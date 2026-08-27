"""LeRobot episode recorder for the mantis teleop (config section `record:`)."""
import collections
import os
import re
import queue
import threading
import time
from pathlib import Path

import numpy as np

os.environ.setdefault("HF_HUB_OFFLINE", "1")

CAM_STALE_S = 1.5


class EpisodeRecorder:
    def __init__(self, node, cfg):
        import teleop_mantis as T
        from sensor_msgs.msg import Image
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

        self.T = T
        self.node = node
        self.log = node.get_logger()
        self.fps = int(cfg.get("fps", 15))
        name = re.sub(r"[^A-Za-z0-9_.-]+", "_", os.environ.get("RECORD_NAME", "").strip())
        self.task = os.environ.get("RECORD_TASK", "").strip() \
            or str(cfg.get("task", "mantis teleop"))
        ns = str(cfg.get("hf_namespace", "mantis")).strip().strip("/")
        self.repo_id = f"{ns}/{name}" if name else str(cfg.get("repo_id", f"{ns}/teleop"))
        self.root = Path(T._cfg_path(str(cfg.get("root", "lerobot_data"))))
        if name:
            self.root = self.root / name
        self.min_frames = int(cfg.get("min_frames", 5))
        self.writer_threads = int(cfg.get("image_writer_threads", 4))
        self.vcodec = str(cfg.get("vcodec", "libsvtav1"))
        self.depth_as_video = bool(cfg.get("depth_as_video", True))
        self.streaming = bool(cfg.get("streaming_encoding", False))
        self.batch = max(1, int(cfg.get("batch_encoding_size", 1)))
        self.debug_frames = bool(cfg.get("debug_keep_frames", False))
        cams = dict(cfg.get("cameras") or {"echo": "/camera/echo_camera/color/image_raw"})
        self.color_keys = sorted(cams)
        depth_cams = dict(cfg.get("depth_cameras") or {})
        unknown = set(depth_cams) - set(cams)
        if unknown:
            raise ValueError(f"record.depth_cameras has {sorted(unknown)}, not in record.cameras")
        self.depth_of = {f"{k}_depth": t for k, t in sorted(depth_cams.items())}
        cams = {**cams, **self.depth_of}
        self.cam_keys = self.color_keys + sorted(self.depth_of)
        self.primary = str(cfg.get("primary_cam") or self.color_keys[0])
        if self.primary not in self.color_keys:
            raise ValueError(f"record.primary_cam {self.primary!r} not in record.cameras")
        self.pair_tol_s = float(cfg.get("pair_tol_s", 0.75 / max(self.fps, 1)))
        self._pending = None
        self._dropped = 0
        self._skew_ms = {k: [] for k in self.cam_keys if k != self.primary}
        self._skew_signed = {}

        self.recording = False
        self.episode_index = None
        self._frames = 0
        self._saving = False
        self._save_thread = None
        self._save_q = queue.Queue()
        self._next_ep_idx = None
        self._dataset = None
        self._lr = None
        self._lr_err = None
        self._img_msg = {}
        self._img_hist = {}
        self._skew_est = {}
        self._skip_warn_t = 0.0

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        for key in self.cam_keys:
            node.create_subscription(
                Image, cams[key],
                (lambda k: lambda m: self._on_image(k, m))(key), qos)

        threading.Thread(target=self._preload, daemon=True).start()
        self.log.info(f"recorder: dataset {self.repo_id} at {self.root}, "
                      f"cams {cams} @ {self.fps} fps (lerobot loading in the background)")

    def _rgb_encoder(self):
        """Colour encoder carrying record.vcodec; depth gets DepthEncoderConfig's own defaults."""
        return self._lr["RGBEncoderConfig"](vcodec=self.vcodec)

    def _patch_depth_image_writer(self):
        """Teach lerobot 0.4.4's image writer to save a 16-bit depth PNG."""
        from lerobot.datasets import image_writer as iw
        import PIL.Image

        if getattr(iw, "_mantis_depth_patch", False):
            return
        original = iw.image_array_to_pil_image

        def with_depth(image_array, range_check: bool = True):
            a = np.asarray(image_array)
            if a.ndim == 3 and a.shape[-1] == 1:
                a = a[:, :, 0]
            if a.ndim == 2 and a.dtype == np.uint16:
                return PIL.Image.fromarray(a, mode="I;16")
            return original(image_array, range_check)

        iw.image_array_to_pil_image = with_depth
        iw._mantis_depth_patch = True
        self.log.info("recorder: image writer patched for 16-bit depth PNGs")

    def _preload(self):
        """Import lerobot off the executor: torch alone takes seconds and the Bridge must reach 'Teleop ready' without paying for it."""
        try:
            from lerobot.datasets.lerobot_dataset import LeRobotDataset
            from lerobot.utils.feature_utils import hw_to_dataset_features, build_dataset_frame
            from lerobot.configs.video import DepthEncoderConfig, RGBEncoderConfig
            self._lr = {"LeRobotDataset": LeRobotDataset,
                        "hw_to_dataset_features": hw_to_dataset_features,
                        "build_dataset_frame": build_dataset_frame,
                        "RGBEncoderConfig": RGBEncoderConfig,
                        "DepthEncoderConfig": DepthEncoderConfig}
            if self.depth_of and not self.depth_as_video:
                self._patch_depth_image_writer()
            self.log.info("recorder: lerobot loaded")
        except Exception as exc:
            self._lr_err = exc
            self.log.error(f"recorder: lerobot import FAILED ({type(exc).__name__}: {exc}) "
                           f"-> episode recording unavailable")

    def _hw_features(self, img_shapes):
        """Hardware feature dicts in lerobot's convention; the action names are the contract with MantisFollower.action_features."""
        joints = {f"{j}.pos": float for j in self.T.ARM}
        joints["gripper.pos"] = float
        return dict(joints), {**joints, **img_shapes}

    def _features(self, img_shapes):
        """Build the LeRobot feature dict for the action and observation (incl. depth) streams."""
        h2d = self._lr["hw_to_dataset_features"]
        act, obs = self._hw_features(img_shapes)
        features = {**h2d(act, "action", True), **h2d(obs, "observation", True)}
        if not self.depth_as_video:
            for key in self.depth_of:
                features[f"observation.images.{key}"]["dtype"] = "image"
        return features

    def _open_dataset(self, img_shapes):
        LRD = self._lr["LeRobotDataset"]
        features = self._features(img_shapes)
        if (self.root / "meta").is_dir() and not (self.root / "meta" / "episodes").is_dir():
            bad = self.root.with_name(self.root.name + time.strftime(".bad_%Y%m%d_%H%M%S"))
            self.root.rename(bad)
            self.log.warn(f"recorder: leftover EMPTY dataset (0 episodes saved) "
                          f"moved to {bad} -> creating a fresh one")
        if (self.root / "meta").is_dir():
            try:
                ds = LRD.resume(self.repo_id, root=self.root, rgb_encoder=self._rgb_encoder(),
                                depth_encoder=self._lr["DepthEncoderConfig"](),
                                streaming_encoding=self.streaming,
                                batch_encoding_size=self.batch,
                                image_writer_threads=self.writer_threads)
            except Exception as exc:
                raise RuntimeError(
                    f"existing dataset at {self.root} would not load "
                    f"({type(exc).__name__}: {exc}). If the last session was killed "
                    f"without finalize, move the folder away and re-record; episodes "
                    f"from cleanly finished sessions are inside it") from None
            for key, ft in features.items():
                have = ds.features.get(key)
                if have is None or tuple(have["shape"]) != tuple(ft["shape"]):
                    raise RuntimeError(
                        f"existing dataset at {self.root} has feature {key} = "
                        f"{have and have['shape']}, current setup needs {ft['shape']} "
                        f"-> refusing to mix; move the old dataset away")
            if int(ds.fps) != self.fps:
                raise RuntimeError(f"existing dataset fps {ds.fps} != record.fps {self.fps}")
            self.log.info(f"recorder: resuming dataset ({ds.meta.total_episodes} episodes so far)")
        else:
            ds = LRD.create(self.repo_id, self.fps, features, root=self.root,
                            robot_type="mantis_follower", use_videos=True,
                            image_writer_threads=self.writer_threads,
                            rgb_encoder=self._rgb_encoder(),
                            depth_encoder=self._lr["DepthEncoderConfig"](),
                            streaming_encoding=self.streaming,
                            batch_encoding_size=self.batch)
            self.log.info(f"recorder: created new dataset at {self.root}")
        return ds

    def _on_image(self, key, msg):
        self._img_msg[key] = (time.monotonic(), msg)
        if key != self.primary:
            h = self._img_hist.setdefault(key, collections.deque(maxlen=4))
            h.append((self._stamp_s(msg), msg))
        if not self.recording:
            return
        if key == self.primary:
            if self._pending is not None and not self._commit_pending():
                self._dropped += 1
                now = time.monotonic()
                if now - self._skip_warn_t > 2.0:
                    self._skip_warn_t = now
                    self.log.warn("recorder: frame dropped — no secondary image within "
                                  "pair_tol_s of the primary's trigger (camera down, or "
                                  "the streams are not hw-synced?)")
            self._pending = msg

    def _commit_pending(self):
        """Pair the pending primary with the nearest-in-time image of every secondary."""
        p = self._pending
        if p is None:
            return False
        ps = self._stamp_s(p)
        secs, offsets = {}, {}
        for key in self.cam_keys:
            if key == self.primary:
                continue
            hist = self._img_hist.get(key)
            if not hist:
                return False
            est = self._skew_est.get(key)
            bias = float(np.median(est)) if est is not None and len(est) >= 5 else 0.0
            target = ps + bias
            best = min(hist, key=lambda c: abs(c[0] - target))
            if abs(best[0] - target) > self.pair_tol_s:
                return False
            secs[key], offsets[key] = best[1], best[0] - ps
        self._pending = None
        for key, off in offsets.items():
            self._skew_signed.setdefault(key, []).append(1000.0 * off)
            self._skew_est.setdefault(key, collections.deque(maxlen=60)).append(off)
        self._add_frame(p, secs)
        return True

    @staticmethod
    def _to_rgb(msg):
        """sensor_msgs/Image -> HWC uint8 RGB, honoring the row stride."""
        h, w, step = msg.height, msg.width, msg.step
        buf = np.frombuffer(bytes(msg.data), np.uint8)
        enc = msg.encoding.lower()
        ch = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}.get(enc)
        if ch is None:
            raise ValueError(f"unsupported image encoding {msg.encoding!r}")
        img = buf.reshape(h, step)[:, : w * ch].reshape(h, w, ch)
        if enc == "mono8":
            img = np.repeat(img, 3, axis=2)
        elif enc.startswith("bgr"):
            img = img[:, :, [2, 1, 0]]
        else:
            img = img[:, :, :3]
        return np.ascontiguousarray(img)

    @staticmethod
    def _to_depth(msg):
        """sensor_msgs/Image 16UC1 -> (H, W, 1) uint16 millimetres, honoring the row stride."""
        enc = msg.encoding.lower()
        if enc not in ("16uc1", "mono16"):
            raise ValueError(f"depth topic encoding {msg.encoding!r} is not 16UC1/mono16")
        h, w, step = msg.height, msg.width, msg.step
        buf = np.frombuffer(bytes(msg.data), "<u2" if not msg.is_bigendian else ">u2")
        img = buf.reshape(h, step // 2)[:, :w]
        return np.ascontiguousarray(img.astype(np.uint16)[:, :, None])

    def _robot_values(self):
        """Flat lerobot-convention value dicts (action, observation) sampled NOW, or None while the robot state is incomplete."""
        T, n = self.T, self.node
        if not isinstance(n.pos, dict) or not all(j in n.pos for j in T.ARM):
            return None, None
        meas = [float(n.pos[j]) for j in T.ARM]
        cmd = list(map(float, n._last_cmd)) if n._last_cmd is not None else meas
        grip_cmd = float(T.GRIP_CLOSE if n._grip_want_closed else T.GRIP_OPEN)
        grip_meas = float(n.pos[T.GRIP_JOINT]) if (T.GRIP_JOINT and T.GRIP_JOINT in n.pos) \
            else grip_cmd
        act = dict(zip([f"{j}.pos" for j in T.ARM], cmd))
        act["gripper.pos"] = grip_cmd
        obs = dict(zip([f"{j}.pos" for j in T.ARM], meas))
        obs["gripper.pos"] = grip_meas
        return act, obs

    @staticmethod
    def _stamp_s(msg):
        return msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec

    def _add_frame(self, p_msg, sec_msgs):
        try:
            imgs = {self.primary: self._to_rgb(p_msg)}
            for key, msg in sec_msgs.items():
                imgs[key] = self._to_depth(msg) if key in self.depth_of else self._to_rgb(msg)
                self._skew_ms[key].append(
                    1000.0 * abs(self._stamp_s(msg) - self._stamp_s(p_msg)))
            act, obs = self._robot_values()
            if act is None:
                raise ValueError("joint states incomplete")
            bdf = self._lr["build_dataset_frame"]
            obs.update(imgs)
            frame = {**bdf(self._dataset.features, act, "action"),
                     **bdf(self._dataset.features, obs, "observation"),
                     "task": self.task}
            self._dataset.add_frame(frame)
            self._frames += 1
            now = time.monotonic()
            if self._frames == 1:
                self._t_first = now
            self._t_last = now
        except Exception as exc:
            now = time.monotonic()
            if now - self._skip_warn_t > 2.0:
                self._skip_warn_t = now
                self.log.warn(f"recorder: frame skipped ({exc})")

    def start(self):
        """Begin an episode; refuses (with the reason) instead of queueing."""
        if self.recording:
            return False
        if self._lr is None:
            self.log.warn("EPISODE not started: lerobot still loading"
                          if self._lr_err is None else
                          f"EPISODE unavailable: lerobot import failed ({self._lr_err})")
            return False
        act, _ = self._robot_values()
        if act is None:
            self.log.warn("EPISODE not started: joint states incomplete")
            return False
        now = time.monotonic()
        shapes = {}
        for key in self.cam_keys:
            t, msg = self._img_msg.get(key, (0.0, None))
            if msg is None or now - t > CAM_STALE_S:
                self.log.warn(f"EPISODE not started: camera '{key}' has no fresh image "
                              f"(is the camera stack up?)")
                return False
            shapes[key] = (self._to_depth(msg) if key in self.depth_of else self._to_rgb(msg)).shape
        if self._dataset is None:
            try:
                self._dataset = self._open_dataset(shapes)
            except Exception as exc:
                self.log.error(f"EPISODE not started: dataset open failed ({exc})")
                return False
        self._frames = 0
        self._skew_ms = {k: [] for k in self.cam_keys if k != self.primary}
        self._skew_signed = {}
        self._pending = None
        self._dropped = 0
        self._t_first = self._t_last = 0.0
        if self._next_ep_idx is None:
            self._next_ep_idx = self._dataset.meta.total_episodes
        self.episode_index = self._next_ep_idx
        self.recording = True
        self.log.info(f"EPISODE {self.episode_index} RECORDING "
                      f"(task: {self.task!r}; MENU again to stop)")
        return True

    def stop(self):
        """End the episode; the save (video encode) runs on a worker thread."""
        if not self.recording:
            return False
        if self._pending is not None:
            self._commit_pending()
            self._pending = None
        self.recording = False
        n, idx = self._frames, self.episode_index
        if self._dropped:
            self.log.warn(f"recorder: {self._dropped} frames dropped this episode "
                          f"waiting for a matching secondary image")
        if n >= 2 and self._t_last > self._t_first:
            real = (n - 1) / (self._t_last - self._t_first)
            if abs(real - self.fps) > 0.1 * self.fps:
                self.log.warn(
                    f"recorder: cameras delivered {real:.1f} Hz but the dataset declares "
                    f"fps={self.fps} -> episode duration and REPLAY SPEED will be off by "
                    f"{real / self.fps:.2f}x. Fix the camera rate (bandwidth? point clouds?) "
                    f"or set record.fps + color_fps to the real rate and start a new dataset")
            else:
                self.log.info(f"recorder: measured frame rate {real:.1f} Hz (declared {self.fps})")
        for key, sk in self._skew_ms.items():
            if sk:
                a = np.asarray(sk)
                sg = np.asarray(self._skew_signed.get(key) or [0.0])
                med = float(np.median(sg))
                self.log.info(
                    f"cam sync {key}-{self.primary}: p50={np.percentile(a, 50):.1f}ms "
                    f"p95={np.percentile(a, 95):.1f}ms max={a.max():.1f}ms over {len(a)} frames "
                    f"| signed median {med:+.1f}ms ({'lags' if med > 0 else 'leads'} the primary)"
                    + ("" if np.percentile(a, 95) < 1000.0 / self.fps * 0.5 else
                       " — POOR SYNC, check the femto-mega trigger cable"))
        if n < self.min_frames:
            self.log.warn(f"EPISODE {idx} DISCARDED ({n} frames < min_frames {self.min_frames})")
            w = self._dataset.writer
            w.episode_buffer = w._create_episode_buffer(episode_index=self._next_ep_idx)
            return True
        self._saving = True
        defer = self.batch > 1 and not self.streaming
        self.log.info(f"EPISODE {idx} STOPPED ({n} frames, {n / self.fps:.1f}s) -> "
                      + ("queued (saved and encoded at teleop exit)" if defer
                         else "queued for saving"))
        w = self._dataset.writer
        buf = w.episode_buffer
        self._next_ep_idx += 1
        w.episode_buffer = w._create_episode_buffer(episode_index=self._next_ep_idx)
        if self._save_thread is None or not self._save_thread.is_alive():
            self._save_thread = threading.Thread(target=self._save_loop, daemon=True)
            self._save_thread.start()
        self._save_q.put((idx, n, buf))
        return True

    def _save_loop(self):
        """Drain queued episodes one at a time; recording keeps running alongside."""
        while True:
            item = self._save_q.get()
            try:
                if item is None:
                    return
                self._save_worker(*item)
            finally:
                self._save_q.task_done()

    def _save_worker(self, idx, n, buf):
        try:
            os.setpriority(os.PRIO_PROCESS, 0, 19)
        except OSError:
            pass
        t0 = time.monotonic()
        try:
            if self.debug_frames:
                self._keep_frames(idx)
            self._dataset.save_episode(episode_data=buf, parallel_encoding=False)
            pending = int(getattr(getattr(self._dataset, "writer", None),
                                  "_episodes_since_last_encoding", 0))
            if pending > 0:
                self.log.info(f"EPISODE {idx} SAVED ({n} frames, "
                              f"total {self._dataset.meta.total_episodes} episodes; "
                              f"{pending} video(s) deferred to teleop exit)")
            else:
                self.log.info(f"EPISODE {idx} SAVED ({n} frames, encode {time.monotonic() - t0:.1f}s, "
                              f"total {self._dataset.meta.total_episodes} episodes)")
        except KeyboardInterrupt:
            print(f"recorder: EPISODE {idx} LOST — the encode was interrupted "
                  f"(second Ctrl-C, or the process group was killed)", flush=True)
            try:
                self._dataset.clear_episode_buffer()
            except (Exception, KeyboardInterrupt):
                pass
        except Exception as exc:
            self.log.error(f"EPISODE {idx} SAVE FAILED ({type(exc).__name__}: {exc})")
            try:
                self._dataset.clear_episode_buffer()
            except Exception:
                pass
        finally:
            self._saving = False

    def _keep_frames(self, idx):
        """debug_keep_frames: hardlink the episode's PNGs aside BEFORE save_episode encodes and deletes them (see the flag's comment)."""
        import shutil
        try:
            self._dataset.writer._wait_image_writer()
            dbg = self.root / "frames_debug" / f"episode-{idx:06d}"
            kept = 0
            for key in self.cam_keys:
                src = self.root / "images" / f"observation.images.{key}" / f"episode-{idx:06d}"
                if src.is_dir():
                    shutil.copytree(src, dbg / key, copy_function=os.link)
                    kept += len(list((dbg / key).glob("frame-*.png")))
            self.log.info(f"recorder: {kept} frames kept for the SYNC CHECK at {dbg} "
                          f"(frame-N of left and right = the same trigger)")
        except Exception as exc:
            self.log.warn(f"recorder: frame keep failed ({exc})")

    def _flush_deferred_videos(self, pending):
        """batch_encoding_size > 1: encode the last `pending` episodes' videos after finalize, from the on-disk dataset files."""
        import glob
        import shutil
        import pandas as pd
        from lerobot.datasets.dataset_writer import _encode_video_worker
        from lerobot.datasets.io_utils import get_file_size_in_mb
        from lerobot.datasets.utils import DEFAULT_VIDEO_PATH, update_chunk_file_indices
        from lerobot.datasets.video_utils import (concatenate_video_files,
                                                  get_video_duration_in_s)
        ds = self._dataset
        root = Path(ds.root)
        chunks_size = int(ds.meta.chunks_size)
        cap_mb = float(ds.meta.video_files_size_in_mb)
        total = int(ds.meta.total_episodes)
        ep_paths = sorted(glob.glob(str(root / "meta" / "episodes" / "**" / "*.parquet"),
                                    recursive=True))
        dfs = {p: pd.read_parquet(p) for p in ep_paths}
        t0 = time.monotonic()
        for key in ds.meta.video_keys:
            col = f"videos/{key}/chunk_index"
            chunk_idx = file_idx = None
            for p in reversed(ep_paths):
                df = dfs[p]
                if col in df.columns and df[col].notna().any():
                    last = df[df[col].notna()].iloc[-1]
                    chunk_idx = int(last[col])
                    file_idx = int(last[f"videos/{key}/file_index"])
                    break
            for ep_idx in range(total - pending, total):
                enc = (self._lr["DepthEncoderConfig"]() if key.endswith("_depth")
                       else self._rgb_encoder())
                tmp = _encode_video_worker(key, ep_idx, root, self.fps, enc)
                dur = get_video_duration_in_s(tmp)
                if chunk_idx is None:
                    chunk_idx, file_idx = 0, 0
                dst = root / DEFAULT_VIDEO_PATH.format(
                    video_key=key, chunk_index=chunk_idx, file_index=file_idx)
                if dst.exists() and \
                        get_file_size_in_mb(dst) + get_file_size_in_mb(tmp) >= cap_mb:
                    chunk_idx, file_idx = update_chunk_file_indices(
                        chunk_idx, file_idx, chunks_size)
                    dst = root / DEFAULT_VIDEO_PATH.format(
                        video_key=key, chunk_index=chunk_idx, file_index=file_idx)
                if dst.exists():
                    from_ts = get_video_duration_in_s(dst)
                    concatenate_video_files([dst, tmp], dst)
                else:
                    from_ts = 0.0
                    dst.parent.mkdir(parents=True, exist_ok=True)
                    shutil.move(str(tmp), str(dst))
                shutil.rmtree(tmp.parent, ignore_errors=True)
                for df in dfs.values():
                    m = df["episode_index"] == ep_idx
                    if m.any():
                        df.loc[m, col] = chunk_idx
                        df.loc[m, f"videos/{key}/file_index"] = file_idx
                        df.loc[m, f"videos/{key}/from_timestamp"] = from_ts
                        df.loc[m, f"videos/{key}/to_timestamp"] = from_ts + dur
                        break
        for p, df in dfs.items():
            df.convert_dtypes(dtype_backend="pyarrow").to_parquet(p)
        print(f"recorder: {pending} episode video(s) encoded in "
              f"{time.monotonic() - t0:.1f}s", flush=True)

    def shutdown(self):
        """Clean teardown: finish the open episode, wait for the encoder, and finalize the dataset."""
        if self.recording:
            self.stop()
        t = self._save_thread
        if t is not None and t.is_alive():
            left = self._save_q.qsize()
            print(f"recorder: writing {left or 1} queued episode(s) to disk before exit "
                  f"(a few seconds each). Press Ctrl-C again to abandon them.", flush=True)
            try:
                self._save_q.put(None)
                t.join(timeout=600.0)
            except KeyboardInterrupt:
                print("recorder: save wait interrupted -> the episodes still queued are dropped",
                      flush=True)
        if self._dataset is not None:
            try:
                w = getattr(self._dataset, "writer", None)
                if w is not None:
                    w.stop_image_writer()
                pending = int(getattr(w, "_episodes_since_last_encoding", 0))
                if w is not None and pending:
                    w._episodes_since_last_encoding = 0
                self._dataset.finalize()
                if pending > 0:
                    print(f"recorder: encoding {pending} deferred episode video(s) at "
                          f"full speed — WAIT for 'dataset finalized', a Ctrl-C here "
                          f"loses the videos...", flush=True)
                    try:
                        self._flush_deferred_videos(pending)
                    except KeyboardInterrupt:
                        print("recorder: deferred encode INTERRUPTED — episodes left "
                              "WITHOUT video; the dataset will not load, move it aside "
                              "or re-record", flush=True)
                    except Exception as exc:
                        print(f"recorder: deferred encode FAILED ({type(exc).__name__}: "
                              f"{exc}) — episodes left WITHOUT video", flush=True)
                print(f"recorder: dataset finalized "
                      f"({self._dataset.meta.total_episodes} episodes)", flush=True)
            except (Exception, KeyboardInterrupt) as exc:
                print(f"recorder: finalize failed: {exc}", flush=True)

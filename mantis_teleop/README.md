# Mantis VR teleop (Meta Quest 2)

VR teleoperation of the PRL **mantis** (dual UR5 — left arm + Weiss WSG50 gripper) over
ROS 2, with one-button recording straight into a stock **LeRobot v3** dataset.

A controller publisher turns Quest 2 (or HTC Vive) poses into `/vive/pose` +
`/vive/buttons`; a bridge maps them onto the arm through Pink differential IK with a
collision barrier. Episodes replay with the stock `lerobot-replay` CLI.

```
operator hand ─► quest_pub.py ─► /vive/pose, /vive/buttons ─► teleop_mantis.py ─► /forward_position_controller/commands
   (host: USB adb, or ALVR/SteamVR)                             (container: Pink IK + collision barrier)
```

> This targets the **mantis** rig (two UR5s, a WSG50 on the left arm, Femto Mega cameras,
> a Quest 2). The *software* install below is meant to run start-to-finish with no prior
> knowledge; the hardware-specific values you must set for your own bench are listed under
> [Before the first run on hardware](#before-the-first-run-on-hardware).

**Contents:** [What you need](#what-you-need) · [Install](#install) · [First run](#first-run-smoke-test)
· [Running](#running) · [Controls](#controls-right-touch-controller) · [Recording a dataset](#recording-a-dataset)
· [Replay](#replay) · [Health check & diagnostics](#health-check--diagnostics) · [How it works](#how-it-works)
· [Troubleshooting](TROUBLESHOOTING.md) · [Files](#files) · [Workspace patches](#workspace-patches)
· [Environment variables](#environment-variables-quest_pubpy)

---

## What you need

**Hardware**
- Meta Quest 2 with a Touch controller, in **developer mode**, and a USB-C cable to the workstation.
- The mantis bench: two UR5s, a Weiss WSG50 gripper on the left arm, (optional) Femto Mega cameras for recording.

**Software on the workstation**
- Linux with a working **`docker`** (user in the `docker` group), and an NVIDIA runtime if you use the GPU flags.
- **`git`** and **`git-lfs`**.
- **`adb`** (`android-tools-adb`) for the Quest-over-USB backend.
- Everything else — ROS 2 Jazzy, the teleop Python stack, lerobot — lives inside the container image, so you do **not** install it on the host.

Only one thing in the whole setup needs root: a udev rule so `adb` can see the headset
(and even that is avoidable — see [Troubleshooting](TROUBLESHOOTING.md)). Everything else
installs into your home directory.

---

## Install

Setup is **two passes of the same script** around one container build: the first pass lays
out the workspace, the second (once the container exists) pulls the ROS packages that only
live inside the image and writes the URDF the teleop loads.

```bash
# 1. the repository
git clone https://github.com/khoviadin442/prl_ur5_ros2.git ~/rabota/prl_ur5_ros2
cd ~/rabota/prl_ur5_ros2 && git checkout mantis-teleop
./mantis_teleop/setup_workstation.sh          # first pass: reports the container packages as MISSING (expected)

# 2. the host controller environment (a pixi workspace; no root needed)
git clone https://github.com/khoviadin442/SO-100-HTC-vive-teleop.git ~/rabota/SO-100-HTC-vive-teleop
cd ~/rabota/SO-100-HTC-vive-teleop && pixi install

# 3. build the image and enter the container (first run builds it, ~30 min: torch + lerobot are several GB)
cd ~/rabota/prl_ur5_ros2/docker-ros2
./start_docker.bash mantis ~/rabota/docker_shared

# 4. inside the container
cd ~/share/mantis_ws && colcon build --symlink-install
pip install -e ~/share/lerobot_robot_mantis        # only needed for replay

# 5. back on the host, container still up — completes the prefix and writes the URDF
cd ~/rabota/prl_ur5_ros2 && ./mantis_teleop/setup_workstation.sh   # second pass: "generated (N lines)"
```

If any step behaves differently from what is written here,
**[TROUBLESHOOTING.md](TROUBLESHOOTING.md)** lists every failure this setup is known to
produce and what it means.

### The host controller environment

`quest_pub.py` runs on the host and needs a Python where
`import rclpy, numpy` passes (plus `openvr` for the ALVR backend, or `oculus_reader, ppadb`
for USB). The SO-100 pixi workspace from step 2 provides all of it with no root; its
`pixi.toml` already lists every package. Verify it:

```bash
source ~/rabota/SO-100-HTC-vive-teleop/.pixi/envs/default/setup.bash
which python && python -c "import rclpy, numpy; print('host env OK')"
```

`which python` must point inside `.pixi/envs/default/bin`. If your environment lives
elsewhere, point the launch scripts at it with env vars instead of editing them:

```bash
export ROS_SETUP=/path/to/setup.bash          # host ROS env
export HOST_DEPS=/path/to/fakeprefix          # AMENT_PREFIX_PATH entry with the mesh packages
export OCULUS_READER=/path/to/oculus_reader   # USB backend only
export QUEST_DEPS=/path/to/quest_deps         # USB backend only (ppadb)
```

### The Quest-over-USB backend (recommended)

The USB (adb) backend needs the `oculus_reader` package and its APK on the headset:

```bash
git clone https://github.com/rail-berkeley/oculus_reader "$OCULUS_READER"
pip3 install --target "$QUEST_DEPS" pure-python-adb
```

The APK in that repo is stored in **Git LFS** — without `git-lfs` the clone gets a
132-byte stub and installation later fails with `Failed to parse APK file`. Fetch it
directly if that happens:

```bash
cd "$OCULUS_READER/oculus_reader/APK"
curl -sL -o teleop-debug.apk \
  https://media.githubusercontent.com/media/rail-berkeley/oculus_reader/main/oculus_reader/APK/teleop-debug.apk
```

Plug in the headset, accept the "Allow USB debugging" prompt inside the Quest, and confirm:

```bash
adb devices          # the headset must be listed as 'device', not 'unauthorized' / 'no permissions'
```

### ADB on the lab PC (adb server in a container)

The Quest talks to the workstation over `adb`. Rather than depend on the host's adb version
and USB permissions, this rig runs the **adb server itself inside a small privileged
container** that owns the USB bus and shares the host network — so every adb client (the
host `adb`, the mantis container, and `quest_pub` via `ppadb`) talks to one server on
`127.0.0.1:5037`.

Why a container:
- **Pinned adb binary** — it runs the `platform-tools` adb from a read-only bind (`/pt`), so the version never drifts.
- **USB without host udev fuss** — `--privileged --device=/dev/bus/usb` hands it the headset directly.
- **Auth persists** — `~/.android` is bind-mounted, so you approve "Allow USB debugging" on the headset once.
- **One shared server** — `--net=host` means the daemon on `127.0.0.1:5037` serves the host and every container, avoiding "adb server version mismatch" churn when several clients each start their own.

Create it once (the image is the teleop image, tagged with your username):

```bash
docker run -d --name adb-server \
  --privileged --net=host \
  --device=/dev/bus/usb \
  -v ~/platform-tools:/pt:ro \
  -v ~/.android:/root/.android \
  --entrypoint /pt/adb \
  prl_ros2:$(id -un) \
  nodaemon server
```

Daily use:

```bash
docker start adb-server           # if it is not already up
adb kill-server                   # kill any host-local server so clients use the container's
adb devices                       # the headset should read 'device'
adb shell am broadcast -a com.oculus.vrpowermanager.prox_close   # keep it awake while off your head
```

If tracking degrades mid-session, `adb reboot` the headset (see
[Health check & diagnostics](#health-check--diagnostics)). `unauthorized` in `adb devices`
means re-accept the prompt inside the headset; `no permissions` is the one udev rule that
needs root (see [TROUBLESHOOTING.md](TROUBLESHOOTING.md)).

### What `setup_workstation.sh` does

It copies the teleop files into the container share dir (`~/rabota/docker_shared`), clones
and patches `mantis_ws`, builds the host package prefix, and generates `mantis.urdf`. It
**never overwrites** what is already there — pass `--force-files` / `--force-patches` when
that is what you want, and delete `mantis_ws/mantis.urdf` to have it regenerated. Paths are
overridable via `SHARE_DIR`, `HOST_DEPS`, `CONTAINER_NAME`, `ROS_SETUP`.

The workspace build lives in the share dir, so it survives the container. The editable
`lerobot_robot_mantis` install does **not**, because `start_docker.bash` runs with `--rm` —
repeat step 4's `pip install` in each new container that needs `run_replay.sh`.

---

## First run (smoke test)

Before touching the robot, confirm the pieces talk to each other. With the container up and
the headset plugged in:

```bash
# host: what does the controller actually report? (publishes nothing)
~/rabota/docker_shared/run_quest_pub.sh --scan

# host: start the publisher, then in the container check the stream arrives
~/rabota/docker_shared/run_quest_pub.sh
#   in the container:
ros2 topic hz /vive/pose        # healthy ≈ 70 Hz (adb) / 250 Hz (ALVR); see Health check below
ros2 topic echo /vive/buttons   # 6 fields, changing as you press buttons
```

If `/vive/pose` is silent, the headset is asleep or out of tracking — see
[Health check & diagnostics](#health-check--diagnostics).

---

## Running

Canonical setup: robot stack and teleop **in the container**, controller publisher **on the host**.

```bash
# terminal 1 (container) — hardware; wait for "forward_position_controller ... activated"
ros2 launch prl_ur5_run real.launch.py            # add activate_cameras:=true to record
# terminal 2 (container) — teleop; wait for "Teleop ready"
python3 ~/share/teleop_mantis.py
# terminal 3 (host) — Quest publisher (defaults to the USB adb backend, extrapolate mode)
~/rabota/docker_shared/run_quest_pub.sh
```

- Everything on the host in one command, no container: `./run_teleop_quest.sh`.
- The Vive equivalents are `run_vive_pub.sh` and `run_teleop_mantis.sh`.
- **Run only one publisher at a time** — they share the topics.

`run_quest_pub.sh` is preconfigured for this rig (`QUEST_BACKEND=adb`,
`QUEST_ADB_MODE=extrapolate`, `QUEST_ROT_OFFSET=-90,0,0`); every default is overridable on
the command line, e.g. `QUEST_ADB_MODE=native ./run_quest_pub.sh` for the raw stream.

---

## Controls (right Touch controller)

| Button | Action |
|---|---|
| **thumbstick click** | engage / freeze |
| **trigger**, squeezed fully | gripper toggle: one click closes, the next opens |
| **B** | episode start / stop (start from frozen also engages; stop freezes and drives HOME) |
| **A** | HOME ramp to the home pose; press again to cancel |
| **grip**, held | axis lock ("drawer mode"): orientation frozen, motion constrained to the gripper axis |

The Touch trigger has no mechanical click, so it is derived from the analog value with
hysteresis. Two buttons must never share one physical control (quest_pub warns if they do).

**Tracking is inside-out:** the headset's own cameras must see the controller. Keep the
headset facing your hands (wear it, or aim it at the workspace) in decent, even light. If
you park it on a desk, the controller drifts out of view, tracking drops, and motion turns
choppy — see [Health check & diagnostics](#health-check--diagnostics).

---

## Recording a dataset

The `record:` section of `config_teleop_mantis.yaml` controls the recorder (dataset root,
`fps`, cameras). Recording needs the cameras up (`activate_cameras:=true` on the robot launch).

**Before you record**, confirm the pose stream is healthy — a degraded stream produces
choppy, jittery episodes (see [Health check](#health-check--diagnostics)). Then:

1. **Engage** with the thumbstick and move the arm to a start pose.
2. Press **B** to start an episode. The recorder logs `EPISODE N RECORDING`.
3. Perform the task. Toggle the **gripper** with the trigger as needed.
4. Press **B** again to stop. The arm freezes and drives HOME; the episode is queued to save.
5. Repeat 1–4 for each episode.
6. **Exit the teleop with Ctrl-C** — this is what *finalizes the dataset* and (with
   `batch_encoding_size > 1`) encodes the videos. Wait for `dataset finalized`; do not
   kill it twice or you lose the queued episodes.

Notes:
- `record.fps` **must equal the camera stream rate** — frames are added at the primary
  camera's rate, and a mismatch throws off the replay speed (the recorder warns if it sees one).
- Name a dataset by exporting `RECORD_NAME=my_task` before launching the teleop; it becomes
  the folder under `record.root` and the `<hf_namespace>/my_task` repo id for `push_to_hub`.
- An episode shorter than `record.min_frames` is discarded (this is why MENU is debounced —
  a double-tap would otherwise start and instantly discard one).

## Replay

Replay an episode with the stock CLI through the plugin (teleop must be stopped — both
publish the arm command topic):

```bash
pip install -e lerobot_robot_mantis      # once, in the container
./run_replay.sh 0                        # episode 0
./run_replay.sh 3 my_task                # episode 3 of the RECORD_NAME=my_task dataset
```

---

## Health check & diagnostics

Almost every "teleop feels bad" symptom — choppy motion, flicks, a laggy gripper, jerky
recordings — traces back to a **degraded Quest tracking stream**, not the robot or the code.
Watch the pose rate; it is the single best health signal.

**Healthy vs degraded** (the teleop prints a `POSESTREAM` line every 10 s, and `ros2 topic
hz /vive/pose` shows it live):

| | healthy | degraded |
|---|---|---|
| `/vive/pose` rate | ~70 Hz | ~50 Hz |
| jitter (p95 interval) | ~24 ms | ~52 ms |
| feel | smooth, responsive gripper | choppy, gripper lags |

When the rate drops below `teleop.pose_hz_warn` (65 Hz) the teleop logs
**`QUEST POSE RATE LOW`**. That means the headset lost solid 6-DOF tracking of the
controller (out of camera view, or warm after hours of use). To fix it:

1. **Reboot the headset** — the "reset to morning" button: `adb reboot`, then restart the publisher.
2. Keep the headset **cool** and the controller **in the cameras' view** in good light.
3. Fresh AA in the controller for a long session; take breaks so the headset doesn't overheat.

Confirm the mechanism directly (should read `POSITION`/6-DOF, not `ORIENTATION`):

```bash
adb shell dumpsys OVRRemoteService | grep -iE "TrackingStatus|tracking lost"
```

`run_quest_pub.sh` defaults to **`extrapolate`** mode, which resamples the raw stream to a
steady 250 Hz (predicted from the last two samples) so motion and the gripper stay smooth
even at ~50 Hz — but it cannot repair genuinely bad tracking, so still fix the headset when
the warning fires.

### Session monitor (optional)

`teleop_monitor.py` (host) samples the whole system once a second and, afterward,
correlates it with the teleop log, the UR driver log, and the recorded dataset — UDP drops,
CPU/thermal, robot-network ping, Quest thermals, per-episode camera loss, and flick
signatures — then prints ranked problems and what it ruled out:

```bash
~/rabota/docker_shared/teleop_monitor.py start     # before a session
#   ... record ...
~/rabota/docker_shared/teleop_monitor.py report    # any time, also mid-session
~/rabota/docker_shared/teleop_monitor.py stop
```

`udp_watch.py` is a lighter tool that only tracks UDP receive-buffer drops per teleop state
(`start` / `report` / `stop`).

---

## How it works

Three processes, split across the host and the container.

1. **Controller publisher (`quest_pub.py`, host).** Reads the Quest controller pose over USB
   — adb talks to the `oculus_reader` APK on the headset, which logs poses that
   `oculus_reader` parses via `adb logcat` — and republishes it as `/vive/pose` +
   `/vive/buttons`. In `extrapolate` mode it resamples the raw ~50–70 Hz headset stream to a
   steady 250 Hz predicted from the last two samples.

2. **Teleop bridge (`teleop_mantis.py`, container).** The core loop:
   - on *engage*, anchors the controller pose to the arm and maps hand motion → a target
     end-effector pose (scaled, heading-normalized, one-euro filtered);
   - solves for joint velocities with **Pink differential IK** under a **collision barrier**
     (a control-barrier-function QP that makes the arm *dodge* instead of freeze), plus
     singularity damping, joint-lead clamps, and a not-following watchdog;
   - re-publishes the IK result through an output **shaper thread at 250 Hz** under
     velocity/acceleration limits — this is why arm motion stays smooth even when the input
     stream or the ROS executor hiccups;
   - drives the WSG50 gripper on its own timer, independent of IK health, via a
     `GripperCommand` action.

3. **Recorder (`lerobot_recorder.py`, inside the bridge).** MENU starts/stops an episode;
   each primary-camera frame stores one LeRobot frame (state = measured joints + gripper,
   action = commanded), pairs the secondary/depth cameras by nearest timestamp, and writes a
   stock LeRobot v3 dataset. Video encoding is deferred to teleop exit so it never competes
   with the control loop.

Host ↔ container traffic is ROS 2 DDS forced to **UDP-only** (`fastdds_udp_only.xml`),
because FastDDS shared memory does not cross the container boundary.

**Two design choices that explain the behaviour you see:** the arm command runs on its own
250 Hz thread, decoupled from the ROS executor, so camera and recording load never stutter
the arm — but the gripper and buttons *do* share the executor, which is why they are the
first thing to feel a degraded input stream. And `extrapolate` mode plus the collision
barrier are the two levers that keep motion usable when tracking or IK gets hard.

## Files

| File | What it is |
|---|---|
| `teleop_mantis.py` | the teleop bridge: VR pose → diff-IK → arm + gripper commands |
| `config_teleop_mantis.yaml` | every tuned parameter (IK, teleop mapping, safety gates, gripper, recording) |
| `quest_pub.py` | Meta Quest 2 controller publisher (backends: ALVR/SteamVR or adb) |
| `vive_pub.py` | HTC Vive publisher, same message contract |
| `lerobot_recorder.py` | LeRobot episode recorder driven by the teleop's MENU button |
| `lerobot_robot_mantis/` | lerobot plugin registering `--robot.type=mantis_follower` for replay |
| `run_*.sh` | launch wrappers |
| `teleop_monitor.py`, `udp_watch.py` | host diagnostics (see Health check) |
| `setup_workstation.sh` | one-shot setup of a new machine: share dir, workspace, patches, host prefix, URDF |
| `TROUBLESHOOTING.md` | what the failures seen while setting a machine up actually mean |
| `fastdds_udp_only.xml` | UDP-only DDS profile, needed for host ↔ container traffic |
| `patches/` | changes made to the ROS workspace packages, mirrored by path |

**Container side** (the docker image from `prl_ur5_ros2/docker-ros2`): ROS 2 Jazzy, the
`mantis_ws` workspace built, `pinocchio`, `pin-pink`, `qpsolvers`, `scipy`, and
`lerobot[async,dataset]==0.6.1` for recording/replay. The Dockerfile in this fork installs
all of it. **Host side**: the pixi environment above.

---

## Workspace patches

`patches/` mirrors the paths of the ROS packages that had to be changed; the setup script
copies each over the corresponding file in `mantis_ws/src/` and rebuilds. When this tree is
published inside the `prl_ur5_ros2` fork, its three `prl_ur5_ros2` patches are already
applied in that branch and the copies here are kept only for reference.

| Path | Change |
|---|---|
| `prl_ur5_robot_configuration/config/standard_setup.yaml` | left arm gets the WSG50 gripper and the teleop joint limits |
| `prl_ur5_robot_configuration/config/limits/teleop_joint_limits.yaml` | new file: ±270° wrists so a roll can complete |
| `prl_ur5_robot_configuration/config/controller_setup.yaml` | `forward_position_controller` active instead of the trajectory controllers |
| `prl_ur5_robot_configuration/config/fixed_cameras/cameras_config.yaml` | third camera (golf) disabled |
| `wsg50-ros-pkg/wsg_50_interface/**` | gripper commands moved off the ros2_control RT loop; open uses ACK + MOVE so a latched fast-stop cannot leave the fingers shut |
| `wsg50-ros-pkg/wsg_50_driver/config/wsg50_setup.yaml` | gripper IP, force and speed |
| `wsg50-ros-pkg/wsg_50_simulation/urdf/wsg_50.urdf.xacro` | wider finger collision meshes + the wire-box connector link |
| `prl_ur5_ros2/docker-ros2/Dockerfile` | the teleop python stack (pinocchio + pink + qpsolvers/daqp), CPU-only torch, `lerobot[async,dataset]==0.6.1` (numpy `<2.3`, protobuf `<7`, `setuptools<80`) |
| `prl_ur5_ros2/docker-ros2/start_docker.bash` | `--ipc=host` |
| `prl_ur5_ros2/prl_ur5_gazebo/launch/start_gazebo_sim.launch.py` | bullet-featherstone world for mimic-joint grippers |

### Regenerating the URDF

After changing the robot configuration, regenerate the URDF the teleop loads (`urdf:` in
the config, by default `mantis_ws/mantis.urdf`): delete it and re-run the setup script with
the container up.

```bash
rm ~/rabota/docker_shared/mantis_ws/mantis.urdf
./mantis_teleop/setup_workstation.sh          # "generated (N lines)"
```

The script runs `mantis.urdf.xacro` against the host package prefix it builds in
`$HOST_DEPS` (default `~/rabota/mantis_host_deps/fakeprefix`) and rewrites the absolute mesh
paths back into `package://` URIs, which both the host and the container resolve through
`AMENT_PREFIX_PATH`. Quick sanity check on a regenerated URDF — the gripper wire box must be
present and no absolute mesh path may survive:

```bash
grep -c 'left_gripper_connector_link' ~/rabota/docker_shared/mantis_ws/mantis.urdf   # 2
grep -o 'package://[a-z0-9_]*' ~/rabota/docker_shared/mantis_ws/mantis.urdf | sort -u
```

---

## Before the first run on hardware

These are per-bench values that no script can guess:

- gripper IP, force and speed — `mantis_ws/src/wsg50-ros-pkg/wsg_50_driver/config/wsg50_setup.yaml`
- arm mounting poses — `prl_ur5_robot_configuration/config/standard_setup.yaml`
- camera topics and `fps` — the `record:` section of `config_teleop_mantis.yaml`

The tuned values in `config_teleop_mantis.yaml` depend on each other (speeds, `out_accel`,
`max_joint_lead`); change them **as a set**, not one line at a time. The one safe first-run
change is lowering `teleop.scale` to `0.5`. **Bring up the simulation before the hardware,
and keep the E-stop within reach.**

---

## Environment variables (quest_pub.py)

| Variable | Default | Meaning |
|---|---|---|
| `QUEST_BACKEND` | `openvr` (`run_quest_pub.sh` sets `adb`) | `openvr` (ALVR/SteamVR) or `adb` (oculus_reader over USB) |
| `QUEST_ADB_MODE` | `native` (`run_quest_pub.sh` sets `extrapolate`) | `native` = one message per headset sample (~50–72 Hz, uneven); `extrapolate` = a steady 250 Hz stream predicted from the last two samples, smoother at low tracking rates. `extrapolate` is recommended; `native` gives the raw, unpredicted stream |
| `QUEST_HAND` | `right` | which controller drives the arm |
| `QUEST_RATE` | `250` | publish/tick rate [Hz] |
| `QUEST_POSE_PREDICTION` | `0.05` | s of forward prediction (openvr and extrapolate) |
| `QUEST_ROT_OFFSET` | — (`run_quest_pub.sh` sets `-90,0,0`) | `rx,ry,rz` in degrees, extra rotation of the controller axes |
| `QUEST_ENGAGE_BUTTON` | `thumbstick` | engage / freeze button |
| `QUEST_MENU_BUTTON` | `b` | episode start / stop |
| `QUEST_HOME_BUTTON` | `a` | HOME ramp |
| `QUEST_AXISLOCK_BUTTON` | `grip` | axis lock |
| `QUEST_TRIGGER_CLICK` | `soft` | `soft` = analog hysteresis, `hw` = driver bit |
| `QUEST_CLICK_ON` / `QUEST_CLICK_OFF` | `0.90` / `0.60` | soft click thresholds |
| `QUEST_UNIVERSE` | `standing` | OpenVR universe (`standing` guarantees up = +Y) |
| `QUEST_BIT_*` | see `--scan` | override the button bits (`QUEST_BIT_A`, `QUEST_BIT_B`, …) |
| `QUEST_POSE_TOPIC` / `QUEST_BUTTONS_TOPIC` | `/vive/pose`, `/vive/buttons` | rename the topics (then also edit `topics:` in the config) |
| `QUEST_ADB_IP` | — | adb over Wi-Fi instead of USB (the cable is ~3× faster) |
| `QUEST_ADB_STALE` | `0.15` | s without data before the pose stream goes silent |
| `QUEST_ADB_KEEPALIVE` | `0.05` | s between resends of an unchanged pose |
| `QUEST_ADB_MAX_EXTRAP` / `_MAX_V` / `_MAX_W` | `0.08` / `3.0` / `12.0` | caps on the extrapolation (how far ahead, and the predicted linear/angular speed) |

Recording (`record:` in the config) is also env-overridable: `RECORD_NAME` (dataset name),
`RECORD_TASK` (task string).

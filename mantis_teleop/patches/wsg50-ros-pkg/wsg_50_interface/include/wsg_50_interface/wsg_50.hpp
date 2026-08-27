#ifndef WSG_50_HPP
#define WSG_50_HPP

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <thread>
#include <chrono>
#include <mutex>
#include <limits>
#include <cmath>


#include "wsg_50_driver/common.h"
#include "wsg_50_driver/cmd.h"
#include "wsg_50_driver/msg.h"
#include "wsg_50_driver/functions.h"

#include <rclcpp/rclcpp.hpp>



#define GRIPPER_MIN_OPEN 2.0 // unit: mm
#define GRIPPER_MAX_OPEN 110.0
#define GRIPPER_MIN_SPEED 0.0
#define GRIPPER_MAX_SPEED 420.0
#define GRIPPER_WIDTH_THRESHOLD 1.0


#define GRIPPER_MIN_OPEN 2.0
#define GRIPPER_MAX_OPEN 110.0


class WSG50Driver{
    public:
        std::string ip_, name_,protocol_;
        int port_,local_port_;
        double rate_;
        double grasping_force_;
        double width_, speed_, force_;
        double goal_width_, goal_speed_;
        bool finger_sensors_;
        std::thread auto_update_thread_;
        int connected_;

        // Async command path: write() only stores the goal, cmd_write_thread_ does
        // the slow TCP writes off the ros2_control RT loop.
        std::thread cmd_write_thread_;
        std::mutex write_mutex_;
        double pending_goal_;
        double pending_speed_;


        WSG50Driver();
        ~WSG50Driver();

        // Connexion / déconnexion
        bool connect();
        bool setup();
        bool disconnect();

        // Commandes principales
        int cmd(double pos, double speed,int mode);

        void read_thread(int interval_ms);
        // Non-blocking: hand the latest goal to the async writer (called from RT write()).
        void request(double goal, double speed);
        // Background command loop: ack_fault -> move (open) / grasp (close) on goal change.
        void write_thread(int interval_ms);

};



#endif
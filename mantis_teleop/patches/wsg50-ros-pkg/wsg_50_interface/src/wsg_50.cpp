#include "wsg_50_interface/wsg_50.hpp"
#include <rclcpp/rclcpp.hpp>

/***
 * @brief Constructor for the WSG50Driver class
 */
WSG50Driver::WSG50Driver(){
    // Start "open" so the first goal-vs-measured direction test is sane.
    width_ = GRIPPER_MAX_OPEN / 1000.0;
    speed_ = 0.0;
    force_ = 0.0;
    goal_width_ = GRIPPER_MAX_OPEN / 1000.0;
    goal_speed_ = 0.0;
    pending_goal_ = std::numeric_limits<double>::quiet_NaN();
    pending_speed_ = 0.0;
    connected_ = 0;
}

/***
 * @brief Destructor for the WSG50Driver class
 */
WSG50Driver::~WSG50Driver(){}

/***    
 * @brief Connecting to the real WSG50 gripper
 * @return true if the connection is successful
 */
bool WSG50Driver::connect(){
    int res_con;
    if(protocol_ == "tcp"){
        RCLCPP_INFO(rclcpp::get_logger("WSG50Driver"), "Connecting to %s:%d", ip_.c_str(), port_);
        res_con=cmd_connect_tcp(ip_.c_str(), port_);
    }
    else if(protocol_ == "udp"){
        RCLCPP_INFO(rclcpp::get_logger("WSG50Driver"), "Connecting to %s:%d:%d", ip_.c_str(), port_, local_port_);
        res_con = cmd_connect_udp(local_port_, ip_.c_str(), port_);
    }
    else{
        RCLCPP_ERROR(rclcpp::get_logger("WSG50Driver"), "Invalid protocol");
        return false;
    }

    if (res_con!=0){
        return false;
    }
    connected_ = 1;
    return true;
}
/***
 * @brief Disconnecting from the WSG50 gripper
 * @return true if the disconnection is successful
 * @note This function will wait for the auto-update thread to finish before disconnecting
 */
bool WSG50Driver::disconnect(){
    connected_ = 0; // Stop the auto-update AND command threads
    bool read_running = false;
    if (auto_update_thread_.joinable()) {
        auto_update_thread_.join(); // Wait for the read thread to finish
        read_running = true;
    }
    if (cmd_write_thread_.joinable()) {
        cmd_write_thread_.join(); // Wait for the command thread to finish
    }
    if (!read_running) {
        return false;
    }
    cmd_disconnect(); // Disconnect from the device
    return true;
}
/***
 * @brief Setup the WSG50 gripper
 * @return true if the setup is successful
 * @note This function will homming or enable the gripper and set the grasping force limit and tare the finger sensors
 */
bool WSG50Driver::setup(){
    if (homing()){// Homing if it is not done that means the gripper is in error case
        ack_fault(); // Acknowledge the fault
        homing();   // Homing again
    }
    rclcpp::sleep_for(std::chrono::milliseconds(500));  // Wait for the gripper to be ready
    if (grasping_force_ > 0.0) { // Set the grasping force limit if it is greater than 0
        RCLCPP_INFO(rclcpp::get_logger("WSG50Driver"), "Setting grasping force limit to %f", grasping_force_);
        setGraspingForceLimit(grasping_force_);
    }
    if (finger_sensors_){ // Tare the finger sensors if it is equipped on the gripper
        if(doTare()==0)
            RCLCPP_INFO(rclcpp::get_logger("WSG50Driver"), "Tare done");
        else
            RCLCPP_ERROR(rclcpp::get_logger("WSG50Driver"), "Tare failed");
    }
    auto_update_thread_ = std::thread(std::bind(&WSG50Driver::read_thread, this, (int)(1000.0 / rate_))); // Start the auto-update thread
    cmd_write_thread_ = std::thread(std::bind(&WSG50Driver::write_thread, this, 20)); // Start the async command thread (~50 Hz)
    return true;
}
/***
 * @brief Move the gripper to a specific position
 * @param pos The position to move to in m
 * @param speed The speed to move at in mm/s
 * @param mode The mode to use (0: move, 1: grasp, 2: release)
 * @return 0 if successful, -1 if failed
 */
int WSG50Driver::cmd(double pos, double speed, int mode){
    float goal_pos = pos * 1000.0; // Convert to mm
    float goal_speed = speed; // mm/s
    switch(mode){
        case 0: // Move
            return move(goal_pos, goal_speed, false, false);
        case 1: // Grasp
            return grasp(goal_pos, goal_speed,true);
        case 2: // Release
            return release(goal_pos, goal_speed,true);
        default:
            RCLCPP_ERROR(rclcpp::get_logger("WSG50Driver"), "Invalid mode");
            return -1;
    }
}
/***
 * @brief Get the current informartion transmitted from the gripper
 * @param interval_ms The interval in ms to update the information
 * @note This function will start as a thread to update the information
 * @note The information will be updated every interval_ms ms
 */
void WSG50Driver::read_thread(int interval_ms){
   // Request automatic updates (error checking is done below)
    getOpening(interval_ms);
    getSpeed(interval_ms);
    getForce(interval_ms);
    int res;
    msg_t msg; msg.id = 0; msg.data = 0; msg.len = 0;

    while(connected_==1){
        msg_free(&msg);
        res = msg_receive( &msg );
        if (res < 0 || msg.len < 2) {
            continue;
        }

        float val = 0.0;
        status_t status = cmd_get_response_status(msg.data);

        // Decode float for opening/speed/force
        if (msg.id >= 0x43 && msg.id <= 0x45 && msg.len == 6) {
            if (status != E_SUCCESS) {
                continue;
            }
            val = convert(&msg.data[2]);
        }
        // Handle response types
        switch (msg.id) {
            /*** Opening ***/
            case 0x43:
                width_ = val/1000.0; // Convert to m
                break;

            /*** Speed ***/
            case 0x44:
                speed_ = val/1000.0; // Convert to m/s
                break;

            /*** Force ***/
            case 0x45:
                force_ = val;
                break;
        }
    }
    // Disconnect from the device
    // TODO: cause some weird response message but not very important
    // getOpening(0);
    // getSpeed(0);
    // getForce(0);
    // std::cout << "Thread stopped" << std::endl;
}

/***
 * @brief Store the latest goal for the async command thread. Non-blocking.
 */
void WSG50Driver::request(double goal, double speed){
    std::lock_guard<std::mutex> lock(write_mutex_);
    pending_goal_ = goal;
    pending_speed_ = speed;
}

/***
 * @brief Background command loop, off the RT cycle. On a goal change it ACKs any
 * latched fast-stop, then opens with a position MOVE (0x21) or closes with a
 * force-controlled GRASP (0x25): a GRASP on a wide part latches a fast-stop that
 * RELEASE cannot clear, and only ACK + MOVE reopens the fingers.
 */
void WSG50Driver::write_thread(int interval_ms){
    double last_goal = std::numeric_limits<double>::quiet_NaN();
    bool last_open = false;
    int retries = 0;
    const double TOL = 0.003;          // m, "reached the open target" dead-band
    const int MAX_RETRY = 5;           // bounded re-ack+re-move if an open does not complete
    const auto RETRY_PERIOD = std::chrono::milliseconds(400);
    auto t_cmd = std::chrono::steady_clock::now();

    while (connected_ == 1){
        double goal, speed;
        {
            std::lock_guard<std::mutex> lock(write_mutex_);
            goal = pending_goal_;
            speed = pending_speed_;
        }
        double meas = width_;          // measured opening [m] (published by read_thread)

        if (!std::isnan(goal)){
            bool changed = std::isnan(last_goal) || std::abs(goal - last_goal) > 1e-4;
            float goal_mm = (float)(goal * 1000.0);
            float speed_mm = (float)speed;
            if (changed){
                bool opening = goal >= meas;                 // direction vs the real finger position
                ack_fault(true);                             // clear any latched fast-stop (send-only)
                std::this_thread::sleep_for(std::chrono::milliseconds(15));   // let the ACK take effect
                if (opening)
                    move(goal_mm, speed_mm, false, true);    // pure position open
                else
                    grasp(goal_mm, speed_mm, true);          // force-controlled close (unchanged)
                RCLCPP_INFO(rclcpp::get_logger("WSG50Driver"), "gripper %s -> %.3f m",
                            opening ? "OPEN(move)" : "CLOSE(grasp)", goal);
                last_goal = goal;
                last_open = opening;
                retries = 0;
                t_cmd = std::chrono::steady_clock::now();
            }
            else if (last_open && std::abs(meas - goal) > TOL && retries < MAX_RETRY
                     && (std::chrono::steady_clock::now() - t_cmd) > RETRY_PERIOD){
                // Opening is never physically blocked, so an unreached target
                // means the command was lost: re-issue, bounded.
                ack_fault(true);
                std::this_thread::sleep_for(std::chrono::milliseconds(15));
                move(goal_mm, speed_mm, false, true);
                retries++;
                t_cmd = std::chrono::steady_clock::now();
                RCLCPP_WARN(rclcpp::get_logger("WSG50Driver"),
                            "gripper OPEN retry %d (meas %.3f, goal %.3f)", retries, meas, goal);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
    }
}
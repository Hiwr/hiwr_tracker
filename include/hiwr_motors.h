//Common
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <algorithm>
#include <math.h>
#include <cmath>

//Threads
#include <thread>

//Ros
#include <ros/ros.h>

//Msgs
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/RegionOfInterest.h"
#include "dynamixel_msgs/JointState.h"
#include "dynamixel_controllers/SetSpeed.h"

//Threads
#include <thread>

class Hiwr_motors_node{
private:
    //Debug variables & methods
    std::string debug_name;
    void print_info(const char *value);
    void print_error(const char * value);

    //Node Handle
    ros::NodeHandle nh;

    //Subscribers
    ros::Subscriber sub_image_size;
    ros::Subscriber sub_roi;

    //Publisher
    ros::Publisher pub_tilt_join;
    ros::Publisher pub_pan_join;

    ros::Publisher pub_pan_torque;
    ros::Publisher pub_tilt_torque;

    ros::Subscriber tilt_state;
    ros::Subscriber pan_state;

    //Services
    ros::ServiceClient  service_pan_speed;
    ros::ServiceClient  service_tilt_speed;

    //Subscribers callbacks
    void callback_ROI(const sensor_msgs::RegionOfInterest::ConstPtr&);

    void callback_pan_state(const dynamixel_msgs::JointStateConstPtr &);
    void callback_tilt_state(const dynamixel_msgs::JointState::ConstPtr &);

    //Params
    float default_joint_speed;
    float max_joint_speed;
    float lead_target_angle;

    float pan_threshold;
    float tilt_threshold;

    float gain_pan;
    float gain_tilt;

    std::string pan_name;
    std::string tilt_name;

    float pan_position;
    float tilt_position;

    float current_pan_pos;
    float current_tilt_pos;

    float pan_speed;
    float tilt_speed;

    float last_tilt_speed;
    float last_pan_speed;

    //Limits
    float max_pan,min_pan,max_tilt,min_tilt;

    unsigned int rate;
    bool target_visible;
    unsigned int target_lost_count;
    unsigned int max_target_lost_count;

    void init_servos();
    void center_head_servos();

    void set_pan_speed(float);
    void set_tilt_speed(float);

    void set_pan_position(float);
    void set_tilt_position(float);

    void set_pan_torque(bool);
    void set_tilt_torque(bool);

    float toRadians(float);

    //Processing variables
    int image_height;
    int image_width;

public:
    Hiwr_motors_node(ros::NodeHandle&);
    void onInit();
    //Main loop
    void loop();
};

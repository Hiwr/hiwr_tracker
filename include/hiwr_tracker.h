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

class HiwrTrackerNode{
private:
    //Debug variables & methods
    std::string debug_name_;
    void printInfo(const char *value);
    void printError(const char * value);

    //Node Handle
    ros::NodeHandle nh_;

    //Subscribers
    ros::Subscriber sub_image_size_;
    ros::Subscriber sub_roi_;

    //Publisher
    ros::Publisher pub_tilt_join_;
    ros::Publisher pub_pan_join_;

    ros::Publisher pub_pan_torque_;
    ros::Publisher pub_tilt_torque_;

    ros::Subscriber tilt_state_;
    ros::Subscriber pan_state_;

    //Services
    ros::ServiceClient  service_pan_speed_;
    ros::ServiceClient  service_tilt_speed_;

    //Subscribers callbacks
    void callbackROI(const sensor_msgs::RegionOfInterest::ConstPtr&);

    void callbackPanState(const dynamixel_msgs::JointStateConstPtr &);
    void callbackTiltState(const dynamixel_msgs::JointState::ConstPtr &);

    //Params
    float default_joint_speed_;
    float max_joint_speed_;
    float lead_target_angle_;

    float pan_threshold_;
    float tilt_threshold_;

    float gain_pan_;
    float gain_tilt_;

    std::string pan_name_;
    std::string tilt_name_;

    float pan_position_;
    float tilt_position_;

    float current_pan_pos_;
    float current_tilt_pos_;

    float pan_speed_;
    float tilt_speed_;

    float last_tilt_speed_;
    float last_pan_speed_;

    //Limits
    float max_pan_,min_pan_,max_tilt_,min_tilt_;

    unsigned int rate_;
    bool target_visible_;
    unsigned int target_lost_count_;
    unsigned int max_target_lost_count_;

    void initServos();
    void centerHeadServos();

    void setPanSpeed(float);
    void setTiltSpeed(float);

    void setPanPosition(float);
    void setTiltPosition(float);

    void setPanTorque(bool);
    void setTiltTorque(bool);

    float toRadians(float);

    //Processing variables
    int image_height_;
    int image_width_;

public:
    HiwrTrackerNode(ros::NodeHandle&);
    void onInit();
    //Main loop
    void loop();
};

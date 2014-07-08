/*********************************************************************
*
*
* Copyright 2014 Worldline
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
* 
*     http://www.apache.org/licenses/LICENSE-2.0
* 
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
* 
***********************************************************************/
#include "hiwr_motors.h"

Hiwr_motors_node::Hiwr_motors_node(ros::NodeHandle & node):
    debug_name("Hiwr_tracking"),
    nh(node)
{
}

void Hiwr_motors_node::onInit(){
    //Retrieve parameters

    //Can't properly get float values with getParam, cast to doubles first...
    double tmp_val;
    nh.getParam("default_joint_speed" , tmp_val);
    default_joint_speed = (float)tmp_val;

    nh.getParam("max_joint_speed" , tmp_val);
    max_joint_speed = (float)tmp_val;

    nh.getParam("lead_target_angle" , tmp_val);
    lead_target_angle = (float)tmp_val;

    nh.getParam("pan_threshold" , tmp_val);
    pan_threshold = (float)tmp_val;
    nh.getParam("tilt_threshold" , tmp_val);
    tilt_threshold = (float)tmp_val;

    nh.getParam("gain_pan" , tmp_val);
    gain_pan = (float)tmp_val;
    nh.getParam("gain_tilt" , tmp_val);
    gain_tilt = (float)tmp_val;

    //Can't get unsigned int param, only int ...
    int tmp_int_val;
    nh.getParam("rate" , tmp_int_val);
    rate = (unsigned int)tmp_int_val;
    max_target_lost_count = 5*rate;

    nh.getParam("image_height" , image_height);
    image_height = 240;
    nh.getParam("image_width" , image_width);
    image_width = 320;

    nh.getParam("pan_name" , pan_name);
    pan_name = "pan_joint";
    nh.getParam("tilt_name" , tilt_name);
    tilt_name = "tilt_joint";

    target_visible = false;
    target_lost_count = 0 ;

    //Set limits on the pan and tilt angles
    //Nb : Dynamixel limits prior these limits
    max_pan = toRadians(100);
    min_pan = toRadians(-100);
    max_tilt = toRadians(100);
    min_tilt = toRadians(-100);

    //Init servos
    init_servos();

    //Center servos
    center_head_servos();

    target_visible = false;
    target_lost_count = 0;
}

void Hiwr_motors_node::loop(){
    ros::Rate loop_rate(rate);
    while (ros::ok()){
        //If we have lost the target, stop the servos
        if(!target_visible){
            pan_speed = 0;
            tilt_speed = 0;
            //Keep counting target lost
            target_lost_count ++;
        }else{
            target_visible = false;
            target_lost_count = 0;
        }

        //If the target is lost long enough, re-center servos
        if(target_lost_count > max_target_lost_count){
            center_head_servos();
        }else{
            /// FIXME if exceptions occurs, set new position to last position

            //Only update pan speed if it differs from last value
            if(last_pan_speed != pan_speed){
                set_pan_speed(pan_speed);
                last_pan_speed = pan_speed;
            }
            //Update position
            set_pan_position(pan_position);

            //Only update tilt speed if it differs from last value
            if(last_tilt_speed != tilt_speed){
                set_tilt_speed(tilt_speed);
                last_tilt_speed = tilt_speed;
            }
            //Update position
            set_tilt_position(tilt_position);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    //Shutdown occurs, try to center and release torque
    //FIXME - Not working yet
    /*center_head_servos();
    set_pan_torque(false);
    set_tilt_torque(false);
    */
}

void Hiwr_motors_node::init_servos(){
    ROS_INFO("Initialize servos");
    //Wait until dynamixel ready
    //FIXME waitForMessage isn't correctly working on olimex board
    //ros::topic::waitForMessage<dynamixel_msgs::JointState>(tilt_name+"/state");
    //ros::topic::waitForMessage<dynamixel_msgs::JointState>(pan_name+"/state");

    //Subscribe to pan and tilt state
    pan_state = nh.subscribe("/"+pan_name+"/state", 1, &Hiwr_motors_node::callback_pan_state, this);
    tilt_state = nh.subscribe("/"+tilt_name+"/state", 1, &Hiwr_motors_node::callback_tilt_state, this);

    //Publish to command
    pub_tilt_join = nh.advertise<std_msgs::Float64>("/"+tilt_name+"/command", 1);
    pub_pan_join =  nh.advertise<std_msgs::Float64>("/"+pan_name+"/command", 1);

    //Speed service
    service_pan_speed = nh.serviceClient<dynamixel_controllers::SetSpeed>(pan_name+"/set_speed");
    service_tilt_speed = nh.serviceClient<dynamixel_controllers::SetSpeed>(tilt_name+"/set_speed");

    set_pan_speed(default_joint_speed);
    set_tilt_speed(default_joint_speed);

    //PUblish to torque
    pub_tilt_torque = nh.advertise<std_msgs::Bool>("/"+tilt_name+"/torque_enable",1);
    pub_pan_torque = nh.advertise<std_msgs::Bool>("/"+pan_name+"/torque_enable",1);

    //Start servos in the disabled state, so we can move them by hand
    set_pan_torque(false);
    set_tilt_torque(false);

    //Set position and speed to 0
    pan_position = 0;
    pan_speed = 0.001;
    last_pan_speed = 0;

    tilt_position = 0;
    tilt_speed = 0.001;
    last_tilt_speed = 0;

    //Wait until ROI message
    //FIXME waitForMessage isn't correctly working on olimex board
    //ros::topic::waitForMessage<sensor_msgs::RegionOfInterest>("/uvc_cam_node/roi");

    //Subscribe to ROI
    //sub_image_size = nh.subscribe("/hiwr_motors/image_size", 1, &Hiwr_motors_node::callback_ImageSize, this);
    sub_roi = nh.subscribe<sensor_msgs::RegionOfInterest>("/uvc_cam_node/roi", 1, &Hiwr_motors_node::callback_ROI, this);

    ROS_INFO("Initialize Ok");
}

void Hiwr_motors_node::center_head_servos(){
    set_pan_speed(default_joint_speed);
    set_tilt_speed(default_joint_speed);
    for(int i = 0; i < 3; i++){
        set_pan_position(0.0);
        set_tilt_position(0.0);
        //Need to delay
        usleep(100000);
    }
}

float Hiwr_motors_node::toRadians(float value){
    return value * M_PI / 180.0;
}

void Hiwr_motors_node::set_pan_torque(bool value){
    std_msgs::Bool torque;
    torque.data = value;
    pub_pan_torque.publish(torque);
}

void Hiwr_motors_node::set_tilt_torque(bool value){
    std_msgs::Bool torque;
    torque.data = value;
    pub_tilt_torque.publish(torque);
}

void Hiwr_motors_node::set_pan_position(float value){
    std_msgs::Float64 position;
    position.data = value;
    pub_pan_join.publish(position);
}

void Hiwr_motors_node::set_tilt_position(float value){
    std_msgs::Float64 position;
    position.data = value;
    pub_tilt_join.publish(position);
}

void Hiwr_motors_node::set_pan_speed(float value){
    if(value < 0.0001) value = 0.0001;
    dynamixel_controllers::SetSpeed speed;
    speed.request.speed = value;
    service_pan_speed.call(speed);
}

void Hiwr_motors_node::set_tilt_speed(float value){
    if(value < 0.0001) value = 0.0001;
    dynamixel_controllers::SetSpeed speed;
    speed.request.speed = value;
    service_tilt_speed.call(speed);
}

void Hiwr_motors_node::callback_pan_state(const dynamixel_msgs::JointStateConstPtr & msg){
    //Retrieve current motor state
    current_pan_pos = msg->current_pos;
}

void Hiwr_motors_node::callback_tilt_state(const dynamixel_msgs::JointState::ConstPtr & msg){
    //Retrieve current motor state
    current_tilt_pos = msg->current_pos;
}

void Hiwr_motors_node::callback_ROI(const sensor_msgs::RegionOfInterest::ConstPtr &msg){ 

    target_visible = true;
    int target_offset_x = msg->x_offset + msg->width/2 - image_width/2;
    int target_offset_y = msg->y_offset + msg->height/2 - image_height/2;
    float percent_offset_x = 0, percent_offset_y = 0;

    try{
        percent_offset_x = std::fabs((float)target_offset_x/(float (image_width/2.0)));
        percent_offset_y = std::fabs((float)target_offset_y/(float (image_height/2.0)));
    }catch(...){
        print_error("image_width and image_height are not valid, value is 0");
    }

    //Move pan if x target offest exceeds the threshold
    if(percent_offset_x > pan_threshold){
        //Set pan speed proportionnal to the target offset
        pan_speed = std::min(max_joint_speed, std::max(0.0f, gain_pan* percent_offset_x)); /// FIXME trunc 2 decimals ?
        //Set the target position ahead or behind the current position
        if(target_offset_x > 0){
            pan_position = std::max(min_pan, current_pan_pos - lead_target_angle);
        }else{
            pan_position = std::min(max_pan, current_pan_pos + lead_target_angle);
        }
        set_pan_speed(pan_speed);
        //pan_pos_updated = false;

    }else{
        pan_speed = 0;
    }

    //Move tilt if y target offest exceeds the threshold
    if(percent_offset_y > tilt_threshold){
        //Set pan speed proportionnal to the target offset
        tilt_speed = std::min(max_joint_speed, std::max(0.0f, gain_tilt* percent_offset_y)); /// FIXME trunc 2 decimals ?
        //Set the target position ahead or behind the current position
        if(target_offset_y < 0){

            tilt_position = std::max(min_tilt, current_tilt_pos - lead_target_angle);
        }else{
            tilt_position = std::min(max_tilt, current_tilt_pos + lead_target_angle);
        }
        set_tilt_speed(tilt_speed);

    }else{
        tilt_speed = 0;
    }

}

void Hiwr_motors_node::print_info(const char * value){
    ROS_INFO("%s %s", debug_name.c_str(), value);
}

void Hiwr_motors_node::print_error(const char * value){
    ROS_ERROR("%s %s", debug_name.c_str(), value);
}

void customCallback(const dynamixel_msgs::JointStateConstPtr & msg){
    ROS_INFO("CUSTOM");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hiwr_motors");
    ros::NodeHandle n;

    Hiwr_motors_node node(n);

    //Init node
    node.onInit();

    //Main loop
    node.loop();

    return 0;
}



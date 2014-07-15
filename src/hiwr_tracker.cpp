#include "hiwr_tracker.h"

HiwrTrackerNode::HiwrTrackerNode(ros::NodeHandle & node):
debug_name_("Hiwr_tracking"),
nh_(node)
{
}

void HiwrTrackerNode::onInit(){
    //Retrieve parameters

    //Can't properly get float values with getParam, cast to doubles first...
  double tmp_val;
  nh_.getParam("default_joint_speed" , tmp_val);
  default_joint_speed_ = (float)tmp_val;

  nh_.getParam("max_joint_speed" , tmp_val);
  max_joint_speed_ = (float)tmp_val;

  nh_.getParam("lead_target_angle" , tmp_val);
  lead_target_angle_ = (float)tmp_val;

  nh_.getParam("pan_threshold" , tmp_val);
  pan_threshold_ = (float)tmp_val;
  nh_.getParam("tilt_threshold" , tmp_val);
  tilt_threshold_ = (float)tmp_val;

  nh_.getParam("gain_pan" , tmp_val);
  gain_pan_ = (float)tmp_val;
  nh_.getParam("gain_tilt" , tmp_val);
  gain_tilt_ = (float)tmp_val;

    //Can't get unsigned int param, only int ...
  int tmp_int_val;
  nh_.getParam("rate" , tmp_int_val);
  rate_ = (unsigned int)tmp_int_val;
  max_target_lost_count_ = 5*rate_;

  nh_.getParam("image_height" , image_height_);
  image_height_ = 240;
  nh_.getParam("image_width" , image_width_);
  image_width_ = 320;

  nh_.getParam("pan_name" , pan_name_);
  pan_name_ = "pan_joint";
  nh_.getParam("tilt_name" , tilt_name_);
  tilt_name_ = "tilt_joint";

  target_visible_ = false;
  target_lost_count_ = 0 ;

    //Set limits on the pan and tilt angles
    //Nb : Dynamixel limits prior these limits
  max_pan_ = toRadians(100);
  min_pan_ = toRadians(-100);
  max_tilt_ = toRadians(100);
  min_tilt_ = toRadians(-100);

    //Init servos
  initServos();

    //Center servos
  centerHeadServos();

  target_visible_ = false;
  target_lost_count_ = 0;
}

void HiwrTrackerNode::loop(){
  ros::Rate loop_rate(rate_);
  while (ros::ok()){
        //If we have lost the target, stop the servos
    if(!target_visible_){
      pan_speed_ = 0;
      tilt_speed_ = 0;
            //Keep counting target lost
      target_lost_count_ ++;
    }else{
      target_visible_ = false;
      target_lost_count_ = 0;
    }

        //If the target is lost long enough, re-center servos
    if(target_lost_count_ > max_target_lost_count_){
      centerHeadServos();
    }else{
            /// FIXME if exceptions occurs, set new position to last position

            //Only update pan speed if it differs from last value
      if(last_pan_speed_ != pan_speed_){
        setPanSpeed(pan_speed_);
        last_pan_speed_ = pan_speed_;
      }
            //Update position
      setPanPosition(pan_position_);

            //Only update tilt speed if it differs from last value
      if(last_tilt_speed_ != tilt_speed_){
        setTiltSpeed(tilt_speed_);
        last_tilt_speed_ = tilt_speed_;
      }
            //Update position
      setTiltPosition(tilt_position_);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}

void HiwrTrackerNode::initServos(){
  ROS_INFO("Initialize servos");

    //Subscribe to pan and tilt state
  pan_state_ = nh_.subscribe("/"+pan_name_+"/state", 1, &HiwrTrackerNode::callbackPanState, this);
  tilt_state_ = nh_.subscribe("/"+tilt_name_+"/state", 1, &HiwrTrackerNode::callbackTiltState, this);

    //Publish to command
  pub_tilt_join_ = nh_.advertise<std_msgs::Float64>("/"+tilt_name_+"/command", 1);
  pub_pan_join_ =  nh_.advertise<std_msgs::Float64>("/"+pan_name_+"/command", 1);

    //Speed service
  service_pan_speed_ = nh_.serviceClient<dynamixel_controllers::SetSpeed>(pan_name_+"/set_speed");
  service_tilt_speed_ = nh_.serviceClient<dynamixel_controllers::SetSpeed>(tilt_name_+"/set_speed");

  setPanSpeed(default_joint_speed_);
  setTiltSpeed(default_joint_speed_);

    //PUblish to torque
  pub_tilt_torque_ = nh_.advertise<std_msgs::Bool>("/"+tilt_name_+"/torque_enable",1);
  pub_pan_torque_ = nh_.advertise<std_msgs::Bool>("/"+pan_name_+"/torque_enable",1);

    //Start servos in the disabled state, so we can move them by hand
  setPanTorque(false);
  setTiltTorque(false);

    //Set position and speed to 0
  pan_position_ = 0;
  pan_speed_ = 0.001;
  last_pan_speed_ = 0;

  tilt_position_ = 0;
  tilt_speed_ = 0.001;
  last_tilt_speed_ = 0;


    //Subscribe to ROI
  sub_roi_ = nh_.subscribe<sensor_msgs::RegionOfInterest>("/hiwr_opencv_detector/roi", 1, &HiwrTrackerNode::callbackROI, this);

  ROS_INFO("Initialize Ok");
}

void HiwrTrackerNode::centerHeadServos(){
  setPanSpeed(default_joint_speed_);
  setTiltSpeed(default_joint_speed_);
  for(int i = 0; i < 3; i++){
    setPanPosition(0.0);
    setTiltPosition(0.0);
        //Need to delay
    usleep(100000);
  }
}

float HiwrTrackerNode::toRadians(float value){
  return value * M_PI / 180.0;
}

void HiwrTrackerNode::setPanTorque(bool value){
  std_msgs::Bool torque;
  torque.data = value;
  pub_pan_torque_.publish(torque);
}

void HiwrTrackerNode::setTiltTorque(bool value){
  std_msgs::Bool torque;
  torque.data = value;
  pub_tilt_torque_.publish(torque);
}

void HiwrTrackerNode::setPanPosition(float value){
  std_msgs::Float64 position;
  position.data = value;
  pub_pan_join_.publish(position);
}

void HiwrTrackerNode::setTiltPosition(float value){
  std_msgs::Float64 position;
  position.data = value;
  pub_tilt_join_.publish(position);
}

void HiwrTrackerNode::setPanSpeed(float value){
  if(value < 0.0001) value = 0.0001;
  dynamixel_controllers::SetSpeed speed;
  speed.request.speed = value;
  service_pan_speed_.call(speed);
}

void HiwrTrackerNode::setTiltSpeed(float value){
  if(value < 0.0001) value = 0.0001;
  dynamixel_controllers::SetSpeed speed;
  speed.request.speed = value;
  service_tilt_speed_.call(speed);
}

void HiwrTrackerNode::callbackPanState(const dynamixel_msgs::JointStateConstPtr & msg){
    //Retrieve current motor state
  current_pan_pos_ = msg->current_pos;
}

void HiwrTrackerNode::callbackTiltState(const dynamixel_msgs::JointState::ConstPtr & msg){
    //Retrieve current motor state
  current_tilt_pos_ = msg->current_pos;
}

void HiwrTrackerNode::callbackROI(const sensor_msgs::RegionOfInterest::ConstPtr &msg){

  target_visible_ = true;
  int target_offset_x = msg->x_offset + msg->width/2 - image_width_/2;
  int target_offset_y = msg->y_offset + msg->height/2 - image_height_/2;
  float percent_offset_x = 0, percent_offset_y = 0;

  try{
    percent_offset_x = std::fabs((float)target_offset_x/(float (image_width_/2.0)));
    percent_offset_y = std::fabs((float)target_offset_y/(float (image_height_/2.0)));
  }catch(...){
    printError("image_width and image_height are not valid, value is 0");
  }

    //Move pan if x target offest exceeds the threshold
  if(percent_offset_x > pan_threshold_){
        //Set pan speed proportionnal to the target offset
        pan_speed_ = std::min(max_joint_speed_, std::max(0.0f, gain_pan_* percent_offset_x)); /// FIXME trunc 2 decimals ?
        //Set the target position ahead or behind the current position
        if(target_offset_x > 0){
          pan_position_ = std::max(min_pan_, current_pan_pos_ - lead_target_angle_);
        }else{
          pan_position_ = std::min(max_pan_, current_pan_pos_ + lead_target_angle_);
        }
        setPanSpeed(pan_speed_);
        //pan_pos_updated = false;

      }else{
        pan_speed_ = 0;
      }

    //Move tilt if y target offest exceeds the threshold
      if(percent_offset_y > tilt_threshold_){
        //Set pan speed proportionnal to the target offset
        tilt_speed_ = std::min(max_joint_speed_, std::max(0.0f, gain_tilt_* percent_offset_y)); /// FIXME trunc 2 decimals ?
        //Set the target position ahead or behind the current position
        if(target_offset_y < 0){

          tilt_position_ = std::max(min_tilt_, current_tilt_pos_ - lead_target_angle_);
        }else{
          tilt_position_ = std::min(max_tilt_, current_tilt_pos_ + lead_target_angle_);
        }
        setTiltSpeed(tilt_speed_);

      }else{
        tilt_speed_ = 0;
      }

    }

    void HiwrTrackerNode::printInfo(const char * value){
      ROS_INFO("%s %s", debug_name_.c_str(), value);
    }

    void HiwrTrackerNode::printError(const char * value){
      ROS_ERROR("%s %s", debug_name_.c_str(), value);
    }

    void customCallback(const dynamixel_msgs::JointStateConstPtr & msg){
      ROS_INFO("CUSTOM");
    }

    int main(int argc, char **argv)
    {
      ros::init(argc, argv, "hiwr_tracker");
      ros::NodeHandle n;

      HiwrTrackerNode node(n);

    //Init node
      node.onInit();

    //Main loop
      node.loop();

      return 0;
    }



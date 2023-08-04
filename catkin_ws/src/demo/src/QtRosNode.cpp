#include "QtRosNode.h"

QtRosNode::QtRosNode()
{
    this->gui_closed = false;
    current_speed = 0;
    current_steering = 0;
}

QtRosNode::~QtRosNode()
{
}

void QtRosNode::run()
{
    pub_speed    = n->advertise<std_msgs::Float64>("/speed"   ,10);
    pub_steering = n->advertise<std_msgs::Float64>("/steering",10);
    pub_start_cruise       = n->advertise<std_msgs::Bool>("/cruise/enable", 10);
    pub_start_follow       = n->advertise<std_msgs::Bool>("/follow/enable", 10);
    pub_start_change_left  = n->advertise<std_msgs::Bool>("/start_change_lane_on_left", 10);
    pub_start_change_right = n->advertise<std_msgs::Bool>("/start_change_lane_on_right", 10);
    pub_start_pass_left    = n->advertise<std_msgs::Bool>("/start_pass_on_left", 10);
    pub_start_pass_right   = n->advertise<std_msgs::Bool>("/start_pass_on_right", 10);
    ros::Rate loop(30);
    
    while(ros::ok() && !this->gui_closed)
    {
        ros::spinOnce();
        emit updateGraphics();
        loop.sleep();
    }
    emit onRosNodeFinished();
}

void QtRosNode::setNodeHandle(ros::NodeHandle* nh)
{
    this->n = nh;   
}

void QtRosNode::publish_speed(double speed)
{
    current_speed = speed;
    std_msgs::Float64 msg;
    msg.data = speed;
    pub_speed.publish(msg);
}

void QtRosNode::publish_steering(double steering)
{
    current_steering = steering;
    std_msgs::Float64 msg;
    msg.data = steering;
    pub_steering.publish(msg);
}

void QtRosNode::update_speed_and_publish(double delta_speed)
{
    current_speed += delta_speed;
    if(current_speed >  50.0) current_speed =  50.0;
    if(current_speed < -20.0) current_speed = -20.0;
    std_msgs::Float64 msg;
    msg.data = current_speed;
    pub_speed.publish(msg);
}

void QtRosNode::update_steering_and_publish(double delta_steering)
{
    current_steering += delta_steering;
    if(current_steering >  0.5) current_steering =  0.5;
    if(current_steering < -0.5) current_steering = -0.5;
    std_msgs::Float64 msg;
    msg.data = current_steering;
    pub_steering.publish(msg);
}

void QtRosNode::publish_start_cruise()
{
    std_msgs::Bool msg;
    msg.data = true;
    pub_start_cruise.publish(msg);
}

void QtRosNode::publish_start_follow()
{
    std_msgs::Bool msg;
    msg.data = true;
    pub_start_follow.publish(msg);
}

void QtRosNode::publish_start_change_left()
{
    std_msgs::Bool msg;
    msg.data = true;
    pub_start_change_left.publish(msg);
}

void QtRosNode::publish_start_change_right()
{
    std_msgs::Bool msg;
    msg.data = true;
    pub_start_change_right.publish(msg);
}

void QtRosNode::publish_start_pass_left()
{
    std_msgs::Bool msg;
    msg.data = true;
    pub_start_pass_left.publish(msg);
}

void QtRosNode::publish_start_pass_right()
{
    std_msgs::Bool msg;
    msg.data = true;
    pub_start_pass_right.publish(msg);
}

void QtRosNode::stop_behaviors()
{
    std_msgs::Bool msg;
    msg.data = false;
    pub_start_cruise.publish(msg);
    pub_start_follow.publish(msg);
    pub_start_change_left.publish(msg);
    pub_start_change_right.publish(msg);
    pub_start_pass_left.publish(msg);
    pub_start_pass_right.publish(msg);
}

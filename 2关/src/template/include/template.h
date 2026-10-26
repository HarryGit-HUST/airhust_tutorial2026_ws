#pragma once

#include <cmath>
#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#define ALTITUDE 0.8f

mavros_msgs::PositionTarget setpoint_raw;

// 飞控状态。
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
	current_state = *msg;
}

// 里程计与起飞初始位置。
tf::Quaternion quat;
nav_msgs::Odometry local_pos;
double roll, pitch, yaw;
float init_position_x_take_off = 0;
float init_position_y_take_off = 0;
float init_position_z_take_off = 0;
float init_yaw_take_off = 0;
bool flag_init_position = false;



void local_pos_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
	local_pos = *msg;
	tf::quaternionMsgToTF(local_pos.pose.pose.orientation, quat);
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
	if (flag_init_position == false && (local_pos.pose.pose.position.z != 0))
	{
		init_position_x_take_off = local_pos.pose.pose.position.x;
		init_position_y_take_off = local_pos.pose.pose.position.y;
		init_position_z_take_off = local_pos.pose.pose.position.z;
		init_yaw_take_off = yaw;
		flag_init_position = true;
	}
}

// 起飞初始位置为平移原点的位置目标。
float mission_pos_cruise_last_position_x = 0;
float mission_pos_cruise_last_position_y = 0;
bool mission_pos_cruise_flag = false;
bool mission_pos_cruise(float x, float y, float z, float target_yaw, float error_max)
{
	if (mission_pos_cruise_flag == false)
	{
		mission_pos_cruise_last_position_x = local_pos.pose.pose.position.x;
		mission_pos_cruise_last_position_y = local_pos.pose.pose.position.y;
		mission_pos_cruise_flag = true;
	}
	setpoint_raw.type_mask = /*1 + 2 + 4 */ +8 + 16 + 32 + 64 + 128 + 256 + 512 /*+ 1024 */ + 2048;
	setpoint_raw.coordinate_frame = 1;
	setpoint_raw.position.x = x + init_position_x_take_off;
	setpoint_raw.position.y = y + init_position_y_take_off;
	setpoint_raw.position.z = z + init_position_z_take_off;
	setpoint_raw.yaw = target_yaw;
	ROS_INFO("now (%.2f,%.2f,%.2f,%.2f) to ( %.2f, %.2f, %.2f, %.2f), init_pose(%.2f, %.2f, %.2f, %.2f)", 
        local_pos.pose.pose.position.x ,local_pos.pose.pose.position.y, local_pos.pose.pose.position.z, yaw * 180.0 / M_PI, 
        x, y, z, target_yaw * 180.0 / M_PI, 
        init_position_x_take_off, init_position_y_take_off, init_position_z_take_off, init_yaw_take_off * 180.0 / M_PI);
	if (fabs(local_pos.pose.pose.position.x - x - init_position_x_take_off) < error_max && fabs(local_pos.pose.pose.position.y - y - init_position_y_take_off) < error_max && fabs(local_pos.pose.pose.position.z - z - init_position_z_take_off) < error_max && fabs(yaw - target_yaw) < 0.1)
	{
		ROS_INFO("到达目标点，巡航点任务完成");
		mission_pos_cruise_flag = false;
		return true;
	}
	return false;
}

// 以首次调用时的当前位置为 XY 平移起点；不进行机体航向旋转。
float current_position_cruise_last_position_x = 0;
float current_position_cruise_last_position_y = 0;
bool current_position_cruise_flag = false;
bool current_position_cruise(float x, float y, float z, float target_yaw, float error_max)
{
	if (current_position_cruise_flag == false)
	{
		current_position_cruise_last_position_x = local_pos.pose.pose.position.x;
		current_position_cruise_last_position_y = local_pos.pose.pose.position.y;
		current_position_cruise_flag = true;
	}
	// float p_xy = 0.8;
	setpoint_raw.type_mask = /*1 + 2 + 4 */+ 8 + 16  + 32 + 64 + 128 + 256 + 512 /*+ 1024 */+ 2048;
	setpoint_raw.coordinate_frame = 1;
	setpoint_raw.position.x = x + current_position_cruise_last_position_x;
	setpoint_raw.position.y = y + current_position_cruise_last_position_y;
	setpoint_raw.position.z = z;
	setpoint_raw.yaw = target_yaw;
    ROS_INFO("now (%.2f,%.2f,%.2f,%.2f) to ( %.2f, %.2f, %.2f, %.2f)", local_pos.pose.pose.position.x ,local_pos.pose.pose.position.y, local_pos.pose.pose.position.z, yaw * 180.0 / M_PI, x + current_position_cruise_last_position_x, y + current_position_cruise_last_position_y, z, target_yaw * 180.0 / M_PI);
	if (fabs(local_pos.pose.pose.position.x - current_position_cruise_last_position_x - x) < error_max && fabs(local_pos.pose.pose.position.y - current_position_cruise_last_position_y - y) < error_max && fabs(local_pos.pose.pose.position.z - z) < error_max && fabs(yaw - target_yaw) < 0.1)
	{
		ROS_INFO("到达目标点，巡航点任务完成");
		// current_position_cruise_flag = false;
		setpoint_raw.yaw_rate = 0.0; // 重置yaw_rate
		setpoint_raw.type_mask = /*1 + 2 + 4 */ +8 + 16 + 32 + 64 + 128 + 256 + 512 + /*1024 + */2048;
		setpoint_raw.yaw = yaw; // 确保最终yaw角度正确
		return true;
	}
	return false;
}

// 降落。
float precision_land_init_position_x = 0;
float precision_land_init_position_y = 0;
bool precision_land_init_position_flag = false;
ros::Time precision_land_last_time;
bool precision_land()
{
	if (!precision_land_init_position_flag)
	{
		precision_land_init_position_x = local_pos.pose.pose.position.x;
		precision_land_init_position_y = local_pos.pose.pose.position.y;
        precision_land_last_time = ros::Time::now();
		precision_land_init_position_flag = true;
	}
	setpoint_raw.position.x = precision_land_init_position_x;
	setpoint_raw.position.y = precision_land_init_position_y;
	setpoint_raw.position.z = -0.15;
	setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ +64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
	setpoint_raw.coordinate_frame = 1;
    if(ros::Time::now() - precision_land_last_time > ros::Duration(5.0))
    {
        ROS_INFO("Precision landing complete.");
        precision_land_init_position_flag = false; // Reset for next landing
        return true;
    }
    return false;
}

// 任务计时器。
ros::Time last_time;
bool lib_time_init_flag = true;
bool lib_time_record_func(float duration, ros::Time current_time) {
      // 静态变量，记录上次触发时间
	if(lib_time_init_flag)
	{
		last_time = current_time;
		lib_time_init_flag = false; // 只在第一次调用时初始化
	}

    ros::Duration elapsed = current_time - last_time;

    // 如果已经过去超过 duration 秒，返回 true 并更新时间
    if (elapsed.toSec() >= duration) {
		lib_time_init_flag = true;
		return true;
    }
    return false;
}

// 保持当前位置悬停。
bool hover_started = false;
ros::Time hover_last_time;
float hover_x = 0;
float hover_y = 0;
float hover_z = 0;
float hover_yaw = 0;
bool hover(float time_duration)
{
    if(!hover_started)
    {
        hover_last_time = ros::Time::now();
        hover_started = true;
        hover_x = local_pos.pose.pose.position.x;
        hover_y = local_pos.pose.pose.position.y;
        hover_z = local_pos.pose.pose.position.z;
        hover_yaw = yaw;
    }
    ros::Duration elapsed = ros::Time::now() - hover_last_time;
    setpoint_raw.type_mask = /*1 + 2 + 4 */ +8 + 16 + 32 + 64 + 128 + 256 + 512 /*+ 1024 */ + 2048;
    setpoint_raw.coordinate_frame = 1;
    setpoint_raw.position.x = hover_x;
    setpoint_raw.position.y = hover_y;
    setpoint_raw.position.z = hover_z;
    setpoint_raw.yaw = hover_yaw;
    ROS_INFO("Hovering at (%.2f, %.2f, %.2f) for %.2f seconds", hover_x, hover_y, hover_z, elapsed.toSec());
     // 如果已经过去超过 time_duration 秒，返回 true 并重置状态 
    if (elapsed.toSec() >= time_duration) {
        hover_started = false; // Reset for next hover
        return true;
    }
    return false;
}

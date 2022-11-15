#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sys/time.h>
#include <string>
#include <time.h>
#include <eigen3/Eigen/Dense>
#include <quadrotor_msgs/PositionCommand.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>

using namespace Eigen;
using namespace std;

ros::Publisher position_cmd_pub;

ros::Publisher current_pose_pub;

ros::Subscriber uav_pos_sub;

ros::Subscriber trigger_sub;

int traj_id_send= 0;
bool is_init= 0;
bool is_traj= 0;

Vector3d hover_position;
Vector3d current_position;

double hover_yaw = 0.0;
double now_yaw = 0.0;

double odom_time = 0.0;
double hover_start_time = 0.0; 

double radius = 1.0;
double period = 6.0;
double rev = 2.0;

bool follow_yaw = false;
bool move_z = false;
double z_diff = 0.0;


void uav_pos_call_back(const nav_msgs::Odometry& msg){
	current_position(0)= msg.pose.pose.position.x;
	current_position(1)= msg.pose.pose.position.y;
	current_position(2)= msg.pose.pose.position.z;

	odom_time = msg.header.stamp.toSec();

    now_yaw  = atan2(2 * (msg.pose.pose.orientation.w * msg.pose.pose.orientation.z + msg.pose.pose.orientation.x * msg.pose.pose.orientation.y), 1 - 2 * (msg.pose.pose.orientation.y * msg.pose.pose.orientation.y + msg.pose.pose.orientation.z * msg.pose.pose.orientation.z));
	
	if(is_init){
		if(is_traj) {

			double dt = odom_time - hover_start_time;
			bool traj_finish = dt > rev * period;
			dt = min(dt, rev * period);
            
            double des_x = hover_position(0) + radius * sin(2 * M_PI / period * (dt));
            double des_y = hover_position(1) - radius * cos(2 * M_PI / period * (dt)) + radius;
            double des_z = hover_position(2);


            double des_vx = radius * 2 * M_PI / period * cos(2 * M_PI / period * (dt));
            double des_vy = radius * 2 * M_PI / period * sin(2 * M_PI / period * (dt));
            double des_vz = 0;

		    double des_ax = - 4 * M_PI * M_PI / period / period * radius * sin(2 * M_PI / period * (dt));
		    double des_ay = 4 * M_PI * M_PI / period / period * radius * cos(2 * M_PI / period * (dt));
		    double des_az = 0;

		    double des_yaw = 0.0;

		    if(follow_yaw){
		    	des_yaw = atan2(des_vy, des_vx);//(dt) / period * 2.0 * M_PI;
		    	//if(dt > period) des_yaw = -(dt) / period * 2.0 * M_PI;
			while(des_yaw > M_PI) des_yaw -= (2*M_PI);
			while(des_yaw < -M_PI) des_yaw += (2*M_PI);
		    }
		    else{
		    	des_yaw = 0.0;
		    }

		    if(move_z){
            	des_z = hover_position(2) + z_diff - z_diff * cos(2 * M_PI / period * (dt));
            	des_vz = 2 * M_PI / period * z_diff * sin(2 * M_PI / period * (dt));
            	des_az = 4 * M_PI * M_PI / period / period * z_diff * cos(2 * M_PI / period * (dt));
            }
                
            quadrotor_msgs::PositionCommand position_cmd;
            position_cmd.header.stamp    = msg.header.stamp;
            position_cmd.header.frame_id = "world";
                               
            if(!traj_finish){
	        	position_cmd.position.x      = des_x;
	            position_cmd.position.y      = des_y;
	            position_cmd.position.z      = des_z;
	            position_cmd.velocity.x      = des_vx;
	            position_cmd.velocity.y      = des_vy;
	            position_cmd.velocity.z      = des_vz;
	            position_cmd.acceleration.x  = des_ax;
	            position_cmd.acceleration.y  = des_ay;
	            position_cmd.acceleration.z  = des_az;
	            position_cmd.yaw             = des_yaw;
	            position_cmd.trajectory_flag = position_cmd.TRAJECTORY_STATUS_READY;
	            position_cmd.trajectory_id = traj_id_send;

				position_cmd_pub.publish( position_cmd );
        	}
        	else{
        		position_cmd.position.x      = des_x;
	            position_cmd.position.y      = des_y;
	            position_cmd.position.z      = des_z;
	            position_cmd.velocity.x      = 0;
	            position_cmd.velocity.y      = 0;
	            position_cmd.velocity.z      = 0;
	            position_cmd.acceleration.x  = 0;
	            position_cmd.acceleration.y  = 0;
	            position_cmd.acceleration.z  = 0;
	            position_cmd.yaw             = des_yaw;
	            position_cmd.trajectory_flag = position_cmd.TRAJECTORY_STATUS_COMPLETED;
	            position_cmd.trajectory_id = traj_id_send;       		
        	}
                

            position_cmd.position.x      = msg.pose.pose.position.x;
            position_cmd.position.y      = msg.pose.pose.position.y;
            position_cmd.position.z      = msg.pose.pose.position.z;
            position_cmd.velocity.x      = msg.twist.twist.linear.x;
            position_cmd.velocity.y      = msg.twist.twist.linear.y;
            position_cmd.velocity.z      = msg.twist.twist.linear.z;
            position_cmd.acceleration.x  = msg.twist.twist.angular.x;
            position_cmd.acceleration.y  = msg.twist.twist.angular.y;
            position_cmd.acceleration.z  = msg.twist.twist.angular.z;
            position_cmd.yaw             = now_yaw;

            current_pose_pub.publish( position_cmd );

        }
    }
    else{
        is_init = true;
    }
}

void trigger_callback( const geometry_msgs::PoseStamped::ConstPtr& trigger_msg ){
    if ( is_init ){
        std::cout << "[#INFO] get traj trigger info." << std::endl;
        traj_id_send= trigger_msg->header.seq + 1;
        is_traj    = true;

        hover_position = current_position;
        hover_yaw = now_yaw;

        hover_start_time = odom_time;

		cout<<"-----------------"<<endl;
		ROS_WARN("circle start!");
        cout<<"start position:\n"<< hover_position.transpose()<<"\n Radius: "<<radius<<"\n rev: "<< rev<<"\n period: "<<period<<endl;
		cout<<"follow yaw: "<<follow_yaw<<endl;

		if(move_z){
			ROS_INFO("move z with diff: %lf", z_diff);
		}
		else{
			ROS_INFO("constant height");
		}


		

    }
}

int main(int argc, char *argv[]){
	ros::init(argc, argv, "circle_test");
	ros::NodeHandle nh;

	nh.param("circle_test/radius", radius, 1.0);
	nh.param("circle_test/period", period, 6.0);
	nh.param("circle_test/rev", rev, 2.0);

	nh.param("circle_test/follow_yaw", follow_yaw, false);
	nh.param("circle_test/move_z", move_z, false);
	nh.param("circle_test/z_diff", z_diff, 0.0);


	uav_pos_sub = nh.subscribe("/vins_multi_rgbd/imu_propagate", 100,uav_pos_call_back);
	trigger_sub = nh.subscribe( "/traj_start_trigger", 100, trigger_callback);
	position_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd",1);
	current_pose_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/current_pose",1);

	ros::spin();
	return 0;
}

#include <iostream>
#include <fstream>
#include <cmath>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <sys/time.h>
#include <string>
#include <time.h>
#include <eigen3/Eigen/Dense>
#include <quadrotor_msgs/PositionCommand.h>
#include <visualization_msgs/Marker.h>
#include <trajectory_generator_waypoint.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>

using namespace Eigen;
using namespace std;

#define mean_speed 1.0

#define change_yaw 1

#define follow_yaw_speed_limit 0.00

ros::Publisher traj_pub;

ros::Publisher position_cmd_pub;

ros::Publisher current_pose_pub;

ros::Subscriber uav_pos_sub;

ros::Subscriber trigger_sub;

visualization_msgs::Marker selected_marker;


int traj_id_send= 0;
double traj_start_time= -1.0;
bool is_init= 0;
bool is_traj= 0;

VectorXd uav_state= VectorXd::Zero(11);
VectorXd uav_t = VectorXd::Zero(1);
VectorXd uav_plan_t = VectorXd::Zero(1);
MatrixXd uav_coef;

void traj_viz(){
	double des_x = 0, des_y = 0, des_z = 0;
    geometry_msgs::Point parent, child;

    Eigen::MatrixXd coef_x,coef_y, coef_z;

    selected_marker.header.stamp = ros::Time::now();
    selected_marker.header.frame_id = "world";
    selected_marker.action = visualization_msgs::Marker::ADD;
    selected_marker.pose.orientation.w = 1.0;
    selected_marker.id = 0;
    selected_marker.type = visualization_msgs::Marker::LINE_LIST;
    selected_marker.scale.x = 0.1;
    selected_marker.color.b = selected_marker.color.a = 1.0;

    bool first=true;

    for(double dT = traj_start_time;dT< uav_t(uav_t.rows()-1);dT+=0.05) {
        if(first){
            parent.x=uav_coef(0,0);
            parent.y=uav_coef(0,6);
         	parent.z=uav_coef(0,12);
            selected_marker.points.push_back(parent);
            first=false;
        }
        for (int i = 1; i < uav_t.size(); i++) {
            if (dT < uav_t(i)) {

                double tt = dT - uav_t(i - 1);
                Eigen::Matrix<double, 1, 6> t_p;
                t_p << 1, tt, pow(tt, 2), pow(tt, 3), pow(tt, 4), pow(tt, 5);
                Eigen::Matrix<double, 1, 6> t_v;
                t_v << 0, 1, 2 * tt, 3 * pow(tt, 2), 4 * pow(tt, 3), 5 * pow(tt, 4);

                coef_x = (uav_coef.block(i-1, 0, 1, 6)).transpose();
                coef_y = (uav_coef.block(i-1, 6, 1, 6)).transpose();
                coef_z = (uav_coef.block(i-1, 12, 1, 6)).transpose();

                des_x = (t_p * coef_x)(0,0);
                des_y = (t_p * coef_y)(0,0);
                des_z = (t_p * coef_z)(0,0);

                child.x = des_x;
                child.y = des_y;
                child.z = des_z;
                parent.x = des_x;
                parent.y = des_y;
                parent.z = des_z;
                
                selected_marker.points.push_back(child);
                selected_marker.points.push_back(parent);
                break;
            }
        }
    }

	if (!selected_marker.points.empty()) selected_marker.points.pop_back();


	traj_pub.publish(selected_marker);
    selected_marker.points.clear();
}

void generate_traj(geometry_msgs::PoseArray& msg){
    if(!is_traj) return;

    ros::Time Time1= ros::Time::now();
    
    TrajectoryGeneratorWaypoint T;
    Eigen::MatrixXd Path;
    Eigen::MatrixXd Vel = Eigen::MatrixXd::Zero(2,3);
    Eigen::MatrixXd Acc = Eigen::MatrixXd::Zero(2,3);
    Eigen::VectorXd Time;

    Path.resize(msg.poses.size()+1,3);
    Time.resize(msg.poses.size());
    uav_t.resize(msg.poses.size()+1);

    uav_plan_t.resize(msg.poses.size()+1);

    Path(0,0) = uav_state(0);
    Path(0,1) = uav_state(1);
    Path(0,2) = uav_state(2);

    Vel(0,0) = uav_state(3);
    Vel(0,1) = uav_state(4);
    Vel(0,2) = uav_state(5);
    
    Acc(0,0) = uav_state(6);
    Acc(0,1) = uav_state(7);
    Acc(0,2) = uav_state(8);

    uav_t(0) = uav_state(10);

    traj_start_time = uav_state(10);

    for(int i=0; i<msg.poses.size(); i++){
        Path(i+1,0) = msg.poses[i].position.x;
        Path(i+1,1) = msg.poses[i].position.y;
        Path(i+1,2) = msg.poses[i].position.z;

        Time(i) = ((Path.row(i+1) - Path.row(i)).transpose().norm()) / mean_speed;

	if(i == 0 || i == msg.poses.size()-1) Time(i) *= 1.5;

        uav_t(i+1) = uav_t(i) + Time(i);
    }

    uav_coef = T.PolyQPGeneration(Path,Vel,Acc,Time);
    
    traj_id_send++;
    //cout<<"Time consumed: "<<(ros::Time::now()-Time1).toSec()<<endl;
    traj_viz();
}


void uav_pos_call_back(const nav_msgs::Odometry& msg){
	uav_state(0)= msg.pose.pose.position.x;
	uav_state(1)= msg.pose.pose.position.y;
	uav_state(2)= msg.pose.pose.position.z;

	uav_state(3)= msg.twist.twist.linear.x;
	uav_state(4)= msg.twist.twist.linear.y;
	uav_state(5)= msg.twist.twist.linear.z;

    uav_state(6)= msg.twist.twist.angular.x;
    uav_state(7)= msg.twist.twist.angular.y;
    uav_state(8)= msg.twist.twist.angular.z;

    uav_state(9) = atan2(2 * (msg.pose.pose.orientation.w * msg.pose.pose.orientation.z + msg.pose.pose.orientation.x * msg.pose.pose.orientation.y), 1 - 2 * (msg.pose.pose.orientation.y * msg.pose.pose.orientation.y + msg.pose.pose.orientation.z * msg.pose.pose.orientation.z));

    uav_state(10) = msg.header.stamp.toSec();

	if(traj_start_time< 0 && is_init) return;
	
	if(is_init){
		if(is_traj) {
            double dT = msg.header.stamp.toSec();
            
            double des_x = 0;
            double des_y = 0;
            double des_z = 0;

            double des_vx = 0;
            double des_vy = 0;
            double des_vz = 0;

		    double des_ax = 0;
		    double des_ay = 0;
		    double des_az = 0;

		    double des_yaw = 0;

            bool traj_ok=false;

            for (int i = 1; i < uav_t.size(); i++) {
                if (dT < uav_t(i)) {

                	double tt = dT - uav_t(i - 1);

                    Matrix<double, 1, 6> t_p;
                    t_p << 1, tt, pow(tt, 2), pow(tt, 3), pow(tt, 4), pow(tt, 5);
                    Matrix<double, 1, 6> t_v;
                    t_v << 0, 1, 2 * tt, 3 * pow(tt, 2), 4 * pow(tt, 3), 5 * pow(tt, 4);
					Matrix<double, 1, 6> t_a;
			    	t_a << 0, 0, 2, 6 * tt, 12 * pow(tt,2), 20*pow(tt,3);
                            
                    VectorXd coef_x;
                    VectorXd coef_y;
                    VectorXd coef_z;
                    
                    coef_x = (uav_coef.block(i-1, 0, 1, 6)).transpose();
                    coef_y = (uav_coef.block(i-1, 6, 1, 6)).transpose();
                    coef_z = (uav_coef.block(i-1, 12, 1, 6)).transpose();
                    
                    des_x = (t_p * coef_x)(0,0);
                    des_y = (t_p * coef_y)(0,0);
                    des_z = (t_p * coef_z)(0,0);

                    des_vx = (t_v * coef_x)(0,0);
                    des_vy = (t_v * coef_y)(0,0);
                    des_vz = (t_v * coef_z)(0,0);

			    	des_ax = (t_a * coef_x)(0,0);
			    	des_ay = (t_a * coef_y)(0,0);
			    	des_az = (t_a * coef_z)(0,0);

			    	if(change_yaw) des_yaw = atan2(des_vy, des_vx);

			    	while(des_yaw > M_PI) des_yaw -= 2*M_PI;
			    	while(des_yaw < -M_PI) des_yaw += 2*M_PI;
			    	//cout<<"des_yaw:"<<des_yaw<<endl<<endl;                    
                    traj_ok= true;
                    
                    break;
                }
            }
                
            quadrotor_msgs::PositionCommand position_cmd;
            position_cmd.header.stamp    = msg.header.stamp;
            position_cmd.header.frame_id = "world";
                    
            if(traj_ok){

            	position_cmd.position.x      = des_x;
                position_cmd.position.y      = des_y;
                position_cmd.position.z      = des_z;
                position_cmd.velocity.x      = des_vx;
                position_cmd.velocity.y      = des_vy;
                position_cmd.velocity.z      = des_vz;
                position_cmd.acceleration.x  = des_ax;
                position_cmd.acceleration.y  = des_ay;
                position_cmd.acceleration.z  = des_az;
                position_cmd.yaw             = des_yaw;//(des_vx < follow_yaw_speed_limit && des_vy < follow_yaw_speed_limit) ? uav_state(9) : des_yaw;
                position_cmd.trajectory_flag = position_cmd.TRAJECTORY_STATUS_READY;
                position_cmd.trajectory_id = traj_id_send;

			}
            else{
                position_cmd.position.x      = msg.pose.pose.position.x;
                position_cmd.position.y      = msg.pose.pose.position.y;
                position_cmd.position.z      = msg.pose.pose.position.z;
                position_cmd.velocity.x      = 0.0;
                position_cmd.velocity.y      = 0.0;
                position_cmd.velocity.z      = 0.0;
                position_cmd.acceleration.x  = 0.0;
                position_cmd.acceleration.y  = 0.0;
                position_cmd.acceleration.z  = 0.0;
                position_cmd.yaw             = uav_state(9);
                position_cmd.trajectory_flag = position_cmd.TRAJECTORY_STATUS_EMPTY;
                position_cmd.trajectory_id   = traj_id_send;
            }
                
            position_cmd_pub.publish( position_cmd );


            position_cmd.position.x      = msg.pose.pose.position.x;
            position_cmd.position.y      = msg.pose.pose.position.y;
            position_cmd.position.z      = msg.pose.pose.position.z;
            position_cmd.velocity.x      = msg.twist.twist.linear.x;
            position_cmd.velocity.y      = msg.twist.twist.linear.y;
            position_cmd.velocity.z      = msg.twist.twist.linear.z;
            position_cmd.acceleration.x  = msg.twist.twist.angular.x;
            position_cmd.acceleration.y  = msg.twist.twist.angular.y;
            position_cmd.acceleration.z  = msg.twist.twist.angular.z;
            position_cmd.yaw             = uav_state(9);

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
        traj_id_send= traj_id_send + 1;//trigger_msg->header.seq + 1;
        is_traj    = true;

        geometry_msgs::PoseArray pa;
        pa.header.stamp = trigger_msg->header.stamp;

        geometry_msgs::Pose pose_temp;


        pose_temp.position.x = 1.0;
        pose_temp.position.y = 1.0;
        pose_temp.position.z = 0.8;
        pa.poses.push_back(pose_temp);

        pose_temp.position.x = 0.0;
        pose_temp.position.y = 2.0;
        pose_temp.position.z = 1.0;
        pa.poses.push_back(pose_temp);

        pose_temp.position.x = -1.0;
        pose_temp.position.y = 1.0;
        pose_temp.position.z = 0.9;
        pa.poses.push_back(pose_temp);

        pose_temp.position.x = 0.0;
        pose_temp.position.y = 0.0;
        pose_temp.position.z = 1.0;
        pa.poses.push_back(pose_temp);

        pose_temp.position.x = 1.0;
        pose_temp.position.y = 1.0;
        pose_temp.position.z = 0.8;
        pa.poses.push_back(pose_temp);

        pose_temp.position.x = 0.0;
        pose_temp.position.y = 2.0;
        pose_temp.position.z = 1.0;
        pa.poses.push_back(pose_temp);

        pose_temp.position.x = -1.0;
        pose_temp.position.y = 1.0;
        pose_temp.position.z = 0.9;
        pa.poses.push_back(pose_temp);

        pose_temp.position.x = 0.0;
        pose_temp.position.y = 0.0;
        pose_temp.position.z = 1.0;
        pa.poses.push_back(pose_temp);


        generate_traj(pa);
    }
}

int main(int argc, char *argv[]){
	ros::init(argc, argv, "traj_generator");
	ros::NodeHandle nh;
    //uav_waypoints_sub = nh.subscribe("/position_cmd_arrays", 10,uav_waypoints_call_back);
	uav_pos_sub = nh.subscribe("/vins_estimator/imu_propagate", 10,uav_pos_call_back);
	trigger_sub = nh.subscribe( "/traj_start_trigger", 100, trigger_callback);
	position_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd",1);
    current_pose_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/current_pose",1);
	traj_pub = nh.advertise<visualization_msgs::Marker>("traj_viz", 1);
	
	ros::spin();
	return 0;
}
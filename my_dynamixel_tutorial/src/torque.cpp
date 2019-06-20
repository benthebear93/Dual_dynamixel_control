#include <ros/ros.h>
#include <dynamixel_controllers/SetTorqueLimit.h>
#include <dynamixel_controllers/SetSpeed.h>
#include <dynamixel_controllers/TorqueEnable.h>
#include <std_msgs/Float64.h>
#include <dynamixel_msgs/MotorStateList.h>
#include <iostream>
#include <cstdlib>
#include <math.h>
// #define PI 3.14159265
// #define L1 190.0
// #define L2 140.0

using std::cin;
using std::cout;
using std::endl;

ros::Publisher pub;
ros::Publisher pub2;
std_msgs::Float64 msg;
std_msgs::Float64 msg2;

ros::ServiceClient torque_limit;
ros::ServiceClient torque_limit2;
dynamixel_controllers::SetTorqueLimit torque;
dynamixel_controllers::SetTorqueLimit torque2;
int set_position = 0;
float L1 = 140.0;
float L2 = 140.0;
float PI = 3.14159265;
float theta[] = {0.0,0.0};
int pre_position= 0;
int torque_flag=0;
float theta2_inverse(float x, float y)
{
	// float abs = 0.0;
	float theta2 = 0.0;
	theta2 = acos(((x*x)+(y*y)-(L1*L1)-(L2*L2))/(2*L1*L2));
	// abs = fabs(theta2);

	//cout<<"function theta2  ";
	//cout<<theta2<<endl;
	// return abs;
	return theta2;
}

float theta1_inverse(float x , float y)
{
	float theta1 = 0.0;
	float theta2 = 0.0;
	theta2 = theta2_inverse(x, y);
	theta1 = atan(y/x)-atan((L2*sin(theta2))/(L1+L2*cos(theta2)));
	return theta1;
}

float deg2rad(float degree)
{
    return degree*PI/180.0;
}
float rad2deg(float rad) {
    return rad * 180.0 / PI;
}
void receiver(const dynamixel_msgs::MotorStateList::ConstPtr&ms){
	float x =0.0;
	float y =0.0;
	float temp_d1 =0.0;
	float temp_d2 =0.0;
	pre_position = ms->motor_states[0].position;
	if (set_position == 0)
	{
		cout <<"where x, y? :";
		cin >> x;
		cin >> y;
		cout <<"x :"<< x <<" ,y :"<< y <<endl;
		theta[1] = theta2_inverse(x,y);
		theta[0] = theta1_inverse(x,y);
		if(theta[0] < 0){
			theta[0] = 6.28 + theta[0];
		}
		temp_d2 = rad2deg(theta[1]);
		temp_d1 = rad2deg(theta[0]);
		cout<<"theta1  ";
		cout<<temp_d1<<endl;
		cout<<"theta2  ";
		cout<<temp_d2<<endl;
		set_position = 1;
	}
    //radian*180/PI;
	//cout <<"goal motor [0]:"<< ms-> motor_states[0].position <<endl;
	//cout <<"goal motor [1]:"<< ms-> motor_states[1].position <<endl;
	//cout <<" msg.data : " << msg.data << endl;
	//cout <<" torque : " << torque.request.torque_limit << endl;

	if (sqrt(x*x+y*x)<=(L1+L2))
	{ //ms->motor_states[0].position>100 ||
		cout <<"set here" <<endl;
		msg.data = theta[0];
		msg2.data = theta[1];
		pub.publish(msg);
		pub2.publish(msg2);
		//ROS_INFO("msg.data", msg);
		torque.request.torque_limit += 0.1f;
		torque_limit.call(torque);
		torque2.request.torque_limit += 0.1f;
		torque_limit.call(torque2);
		cout <<" torque : " << torque.request.torque_limit << endl;
		cout <<" torque2 : " << torque2.request.torque_limit << endl;
		/*if(abs(pre_position -ms-> motor_states[0].position<3))
		{
			cout <<"abs ?" <<endl;
			cout <<"tourq flag ?"<<torque_flag <<endl;
			while(torque_flag==0){
				cout <<"torque_flag" <<endl;
				torque.request.torque_limit += 0.1f;
				torque_limit.call(torque);
				torque2.request.torque_limit += 0.1f;
				torque_limit.call(torque2);
				cout<<"Torque mode on:";
				cin>> torque_flag ;
				cout<<"changed"<< torque_flag<<endl;
				while(torque_flag=1){
					torque.request.torque_limit -= 0.1f;
					torque_limit.call(torque);
					torque2.request.torque_limit -= 0.1f;
					torque_limit.call(torque2);
					cout<<"Torque mode off:";
					cin>> torque_flag ;
					cout<<"changed"<< torque_flag<<endl;
					if(torque_flag ==0)
					{break;}
				}
			}
		}*/
	}
    //////first motor ////
	// if(ms->motor_states[0].position>2000&&msg.data !=0){
	// 	ROS_INFO("Switch to decrease");
	// 	msg.data = 1.28f;
	// 	torque.request.torque_limit += 0.1f;
	// 	cout <<"here? "<<endl;
	// }else if(ms->motor_states[0].position< 1500&&msg.data !=4000&&(torque.request.torque_limit>0)){
	// 	ROS_INFO("Switch to increase");
	// 	msg.data = 2.28f;
	// 	torque.request.torque_limit += 0.1f;
	// 	torque_limit.call(torque);
	// 	cout <<"here? ?? "<<endl;
	// 	}
	// //////first motor ////
	// if(ms->motor_states[1].position>2000&&msg2.data !=0){
	// 	ROS_INFO("Switch to decrease");
	// 	msg2.data = 1.28f;
	// 	torque2.request.torque_limit += 0.1f;
	// 	cout <<"here? "<<endl;
	// }else if(ms->motor_states[0].position< 1500&&msg2.data !=4000&&(torque2.request.torque_limit>0)){
	// 	ROS_INFO("Switch to increase");
	// 	msg2.data = 2.28f;
	// 	torque2.request.torque_limit -= 0.1f;
	// 	torque_limit.call(torque2);
	// 	cout <<"here? ?? "<<endl;
	// 	}
	pub.publish(msg);
	pub2.publish(msg2);
}

int main(int argc, char** argv){

	ros::init(argc,argv,"start");
	ros::NodeHandle *nh;
	nh = new ros::NodeHandle("mcm");
	ros::Rate r(10);
	ros::Subscriber sub;
	sub = nh->subscribe<dynamixel_msgs::MotorStateList>("/motor_states/motor_my",1000,receiver);
	pub = nh->advertise<std_msgs::Float64>("/first_motor/command",100);
	pub2 = nh->advertise<std_msgs::Float64>("/second_motor/command",100);
	msg.data = 0;
	pub.publish(msg);
	pub2.publish(msg2);
////////////////////first motor/////////////////////
	ros::ServiceClient motor_speed;
	motor_speed = nh->serviceClient<dynamixel_controllers::SetSpeed>("/first_motor/set_speed");
	torque_limit = nh->serviceClient<dynamixel_controllers::SetTorqueLimit>("/first_motor/set_torque_limit");

	dynamixel_controllers::SetSpeed speed;
	speed.request.speed = 3.28f;
	torque.request.torque_limit=1.0f;

///////////////////second motor////////////////////
	ros::ServiceClient motor_speed2;
	motor_speed2 = nh->serviceClient<dynamixel_controllers::SetSpeed>("/second_motor/set_speed");
	torque_limit2 = nh->serviceClient<dynamixel_controllers::SetTorqueLimit>("/second_motor/set_torque_limit");

	dynamixel_controllers::SetSpeed speed2;
	speed2.request.speed = 3.28f;
	torque2.request.torque_limit=1.0f;

	if(!motor_speed.call(speed)) {
		cout<<"here?"<<endl;
		return 0;
	}
	if(!torque_limit.call(torque)) {
		return 0;
	}
	if(!motor_speed2.call(speed2)) {
		cout<<"here?"<<endl;
		return 0;
	}
	if(!torque_limit2.call(torque2)) {
		return 0;
	}
	cout <<"here spin?"<<endl;
	while(ros::ok()){
		ros::spinOnce();
		r.sleep();
	}
	return 0;
}

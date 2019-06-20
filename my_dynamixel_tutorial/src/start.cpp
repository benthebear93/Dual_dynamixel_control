//////////////////////////////////////////////////////////
///////////////  Robot Control Class 101 /////////////////
////////  Haegu Lee, GwangYeol Cha, Seungtake Oh  ////////
////////        Date : 2019. 06. 21 (Fri)         ////////
//////////////                             ///////////////
//////////////////////////////////////////////////////////
#include <ros/ros.h>
#include <dynamixel_controllers/SetTorqueLimit.h>
#include <dynamixel_controllers/SetSpeed.h>
#include <dynamixel_controllers/TorqueEnable.h>
#include <std_msgs/Float64.h>
#include <dynamixel_msgs/MotorStateList.h>
#include <iostream>
#include <cstdlib>
#include <math.h>

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
int torque_flag = 0; //changing torque mode flag
int pre_position =0; //save previous motor position

/////////Inverse Kinematic///////
float theta2_inverse(float x, float y)
{
	float theta2 = 0.0;
	theta2 = acos(((x*x)+(y*y)-(L1*L1)-(L2*L2))/(2*L1*L2));
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
/////////degree to radian , radian to degree//////
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
	//temp value to see the degree value
	float temp_d1 =0.0;
	float temp_d2 =0.0; 
	pre_position = ms-> motor_states[0].position;
	if (set_position == 0) //// set position 0 = Position Control 
	{
		cout<<"theta[0] : "<<theta[0]<<" theta[1] : "<<theta[1]<<endl;
		torque.request.torque_limit = +100.1f;
		torque_limit.call(torque);
		torque2.request.torque_limit = +100.1f;
		torque_limit.call(torque2);

		cout <<"where x, y? :";
		cin >> x;
		cin >> y;
		theta[1] = theta2_inverse(x,y);
		theta[0] = theta1_inverse(x,y);

		if((y<0)&&(x>=0))
		{
			theta[0]=6.28 + theta[0];
			cout <<theta[0]<<endl;
		}
		if(x<0)
		{
			theta[0]=3.14 + theta[0];
		}
		temp_d2 = rad2deg(theta[1]);
		temp_d1 = rad2deg(theta[0]);

		set_position = 1;
	}
	if (set_position ==2){ //// set position 2 = stop torque control
		cout<<"setpoint 2?"<<endl;
		float msp=0.0;
		float msp2=0.0;
		msp = (ms->motor_states[0].position)/11.377;
		msp2 = (ms->motor_states[1].position)/11.377;

		msg.data = 0;
		msg2.data = 0;
		pub.publish(msg);
		pub2.publish(msg2);

		torque.request.torque_limit = 100.1f;
		torque_limit.call(torque);
		torque2.request.torque_limit = 100.1f;
		torque_limit.call(torque2);
	}
	
	if (sqrt(x*x+y*x)<=(L1+L2)) /// work space
	{ 
		msg.data = theta[0];
		msg2.data = theta[1];
		pub.publish(msg);
		pub2.publish(msg2);
		if(abs(pre_position - ms-> motor_states[0].position)<3){
			if(torque_flag ==0){
				torque.request.torque_limit = 100.1f;
				torque_limit.call(torque);
				torque2.request.torque_limit = 100.1f;
				torque_limit.call(torque2);
				cout <<" on torque : " << torque.request.torque_limit << endl;
				cout<<"set torque off   "<<endl;
				cin>>torque_flag;
				if(torque_flag ==1){ /// Start Torque Control
					torque.request.torque_limit = -100.1f;
					torque_limit.call(torque);
					torque2.request.torque_limit = -100.1f;
					torque_limit.call(torque2);	
					cout <<" off torque : " << torque.request.torque_limit << endl;
					cout<<"set torque on"<<endl;\
					cin>>torque_flag;
					set_position =2;
				}
			}
		}

		torque.request.torque_limit = 100.1f;
		torque_limit.call(torque);
		torque2.request.torque_limit = 100.1f;
		torque_limit.call(torque2);
	}
	// pub.publish(msg);
	// pub2.publish(msg2);
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
		return 0;
	}
	if(!torque_limit.call(torque)) {
		return 0;
	}
	if(!motor_speed2.call(speed2)) {
		return 0;
	}
	if(!torque_limit2.call(torque2)) {
		return 0;
	}
	while(ros::ok()){
		ros::spinOnce();
		r.sleep();
	}
	return 0;
}

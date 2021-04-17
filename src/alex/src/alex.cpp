#include <stdio.h>
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>
#include <stdint.h>
#include "alex/packet.h"
#include "alex/serial.h"
#include "alex/serialize.h"
#include "alex/constants.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "visualization_msgs/InteractiveMarkerFeedback.h"
#include "geometry_msgs/Pose.h"
#include <math.h>
#include "tf2_msgs/TFMessage.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_ros/transform_listener.h"

#define PORT_NAME			"/dev/ttyACM0"
#define BAUD_RATE			B9600
#define PI                              3.14159265

int exitFlag=0;
sem_t _xmitSema;

void handleError(TResult error)
{
	switch(error)
	{
		case PACKET_BAD:
			ROS_INFO("ERROR: Bad Magic Number\n");
			break;

		case PACKET_CHECKSUM_BAD:
			ROS_INFO("ERROR: Bad checksum\n");
			break;

		default:
			ROS_INFO("ERROR: UNKNOWN ERROR\n");
	}
}

void handleStatus(TPacket *packet)
{
	printf("\n ------- ALEX STATUS REPORT ------- \n\n");
	printf("Left Forward Ticks:\t\t%d\n", packet->params[0]);
	printf("Right Forward Ticks:\t\t%d\n", packet->params[1]);
	printf("Left Reverse Ticks:\t\t%d\n", packet->params[2]);
	printf("Right Reverse Ticks:\t\t%d\n", packet->params[3]);
	printf("Left Forward Ticks Turns:\t%d\n", packet->params[4]);
	printf("Right Forward Ticks Turns:\t%d\n", packet->params[5]);
	printf("Left Reverse Ticks Turns:\t%d\n", packet->params[6]);
	printf("Right Reverse Ticks Turns:\t%d\n", packet->params[7]);
	printf("Forward Distance:\t\t%d\n", packet->params[8]);
	printf("Reverse Distance:\t\t%d\n", packet->params[9]);
	if (packet->params[10] == 1) printf("Colour Detected : Red");
	else if (packet->params[10] == 0) printf("Colour Detected : Green");
	else printf("Colour Sensor Failed to detect colour");
	//printf("Colour Detected:\t\t%d\n",packet->params[10]);
	printf("\n---------------------------------------\n\n");
}

void handleResponse(TPacket *packet)
{
	// The response code is stored in command
	switch(packet->command)
	{
		case RESP_OK:
			ROS_INFO("Command OK\n");
		break;

		case RESP_STATUS:
			handleStatus(packet);
		break;

		default:
			ROS_INFO("Arduino is confused\n");
	}
}

void handleErrorResponse(TPacket *packet)
{
	// The error code is returned in command
	switch(packet->command)
	{
		case RESP_BAD_PACKET:
			ROS_INFO("Arduino received bad magic number\n");
		break;

		case RESP_BAD_CHECKSUM:
			ROS_INFO("Arduino received bad checksum\n");
		break;

		case RESP_BAD_COMMAND:
			ROS_INFO("Arduino received bad command\n");
		break;

		case RESP_BAD_RESPONSE:
			ROS_INFO("Arduino received unexpected response\n");
		break;

		default:
			ROS_INFO("Arduino reports a weird error\n");
	}
}

void handleMessage(TPacket *packet)
{
	ROS_INFO("Message from Alex: %s\n", packet->data);
}

void handlePacket(TPacket *packet)
{
	switch(packet->packetType)
	{
		case PACKET_TYPE_COMMAND:
				// Only we send command packets, so ignore
			break;

		case PACKET_TYPE_RESPONSE:
				handleResponse(packet);
			break;

		case PACKET_TYPE_ERROR:
				handleErrorResponse(packet);
			break;

		case PACKET_TYPE_MESSAGE:
				handleMessage(packet);
			break;
	}
}

void sendPacket(TPacket *packet)
{
	char buffer[PACKET_SIZE];
	int len = serialize(buffer, packet, sizeof(TPacket));

	serialWrite(buffer, len);
}

void *receiveThread(void *p)
{
	char buffer[PACKET_SIZE];
	int len;
	TPacket packet;
	TResult result;
	int counter=0;

	while(1)
	{
		len = serialRead(buffer);
		counter+=len;
		if(len > 0)
		{
			result = deserialize(buffer, len, &packet);

			if(result == PACKET_OK)
			{
				counter=0;
				handlePacket(&packet);
			}
			else 
				if(result != PACKET_INCOMPLETE)
				{
					ROS_INFO("PACKET ERROR\n");
					handleError(result);
				}
		}
	}
}

void flushInput()
{
	char c;

	while((c = getchar()) != '\n' && c != EOF);
}

void getParams(TPacket *commandPacket)
{
	printf("Enter distance/angle in cm/degrees (e.g. 50) and power in %% (e.g. 75) separated by space.\n");
	printf("E.g. 50 75 means go at 50 cm at 75%% power for forward/backward, or 50 degrees left or right turn at 75%%  power\n");
	scanf("%d %d", &commandPacket->params[0], &commandPacket->params[1]);
	flushInput();
}

void sendCommand(char command)
{
	TPacket commandPacket;

	commandPacket.packetType = PACKET_TYPE_COMMAND;

	switch(command)
	{
		case 'f':
		case 'F':
			//getParams(&commandPacket);
			commandPacket.command = COMMAND_FORWARD;
			sendPacket(&commandPacket);
			break;

		case 'b':
		case 'B':
			//getParams(&commandPacket);
			commandPacket.command = COMMAND_REVERSE;
			sendPacket(&commandPacket);
			break;

		case 'l':
		case 'L':
			//getParams(&commandPacket);
			commandPacket.command = COMMAND_TURN_LEFT;
			sendPacket(&commandPacket);
			break;

		case 'r':
		case 'R':
			//getParams(&commandPacket);
			commandPacket.command = COMMAND_TURN_RIGHT;
			sendPacket(&commandPacket);
			break;

		case 's':
		case 'S':
			commandPacket.command = COMMAND_STOP;
			sendPacket(&commandPacket);
			break;

		case 'c':
		case 'C':
			commandPacket.command = COMMAND_CLEAR_STATS;
			commandPacket.params[0] = 0;
			sendPacket(&commandPacket);
			break;

		case 'g':
		case 'G':
			commandPacket.command = COMMAND_GET_STATS;
			sendPacket(&commandPacket);
			break;

		case 'q':
		case 'Q':
			exitFlag=1;
			break;

		default:
			printf("Bad command\n");

	}
}


void getKeyboardPress(const geometry_msgs::Twist::ConstPtr& msg) {
    //ROS_INFO("Until get keyboaord press is okay");
    TPacket Command_Packet;
    Command_Packet.packetType = PACKET_TYPE_COMMAND;
    if(msg->linear.x > 0 && msg->angular.z > 0) {
	    Command_Packet.params[0] = 0;
	    Command_Packet.params[1] = 100;
	    Command_Packet.command = COMMAND_FORWARD;
	    sendPacket(&Command_Packet);
    }
    else if (msg->linear.x > 0 && msg->linear.x < 1) {
	    Command_Packet.params[0] = 5;
	    Command_Packet.params[1] = 95;
	    Command_Packet.command = COMMAND_FORWARD;
	    sendPacket(&Command_Packet);
    } else if (msg->linear.x < 0) {
	    Command_Packet.params[0] = 2;
	    Command_Packet.params[1] = 90;
	    Command_Packet.command = COMMAND_REVERSE;
	    sendPacket(&Command_Packet);
    } else if (msg->angular.z > 0) {
	    Command_Packet.params[0] = 5;
	    Command_Packet.params[1] = 100;
	    Command_Packet.command = COMMAND_TURN_LEFT;
	    sendPacket(&Command_Packet);
    } else if (msg->angular.z < 0) {
	    Command_Packet.params[0] = 5;
	    Command_Packet.params[1] = 100;
	    Command_Packet.command = COMMAND_TURN_RIGHT;
	    sendPacket(&Command_Packet);
    } else if (msg->linear.x > 1) {
	    Command_Packet.command = COMMAND_CLEAR_STATS;
	    sendPacket(&Command_Packet);
    } else if (msg->linear.x == 0) {
	    Command_Packet.command = COMMAND_STOP;
	    sendPacket(&Command_Packet);
    }
}

volatile double x_pose = 0;
volatile double y_pose = 0;
volatile double alex_x = 0;
volatile double alex_y = 0;
volatile double alex_z = 0;
volatile double alex_w = 0;
volatile double alex_angle = 0;
volatile int turning_in_progress = 0;
volatile int movement_in_progress = 0;

void updateAlexPose(const tf2_msgs::TFMessageConstPtr &msg) {
	if(msg->transforms[0].header.frame_id == "map") {
		double scan_x = msg->transforms[0].transform.translation.x;
		double scan_y = msg->transforms[0].transform.translation.y;
		double scan_z = msg->transforms[0].transform.rotation.z;
		double scan_w = msg->transforms[0].transform.rotation.w;
		double sinr_cosp = 2*(scan_z*scan_w);
		double cosr_cosp = 1 - 2*(scan_z*scan_z);
		double unit_angle;
		//gonna leave the angle in radians until we wanna send message
		alex_angle = std::atan2(sinr_cosp,cosr_cosp);
		ROS_INFO("alex_angle is %lf", alex_angle);
		if ( alex_angle > PI/2 ) unit_angle = PI - alex_angle;
		else if (alex_angle < -PI/2) unit_angle = -PI - alex_angle;
		else unit_angle = alex_angle;
		ROS_INFO("unit_angle is %lf", unit_angle);
		double unit_x = cos(unit_angle);
		double unit_y = sin(unit_angle);
		alex_x = scan_x + unit_x;
		alex_y = scan_y + unit_y;
		ROS_INFO("alex_x is %lf and alex_y is %lf",alex_x,alex_y);
	}
}


void go_left(int times) {
	TPacket cmd;
	cmd.packetType = PACKET_TYPE_COMMAND;
	cmd.params[0] = 4;
	cmd.params[1] = 90;
	cmd.command = COMMAND_TURN_LEFT;
	for (int i = 0; i < times; i++) {
		sendPacket(&cmd);
		sleep(2);
	}
}

void go_right(int times) {
	TPacket cmd;
	cmd.packetType = PACKET_TYPE_COMMAND;
	cmd.params[0] = 4;
	cmd.params[1]= 100;
	cmd.command = COMMAND_TURN_RIGHT;
	for (int i = 0; i < times; i++) {
		sendPacket(&cmd);
		sleep(2);
	}
}

void go_straight(int times) {
	TPacket cmd;
	cmd.packetType = PACKET_TYPE_COMMAND;
	cmd.params[0] = 5;
	cmd.params[1] = 100;
	cmd.command = COMMAND_FORWARD;
	for (int i = 0; i < times; i ++) {
		sendPacket(&cmd);
		sleep(2);
		//go_right(2);
	}
}

	

void move_alex(){
	double dist_forward = sqrt((x_pose - alex_x) * (x_pose - alex_x) + (y_pose - alex_y) * (y_pose - alex_y));
	double for_angle_x = x_pose - alex_x;
	double for_angle_y = y_pose - alex_y;
	double angle_pos = atan2(for_angle_y,for_angle_x);//
	if (fabs(angle_pos - 180) <= 2.00 || fabs(angle_pos + 180) <= 2.00) angle_pos = 180; //
	double angle = (angle_pos - alex_angle) * 180/PI;
	if(angle_pos < 0 && alex_angle > 0 && angle < -180)  angle = 360 + angle;
	else if(angle_pos > 0 && alex_angle < 0 && angle > 180) angle = -360 +  angle;
	ROS_INFO("marker x is %lf and markery_y is %lf",x_pose,y_pose);//
	ROS_INFO("angle of marker is %lf",angle_pos * 180/PI);//
	ROS_INFO("dist_forward  %lf",dist_forward);
	if  (angle > 0) ROS_INFO("go left %lf degrees",angle);
	else ROS_INFO("go right %lf degrees", angle*-1);
	//ROS_INFO("angle to move %lf",angle);
		ROS_INFO("alex_x is %lf and alex_y is %lf",alex_x,alex_y);
		ROS_INFO("alex_angle is %lf", alex_angle * 180/PI);
//	ROS_INFO(" marker_angle %f " atan2(y_pose,x_pose));
	int angle_no = round(angle / 20);
	ROS_INFO("Turning %d times", angle_no);
	int dist_no = round(dist_forward/0.12);
	if(turning_in_progress) {
		if (angle > 0 ) {
			go_left(1);
		} else if (angle < 0) {
			go_right(1);
		}
		if(fabs(angle) < 15) turning_in_progress = 0;
	}
	else {go_straight(1);
		turning_in_progress = 1;
	}
	if ( dist_forward - 0.1 <= 0.06) {
		movement_in_progress = 0;
	}
	/**if (angle > 0 ) {
		go_left(angle_no);
	} else if (angle < 0) {
		go_right(angle_no * -1);
	}
	go_straight(dist_no);*/

}

void poseUpdate(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
	if(feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE) {
		x_pose = feedback->pose.position.x;
		y_pose = feedback->pose.position.y;
	}
	if(feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP || movement_in_progress) {
		turning_in_progress = 1;
		movement_in_progress = 1;
		//move_alex();
		//x_pose = 0;
		//y_pose = 0;
	}
}


int main(int argc,char **argv)
{
	// Connect to the Arduino
	startSerial(PORT_NAME, BAUD_RATE, 8, 'N', 1, 5);

	// Sleep for two seconds
	printf("WAITING TWO SECONDS FOR ARDUINO TO REBOOT\n");
	sleep(2);
	printf("DONE\n");

	// Spawn receiver thread
	pthread_t recv;

	pthread_create(&recv, NULL, receiveThread, NULL);

	// Send a hello packet
	TPacket helloPacket;

	helloPacket.packetType = PACKET_TYPE_HELLO;
	sendPacket(&helloPacket);

    ros::init(argc,argv,"alex");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("cmd_vel",1,getKeyboardPress);
    //ros::Subscriber lub = n.subscribe("/tf",1,updateAlexPose);
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Subscriber dub = n.subscribe("/basic_controls/feedback",1,poseUpdate);
  //  ros::spin();
    ros::Rate rate(10.0);
     while(n.ok()) {
    geometry_msgs::TransformStamped transformStamped;
    try {
    transformStamped = tfBuffer.lookupTransform("map","base_link",ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
	    ROS_WARN("%s",ex.what());
	    ros::Duration(1.0).sleep();
	    continue;
    }
    
		double scan_x = transformStamped.transform.translation.x;
		double scan_y = transformStamped.transform.translation.y;
		double scan_z = transformStamped.transform.rotation.z;
		double scan_w = transformStamped.transform.rotation.w;
		double sinr_cosp = 2*(scan_z*scan_w);
		double cosr_cosp = 1 - 2*(scan_z*scan_z);
		double unit_angle;
		//gonna leave the angle in radians until we wanna send message
		alex_angle = std::atan2(sinr_cosp,cosr_cosp);
		//ROS_INFO("alex_angle is %lf", alex_angle * 180/PI);
		int unit_x_mult = 1;
		int unit_y_mult = 1;
		if ( alex_angle < -PI/2 ) {
		     unit_angle = PI - alex_angle;
		     unit_x_mult = -1;
		     unit_y_mult = -1;
		}
		else if (alex_angle > PI/2) {
			unit_angle = -PI - alex_angle;
			unit_x_mult = -1;
		}
		else if (alex_angle < 0 && alex_angle > -PI/2) {
			unit_angle = alex_angle;
			unit_y_mult = -1;
		} else unit_angle = alex_angle;
                unit_angle = fabs(std::atan(sinr_cosp/cosr_cosp));
		//ROS_INFO("unit_angle is %lf", unit_angle * 180/PI);
		//ROS_INFO("x_mul is %d and y_mult is %d",unit_x_mult,unit_y_mult);
		double unit_x = cos(fabs(unit_angle)) * unit_x_mult;
		double unit_y = sin(fabs(unit_angle)) * unit_y_mult;
		alex_x = scan_x + unit_x;
		alex_y = scan_y + unit_y;
		//ROS_INFO("alex_x is %lf and alex_y is %lf",alex_x,alex_y);
		//ROS_INFO("unit_x is %lf and unit_y is %lf",unit_x,unit_y);
    if(movement_in_progress) move_alex();
    rate.sleep();
    ros::spinOnce();
    }
    /**
	while(!exitFlag)
	{
		char ch;
		printf("Command (f=forward, b=reverse, l=turn left, r=turn right, s=stop, c=clear stats, g=get stats q=exit)\n");
		scanf("%c", &ch);

		// Purge extraneous characters from input stream
		flushInput();

		sendCommand(ch);
	}*/

	printf("Closing connection to Arduino.\n");
    return 0;
	endSerial();
}

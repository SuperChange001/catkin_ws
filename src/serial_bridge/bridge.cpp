#include <string>
#include <ros/ros.h>                           // 包含ROS的头文件
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <boost/asio.hpp>                  //包含boost库函数
#include <boost/bind.hpp>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include "std_msgs/String.h"              //ros定义的String数据类型

using namespace std;
using namespace boost::asio;           //定义一个命名空间，用于后面的读写操作
io_service iosev;
serial_port sp(iosev, "/dev/ttyUSB0");         //定义传输的串口

void cmdMessageReceived(const geometry_msgs::Twist&msg)//
{
	ROS_INFO_STREAM("Getting command:"
		<<"linear, "<<msg.linear.x
		<<"angular"<<msg.angular.z
	);
	// v m/s * 100
	// r degree/s 
	char commandbuf[]="v:+1000,r:+360";
	
	float vsf = 9876;//msg.linear.x * 100;
	if(vsf<0)
	{
		commandbuf[2] = '-';
		vsf = -vsf;
	}
	unsigned int	vsi = (unsigned int)vsf;
	commandbuf[3] = 0x30 + vsi%10000/1000;
	commandbuf[4] = 0x30 + vsi%1000/100;
	commandbuf[5] = 0x30 + vsi%100/10;
	commandbuf[6] = 0x30 + vsi%10;
	
	float rsf = 359;//msg.angular.z;
	if(rsf<0)
	{
		commandbuf[10] = '-';
		rsf = -rsf;		
	}
	unsigned int rsi = (unsigned int)rsf;
	commandbuf[11] = 0x30 + rsi%1000/100;
	commandbuf[12] = 0x30 + rsi%100/10;
	commandbuf[13] = 0x30 + rsi%10;

	// Serial send Demo just for test
	write(sp, buffer(commandbuf, 14));  
}


unsigned int readlen;
int main(int argc, char** argv) {

    ros::init(argc, argv, "boost");       //初始化节点
	ros::NodeHandle n;
    
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);     
	
	ros::Subscriber chatter_sub = n.subscribe("turtle1/cmd_vel",1000,&cmdMessageReceived);

	ros::spin();

  	ros::Rate loop_rate(2);

    sp.set_option(serial_port::baud_rate(115200));   
    sp.set_option(serial_port::flow_control());
    sp.set_option(serial_port::parity());
    sp.set_option(serial_port::stop_bits());
    sp.set_option(serial_port::character_size(8));
write(sp, buffer("Hello world", 12));  
	// Test if the package is running
	//ROS_INFO("hello\n");//打印接受到的字符串
    //ros::spinOnce();

	while (ros::ok()) {
        
		
		
		boost::asio::streambuf buf;			// new an stream buffer		
		read_until (sp,buf,0xBA);			// read out the data from serial
		readlen = buf.size();				// To get the data length
	
		char Charbuf[readlen+1];            // new a normal buffer to copy the data for streabuf
		buf.sgetn(Charbuf,readlen);			// copy the data
		Charbuf[readlen-1]='\0';			// put an '\0' for making a string smoothly
		
		if(Charbuf[0]==0xAB)//to check if there has a frame header
		{
			// remode frame headr(0xAB) and rear(0xBA)
			string str(&Charbuf[1],&Charbuf[readlen-1]);            //将数组转化为字符串
			std_msgs::String msg;
			std::stringstream ss;
			ss <<str;

			msg.data = ss.str();
		 
			ROS_INFO("Get %dBytes Data: %s", readlen, msg.data.c_str());//打印接受到的字符串

			chatter_pub.publish(msg);   //发布消息

			ros::spinOnce();

			//loop_rate.sleep();
		}
    }
	
    iosev.run(); 
    return 0;
}

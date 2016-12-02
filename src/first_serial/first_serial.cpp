#include <string>
#include <ros/ros.h>                           // 包含ROS的头文件
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <boost/asio.hpp>                  //包含boost库函数
#include <boost/bind.hpp>
#include <math.h>
#include "std_msgs/String.h"              //ros定义的String数据类型

using namespace std;
using namespace boost::asio;           //定义一个命名空间，用于后面的读写操作


unsigned int readlen;
int main(int argc, char** argv) {

    ros::init(argc, argv, "boost");       //初始化节点
    ros::NodeHandle n;
    
  	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);      //定义发布消息的名称及sulv

  	ros::Rate loop_rate(10);


    io_service iosev;
    serial_port sp(iosev, "/dev/ttyUSB0");         //定义传输的串口
    sp.set_option(serial_port::baud_rate(115200));   
    sp.set_option(serial_port::flow_control());
    sp.set_option(serial_port::parity());
    sp.set_option(serial_port::stop_bits());
    sp.set_option(serial_port::character_size(8));

	// Test if the package is running
	//ROS_INFO("hello\n");//打印接受到的字符串
    
	while (ros::ok()) {
        
		// Serial send Demo just for test
		 write(sp, buffer("Hello world", 12)); 
		
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

			loop_rate.sleep();
		}
    }

    iosev.run(); 
    return 0;
}

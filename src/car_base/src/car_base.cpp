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
int linearx=0;
int angularz=0;

ros::Time current_time;
unsigned int readlen;
double x = 0.0;
double y = 0.0;
double th = 0.0;

// v m/s * 100
	// r degree/s 
char commandbuf[]="a2v:+aaaa,w:+000,r:360b";

void cmdMessageReceived(const geometry_msgs::Twist&msg)//
{
	

	if(msg.linear.x==2){
		linearx = linearx+10;
        angularz = 0;
	}
	if(msg.linear.x==-2){
		linearx = linearx-10;
        angularz=0;
	}
	if(msg.angular.z==2){
		angularz = angularz+3;
		linearx =0;
	}
	if(msg.angular.z==-2){
		angularz = angularz-3;
		linearx =0;
	}


	if(linearx>80)
	    linearx=80;
    if(linearx<-80)
        linearx=-80;

    if(angularz>90)
	    angularz=90;
    if(angularz<-90)
        angularz=-90;
    if(angularz!=0)
    {
    linearx =0;
    }

	float vsf = linearx;//msg.linear.x * 100;
	if(vsf<0)
	{
		commandbuf[4] = '-';
		vsf = -vsf;
	}
    else
    {
        commandbuf[4] = '+';
    }
	unsigned int	vsi = (unsigned int)vsf;
	commandbuf[5] = 0x30 + vsi%10000/1000;
	commandbuf[6] = 0x30 + vsi%1000/100;
	commandbuf[7] = 0x30 + vsi%100/10;
	commandbuf[8] = 0x30 + vsi%10;
	
	float rsf = angularz;//msg.angular.z;
	if(rsf<0)
	{
		commandbuf[12] = '-';
		rsf = -rsf;		
	}
    else
    {
        commandbuf[12] = '+';
    }
	unsigned int rsi = (unsigned int)rsf;
	commandbuf[13] = 0x30 + rsi%1000/100;
	commandbuf[14] = 0x30 + rsi%100/10;
	commandbuf[15] = 0x30 + rsi%10;
//        angularz = 0;

	ROS_INFO_STREAM("Getting command:"
		<<"linear"<<msg.linear.x
		<<", angular"<<msg.angular.z
		<<" ==== vs:"<<  linearx
		<<", ws:"<<angularz
	);
}

void odom_calculate(tf::TransformBroadcaster&ob,ros::Publisher&p,double vx, double vy, double vth,double x, double y, double th)
{

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    ob.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    p.publish(odom);

//ROS_INFO("publish once");
}


void handle_read(char *buf,boost::system::error_code ec,
    std::size_t bytes_transferred)
{
    //cout.write(buf, bytes_transferred);
ROS_INFO("Get Bytes Data");
}

int cnt;
int main(int argc, char** argv) {

    ros::init(argc, argv, "move_base");       //初始化节点

    ros::NodeHandle move_base_NodeHandle;

    tf::TransformBroadcaster odom_broadcaster;


      
    ros::Subscriber move_base_sub = move_base_NodeHandle.subscribe("turtle1/cmd_vel",1000,&cmdMessageReceived);
   ros::Publisher move_base_pub = move_base_NodeHandle.advertise<nav_msgs::Odometry>("odom", 1000);   
  	
    
    sp.set_option(serial_port::baud_rate(115200));   
    sp.set_option(serial_port::flow_control());
    sp.set_option(serial_port::parity());
    sp.set_option(serial_port::stop_bits());
    sp.set_option(serial_port::character_size(8));
   // write(sp, buffer("Hello world", 12)); 
    write(sp, buffer(commandbuf, 23));
    commandbuf[5] = 0x30;
        commandbuf[6] = 0x30;
        commandbuf[7] = 0x30;
        commandbuf[8] = 0x30;

    ros::Rate loop_rate(50);
    while (ros::ok()) {
        ros::spinOnce();
        boost::asio::streambuf buf;			// new an stream buffer		
		read_until (sp,buf,'b');			// read out the data from serial
		readlen = buf.size();				// To get the data length
	
		char Charbuf[readlen+1];            // new a normal buffer to copy the data for streabuf
		buf.sgetn(Charbuf,readlen);			// copy the data
		Charbuf[readlen-1]='\0';			// put an '\0' for making a string smoothly
	    

		if(Charbuf[0]=='a')//to check if there has a frame header
		{
			// remode frame headr(0xAB) and rear(0xBA)
			string str(&Charbuf[1],&Charbuf[readlen-1]);            //将数组转化为字符串
			std_msgs::String msg;
			std::stringstream ss;
			ss <<str;

			msg.data = ss.str();
            current_time = ros::Time::now();		 
		//	ROS_INFO("Get %dBytes Data: %s", readlen, msg.data.c_str());//打印接受到的字符串
           double vx=0,vth=0,x,y,th;
	    vx = (Charbuf[5]-0x30)+(Charbuf[7]-0x30)*0.1+(Charbuf[8]-0x30)*0.01;
	    if(Charbuf[4]=='-')
		vx = -vx;

	    vth = (Charbuf[15]-0x30)*10+(Charbuf[16]-0x30)+(Charbuf[18]-0x30)*0.1;        
	    if(Charbuf[14]=='-')
		vth = -vth;
	    
	    x = (Charbuf[24]-0x30)*10+(Charbuf[25]-0x30)+(Charbuf[26]-0x30)*0.1+(Charbuf[27]-0x30)*0.01+(Charbuf[28]-0x30)*0.001;
	    if(Charbuf[23]=='-')
		x=-x;

	    y = (Charbuf[34]-0x30)*10+(Charbuf[35]-0x30)+(Charbuf[36]-0x30)*0.1+(Charbuf[37]-0x30)*0.01+(Charbuf[38]-0x30)*0.001;
	    if(Charbuf[33]=='-')		
		y =-y;

            th = (Charbuf[43]-0x30)*100+(Charbuf[44]-0x30)*10+(Charbuf[45]-0x30);
	    double rvth = vth/180*3.14159265;
	    double rth = th/180*3.14159265;

            odom_calculate(odom_broadcaster, move_base_pub, vx , 0, rvth,x,y,rth);

// odom_calculate(odom_broadcaster, move_base_pub, 0,0,0,0,0,0);
	  
	   cnt++;
	if(cnt==20)
	{
	    cnt =0;
     	ROS_INFO_STREAM("set_vx:"<<linearx
		    <<", set_vth:"<<angularz
            <<", vx:"<<vx<<", vth:"<<vth<<", x:"<<x<<", y:"<<y<<", th:"<<th);

	}
	//since all odometry is 6DOF we'll need a quaternion created from yaw

       write(sp, buffer(commandbuf, 23));  
	   loop_rate.sleep();
            
		}
        

    }

    	commandbuf[5] = '0';
        commandbuf[6] = '0';
        commandbuf[7] = '0';
        commandbuf[8] = '0';
     write(sp, buffer(commandbuf, 23));


    iosev.run(); 
    return 0;

}



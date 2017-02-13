#include <ros/ros.h>
#include <image_transport/image_transport.h>


image_transport::Publisher my_image_pubisher;


void imageCB(const sensor_msgs::ImageConstPtr& msg)
{
    int i,j;
    sensor_msgs::Image temp_msg;
    
    temp_msg = *msg;
    temp_msg.width = msg->height;
    temp_msg.height = msg->width;

    for(i=0;i<480;i++)
    {
      for(j=0;j<640;j++)
      {
        temp_msg.data[((639-j)*480+i)*4]= msg->data[(i*640+j)*4];
        temp_msg.data[((639-j)*480+i)*4+1]= msg->data[(i*640+j)*4+1];
        temp_msg.data[((639-j)*480+i)*4+2]= msg->data[(i*640+j)*4+2];
        temp_msg.data[((639-j)*480+i)*4+3]= msg->data[(i*640+j)*4+3];
      }
    }
    my_image_pubisher.publish(temp_msg);
}


int main(int argc, char** argv)
{
 //  image_transport::Publisher my_image_pubisher;
   ros::init(argc, argv, "image_rotation_node");
   ros::NodeHandle image_rotation_nh;

   image_transport::ImageTransport my_image_transport(image_rotation_nh);

   my_image_pubisher = my_image_transport.advertise("/camera/rotated_image", 1);
   image_transport::Subscriber sub = my_image_transport.subscribe("/camera/depth/image", 1, imageCB);
    ros::spin();
}



#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <sstream>
#include <pthread.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>

#define DEFAULT_BAUD 9600
#define DEFAULT_PORT "/dev/ttyUSB0"
/*
FILE *fpSerial = NULL;

//Initialize serial port, return file descriptor
FILE *serialInit(char * port, int baud)
{
  int BAUD = 0;
  int fd = -1;
  struct termios newtio;
  FILE *fp = NULL;


 // read/write, not controlling terminal for process,
  fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY );
  if ( fd<0 )
  {
    ROS_ERROR("serialInit: Could not open serial device %s",port);
    return fp;
  }

  // set up new settings
  memset(&newtio, 0,sizeof(newtio));
  newtio.c_cflag =  CS8 | CLOCAL | CREAD;  //no parity, 1 stop bit

  newtio.c_iflag = IGNCR;    //ignore CR, other options off
  newtio.c_iflag |= IGNBRK;  //ignore break condition

  newtio.c_oflag = 0;        //all options off

  newtio.c_lflag = ICANON;     //process input as lines

  // activate new settings
  tcflush(fd, TCIFLUSH);
  //Look up appropriate baud rate constant
  switch (baud)
  {
     case 38400:
     default:
        BAUD = B38400;
        break;
     case 19200:
        BAUD  = B19200;
        break;
     case 9600:
        BAUD  = B9600;
        break;
     case 4800:
        BAUD  = B4800;
        break;
     case 2400:
        BAUD  = B2400;
        break;
     case 1800:
        BAUD  = B1800;
        break;
     case 1200:
        BAUD  = B1200;
        break;
  }  //end of switch baud_rate
  if (cfsetispeed(&newtio, BAUD) < 0 || cfsetospeed(&newtio, BAUD) < 0)
  {
    ROS_ERROR("serialInit: Failed to set serial baud rate: %d", baud);
    close(fd);
    return NULL;
  }
  tcsetattr(fd, TCSANOW, &newtio);
  tcflush(fd, TCIOFLUSH);

  //Open file as a standard I/O stream
  fp = fdopen(fd, "r+");
  if (!fp) {
    ROS_ERROR("serialInit: Failed to open serial stream %s", port);
    fp = NULL;
  }
  return fp;
} //serialInit */



class BUD_E_BASE
{
public:
  BUD_E_BASE()
   {
    pub_ =n_.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel",100);

    sub_ = n_.subscribe("/cmd_vel", 500, &BUD_E_BASE::commandBase, this);
    ROS_INFO("Initialized BUD_E_Base");
  }

  void commandBase(const geometry_msgs::Twist& cmd)
  {
   ROS_INFO("Command received: Linear.x=[%f]  Angular.z=[%f]", cmd.linear.x,cmd.angular.z);
    pub_.publish(cmd);
  }

private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;

};//End of class 




/*void commandMD(const geometry_msgs::Twist& cmd)
{
  ROS_INFO("Command received: Linear.x=[%f]  Angular.z=[%f]", cmd.linear.x,cmd.angular.z);
    //ROS_DEBUG("Command received: [%s]", msg->data.c_str());
  fprintf(fpSerial, "%f %f", cmd.linear.x,cmd.angular.z); //appends newline
}

void commandB(const geometry_msgs::Twist& cmd)
{
  ROS_INFO("Command received: Linear.x=[%f]  Angular.z=[%f]", cmd.linear.x,cmd.angular.z);
  
}

*/

int main(int argc, char **argv)
{

  ros::init(argc, argv, "BUD_E_Base");
ROS_INFO("Started BUDE_E_Base node");
BUD_E_BASE start;
  //ros::NodeHandle n;


/*  ROS_INFO("connection initializing (%s) at %d baud", DEFAULT_PORT, DEFAULT_BAUD);
     fpSerial = serialInit(DEFAULT_PORT, DEFAULT_BAUD);
    if (!fpSerial )
    {
      ROS_ERROR("unable to create a new serial port");
      return 1;
    }
    ROS_INFO("serial connection successful");
*/

  //ros::Subscriber BUDEBASE = n.subscribe("/cmd_vel", 500, commandBase);
  //ros::Publisher pub=n.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel",100);


  ros::spin();

  //fclose(fpSerial);
//  ROS_INFO("Stopping Mini-Dora");
  ROS_INFO("Stopping BUD-E");
  return 0;
}


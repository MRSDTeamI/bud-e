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
#define DEFAULT_PORT "/dev/ttyACM1"

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
} //serialInit




void commandMD(const geometry_msgs::Twist& cmd)
{
  ROS_INFO("Command received: Linear.x=[%f]  Angular.z=[%f]", cmd.linear.x,cmd.angular.z);
    //ROS_DEBUG("Command received: [%s]", msg->data.c_str());
 float linVel=cmd.linear.x;
float angVel=cmd.angular.z;
//Currently MiniDora is programmed to accept Linear_velocities_x in: 0.2,0.4,0.6,0.8,1.0,1.2,1.4,-0.2,-0.4 m/s
// Angular_velocities_z in : 0.2,0.4,0.6,0.8,-0.2,-0.4,-0.6,-0.8s; +ve==> CWR; -ve==> CCWR




if(angVel>-0.2&&angVel<0.2)
{



if(linVel>1.2&&linVel<=1.4)
	fprintf(fpSerial,"r1350 l1930;\n");
	
else if(linVel>1&&linVel<=1.2)
	fprintf(fpSerial,"r1370 l1872;\n");
else if(linVel>0.8&&linVel<=1)
	fprintf(fpSerial,"r1389 l1822;\n");
else if(linVel>0.6&&linVel<=0.8)
	fprintf(fpSerial,"r1408 l1772;\n");	 
else if(linVel>0.4&&linVel<=0.6)
	fprintf(fpSerial,"r1427 l1772;\n");
else if(linVel>0.2&&linVel<=0.4)
	fprintf(fpSerial,"r1446 l1672;\n");
else if(linVel>0&&linVel<=0.2)
	fprintf(fpSerial,"r1460 l1622;\n");
else if(linVel==0)
	fprintf(fpSerial,"r1500 l1500;\n");
else if(linVel>-0.2&&linVel<0)
	fprintf(fpSerial,"r1608 l1452;\n");
else if(linVel>=-0.4&&linVel<=-0.2)
	fprintf(fpSerial,"r1658 l1430;\n");
else
   fprintf(fpSerial,"r1500 l1500;\n");

}
	


else 
{

if(angVel>0.6&&angVel<=0.8)
	fprintf(fpSerial,"r1758 l1772;\n");
else if(angVel>0.4&&angVel<=0.6)
	fprintf(fpSerial,"r1708 l1722;\n");
else if(angVel>0.2&&angVel<=0.4)
	fprintf(fpSerial,"r1658 l1672;\n");
else if(angVel>0&&angVel<=0.2)
	fprintf(fpSerial,"r1608 l1600;\n");
else if(angVel==0)
	fprintf(fpSerial,"r1500 l1500;\n");
else if(angVel>-0.2&&angVel<0)
	fprintf(fpSerial,"r1465 l1452;\n");
else if(angVel>-0.4&&angVel<=-0.2)
	fprintf(fpSerial,"r1446 l1430;\n");
else if(angVel>-0.6&&angVel<=-0.4)
	fprintf(fpSerial,"r1427 l1410;\n");
else if(angVel>=-0.8&&angVel<=-0.6)
	fprintf(fpSerial,"r1408 l1390;\n");
else
   fprintf(fpSerial,"r1500 l1500;\n");
  
}




//fprintf(fpSerial, "%f %f", cmd.linear.x,cmd.angular.z); //appends newline
  
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "MiniDora");

  ros::NodeHandle n;


  ROS_INFO("connection initializing (%s) at %d baud", DEFAULT_PORT, DEFAULT_BAUD);
     fpSerial = serialInit(DEFAULT_PORT, DEFAULT_BAUD);
    if (!fpSerial )
    {
      ROS_ERROR("unable to create a new serial port");
      return 1;
    }
  sleep(1);
    ROS_INFO("serial connection successful");


  ros::Subscriber MiniDora = n.subscribe("/cmd_vel", 5, commandMD);



  ros::spin();

  fclose(fpSerial);
  ROS_INFO("Stopping Mini-Dora");
  return 0;
}

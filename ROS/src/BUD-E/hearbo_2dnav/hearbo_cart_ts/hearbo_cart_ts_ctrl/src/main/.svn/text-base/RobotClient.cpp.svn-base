#include "RobotClient.h"

#include "constants.h"

#include "ros/ros.h"

// Comm
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <string>
#include <cstdio>
#include <errno.h>


void* receiveThread( void* data );
void* statusRequestThread( void* pData );

RobotClient::RobotClient()
{  
  strcpy(m_oConnection.pcIp, ROBOT_CART_IP);
  strcpy(m_oConnection.pcName, "cart");
  m_oConnection.nPort = ROBOT_CART_PORT;
  m_oConnection.nSocket = -1;
}

bool RobotClient::initialize()
{ 
  if( !connectTo( m_oConnection ) )
  {
    terminate();
    return false;
  }
  
  return true;
}

bool RobotClient::connectTo( T_ConnectionData& oConnectionData )
{
  ROS_INFO("Connecting to %s:%d.", oConnectionData.pcIp, oConnectionData.nPort);
  
  // Construct the socket address
  sockaddr_in dstAddr;
  bzero((char *)&dstAddr, sizeof(dstAddr));  
  
  // Construct a new socket
  oConnectionData.nSocket = socket(AF_INET, SOCK_STREAM, 0);
  if(oConnectionData.nSocket < 0)
  {
    ROS_ERROR("Socket: %s",strerror(errno));
    return false;
  }

  dstAddr.sin_port = htons(oConnectionData.nPort);
  dstAddr.sin_family = AF_INET;
  //dstAddr.sin_addr.s_addr = inet_addr(ROBOT_IP);

  // Find the host by name
  {
    hostent *phostent;
    
    if( ( phostent = gethostbyname(oConnectionData.pcIp) ) != NULL){
      bcopy(phostent->h_addr, &dstAddr.sin_addr, phostent->h_length);
    }else{
      ROS_ERROR("Get host: %s",strerror(errno));
      close(oConnectionData.nSocket);
      oConnectionData.nSocket = -1;
      return false;
    }
  }

  // Connect
  if (connect(oConnectionData.nSocket, (struct sockaddr *)&dstAddr, sizeof(dstAddr)) < 0)
  {
    ROS_ERROR("Connect: %s",strerror(errno));
    close(oConnectionData.nSocket);
    oConnectionData.nSocket = -1;
    return false;
  }
  
  ROS_INFO("Connected to %s:%d", oConnectionData.pcIp, oConnectionData.nPort);//*/

  return true;//*/
}
  
bool RobotClient::sendCommand( char* pcCommand )
{
  char pcCommandBuffer[520];
  sprintf(pcCommandBuffer, "%s\r", pcCommand ); // Append carriage return
  
  ROS_INFO("Sending to %s server: %s", m_oConnection.pcName, pcCommandBuffer);
  
  if( send(m_oConnection.nSocket, pcCommandBuffer, strlen( pcCommandBuffer ), 0) < 0)
  {
    ROS_WARN("Could not send command data to robot: %s", strerror(errno));
    return true; // hide this failure
  }//*/
  
	return true;
}

bool RobotClient::terminate()
{
  if( m_oConnection.nSocket >= 0 )
  {
    close(m_oConnection.nSocket);
    m_oConnection.nSocket = -1;
  }
  
  ROS_INFO("Disconnected from robot.");
  return true;
}






#ifndef ROBOT_CLIENT_H_INCLUDED
#define ROBOT_CLIENT_H_INCLUDED

#include <fcntl.h>

struct T_ConnectionData
{
  char pcIp[32];
  char pcName[32];
  int nPort;
  int nSocket;
};

class RobotClient
{
public:

  RobotClient();
  bool initialize();
  bool sendCommand( char* pcCommand );
  bool terminate();
  
private:

  T_ConnectionData m_oConnection;
    
  bool connectTo( T_ConnectionData& oConnectionData );
  
};

#endif // ROBOT_CLIENT_H_INCLUDED

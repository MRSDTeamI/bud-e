#include "RobotClient.h"

#define CART_COMMAND_LENGTH 257

class CartCommandMaker
{
public:
  bool initialize( bool bDoConnect );
  bool terminate();
  
  bool sendActivationCommand();
  bool sendTranslationalVelocityCommand( double fVelX, double fVelY );
  bool sendRotationalVelocityCommand( double fVelRot );
  
private:

  RobotClient m_oClient;
  char m_pcCommand[CART_COMMAND_LENGTH];
};

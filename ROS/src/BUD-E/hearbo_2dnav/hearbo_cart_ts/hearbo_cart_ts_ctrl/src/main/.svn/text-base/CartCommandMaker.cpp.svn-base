#include "CartCommandMaker.h"

#include "ros/ros.h"
#include "constants.h"

#include <strings.h>
#include <stdio.h>

bool CartCommandMaker::initialize( bool bDoConnect )
{  
  bzero( m_pcCommand, CART_COMMAND_LENGTH );
  
  if( bDoConnect ) m_oClient.initialize();
  
  return true;
}

bool CartCommandMaker::terminate()
{
  return m_oClient.terminate();
}

bool CartCommandMaker::sendActivationCommand()
{
  sprintf( m_pcCommand, "%s", "#b1" );
  return m_oClient.sendCommand( m_pcCommand );
}

bool CartCommandMaker::sendTranslationalVelocityCommand( double fVelX, double fVelY )
{
  // int nVelX = (int)(fVelX * 1000.0); // mm/sec
  // int nVelY = (int)(fVelY * 1000.0); // mm/sec

  int nVelX = (int)(fVelX); // mm/sec
  int nVelY = (int)(fVelY); // mm/sec

  // ROS_INFO("fVelx : %f, fVely : %f",fVelX,fVelY);
  // ROS_INFO("nVelx : %d, nVely : %d",nVelX,nVelY);
  
  if( nVelX > MAX_TRANS_VELOCITY )
  {
    nVelX = MAX_TRANS_VELOCITY;
    ROS_WARN("X translational velocity was too high. It was set to %f.", MAX_TRANS_VELOCITY/1000.0);
  }
  else if( nVelX < MIN_TRANS_VELOCITY )
  {
    nVelX = MIN_TRANS_VELOCITY;
    ROS_WARN("X translational velocity was too low. It was set to %f.", MIN_TRANS_VELOCITY/1000.0);
  }
  
  if( nVelY > MAX_TRANS_VELOCITY )
  {
    nVelY = MAX_TRANS_VELOCITY;
    ROS_WARN("Y translational velocity was too high. It was set to %f.", MAX_TRANS_VELOCITY/1000.0);
  }
  else if( nVelY < MIN_TRANS_VELOCITY )
  {
    nVelY = MIN_TRANS_VELOCITY;
    ROS_WARN("Y translational velocity was too low. It was set to %f.", MIN_TRANS_VELOCITY/1000.0);
  }
  
  sprintf(m_pcCommand, "#bm%+04d%+04d000", nVelX, nVelY);
  
  return m_oClient.sendCommand( m_pcCommand );
}

bool CartCommandMaker::sendRotationalVelocityCommand( double fVelRot )
{
  // int nVelNorm = (int)abs(fVelRot*1000.0);

  int nVelNorm = (int)abs(fVelRot);

  // ROS_INFO("fVelRot : %f",fVelRot);
  // ROS_INFO("nVelRot : %d",nVelNorm);

  if( nVelNorm > MAX_ROT_VELOCITY )
  {
    nVelNorm = MAX_ROT_VELOCITY;
    ROS_WARN("Rotational velocity was too high. It was set to %f.", MAX_ROT_VELOCITY/1000.0);
  }
  else if( nVelNorm < MIN_ROT_VELOCITY )
  {
    nVelNorm = MIN_ROT_VELOCITY;
    ROS_WARN("Rotational velocity was too low. It was set to %f.", MIN_ROT_VELOCITY/1000.0);
  }
  
  bool bOk = true;
  
  // set the velocity norm
  sprintf(m_pcCommand, "#bs%03d", nVelNorm);
  bOk = m_oClient.sendCommand( m_pcCommand );
  
  // set the velocity direction
  if(fVelRot > 0)
  {
    sprintf(m_pcCommand, "#bl");
    bool tmpOk = m_oClient.sendCommand( m_pcCommand );
    bOk = bOk && tmpOk;
  }
  else if(fVelRot < 0)
  {
    sprintf(m_pcCommand, "#br");
    bool tmpOk = m_oClient.sendCommand( m_pcCommand );
    bOk = bOk && tmpOk;
  }
  
  return bOk;
}




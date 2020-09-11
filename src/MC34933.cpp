#include <Arduino.h>
#include "MC34933.h"

void MC34933::begin(void)
{
    pinMode(m_MA, OUTPUT);
    pinMode(m_MB, OUTPUT);
    //init pins Hi-Z
    digitalWrite(m_MA, 1);
    digitalWrite(m_MB, 1);  


}

void MC34933::setDirection(e_direction dir)
{
  m_prevDirection = m_direction;
  m_direction = dir;
}

void MC34933::start( void )
{
  m_startCmd = true;
}

bool MC34933::start( int speed )
{
  if(setSpeed(speed))
  {
    m_startCmd = true;
    return true;
  }
  return false;
}

bool MC34933::start( int speed, int duration_ms)
{
  if(setSpeed(speed))
  {
    m_startCmd = true;
    return true;
  }
  return false;
}

bool MC34933::setSpeed(int speed)
{
  if(speed <= 255)
  {
    m_speed = speed;
    return true;
  }
  return false;
}

void MC34933::stop ( void )
{
  m_stopCmd = true;
}

void MC34933::forward(int speed)
{
  setDirection(LEFT);
  if(setSpeed(speed))
  {
    start();
  }
  
}

void MC34933::backward(int speed)
{
  setDirection(RIGHT);
  if(setSpeed(speed))
  {
    start();
  }
}


//motor statemachine
void MC34933::run( void )
{
  switch(m_status)
  {
    case STARTING:
    {
      m_startCmd = false;     //clear start flag
      if(m_direction == LEFT)
      {
        analogWrite(m_MA, m_speed);
        digitalWrite(m_MB, 1); 
      }
      if(m_direction == RIGHT)
      {
        analogWrite(m_MB, m_speed);
        digitalWrite(m_MA, 1); 
      }
      m_status = RUN;
    }
    break;

    case RUN:
    {
      if(m_stopCmd )
      {
        m_status = STOPPING;
        
      }

      if(m_direction != m_prevDirection)
      {
        m_status = STOPPING;
        m_startCmd = true;
      }
    }
    break;

    case STOPPING:
    {
      m_stopCmd = false;      //clear stopflag
      //set both pins to break;
      analogWrite(m_MA, 0);
      analogWrite(m_MB, 0);
      digitalWrite(m_MA, 0);
      digitalWrite(m_MB, 0); 
      m_status = IDLE;
    }
    break;

    default:
    case IDLE:
    {
      if(m_startCmd)
      {
        m_status = STARTING;
      }
      //write pins Hi-z
      analogWrite(m_MA, 255);
      analogWrite(m_MB, 255);
      digitalWrite(m_MA, 1);
      digitalWrite(m_MB, 1); 
    }
    break;
  }
}
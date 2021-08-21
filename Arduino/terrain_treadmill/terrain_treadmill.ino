// ROS Utilities
#include <ros.h>               // ROS serial interface
#include <terrain_treadmill/MotorVelocity.h>

// Utilities from my library
#include <Encoder.h>
#include <Utilities.h> 
#include <Controller.h>
#include <Motors.h>

// Definitions
#define BAUD 57600
#define MOTOR1 1
#define MOTOR2 2
#define MOTOR3 3

// Parameters
#define ControllerFreq 200
#define Kp 400
#define Ki 2
#define Kd 400
#define SampleNumber 10

ros::NodeHandle nh;
terrain_treadmill::MotorVelocity vel_msg;

// Publisher publishes actual velocity information from motor encoders
ros::Publisher vel_pub("arduino/actual_velocity_counts", &vel_msg);

// Funcion called each time a new velocity command is recieved
void vel_cb( const terrain_treadmill::MotorVelocity& vel_in)
{
  // Set desired velocities for controller
  set_desired_velocity(vel_in.w1,MOTOR1);
  set_desired_velocity(vel_in.w2,MOTOR2);
  set_desired_velocity(vel_in.w3,MOTOR3);

  // Publish actual velocity information
  vel_msg.w1 = get_velocity(MOTOR1);
  vel_msg.w2 = get_velocity(MOTOR2);
  vel_msg.w3 = get_velocity(MOTOR3);
  vel_pub.publish( &vel_msg );
}

// Subscriber subcribes to velocity commands from Computer
ros::Subscriber<terrain_treadmill::MotorVelocity> vel_sub("/arduino/desired_velocity_counts", &vel_cb );

void setup()
{
  // Initialize hardware
  noInterrupts();
  encoder_setup(SampleNumber);      // Setup encoder pins/interrupts, set velocity moving average sample number
  set_gains(Kp,Ki,Kd);              // Set motor controller gains
  controller_setup(ControllerFreq); // Start fixed frequency controller interrupt
  motor_setup();                    // Setup motor control pins          
  interrupts();
  
  // Initialize ROS communication
  nh.getHardware()->setBaud(BAUD);
  nh.initNode();
  nh.subscribe(vel_sub);
  nh.advertise(vel_pub);
  
  // Start velocity control mode
  setMODE(VELOCITY_HOLD);
}

void loop()
{ 
  nh.spinOnce();  // Check for callbacks
  delayMicroseconds(1);
}



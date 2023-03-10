// File:          my_controller.cpp
// Date:          14-02-2023
// Description:   Controller file to perform the motion of a turtlebot burguer with Webots
// Author:        Jaime Godoy-Calvo
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Lidar.hpp>
//// #include <webots/GPS.hpp>
//// #include <webots/InertialUnit.hpp>

#include <iostream>
#include <limits>
#include <stdio.h>
#include <unistd.h>

void straight(webots::Motor* &motor_left, webots::Motor* &motor_right, char &u);
void turn_left(webots::Motor* &motor_left, webots::Motor* &motor_right, char &u);
void turn_right(webots::Motor* &motor_left, webots::Motor* &motor_right, char &u);
void stop(webots::Motor* &motor_left, webots::Motor* &motor_right, char &u);
// All the webots classes are defined in the "webots" namespace
using namespace webots;

int cont = 0;

void straight(webots::Motor* &motor_left, webots::Motor* &motor_right, char &u){
  motor_left->setVelocity(-5.0);
  motor_right->setVelocity(-5.0);
  cont++;
  if(cont>1000){
    if(rand()%2){
       u = 'r';
       cont = 0;
    }
    else{
       u = 'l';
       cont = 0;
    }
  }
}

void turn_left(webots::Motor* &motor_left, webots::Motor* &motor_right, char &u){
  motor_left->setVelocity(-2.5);
  motor_right->setVelocity(2.5);
  u = 'l';
  cont++;
}

void turn_right(webots::Motor* &motor_left, webots::Motor* &motor_right, char &u){
  motor_left->setVelocity(3.5);
  motor_right->setVelocity(-3.5);
  u = 'r';
  cont++;
}

void stop(webots::Motor* &motor_left, webots::Motor* &motor_right, char &u){
  motor_left->setVelocity(0.0);
  motor_right->setVelocity(0.0);
}

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();

  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();

  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  webots::Motor* motor_left = robot->getMotor("left wheel motor");
  webots::Motor* motor_right = robot->getMotor("right wheel motor");
  webots::Lidar* lidar =  robot->getLidar("LDS-01");
  //// DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //// webots::GPS* gps = robot->getGPS("gps");
  //// webots::InertialUnit* imu = robot->getInertialUnit("inertial unit");
  
  lidar->enable(timeStep);
  lidar->enablePointCloud();
  //// ds->enable(timeStep);
  //// gps->enable(timeStep);
  //// imu->enable(timeStep);

  motor_left->setVelocity(0.0);
  motor_right->setVelocity(0.0);
  
  motor_left->setPosition(std::numeric_limits<double>::infinity());
  motor_right->setPosition(std::numeric_limits<double>::infinity());
  char u = 's';
  
  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    const float *lidar_image = lidar->getRangeImage();
    
    // std::cout << lidar_image[315] << ", " << lidar_image[0]  << ", " << lidar_image[45];
    // printf("\n"); 
    //// double val = ds->getValue();
    //// const double * pos = gps->getValues();
    /* const double * imu_rads = imu->getRollPitchYaw();
       std::cout << "Hello World from c++! ["
                  << pos[0] << " "
                  << pos[1] << " "
                  << pos[2] << "] ["
                  << imu_rads[0]*180.0/3.14159 << " "
                  << imu_rads[1]*180.0/3.14159 << " "
                  << imu_rads[2]*180.0/3.14159 << "]" << std::endl;*/

    // Process sensor data here.

    // Enter here functions to send actuator commands, like:
    if(lidar_image[0]>0.5 && lidar_image[45]>0.3 && lidar_image[315]>0.3){
      if(cont>10){
        straight(motor_left, motor_right, u);
      }
      if(rand()%100>95) cont=0;
      cont++;
    }
    else{
      stop(motor_left, motor_right, u);
      if(u == 'l'){
        turn_left(motor_left, motor_right, u);
      }
      else{
        turn_right(motor_left, motor_right, u);
      }
    }
    // if((lidar_image[45]<0.5) && (lidar_image[45]<lidar_image[315])){
      // turn_left(motor_left, motor_right, u);
    // }
    // if((lidar_image[45]<0.5) && (lidar_image[45]>lidar_image[315])){
     // turn_right(motor_left, motor_right, u);
    // }
   
  };
  
  // Enter here exit cleanup code.
  std::cout << "Bye from c++!" << std::endl;

  delete robot;
  return 0;
}

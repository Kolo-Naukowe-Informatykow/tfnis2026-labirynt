#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>

#include <iostream>

using namespace webots;

int main(int argc, char **argv) {
  Robot *robot = new Robot();

  int timeStep = (int)robot->getBasicTimeStep();

  Motor *left_motor = robot->getMotor("left wheel motor");
  Motor *right_motor = robot->getMotor("right wheel motor");
  
  left_motor->setPosition(INFINITY);
  right_motor->setPosition(INFINITY);
  left_motor->setVelocity(0.0);
  right_motor->setVelocity(0.0);
  
  DistanceSensor *ds[6];
  const char* ds_names[6] = {
    "ll lidar", "ld lidar", "lf lidar",
    "rf lidar", "rd lidar", "rr lidar"
  };
  for (int i = 0; i < 6; i++) {
    ds[i] = robot->getDistanceSensor(ds_names[i]);
    ds[i]->enable(timeStep);
  }

  while (robot->step(timeStep) != -1) {
    double distance_values[6];
    for(int i=0; i<6;i++){
      distance_values[i] = ds[i]->getValue();
    }
    double diff = 0.0;
    if(distance_values[1]<0.1){
      diff = 1.0;
    }
    
    left_motor->setVelocity(1.0+diff);
    right_motor->setVelocity(1.0-diff);
  };

  delete robot;
  return 0;
}

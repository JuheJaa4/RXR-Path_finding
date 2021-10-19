/*
 * Copyright 1996-2021 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description:   A simple controller for the Thymio II robot
 */

#include <webots/accelerometer.h>
#include <webots/distance_sensor.h>
#include <webots/keyboard.h>
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/plugins/robot_window/default.h>
#include <webots/robot.h>
#include <webots/touch_sensor.h>

#include <math.h>
#include <stdio.h>

#define TIME_STEP 10
#define MAX_SPEED 9.0

static WbDeviceTag motor_left;
static WbDeviceTag motor_right;
static WbDeviceTag outerLeftSensor;
static WbDeviceTag centralLeftSensor;
static WbDeviceTag centralSensor;
static WbDeviceTag centralRightSensor;
static WbDeviceTag outerRightSensor;
static WbDeviceTag acc;
double static maxMotorVelocity;
double static initialVelocity;
int static distanceSensorCalibrationConstant;

static void init_devices() {

  motor_left = wb_robot_get_device("motor.left");
  motor_right = wb_robot_get_device("motor.right");
  wb_motor_set_position(motor_left, INFINITY);
  wb_motor_set_position(motor_right, INFINITY);

  acc = wb_robot_get_device("acc");
  wb_accelerometer_enable(acc, TIME_STEP);

  //Get frontal distance sensors.
  outerLeftSensor = wb_robot_get_device("prox.horizontal.0");
  centralLeftSensor = wb_robot_get_device("prox.horizontal.1");
  centralSensor = wb_robot_get_device("prox.horizontal.2");
  centralRightSensor = wb_robot_get_device("prox.horizontal.3");
  outerRightSensor = wb_robot_get_device("prox.horizontal.4");
  
  //Enable distance sensors.
  wb_distance_sensor_enable(outerLeftSensor, TIME_STEP);
  wb_distance_sensor_enable(centralLeftSensor, TIME_STEP);
  wb_distance_sensor_enable(centralSensor, TIME_STEP);
  wb_distance_sensor_enable(centralRightSensor, TIME_STEP);
  wb_distance_sensor_enable(outerRightSensor, TIME_STEP);

  //Constants of the Thymio II motors and distance sensors.
  maxMotorVelocity = 9.53;
  distanceSensorCalibrationConstant = 360;
 
  //Set ideal motor velocity.
  initialVelocity = 0.7 * maxMotorVelocity;

  //Set the initial velocity of the left and right wheel motors.
  wb_motor_set_velocity(motor_left, initialVelocity);
  wb_motor_set_velocity(motor_right, initialVelocity);
}

int main(int argc, char **argv) {
  wb_robot_init();
  init_devices();

  while (wb_robot_step(TIME_STEP) != -1) {
    //Read values from four distance sensors and calibrate.
    double outerLeftSensorValue = wb_distance_sensor_get_value(outerLeftSensor) / distanceSensorCalibrationConstant;
    double centralLeftSensorValue = wb_distance_sensor_get_value(centralLeftSensor) / distanceSensorCalibrationConstant;
    double centralSensorValue = wb_distance_sensor_get_value(centralSensor) / distanceSensorCalibrationConstant;
    double centralRightSensorValue = wb_distance_sensor_get_value(centralRightSensor) / distanceSensorCalibrationConstant;
    double outerRightSensorValue = wb_distance_sensor_get_value(outerRightSensor) / distanceSensorCalibrationConstant;

    //Set wheel velocities based on sensor values, prefer right turns if the central sensor is triggered.
    wb_motor_set_velocity(motor_left, (initialVelocity - (centralRightSensorValue + outerRightSensorValue) / 2));
    wb_motor_set_velocity(motor_right, (initialVelocity - (centralLeftSensorValue + outerLeftSensorValue) / 2 - centralSensorValue));
  }

  wb_robot_cleanup();
  return 0;
}

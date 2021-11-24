#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/compass.h>
#include <webots/gps.h>
#include <time.h>

// maximal speed allowed
#define MAX_SPEED 5.24

// how many sensors are on the robot
#define MAX_SENSOR_NUMBER 16

// maximal value returned by the sensors
#define MAX_SENSOR_VALUE 1024

// minimal distance, in meters, for an obstacle to be considered
#define MIN_DISTANCE 0.5

// minimal weight for the robot to turn
#define WHEEL_WEIGHT_THRESHOLD 100

// clock to measure passed time
static clock_t start;
static bool grid[7][5];
static bool walls[4] = {false, false, false, false};

// structure to store the data associated to one sensor
typedef struct {
  WbDeviceTag device_tag;
  double wheel_weight[2];
} SensorData;

// enum to represent the state of the robot
typedef enum { FORWARD, LEFT, RIGHT } State;

double getBearing(WbDeviceTag tag) {
  const double *north = wb_compass_get_values(tag);
  double rad = atan2(north[0], north[2]);
  double dir = (rad - 1.5708) / M_PI * 180.0;
  //set direction back to positive 
  if (dir < 0.0) {
    dir += 360.0;
  }
  return dir;
}

bool* formatWalls(WbDeviceTag tag) {
  double dir = getBearing(tag);
  static bool tmp[4];
  if (dir > 335 || dir < 45) {
    for (int i = 0; i < 4; i++) {
      tmp[i] = walls[i];
    }
  } else if (dir < 135 && dir > 45) {
    for (int i = 0; i < 4; i++) {
      if ((i - 1) > 0) {
        tmp[i - 1 + 4] = walls[i];
      } else {
        tmp[i - 1] = walls[i];
      }
    }
  } else if (dir < 225 && dir > 135) {
    for (int i = 0; i < 4; i++) {
      if ((i - 2) > 0) {
        tmp[i - 2 + 4] = walls[i];
      } else {
        tmp[i - 2] = walls[i];
      }
    }
  } else if (dir > 135 && dir < 335) {
    for (int i = 0; i < 4; i++) {
      if ((i - 3) > 0) {
        tmp[i - 3 + 4] = walls[i];
      } else {
        tmp[i - 3] = walls[i];
      }
    }
  }
  return tmp;
}

int* getGrid(WbDeviceTag tag) {
  const double *gpsVector = wb_gps_get_values(tag);
  double y = gpsVector[2] - 2.0;
  double x = gpsVector[0] + 2.0;
  int gridY = (y - 0.50 + 5);
  int gridX = (x - 0.50);
  //printf("x: %f y: %f\n", x, y);
  //printf("x: %d y: %d\n", gridX, gridY);
  static int grid1[2];
  grid1[0] = gridX;
  grid1[1] = gridY;
  //Update what grids have been visited
  grid[gridX][gridY] = true;
  printf("x: %d y: %d visited: %d\n", gridX, gridY, grid[gridX][gridY]);

  return grid1;
}

State getNextDirection(WbDeviceTag gps, WbDeviceTag cmps) {
  int nextGrids[4][2];
  int *tmp = getGrid(gps);
  bool* wallsFormatted = formatWalls(cmps);
  //Upwards next grid
  if ((tmp[1] - 1) < 0) {
    nextGrids[0][1] = 0;
  } else {
    nextGrids[0][1] = tmp[1] - 1;
  }
  nextGrids[0][0] = tmp[0];
  //Rightwards next grid
  nextGrids[1][1] = tmp[1];
  if ((tmp[0] + 1) > 6) {
    nextGrids[1][0] = 6;
  } else {
    nextGrids[1][0] = tmp[0] + 1;
  }
  //Downwards next grid
  if ((tmp[1] + 1) > 6) {
    nextGrids[2][1] = 6;
  } else {
    nextGrids[2][1] = tmp[1] + 1;
  }
  
  nextGrids[2][0] = tmp[0];
  //Leftwards next grid
  nextGrids[3][1] = tmp[1];
  if ((tmp[0] - 1) < 0) {
    nextGrids[3][0] = 0;
  } else {
    nextGrids[3][0] = tmp[0] - 1;
  }


  if (wallsFormatted[0] == false) {
    if (grid[nextGrids[0][0]][nextGrids[0][1]] == false) {
      printf("Forward\n");
      return FORWARD;
    }
  } else if (wallsFormatted[1] == false) {
    if (grid[nextGrids[1][0]][nextGrids[1][1]] == false) {
      printf("Right\n");
      return RIGHT;
    }
  } else if (wallsFormatted[2] == false) {
    if (grid[nextGrids[2][0]][nextGrids[2][1]] == false) {
      printf("Down\n");
      //return DOWNWARD;
      return FORWARD;
    }
  } else if (wallsFormatted[3] == false) {
    if (grid[nextGrids[3][0]][nextGrids[3][1]] == false) {
      printf("Left\n");
      return LEFT;
    }
  }
  printf("default\n");
  return FORWARD;
}

int getPassedTime() {
  return (clock() - start) * 1000 / CLOCKS_PER_SEC;
}

void startTimer() {
  start = clock();
}

// how much each sensor affects the direction of the robot
static SensorData sensors[MAX_SENSOR_NUMBER] = {
  {.wheel_weight = {150, 0}}, {.wheel_weight = {200, 0}}, {.wheel_weight = {300, 0}}, {.wheel_weight = {600, 0}},
  {.wheel_weight = {0, 600}}, {.wheel_weight = {0, 300}}, {.wheel_weight = {0, 200}}, {.wheel_weight = {0, 150}},
  {.wheel_weight = {0, 0}},   {.wheel_weight = {0, 0}},   {.wheel_weight = {0, 0}},   {.wheel_weight = {0, 0}},
  {.wheel_weight = {0, 0}},   {.wheel_weight = {0, 0}},   {.wheel_weight = {0, 0}},   {.wheel_weight = {0, 0}}};

int main() {
  // necessary to initialize Webots
  wb_robot_init();

  // stores simulation time step
  int time_step = wb_robot_get_basic_time_step();

  // stores device IDs for the wheels
  WbDeviceTag left_wheel = wb_robot_get_device("left wheel");
  WbDeviceTag right_wheel = wb_robot_get_device("right wheel");
  WbDeviceTag compass = wb_robot_get_device("compass");
  WbDeviceTag gps = wb_robot_get_device("gps");
  // Sensors
  //Mapping grid when arena size 7mx5m
  for (int x = 0; x < 5; x++) {
    for (int y = 0; y < 7; y++) {
      grid[x][y] = false;
    }
  }
  //Enable devices
  wb_compass_enable(compass, time_step);
  wb_gps_enable(gps, time_step);

  char sensor_name[5] = "";
  int i;

  // sets up sensors and stores some info about them
  for (i = 0; i < MAX_SENSOR_NUMBER; ++i) {
    sprintf(sensor_name, "so%d", i);
    sensors[i].device_tag = wb_robot_get_device(sensor_name);
    wb_distance_sensor_enable(sensors[i].device_tag, time_step);
  }

  // sets up wheels
  wb_motor_set_position(left_wheel, INFINITY);
  wb_motor_set_position(right_wheel, INFINITY);
  wb_motor_set_velocity(left_wheel, 0.0);
  wb_motor_set_velocity(right_wheel, 0.0);

  double speed[2] = {0.0, 0.0};
  double wheel_weight_total[2] = {0.0, 0.0};
  double distance, speed_modifier, sensor_value;
  int j;
  // by default, the robot goes forward
  State state = FORWARD;

  // run simulation
  while (wb_robot_step(time_step) != -1) {
    // initialize speed and wheel_weight_total arrays at the beginning of the loop
    double bearing = getBearing(compass);
    printf("Bearing: %f\n", bearing);
    
    memset(speed, 0, sizeof(double) * 2);
    memset(wheel_weight_total, 0, sizeof(double) * 2);



    for (i = 0; i < MAX_SENSOR_NUMBER; ++i) {
      sensor_value = wb_distance_sensor_get_value(sensors[i].device_tag);
      //distance = 5.0 * (1.0 - (sensor_value / MAX_SENSOR_VALUE));
      //printf("Distance of sensor%d: %f, sensorVal: %f\n", i, distance, sensor_value);
      // if the sensor doesn't see anything, we don't use it for this round
      if (sensor_value == 0.0) {
        speed_modifier = 0.0;

        // Update walls
        switch (i) {
          case 4:
            walls[0] = false;
            break;
          case 7:
            walls[1] = false;
            break;
          case 12:
            walls[2] = false;
            break;
          case 0:
            walls[3] = false;
            break;
          default:
            break;
        }
      } else {
        // computes the actual distance to the obstacle, given the value returned by the sensor
        distance = 5.0 * (1.0 - (sensor_value / MAX_SENSOR_VALUE));  // lookup table inverse.
        
        // if the obstacle is close enough, we may want to turn
        // here we compute how much this sensor will influence the direction of the robot
        if (distance < MIN_DISTANCE) {
          speed_modifier = 1 - (distance / MIN_DISTANCE);
          // Update walls
          switch (i) {
            case 4:
              walls[0] = true;
              break;
            case 7:
              walls[1] = true;
              break;
            case 12:
              walls[2] = true;
              break;
            case 0:
              walls[3] = true;
              break;
            default:
              break;
          }
        }
        else
          speed_modifier = 0.0;
      }

      // add the modifier for both wheels
      for (j = 0; j < 2; ++j)
        wheel_weight_total[j] += sensors[i].wheel_weight[j] * speed_modifier;
    }
    State state2 = getNextDirection(gps, compass);
    //Print wall situation
    printf("Forward wall: %d\nRight wall: %d\nBackward wall: %d\nLeft wall %d\n", walls[0], walls[1], walls[2], walls[3]);
    
    // (very) simplistic state machine to handle the direction of the robot
    switch (state) {
      // when the robot is going forward, it will start turning in either direction when an obstacle is close enough
      case FORWARD:
        if (wheel_weight_total[0] > WHEEL_WEIGHT_THRESHOLD) {
          speed[0] = 0.7 * MAX_SPEED;
          speed[1] = -0.7 * MAX_SPEED;
          state = LEFT;
        } else if (wheel_weight_total[1] > WHEEL_WEIGHT_THRESHOLD) {
          speed[0] = -0.7 * MAX_SPEED;
          speed[1] = 0.7 * MAX_SPEED;
          state = RIGHT;
        } else {
          speed[0] = MAX_SPEED;
          speed[1] = MAX_SPEED;
        }
        break;
      // when the robot has started turning, it will go on in the same direction until no more obstacle are in sight
      // this will prevent the robot from being caught in a loop going left, then right, then left, and so on.
      case LEFT:
        if (wheel_weight_total[0] > WHEEL_WEIGHT_THRESHOLD || wheel_weight_total[1] > WHEEL_WEIGHT_THRESHOLD) {
          speed[0] = 0.7 * MAX_SPEED;
          speed[1] = -0.7 * MAX_SPEED;
        } else {
          speed[0] = MAX_SPEED;
          speed[1] = MAX_SPEED;
          state = FORWARD;
        }
        break;
      case RIGHT:
        if (wheel_weight_total[0] > WHEEL_WEIGHT_THRESHOLD || wheel_weight_total[1] > WHEEL_WEIGHT_THRESHOLD) {
          speed[0] = -0.7 * MAX_SPEED;
          speed[1] = 0.7 * MAX_SPEED;
        } else {
          speed[0] = MAX_SPEED;
          speed[1] = MAX_SPEED;
          state = FORWARD;
        }
        break;
    }
    
    // sets the motor speeds
    wb_motor_set_velocity(left_wheel, speed[0]);
    wb_motor_set_velocity(right_wheel, speed[1]);
  }

  wb_robot_cleanup();

  return 0;
}

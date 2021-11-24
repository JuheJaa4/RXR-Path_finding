#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/compass.h>
#include <webots/gps.h>

// maximal speed allowed
#define MAX_SPEED 5.24
// how many sensors are on the robot
#define MAX_SENSOR_NUMBER 16
// maximal value returned by the sensors
#define MAX_SENSOR_VALUE 1024
// minimal distance, in meters, for an obstacle to be considered
// 0.5 toimivin tähän mennessä
#define MIN_DISTANCE 0.5
// minimal weight for the robot to turn
#define WHEEL_WEIGHT_THRESHOLD 100

//Julkisia muuttujia
static bool grid[7][5];
static bool walls[4] = {false, false, false, false};
static bool tmp[4];
static double bearing;
static bool sensorsEnabled;
static bool isStopped;

// structure to store the data associated to one sensor
typedef struct {
  WbDeviceTag device_tag;
  double wheel_weight[2];
} SensorData;

// enum to represent the state of the robot
typedef enum { FORWARD, LEFT, RIGHT, DOWNWARD, UPWARD, STOP, NONE, FINISH } State;

double getBearing(WbDeviceTag tag) {
  const double *north = wb_compass_get_values(tag);
  double rad = atan2(north[0], north[2]);
  double dir = (rad - 1.5708) / M_PI * 180.0;
  //Pitää arvon 0 - 360 välillä
  if (dir < 0.0) {
    dir += 360.0;
  }
  return dir;
}

bool* formatWalls() {
  //Käännetään taulukkoa vain silloin kun sensorit ovat päällä, koska kesken käännöksen
  //ja ilman sensoridataa taulukko kääntyy liian useasti/väärään suuntaan tms.
  if (sensorsEnabled) {
    if (bearing > 355 || bearing < 5) {
      //printf("Ei käännetä taulukkoa, koska ylöspäin suuntautuessa ei tarvitse muuttaa taulukkoa\n");
      for (int i = 0; i < 4; i++) {
        tmp[i] = walls[i];
      }
    } else if (bearing > 265 && bearing < 275) {
      //printf("Käännetään taulukko kun robotti on suuntautunut oikealle\n");
      for (int i = 0; i < 4; i++) {
        if ((i - 1) < 0) {
          tmp[i - 1 + 4] = walls[i];
        } else {
          tmp[i - 1] = walls[i];
        }
      }
    } else if (bearing < 185 && bearing > 175) {
      //printf("Käännetään taulukko kun robotti on suuntautunut alaspäin\n");
      for (int i = 0; i < 4; i++) {
        if ((i - 2) < 0) {
          tmp[i - 2 + 4] = walls[i];
        } else {
          tmp[i - 2] = walls[i];
        }
      }
    } else if (bearing < 95 && bearing > 85) {
      //printf("Käännetään taulukko kun robotti on suuntautunut vasemmalle\n");
      for (int i = 0; i < 4; i++) {
        if ((i - 3) < 0) {
          tmp[i - 3 + 4] = walls[i];
        } else {
          tmp[i - 3] = walls[i];
        }
      }
    }
  }
  return tmp;
}

double* getCoordinates(WbDeviceTag tag) {
  const double *gpsVector2 = wb_gps_get_values(tag);
  static double xy[2];
  xy[1] = gpsVector2[2] - 2.0;
  xy[0] = gpsVector2[0] + 2.0;
  //printf("getCoordinates() x: %f y: %f\n", xy[0], xy[1]);
  return xy;
}

int* getGrid(WbDeviceTag tag) {
  const double *gpsVector = getCoordinates(tag);
  double x = gpsVector[0];
  double y = gpsVector[1];
  int gridY = (y - 0.50 + 5);
  int gridX = (x - 0.50);
  //printf("x: %f y: %f\n", x, y);
  static int grid1[2];
  grid1[0] = gridX;
  grid1[1] = gridY;
  //Päivitä jo käydyt gridit taulukkoon
  grid[gridX][gridY] = true;
  printf("Current coordinates x: %d y: %d visited: %d\n", gridX, gridY, grid[gridX][gridY]);

  //Palauttaa tämän hetkisen gridin muodossa int x, y
  return grid1;
}

bool stop(WbDeviceTag gps) {
  const double *gpsVector = getCoordinates(gps);
  double x = gpsVector[0];
  double y = gpsVector[1] + 4;
  
  int *grid = getGrid(gps);
  //printf("stop() x: %f y: %f\n", x, y);
  int gridX = grid[0];
  int gridY = grid[1];

  double deltaX = fabs(x - gridX);
  double deltaY = fabs(y - gridY);
  //printf("Current grid x: %d y: %d, current x: %f y: %f, delta x: %f y: %f\n", gridX, gridY, x, y, deltaX, deltaY);
  if (deltaY < 0.02) {
    return true;
  } else if (deltaX >= 0.98 && deltaX <= 1.02) {
    return true;
  }

  return false;
}

State getNextDirection(WbDeviceTag gps, WbDeviceTag cmps) {
  int nextGrids[4][2];
  //Palauttaa tämän hetkisen sijainnin grid taulukossa x, y arvoina
  int *tmp = getGrid(gps);
  //Formatoi seinien sijainnit robotin kääntöasteen mukaan "meille loogiseen järjestykseen"
  bool* wallsFormatted = formatWalls();
  //Tarkistetaan tuleeko robotin pysähtyä
  bool tmpState = stop(gps);
  if (tmpState == true) {
    isStopped = true;
  } else {
    isStopped = false;
  }
  printf("isStopped: %d\n", isStopped);

  if (isStopped) {
    //Tarkistetaan onko robotti maalissa
    if (tmp[0] == 5 && tmp[1] == 0) {
      return FINISH;
    }
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

    //Valitaan suunta, jonne mennä
    //Sen mukaan missä ei ole seinää ja missä suunnassa ei ole VIELÄ käyty
    //Ja palautetaan komento suunnasta
    printf("Forward wall: %d\nRight wall: %d\nBackward wall: %d\nLeft wall %d\n", wallsFormatted[0], wallsFormatted[1], wallsFormatted[2], wallsFormatted[3]);
    if (wallsFormatted[0] == false) {
      if (grid[nextGrids[0][0]][nextGrids[0][1]] == false) {
        printf("Up\n");
        return UPWARD;
      }
    }
    if (wallsFormatted[1] == false) {
      //printf("no wall on right\n");
      if (grid[nextGrids[1][0]][nextGrids[1][1]] == false) {
        //printf("Seuraava vapaa grid oikealla on x: %d , y: %d\n", nextGrids[1][0], nextGrids[1][1]);
        printf("Right\n");
        return RIGHT;
      }
    }
    if (wallsFormatted[2] == false) {
      if (grid[nextGrids[2][0]][nextGrids[2][1]] == false) {
        printf("Down\n");
        return DOWNWARD;
      }
    }
    if (wallsFormatted[3] == false) {
      if (grid[nextGrids[3][0]][nextGrids[3][1]] == false) {
        printf("Left\n");
        return LEFT;
      }
    }

    //Mikäli ei löytynyt seinätöntä ja VIELÄ käymätöntä aluetta täytyy mennä takaisin päin.
    //Talletetaan taulukkoon kaikki seinättömät vaihtoehdot
    printf("Ei löytynyt suuntaa jossa ei olisi käyty. Valitaan vapaista jokin suunta.\n");
    int vapaat[4];

    for (int c = 0; c < 4; c++) {
      vapaat[c] = -1;
    }

    for (int i = 0; i < 4; i++) {
      if (wallsFormatted[i] == false) {
        for (int a = 0; a < 4; a++) {
          if (vapaat[a] == -1) {
            vapaat[a] = i;
            break;
          }
        }
      }
    }

    //Otetaan ensimmäinen vapaa vaihtoehto ja lähdetään sinne(Tää ei kyl toimi niin ku pitäis)
    //Todo joku logiikka minkä valitsee
    for (int i = 3; i > -1; i--) {
      //printf("vapaat[i]: %d\n", vapaat[i]);
      if (vapaat[i] != -1) {
        switch (vapaat[i]) {
          case 0:
            printf("Vapaista valittu up\n");
            return UPWARD;
          case 1:
            printf("Vapaista valittu right\n");
            return RIGHT;
          case 2:
            printf("Vapaista valittu down\n");
            return DOWNWARD;
          case 3:
            printf("Vapaista valittu left\n");
            return LEFT;
        }
      }
    }

    //printf("Tänne ei pitäisi ikinä päästä\n");
    return DOWNWARD;
  }

  
  return FORWARD;
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
  sensorsEnabled = true;
  isStopped = false;

  // sets up wheels
  wb_motor_set_position(left_wheel, INFINITY);
  wb_motor_set_position(right_wheel, INFINITY);
  wb_motor_set_velocity(left_wheel, 0.0);
  wb_motor_set_velocity(right_wheel, 0.0);

  double speed[2] = {0.0, 0.0};
  double wheel_weight_total[2] = {0.0, 0.0};
  double distance, speed_modifier, sensor_value;
  int j;

  // run simulation
  while (wb_robot_step(time_step) != -1) {
    bearing = getBearing(compass);
    printf("Bearing: %f\n", bearing);

    // initialize speed and wheel_weight_total arrays at the beginning of the loop
    memset(speed, 0, sizeof(double) * 2);
    memset(wheel_weight_total, 0, sizeof(double) * 2);

    if (sensorsEnabled) {
      for (i = 0; i < MAX_SENSOR_NUMBER; ++i) {
        sensor_value = wb_distance_sensor_get_value(sensors[i].device_tag);
        distance = 5.0 * (1.0 - (sensor_value / MAX_SENSOR_VALUE));
        if (i == 4 || i == 7 || i == 12 || i == 0) {
          //printf("Distance of sensor%d: %f, sensorVal: %f\n", i, distance, sensor_value);
        }
        // if the sensor doesn't see anything, we don't use it for this round
        if (sensor_value < 900 || distance > 1) {
          speed_modifier = 0.0;

          //Päivitä seinien sijainti jos sensori ei näe seinää
          /* 4 = etummainen sensori
          *  7 = oikean puoleinen
          *  12 = taka
          *  0 = vasen
          */
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

            //Päivitä seinien sijainti jos seinä tunnistetaan
            /* 4 = etummainen sensori
            *  7 = oikean puoleinen
            *  12 = taka
            *  0 = vasen
            */
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
    }


    State state2 = getNextDirection(gps, compass);
    printf("State: %d\n", state2);

    //Käännytään tai ajetaan suoraan ohjeiden mukaisesti
    switch (state2) {
      case STOP:
        printf("STOP\n");
        isStopped = true;
        break;
      case FORWARD:
        //todo
        speed[0] = MAX_SPEED;
        speed[1] = MAX_SPEED;
        break;
      case LEFT:
        if (!(bearing < 272 && bearing > 268)) {
          printf("Käännytään\n");
          sensorsEnabled = false;
          speed[0] = -0.2 * MAX_SPEED;
          speed[1] = 0.2 * MAX_SPEED;
        } else {
          printf("Jatketaan suoraan\n");
          sensorsEnabled = true;
          speed[0] = MAX_SPEED;
          speed[1] = MAX_SPEED;
        }
        break;
      case RIGHT:
        if ((bearing > 92 || bearing < 88)) {
          printf("Käännytään\n");
          sensorsEnabled = false;
          speed[0] = -0.2 * MAX_SPEED;
          speed[1] = 0.2 * MAX_SPEED;
        } else {
          printf("Jatketaan suoraan\n");
          sensorsEnabled = true;
          speed[0] = MAX_SPEED;
          speed[1] = MAX_SPEED;
        }
        break;
      case DOWNWARD:
        if ((bearing > 182 || bearing < 178)) {
          printf("Käännytään\n");
          sensorsEnabled = false;
          speed[0] = -0.2 * MAX_SPEED;
          speed[1] = 0.2 * MAX_SPEED;
        } else {
          printf("Jatketaan suoraan\n");
          sensorsEnabled = true;
          speed[0] = MAX_SPEED;
          speed[1] = MAX_SPEED;
        }
        break;
      case UPWARD:
        if (!(bearing > 356 || bearing < 4)) {
          printf("Käännytään\n");
          sensorsEnabled = false;
          speed[0] = -0.2 * MAX_SPEED;
          speed[1] = 0.2 * MAX_SPEED;
        } else {
          printf("Jatketaan suoraan\n");
          sensorsEnabled = true;
          speed[0] = MAX_SPEED;
          speed[1] = MAX_SPEED;
        }
        break;
      case FINISH:
        speed[0] = 0.0;
        speed[1] = 0.0;
      default:
        printf("ei tehä mitään\n");
        break;
    }
    
    // sets the motor speeds
    wb_motor_set_velocity(left_wheel, speed[0]);
    wb_motor_set_velocity(right_wheel, speed[1]);
  }

  wb_robot_cleanup();

  return 0;
}

#ifndef EWENSBOATLIB_H
#define EWENSBOATLIB_H

#include <Arduino.h>
#include <Wire.h>
#include "controlMotor.h"
#include "IMU.h"
#include "config.h"
#include "windSensor.h"
#include "GPS.h"
#include <SD.h>
#include <SPI.h>
#include "controler.h"

#define PIN_SPI_CS 53

double scalprod(Cartcoord A, Cartcoord B);

class nav {
public:
  nav();  // Déclaration correcte du constructeur
  ~nav();
  void linefollowing(float lata, float longa, float latb, float longb, bool integral = false);
  void update_logs();
  void update();
  void path_following(GPScoord list_points[], int nb_points, bool integral = false);
  void non_blocking_path_following(GPScoord list_points[], int nb_points, bool integral = false);
  void run_mission();
private:
    IMU* imu;
    controlMotor* powerboard;
    WindSensor* wind;
    GPS* gps;
    Controler* controler;
    char filename[32];
    void init_sequence_rud();
    float get_true_wind_dir();
    // % q --- the tacking variable for the linefollowing;
    float q = 1;
    float z = 0;
    float dt = 0.1;
    float alpha = 0.02;
    int scenario = -1;
};

int getMaxLogIndex();

#endif

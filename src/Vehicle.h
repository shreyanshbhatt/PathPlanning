#ifndef __Vehicle_h_
#define __Vehicle_h_
#include "math.h"
#include <string.h>
#include <iostream>
#include <assert.h>
#include <vector>
#include "Cost.h"
#include <map>

class VehicleState {
 public:
  VehicleState();
  VehicleState(double s, float d, double vel) {
    s_ = s;
    d_ = d;
    velocity_ = vel;
  }
 public:
  double s_;
  float d_;
  double velocity_;
};

// Class for the current car.
class Vehicle {
  VehicleState vs_;
  VehicleState vs_temp_;
  std::vector<std::vector<double>> sensor_data;
  std::tuple<double, double, double, double> get_front_car_(float l);
  float get_possible_left_lane(float l);
  float get_possible_right_lane(float l);
  double get_vel_reducer_(double buffer, double fval);
  double getPLCTrajectory_(float il, float il2);
  double getLCTrajectory_(float il1, float il2);
 public:
  // Possible states. Note that KLBuf and KLBuf2 are not really required
  // It is to make sure that once we change lange we don't start chaging
  // lanes immediately.
  enum states:int  {KL, PLCL, LCL, PLCR, LCR, KLBuf, KLBuf2};

  Vehicle(VehicleState vs) {
    vs_ = vs;
  }

  std::vector<states> get_next_states();
  // Use CostCalculator within each
  double getKLTrajectory();
  double getPLCLTrajectory();
  double getPLCRTrajectory();
  double getLCLTrajectory();
  double getLCRTrajectory();

  VehicleState getBestTrajectory(std::vector<std::vector<double>> sensor_data, int prev_size, double s);
 private:
  states cur_state_ = KL;
  states temp_state_ = KL;
  int prev_size_ = 0;
};

#endif

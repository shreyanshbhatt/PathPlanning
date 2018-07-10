#ifndef __Cost_h_
#define __Cost_h_
#include<vector>
#include<iostream>
class CostCalculator {
  float d_;
  double buffer_back_;
  double buffer_front_;
  double velocity_;
  float cur_lane_;

  double collision_cost_ = 1.0;
  double buffer_cost_ = 0.5;
  double b_buffer_cost_ = 0.8;
  double vel_cost_ = 0.3;
  double lane_change_cost_ = 0.2;
  double val_c_req_cost_ = 0.5;
 public:
  CostCalculator();
  CostCalculator(double d, double bback, double bfront, double vel, float cur_lane) {
    d_ = d;
    buffer_back_ = bback;
    buffer_front_ = bfront;
    velocity_ = vel;
    cur_lane_ = cur_lane;
  }
  double getCost(std::vector<std::vector<double>> sensor_data, double valcreq);
};
#endif

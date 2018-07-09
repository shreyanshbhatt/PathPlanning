#include "Cost.h"

double CostCalculator::getCost(std::vector<std::vector<double>> sensor_data, double valcreq) {
  int has_collision = 0;
  if (buffer_front_ <= 10.0 || buffer_back_ <= 10.0) {
    // std::cout << " found collision " << d_ << std::endl;
    has_collision = 100;
  }
  double cost = 0.0;
  if (cur_lane_ != d_) {
    cost += 4.0;
  }
  double buffer_val = std::min(buffer_front_, 100.0);
  double b_buffer_val = std::min(buffer_back_, 30.0);
  cost += (buffer_cost_)*(100.0 - buffer_val)
      + (b_buffer_cost_)*(30.0 - b_buffer_val)
      + has_collision*collision_cost_
      + (49.5 - velocity_)*vel_cost_
      + valcreq * val_c_req_cost_ * 200;
  return cost;
}

#include "Vehicle.h"

std::vector<Vehicle::states> Vehicle::get_next_states() {
  switch(cur_state_) {
    case KL:
      return {KL, PLCL, PLCR};
    case PLCL:
      return {PLCL, LCL, KL};
    case PLCR:
      return {PLCR, LCR, KL};
    case LCL:
      return {KLBuf};
    case LCR:
      return {KLBuf};
    case KLBuf:
      return {KLBuf2};
    case KLBuf2:
      return {KL};
    default:
      assert(1 == 2);
  }
}

//TODO: Not handled what happens when car behind you
// is above the speed limit and honking to get ahead
double Vehicle::getKLTrajectory() {
  // Get the next vehicle
  // Get the vehicle behind you
  // Decrease the velocity by a constant factor
  // Make sure you don't decrese less than 0
  // if you find the next vehicle within some dist
  // Make sure the velocity of vehicle behind you
  // Update buffer in s
  // return
  float intended_lane = vs_.d_;
  const auto &f_v = get_front_car_(intended_lane);
  double buffer_f = std::get<0>(f_v);
  double fvel = std::get<1>(f_v);
  double buffer_b = std::get<2>(f_v);
  double val_change_req = 0.0;
  if (buffer_f < 30.0)
    val_change_req = get_vel_reducer_(buffer_f, fvel);

  double intended_velocity = (fvel > 49.5)?fvel:49.5;
  //std::cout << f_v.velocity_ << std::endl;
  CostCalculator cc(intended_lane, buffer_b, buffer_f, intended_velocity, vs_.d_);
  double cost = cc.getCost(sensor_data, val_change_req);
  vs_temp_.s_ = vs_.s_;
  vs_temp_.d_ = intended_lane;
  vs_temp_.velocity_ = vs_.velocity_;
  if (buffer_f < 30.0 && buffer_b > 10.0) {
    if (vs_temp_.velocity_ > fvel) {
      vs_temp_.velocity_ -= val_change_req;
    }
  } else if (buffer_f >= 40.0 && vs_temp_.velocity_ < 49.5) {
    float val_to_consider = std::min(50.0, buffer_f);
    vs_temp_.velocity_ += (val_to_consider/20.0)*0.045;
  } else if (vs_temp_.velocity_ < fvel && vs_temp_.velocity_ < 49.5) {
    vs_temp_.velocity_ += (buffer_f/20.0)*0.045;
  }
  temp_state_ = KL;
  // std::cout << cost << std::endl;
  return cost;
}

float Vehicle::get_possible_left_lane(float l) {
  if (l == 1.0) {
    return float(0.0);
  }
  if (l == 2.0) {
    return float(1.0);
  }
  return -1.0f;
}

float Vehicle::get_possible_right_lane(float l) {
  if (l == 1.0) {
    return float(2.0);
  }
  if (l == 0.0) {
    return float(1.0);
  }
  return -1.0f;
}

double Vehicle::getPLCTrajectory_(float intended_lane, float intended_lane_2) {
  if (intended_lane == -1.0f)
    return 3000.0;
  const auto &f_v = get_front_car_(intended_lane);
  const auto &cf_v = get_front_car_(vs_.d_);
  double f_buff = std::get<0>(f_v);
  double cf_buff = std::get<0>(cf_v);
  double fval = std::get<1>(f_v);
  double fval_this = std::get<1>(f_v);
  double cfval = std::get<1>(cf_v);
  double val_change_req = 0.0;
  double min_cf_buff = cf_buff;
  double min_f_val = std::get<1>(cf_v);
  if (cf_buff < f_buff) {
    min_cf_buff = f_buff;
    min_f_val = fval_this;
  }
  double min_cb_buff = std::min(std::get<2>(cf_v), std::get<2>(f_v));
  if (min_cf_buff < 30.0)
    val_change_req = get_vel_reducer_(min_cf_buff, fval_this);

  if (intended_lane_2 != -1.0f) {
    const auto &rmf_v = get_front_car_(intended_lane_2);
    double left_most_val = std::get<1>(rmf_v);
    double left_most_buff = std::get<0>(rmf_v);
    fval = (left_most_val <= fval)?fval:left_most_val;
    f_buff = (left_most_buff <= f_buff)?f_buff:left_most_buff;
  }

  double intended_velocity = (fval > 49.5)?fval:49.5;
  CostCalculator cc(intended_lane, min_cb_buff, f_buff, intended_velocity, vs_.d_);
  double cost = cc.getCost(sensor_data, val_change_req);
  vs_temp_.s_ = vs_.s_;
  vs_temp_.d_ = vs_.d_;
  vs_temp_.velocity_ = vs_.velocity_;
  if (min_cf_buff < 30.0 && min_cb_buff > 10.0) {
    if (vs_temp_.velocity_ > std::min(cfval, fval_this)) {
      vs_temp_.velocity_ -= val_change_req;
    }
  } else if (min_cf_buff > 40.0 && vs_temp_.velocity_ < 49.5) {
    float val_to_consider = std::min(50.0, min_cf_buff);
    vs_temp_.velocity_ += (val_to_consider/20.0)*0.045;
  } else if (vs_temp_.velocity_ < std::min(cfval, fval_this)) {
    vs_temp_.velocity_ += (min_cf_buff/20.0)*0.045;
  }
  return cost;
}

double Vehicle::getPLCLTrajectory() {
  // Get the car behind you
  // check the velocity of car in the left lane
  // Increase the velocity by a constant factor to
  // adjust yourself to next lane
  // Set the intended velocity to the car in left lane
  // set the velocity to atleast the car behind you
  // Update the buffer in s
  float intended_lane = get_possible_left_lane(vs_.d_);
  float intended_lane_2 = get_possible_left_lane(intended_lane);
  temp_state_ = PLCL;
  return getPLCTrajectory_(intended_lane, intended_lane_2);
}

double Vehicle::getPLCRTrajectory() {
  // Get the car behind you
  // check the velocity of car in the right lane
  // Increase the velocity by a constant factor to
  // adjust yourself to next lane
  // Set the intended velocity to the car in left lane
  // set the velocity to atleast the car behind you
  // Update the buffer in s
  float intended_lane = get_possible_right_lane(vs_.d_);
  float intended_lane_2 = get_possible_right_lane(intended_lane);
  temp_state_ = PLCR;
  return getPLCTrajectory_(intended_lane, intended_lane_2);
}

double Vehicle::get_vel_reducer_(double buffer, double f_v_vel) {
  double change = (vs_.velocity_*0.447) - f_v_vel*0.447;
  double factor = buffer/change;
  double ret = (change/factor) * 0.045;
  // double ret = (0.0 + ((50.0 - buffer)/4.0)) * 0.045;
  return ret;
}

double Vehicle::getLCTrajectory_(float intended_lane, float intended_lane_2) {
  assert(intended_lane != -1.0f);
  const auto &tf_buff = get_front_car_(intended_lane);
  double f_buff = std::get<0>(tf_buff);
  double b_buff = std::get<2>(tf_buff);
  if (f_buff < 10.0 || b_buff < 10.0)
      return 3000.0;
  double f_buff_this = std::get<0>(tf_buff);
  double f_v = std::get<1>(tf_buff);
  double f_v_this = std::get<1>(tf_buff);
  double val_change_req = 0.0;
  if (f_buff < 30.0)
    val_change_req = get_vel_reducer_(f_buff, f_v_this);
  if (intended_lane_2 != -1.0f) {
    const auto &rmf_v = get_front_car_(intended_lane_2);
    double left_most_val = std::get<1>(rmf_v);
    double left_most_buff = std::get<0>(rmf_v);
    f_v = (left_most_val <= f_v)?f_v:left_most_val;
    f_buff = (left_most_buff <= f_buff)?f_buff:left_most_buff;
  }
  double intended_velocity = (f_v > 49.5)?f_v:49.5;
  CostCalculator cc(intended_lane, b_buff, f_buff, intended_velocity, intended_lane);
  double cost = cc.getCost(sensor_data, val_change_req);
  vs_temp_.s_ = vs_.s_;
  vs_temp_.d_ = intended_lane;
  vs_temp_.velocity_ = vs_.velocity_;
  if (f_buff_this < 30.0) {
    // if (vs_temp_.velocity_ > f_v_this)
      vs_temp_.velocity_ -= std::min(0.2, val_change_req);
  }

  return cost;
}

double Vehicle::getLCLTrajectory() {
  // Set next lane
  // Increase the velocity by constant factor
  // Set the intended velocity to the car in left lane or 50 whatever is greater
  // Calculate the buffer accordingly
  float intended_lane = get_possible_left_lane(vs_.d_);
  assert(intended_lane != -1.0f);
  float intended_lane_2 = get_possible_left_lane(intended_lane);
  temp_state_ = LCL;
  return getLCTrajectory_(intended_lane, intended_lane_2);
}

double Vehicle::getLCRTrajectory() {
  // Set right lane
  // Increase the velocity by constant factor
  // Set the resulting velocity to the car in right lane
  // Update buffers accordingly
  float intended_lane = get_possible_right_lane(vs_.d_);
  assert(intended_lane != -1.0f);
  float intended_lane_2 = get_possible_right_lane(intended_lane);
  temp_state_ = LCR;
  return getLCTrajectory_(intended_lane, intended_lane_2);
}

VehicleState Vehicle::getBestTrajectory(std::vector<std::vector<double>> s_data, int prev_size, double s) {
  vs_.s_ = s;
  prev_size_ = prev_size;
  sensor_data = s_data;
  const std::vector<Vehicle::states> & next_states = get_next_states();
  double best_cost = 30000.0;
  for (const Vehicle::states &s : next_states) {
    double cur_cost = 0.0;
    switch(s) {
      case KL:
        cur_cost = getKLTrajectory();
        break;
      case PLCL:
        cur_cost = getPLCLTrajectory();
        break;
      case LCL:
        cur_cost = getLCLTrajectory();
        break;
      case PLCR:
        cur_cost = getPLCRTrajectory();
        break;
      case LCR:
        cur_cost = getLCRTrajectory();
        break;
      case KLBuf:
        cur_cost = getKLTrajectory();
        break;
      case KLBuf2:
        cur_cost = getKLTrajectory();
        break;
      default:
        assert(1 == 2);
    }
    if (cur_cost < best_cost) {
      vs_.d_ = vs_temp_.d_;
      vs_.s_ = vs_temp_.s_;
      vs_.velocity_ = vs_temp_.velocity_;
      best_cost = cur_cost;
      cur_state_ = temp_state_;
    }
  }
  // std::cout << cur_state_ << ">" << vs_.d_ << std::endl;
  return vs_;
}

std::tuple<double, double, double, double> Vehicle::get_front_car_(float l) {
  double f_buff = 1000.0;
  double b_buff = 1000.0;
  double fval = 50.0;
  double bval = 50.0;
  for (const std::vector<double> &car : sensor_data) {
    float d = car[6];
    if ((d < (2 + 4*l + 2) && d > (2 + 4*l - 2))) {
      double vx = car[3];
      double vy = car[4];
      double vel = sqrt(vx*vx + vy*vy);
      double s = car[5];
      s += ((double)prev_size_ * 0.02 * vel);
      double buff_now = s - vs_.s_;
      double buff_now_b = vs_.s_ - s;
      if (buff_now >= 0.0 && buff_now < f_buff) {
        f_buff = buff_now;
        fval = vel;
      }
      if (buff_now_b >= 0.0 && buff_now_b < b_buff) {
        b_buff = buff_now_b;
        bval = vel;
      }
    }
  }
  return std::make_tuple(f_buff, fval, b_buff, bval);
}

VehicleState::VehicleState() {
  // Do nothing
}

#include <boost/algorithm/string.hpp>
#include <iostream>
#include <math.h>

#include "pendulum_utils.hpp"

double discretize_pos(double p, double pos_res) {
  // //p = std::fmod((p + M_PI), (2*M_PI)) - M_PI;
  // //p = std::fmod(p, (2*M_PI));
  // p = round(p / pos_res)*pos_res;
  // p = round(100000*p) / 100000.0;
  // //p = std::fmod((p + M_PI), (2*M_PI)) - M_PI;
  // //p = std::fmod(p, (2*M_PI));
  // p = -M_PI + std::fmod(2*M_PI + std::fmod(p - M_PI, 2*M_PI), 2*M_PI);
  p = -M_PI + std::fmod(2 * M_PI + std::fmod(p - M_PI, 2 * M_PI), 2 * M_PI);
  p = round(p / pos_res) * pos_res;
  p = round(100000 * p) / 100000.0;
  return p;
}

double discretize_vel(double v, double vel_res) {
  v = round(v / vel_res) * vel_res;
  v = round(1000 * v) / 1000.0;
  return v;
}

long pos_vel_to_index(double pos, double vel, double pos_res, double pos_min,
                      double pos_max, double vel_res, double vel_min,
                      double vel_max) {
  pos = -M_PI + std::fmod(2 * M_PI + std::fmod(pos - M_PI, 2 * M_PI), 2 * M_PI);
  // while (pos > 2 * M_PI) {
  //   pos -= 2 * M_PI;
  // }
  // while (pos < -2 * M_PI) {
  //   pos += 2 * M_PI;
  // }
  long n_pos = (long)round((pos_max - pos_min) / pos_res);
  long n_vel = (long)round((vel_max - vel_min) / vel_res);
  long pos_id = (long)round((pos - pos_min) / pos_res);
  long vel_id = (long)round((vel - vel_min) / vel_res);
  long index = n_pos * vel_id + pos_id;
  return index;
}

double index_to_pos(long index, double pos_res, double pos_min, double pos_max,
                    double vel_res, double vel_min, double vel_max) {
  long n_pos = (long)round((pos_max - pos_min) / pos_res);
  long pos_id = index % n_pos;
  double pos = pos_id * pos_res + pos_min;
  return pos;
}

double index_to_vel(long index, double pos_res, double pos_min, double pos_max,
                    double vel_res, double vel_min, double vel_max) {
  long n_pos = (long)round((pos_max - pos_min) / pos_res);
  long n_vel = (long)round((vel_max - vel_min) / vel_res);
  long pos_id = index % n_pos;
  long vel_id = (index - pos_id) / n_pos;
  double vel = vel_id * vel_res + vel_min;
  return vel;
}

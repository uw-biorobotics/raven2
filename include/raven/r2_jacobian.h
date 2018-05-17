/* Raven 2 Control - Control software for the Raven II robot
 * Copyright (C) 2005-2012  H. Hawkeye King, Blake Hannaford, and the University
 *of Washington BioRobotics Laboratory
 *
 * This file is part of Raven 2 Control.
 *
 * Raven 2 Control is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Raven 2 Control is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU  General Public License
 * along with Raven 2 Control.  If not, see <http://www.gnu.org/licenses/>.
 */

/** Class header for jacobian class
 *
 *
 *  \date May 17, 2016
 *  \author Andy Lewis
 */

#ifndef R2_JACOBIAN_H_
#define R2_JACOBIAN_H_

#include <Eigen/Dense>

struct robot_device;

/**
 * \class The r2_jacobian class holds the 6-DOF velocity and force vectors for a
 *RAVEN mechanism
 *
 *
 */
class r2_jacobian {
 private:
  Eigen::VectorXf velocity;
  Eigen::VectorXf force;
  Eigen::Matrix<float, 6, 6> j_matrix;

  void set_vel(Eigen::VectorXf);

  void set_force(Eigen::VectorXf);

  int calc_jacobian(float[6], tool, int);

  int calc_velocities(float[6]);

  int calc_forces(float[6]);

  // methods
 public:
  r2_jacobian(){};

  r2_jacobian(Eigen::VectorXf, Eigen::VectorXf);

  ~r2_jacobian(){};

  void get_vel(float *);

  void get_force(float *);

  int update_r2_jacobian(float[6], float[6], float[6], tool, int);
};

int r2_device_jacobian(robot_device *d0, int runlevel);

#endif /* R2_JACOBIAN_H_ */

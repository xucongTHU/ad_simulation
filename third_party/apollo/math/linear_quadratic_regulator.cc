/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include <limits>

#include "Eigen/Dense"
#include <iostream>

#include "third_party/smartsim/math/linear_interpolation.h"

namespace smartsim {
namespace math {

using Matrix = Eigen::MatrixXd;

// solver with cross term
void SolveLQRProblem(const Matrix &A, const Matrix &B, const Matrix &Q,
                     const Matrix &R, const Matrix &M, const double tolerance,
                     const uint max_num_iteration, Matrix *ptr_K,
                     uint &result_num_iter, double &result_diff, Eigen::MatrixXd *ptr_P_last) {
  if (A.rows() != A.cols() || B.rows() != A.rows() || Q.rows() != Q.cols() ||
      Q.rows() != A.rows() || R.rows() != R.cols() || R.rows() != B.cols() ||
      M.rows() != Q.rows() || M.cols() != R.cols()) {
    std::cerr << "LQR solver: one or more matrices have incompatible dimensions.";
    return;
  }

  Matrix AT = A.transpose();
  Matrix BT = B.transpose();
  Matrix MT = M.transpose();

  // Solves a discrete-time Algebraic Riccati equation (DARE)
  // Calculate Matrix Difference Riccati Equation, initialize P and Q
  Matrix P = Q;
  if (ptr_P_last != nullptr) {
    P = *ptr_P_last;
  }
  uint num_iteration = 0;
  double diff = std::numeric_limits<double>::max();
  while (num_iteration++ < max_num_iteration && diff > tolerance) {
    Matrix P_next =
        AT * P * A -
        (AT * P * B + M) * (R + BT * P * B).inverse() * (BT * P * A + MT) + Q;
    // check the difference between P and P_next
    diff = fabs((P_next - P).maxCoeff());
    P = P_next;
  }

  if (num_iteration >= max_num_iteration) {
    std::cerr << "LQR solver cannot converge to a solution, "
              "last consecutive result diff is: "
           << diff;
  } else {
    // std::cerr << "LQR solver converged at iteration: " << num_iteration
    //        << ", max consecutive result diff.: " << diff;
  }
  result_num_iter = num_iteration;
  result_diff = diff;
  *ptr_K = (R + BT * P * B).inverse() * (BT * P * A + MT);
  if (ptr_P_last != nullptr) {
    *ptr_P_last = P;
  }
}

void SolveLQRProblem(const Matrix &A, const Matrix &B, const Matrix &Q,
                     const Matrix &R, const double tolerance, const uint max_num_iteration,
                     Matrix *ptr_K, uint &result_num_iter, double &result_diff, Eigen::MatrixXd *ptr_P_last) {
  // create M as zero matrix of the right size:
  // M.rows() == Q.rows() && M.cols() == R.cols()
  Matrix M = Matrix::Zero(Q.rows(), R.cols());
  SolveLQRProblem(A, B, Q, R, M, tolerance, max_num_iteration, ptr_K,
                  result_num_iter, result_diff, ptr_P_last);
}

}  // namespace math
}  // namespace smartsim

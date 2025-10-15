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

/**
 * @file
 * @brief Defines some useful matrix operations.
 */

#pragma once

#include <cmath>
#include <utility>
#include <vector>

#include "Eigen/Dense"
#include "Eigen/SVD"

/**
 * @namespace smartsim::common::math
 * @brief The math namespace deals with a number of useful mathematical objects.
 */
namespace smartsim {
namespace common {
namespace math {

template <typename T, int M, int N, typename D>
void DenseToCSCMatrix(const Eigen::Matrix<T, M, N> &dense_matrix,
                      std::vector<T> *data, std::vector<D> *indices,
                      std::vector<D> *indptr) {
  static constexpr double epsilon = 1e-9;
  int data_count = 0;
  for (int c = 0; c < dense_matrix.cols(); ++c) {
    indptr->emplace_back(data_count);
    for (int r = 0; r < dense_matrix.rows(); ++r) {
      if (std::fabs(dense_matrix(r, c)) < epsilon) {
        continue;
      }
      data->emplace_back(dense_matrix(r, c));
      ++data_count;
      indices->emplace_back(r);
    }
  }
  indptr->emplace_back(data_count);
}

}  // namespace math
}  // namespace common
}  // namespace smartsim

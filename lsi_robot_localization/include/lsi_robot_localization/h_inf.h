/*
 * Copyright (c) 2014, 2015, 2016, Charles River Analytics, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef LSI_ROBOT_LOCALIZATION_h_inf_H
#define LSI_ROBOT_LOCALIZATION_h_inf_H

#include "lsi_robot_localization/filter_base.h"

#include <fstream>
#include <iostream>
#include <vector>
#include <set>
#include <queue>

namespace RobotLocalization
{

//! @brief Extended Kalman filter class
//!
//! Implementation of an extended Kalman filter (h_inf). This
//! class derives from FilterBase and overrides the predict()
//! and correct() methods in keeping with the discrete time
//! h_inf algorithm.
//!
class h_inf: public FilterBase
{
  public:
    //! @brief Constructor for the h_inf class
    //!
    //! @param[in] args - Generic argument container (not used here, but
    //! needed so that the ROS filters can pass arbitrary arguments to
    //! templated filter types).
    //!
    explicit h_inf(std::vector<double> args = std::vector<double>());

    //! @brief Destructor for the h_inf class
    //!
    ~h_inf();

    //! @brief Carries out the correct step in the predict/update cycle.
    //!
    //! @param[in] measurement - The measurement to fuse with our estimate
    //!
    void correct(const Measurement &measurement);

    //! @brief Carries out the predict step in the predict/update cycle.
    //!
    //! Projects the state and error matrices forward using a model of
    //! the vehicle's motion.
    //!
    //! @param[in] referenceTime - The time at which the prediction is being made
    //! @param[in] delta - The time step over which to predict.
    //!
    void predict(const double referenceTime, const double delta);

protected:
    //!@brief Gamma is the performance bound used for solving the optimization problem
    //! of the H-infinity filter
    double Gamma;
};

}  // namespace RobotLocalization

#endif  // LSI_ROBOT_LOCALIZATION_h_inf_H

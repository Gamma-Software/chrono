// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// HMMWV simple driveline model.
//
// =============================================================================

#pragma once

#include "chrono_vehicle/wheeled_vehicle/driveline/ChSimpleDriveline.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace tract {


struct DriveLineData
{
    double front_torque_frac = 0.5;
    double front_diff_bias = 2.0;
    double rear_diff_bias = 2.0;
};

/// @addtogroup vehicle_models_hmmwv
/// @{

/// Simple HMMWV driveline subsystem (purely kinematic).
class CH_MODELS_API Tract_SimpleDriveline : public ChSimpleDriveline {
  public:
    Tract_SimpleDriveline(const std::string& name, const DriveLineData& data);

    ~Tract_SimpleDriveline() {}

    virtual double GetFrontTorqueFraction() const override { return m_data.front_torque_frac; }
    virtual double GetFrontDifferentialMaxBias() const override { return m_data.front_diff_bias; }
    virtual double GetRearDifferentialMaxBias() const override { return m_data.rear_diff_bias; }

  private:
      DriveLineData m_data;
};

/// @} vehicle_models_hmmwv

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono

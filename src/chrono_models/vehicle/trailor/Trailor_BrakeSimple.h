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
// Authors: Alessandro Tasora
// =============================================================================
//
// HMMWV simple brake models (front and rear).
//
// =============================================================================

#pragma once

#include "chrono_vehicle/wheeled_vehicle/brake/ChBrakeSimple.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace trailor {


struct BrakeData
{
    double maxtorque = 4000;
};

/// @addtogroup vehicle_models_hmmwv
/// @{

/// Simple HMMWV brake subsystem (torque applied directly to the spindle joint).
class CH_MODELS_API Trailor_BrakeSimple : public ChBrakeSimple {
  public:
    Trailor_BrakeSimple(const std::string& name, const BrakeData& data);
    virtual ~Trailor_BrakeSimple() {}

    virtual double GetMaxBrakingTorque() override { return m_data.maxtorque; }

  private:
    BrakeData m_data;
};

/// @} vehicle_models_hmmwv

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono

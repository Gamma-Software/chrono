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
// HMMWV rack-pinion steering model.
//
// =============================================================================

#pragma once

#include "chrono_vehicle/wheeled_vehicle/steering/ChRackPinion.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace tract {

struct SteeringData
{
    double steeringLinkMass = 9.072;
    ChVector<> steeringLinkInertia = ChVector<>(1., 1., 1.);
    double steeringLinkCOM = 0;
    double steeringLinkLength = 0.896;
    double steeringLinkRadius = 0.03;

    double pinionRadius = 0.1;

    double maxAngle = 50.0 * (CH_C_PI / 180);
};

/// @addtogroup vehicle_models_hmmwv
/// @{

/// Rack-pinion steering subsystem for the HMMWV vehicle.
class CH_MODELS_API Tract_RackPinion : public ChRackPinion {
  public:
    Tract_RackPinion(const std::string& name, const SteeringData& data);
    ~Tract_RackPinion() {}

    virtual double GetSteeringLinkMass() const override { return m_data.steeringLinkMass; }
    virtual ChVector<> GetSteeringLinkInertia() const override { return m_data.steeringLinkInertia; }
    virtual double GetSteeringLinkCOM() const override { return m_data.steeringLinkCOM; }
    virtual double GetSteeringLinkRadius() const override { return m_data.steeringLinkRadius; }
    virtual double GetSteeringLinkLength() const override { return m_data.steeringLinkLength; }

    virtual double GetPinionRadius() const override { return m_data.pinionRadius; }

    virtual double GetMaxAngle() const override { return m_data.maxAngle; }

  private:
    SteeringData m_data;
};

/// @} vehicle_models_hmmwv

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono

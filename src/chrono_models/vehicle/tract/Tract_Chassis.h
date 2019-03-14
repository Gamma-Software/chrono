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
// HMMWV chassis subsystem.
//
// =============================================================================

#pragma once
#include <string>

#include "chrono_vehicle/chassis/ChRigidChassis.h"

#include "chrono_models/ChApiModels.h"
#include "chrono_models/vehicle/ChVehicleModelDefs.h"

namespace chrono {
namespace vehicle {
namespace tract {

struct ChassisData
{
    double mass = 2086.52;
    ChMatrix33<> inertia = ChMatrix33<>(0.);
    ChVector<> inertiaXX = ChVector<>(1078.52, 2955.66, 3570.20);
    ChVector<> inertiaXY = ChVector<>(0, 0, 0);
    ChVector<> COM_loc = ChVector<>(0.056, 0, 0.523);
    ChCoordsys<> driverCsys = ChCoordsys<>(ChVector<>(0.87, -0.27, 1.05), ChQuaternion<>(1, 0, 0, 0));
};

/// @addtogroup vehicle_models_hmmwv
/// @{

/// HMMWV chassis subsystem.
class CH_MODELS_API Tract_Chassis : public ChRigidChassis {
  public:
    Tract_Chassis(const std::string& name,
                  const ChassisData& data,
                  bool fixed = false,
                  ChassisCollisionType chassis_collision_type = ChassisCollisionType::NONE);
    ~Tract_Chassis() {}

    /// Return the mass of the chassis body.
    virtual double GetMass() const override { return m_data.mass; }

    /// Return the inertia tensor of the chassis body.
    virtual const ChMatrix33<>& GetInertia() const override { return m_data.inertia; }

    /// Get the location of the center of mass in the chassis frame.
    virtual const ChVector<>& GetLocalPosCOM() const override { return m_data.COM_loc; }

    /// Get the local driver position and orientation.
    /// This is a coordinate system relative to the chassis reference frame.
    virtual ChCoordsys<> GetLocalDriverCoordsys() const override { return m_data.driverCsys; }

  protected:
    ChassisData m_data;
};

/// @} vehicle_models_hmmwv

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono

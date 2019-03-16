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
// Authors: Radu Serban, Justin Madsen
// =============================================================================
//
// HMMWV 9-body vehicle model...
//
// =============================================================================

#pragma once

#include "chrono_models/ChApiModels.h"
#include "chrono_models/vehicle/ChVehicleModelDefs.h"

#include "chrono_models/vehicle/tract/Tract_Vehicle.h"
#include "chrono_models/vehicle/tract/Tract_Chassis.h"
#include "chrono_models/vehicle/tract/Tract_BrakeSimple.h"
#include "chrono_models/vehicle/tract/Tract_DoubleWishboneReduced.h"
//#include "chrono_models/vehicle/tract/Tract_LeafspringAxle.h"
#include "chrono_models/vehicle/tract/Tract_RigidSuspension.h"
#include "chrono_models/vehicle/tract/Tract_Driveline2WD.h"
#include "chrono_models/vehicle/tract/Tract_SimpleDriveline.h"
#include "chrono_models/vehicle/tract/Tract_RackPinion.h"
#include "chrono_models/vehicle/tract/Tract_Wheel.h"

namespace chrono {
namespace vehicle {
namespace tract {

/// @addtogroup vehicle_models_hmmwv
/// @{

/// HMMWV vehicle system using reduced double wishbone suspension (control arms modeled using distance constraints)
/// and rack-pinion steering mechanism.
class CH_MODELS_API Tract_VehicleReduced : public Tract_Vehicle {
  public:
    Tract_VehicleReduced(const bool fixed,
                         DrivelineType drive_type,
                         ChMaterialSurface::ContactMethod contact_method,
                         ChassisCollisionType chassis_collision_type);

    Tract_VehicleReduced(ChSystem* system,
                         const bool fixed,
                         DrivelineType drive_type,
                         ChassisCollisionType chassis_collision_type);

    ~Tract_VehicleReduced();

    virtual void Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel = 0) override;

  private:
    void Create(bool fixed, ChassisCollisionType chassis_collision_type);
};

/// @} vehicle_models_hmmwv

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono

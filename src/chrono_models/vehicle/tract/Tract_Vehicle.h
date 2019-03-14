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
// Base class for the HMMWV vehicle models
//
// =============================================================================

#pragma once

#include <vector>
#include "chrono/core/ChCoordsys.h"
#include "chrono/physics/ChMaterialSurface.h"
#include "chrono/physics/ChSystem.h"

#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace tract {

/// @addtogroup vehicle_models_hmmwv
/// @{

/// Base class for a HMMWV vehicle.
class CH_MODELS_API Tract_Vehicle : public ChWheeledVehicle {
  public:
    virtual ~Tract_Vehicle() {}

    virtual int GetNumberAxles() const override { return 2; }

    virtual double GetMinTurningRadius() const override { return GetWheelbase(0)/std::tan(GetMaxSteeringAngle()); }
    virtual double GetMaxSteeringAngle() const override { return 26.9 * CH_C_DEG_TO_RAD; }

    void SetInitWheelAngVel(const std::vector<double>& omega) {
        assert(omega.size() == 4);
        m_omega = omega;
    }

  protected:
    Tract_Vehicle(const std::string& name, ChMaterialSurface::ContactMethod contactMethod, DrivelineType driveType)
        : ChWheeledVehicle(name, contactMethod), m_driveType(driveType), m_omega({0, 0, 0, 0}) {}

    Tract_Vehicle(const std::string& name, ChSystem* system, DrivelineType driveType)
        : ChWheeledVehicle(name, system), m_driveType(driveType), m_omega({0, 0, 0, 0}) {}

    DrivelineType m_driveType;
    std::vector<double> m_omega;
};

/// @} vehicle_models_hmmwv

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono
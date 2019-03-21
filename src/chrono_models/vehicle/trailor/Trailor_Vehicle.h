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
#include "chrono_vehicle/ChVehicle.h"

#include "chrono_models/vehicle/Trailor/Trailor_Vehicle.h"
#include "chrono_models/vehicle/Trailor/Trailor_Chassis.h"
#include "chrono_models/vehicle/Trailor/Trailor_BrakeSimple.h"
#include "chrono_models/vehicle/Trailor/Trailor_LeafspringAxle.h"
#include "chrono_models/vehicle/Trailor/Trailor_RigidSuspension.h"
#include "chrono_models/vehicle/Trailor/Trailor_Wheel.h"

namespace chrono {
namespace vehicle {
namespace trailor {

/// @addtogroup vehicle_models_hmmwv
/// @{

/// Base class for a HMMWV vehicle.
class CH_MODELS_API Trailor_Vehicle : public ChVehicle
{
public:
    virtual ~Trailor_Vehicle()
    {
    }

    void Create(bool fixed, ChassisCollisionType chassis_collision_type);


    void SetInitWheelAngVel(const std::vector<double>& omega)
    {
        assert(omega.size() == 4);
        m_omega = omega;
    }


    /// Get a handle to the specified wheel body.
    std::shared_ptr<ChBody> GetWheelBody(const WheelID& wheelID) const;

    /// Get the complete state for the specified wheel.
    /// This includes the location, orientation, linear and angular velocities,
    /// all expressed in the global reference frame, as well as the wheel angular
    /// speed about its rotation axis.
    WheelState GetWheelState(const WheelID& wheel_id) const;

    /// Get the name of the vehicle system template.
    virtual std::string GetTemplateName() const override;

    /// Get the vehicle total mass.
    /// This includes the mass of the chassis and all vehicle subsystems.
    virtual double GetVehicleMass() const override;

    /// Get the current global vehicle COM location.
    virtual ChVector<> GetVehicleCOMPos() const override;

    /// Get a handle to the vehicle's driveshaft body.
    virtual std::shared_ptr<ChShaft> GetDriveshaft() const override;

    /// Get the angular speed of the driveshaft.
    /// This function provides the interface between a vehicle system and a
    /// powertrain system.
    virtual double GetDriveshaftSpeed() const override;


    /// Initialize this vehicle at the specified global location and orientation.
    virtual void Initialize(const ChCoordsys<>& chassisPos,  ///< [in] initial global position and orientation
        double chassisFwdVel = 0         ///< [in] initial chassis forward velocity
    ) override;

    void Synchronize(double time,
        double braking,
        const TerrainForces& tire_forces);

    /// Log current constraint violations.
    virtual void LogConstraintViolations() override;

    /// Return a JSON string with information on all modeling components in the vehicle system.
    /// These include bodies, shafts, joints, spring-damper elements, markers, etc.
    virtual std::string ExportComponentList() const override;

    /// Write a JSON-format file with information on all modeling components in the vehicle system.
    /// These include bodies, shafts, joints, spring-damper elements, markers, etc.
    virtual void ExportComponentList(const std::string& filename) const override;

    /// Output data for all modeling components in the vehicle system.
    virtual void Output(int frame, ChVehicleOutput& database) const override;

    Trailor_Vehicle(const std::string& name, bool fixed, ChassisCollisionType chassis_collision_type, ChMaterialSurface::ContactMethod contactMethod)
        : ChVehicle(name, contactMethod), m_omega({ 0, 0, 0, 0 })
    {
        Create(fixed, chassis_collision_type);
    }

    Trailor_Vehicle(const std::string& name, bool fixed, ChassisCollisionType chassis_collision_type, ChSystem* system)
        : ChVehicle(name, system), m_omega({ 0, 0, 0, 0 })
    {
        Create(fixed, chassis_collision_type);
    }
    std::vector<double> m_omega;

protected:
    ChSuspensionList m_suspensions;            ///< list of handles to suspension subsystems
    ChWheelList m_wheels;                      ///< list of handles to wheel subsystems
    ChBrakeList m_brakes;                      ///< list of handles to brake subsystems

};

/// @} vehicle_models_hmmwv

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono

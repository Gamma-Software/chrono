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

#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/tract/Tract_VehicleReduced.h"

namespace chrono {
namespace vehicle {
namespace tract {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

static const double in2m = 0.0254;
static const double lb2kg = 0.453592;
static const double lbf2N = 4.44822162;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Tract_VehicleReduced::Tract_VehicleReduced(const bool fixed,
                                           DrivelineType drive_type,
                                           ChMaterialSurface::ContactMethod contact_method,
                                           ChassisCollisionType chassis_collision_type)
    : Tract_Vehicle("Tractreduced", contact_method, drive_type) {
    Create(fixed, chassis_collision_type);
}

Tract_VehicleReduced::Tract_VehicleReduced(ChSystem* system,
                                           const bool fixed,
                                           DrivelineType drive_type,
                                           ChassisCollisionType chassis_collision_type)
    : Tract_Vehicle("Tractreduced", system, drive_type) {
    Create(fixed, chassis_collision_type);
}

void Tract_VehicleReduced::Create(bool fixed, ChassisCollisionType chassis_collision_type) {
    // -------------------------------------------
    // Create the chassis subsystem
    // -------------------------------------------
    ChassisData chassis_data;
    chassis_data.mass = 4215.;
    chassis_data.COM_loc.x() = -0.05;
    chassis_data.COM_loc.z() = 0.;
    chassis_data.driverCsys = ChCoordsys<>(ChVector<>(0.8, 0., 0.805), ChQuaternion<>(1, 0, 0, 0));
    m_chassis = std::make_shared<Tract_Chassis>("Chassis", chassis_data, fixed, chassis_collision_type);

    // -------------------------------------------
    // Create the suspension subsystems
    // -------------------------------------------
    m_suspensions.resize(2);
    /*SuspensionData suspension_data;
    suspension_data.springCoefficient *= 10;*/
    m_suspensions[0] = std::make_shared<Tract_DoubleWishboneReducedFront>("FrontSusp");
    //suspension_data.wheelTrack = 1.24; // The rear wheel track is different
    m_suspensions[1] = std::make_shared<Tract_DoubleWishboneReducedRear>("RearSusp");

    // -----------------------------
    // Create the steering subsystem
    // -----------------------------
    m_steerings.resize(1);
    SteeringData steering_data;
    steering_data.steeringLinkLength = 0.654;
    m_steerings[0] = std::make_shared<Tract_RackPinion>("Steering", steering_data);

    // -----------------
    // Create the wheels
    // -----------------
    m_wheels.resize(4);
    WheelData wheel_data;
    wheel_data.radius = 0.4572/2;
    wheel_data.width = 0.1524;
    m_wheels[0] = std::make_shared<Tract_Wheel>("Wheel_FL", wheel_data);
    m_wheels[1] = std::make_shared<Tract_Wheel>("Wheel_FR", wheel_data);
    wheel_data.radius = 0.6096 / 2;
    wheel_data.width = 0.1778;
    m_wheels[2] = std::make_shared<Tract_Wheel>("Wheel_RL", wheel_data);
    m_wheels[3] = std::make_shared<Tract_Wheel>("Wheel_RR", wheel_data);

    // --------------------
    // Create the driveline
    // --------------------
    switch (m_driveType) {
        case DrivelineType::RWD:
            m_driveline = std::make_shared<Tract_Driveline2WD>("Driveline");
            break;
        case DrivelineType::SIMPLE:
            m_driveline = std::make_shared<Tract_SimpleDriveline>("Driveline", DriveLineData());
            break;
    }

    // -----------------
    // Create the brakes
    // -----------------
    m_brakes.resize(4);
    m_brakes[0] = std::make_shared<Tract_BrakeSimple>("Brake_FL", BrakeData());
    m_brakes[1] = std::make_shared<Tract_BrakeSimple>("Brake_FR", BrakeData());
    m_brakes[2] = std::make_shared<Tract_BrakeSimple>("Brake_RL", BrakeData());
    m_brakes[3] = std::make_shared<Tract_BrakeSimple>("Brake_RR", BrakeData());
}

Tract_VehicleReduced::~Tract_VehicleReduced() {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Tract_VehicleReduced::Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel) {
    // Invoke base class method to initialize the chassis.
    ChWheeledVehicle::Initialize(chassisPos, chassisFwdVel);

    // Initialize the steering subsystem (specify the steering subsystem's frame
    // relative to the chassis reference frame).
    ChVector<> offset = ChVector<>(0.5305, 0, 0.);
    m_steerings[0]->Initialize(m_chassis->GetBody(), offset, ChQuaternion<>(1, 0, 0, 0));

    // Initialize the suspension subsystems (specify the suspension subsystems'
    // frames relative to the chassis reference frame).
    double wheelBase = 1.521;
    m_suspensions[0]->Initialize(m_chassis->GetBody(), ChVector<>(wheelBase/2, 0, 0.0264),
                                 m_steerings[0]->GetSteeringLink(), 0, m_omega[0], m_omega[1]);
    m_suspensions[1]->Initialize(m_chassis->GetBody(), ChVector<>(-wheelBase/2, 0, 0.0264), m_chassis->GetBody(), -1,
                                 m_omega[2], m_omega[3]);

    // Initialize wheels
    m_wheels[0]->Initialize(m_suspensions[0]->GetSpindle(LEFT));
    m_wheels[1]->Initialize(m_suspensions[0]->GetSpindle(RIGHT));
    m_wheels[2]->Initialize(m_suspensions[1]->GetSpindle(LEFT));
    m_wheels[3]->Initialize(m_suspensions[1]->GetSpindle(RIGHT));

    // Initialize the driveline subsystem.
    std::vector<int> driven_susp_indexes(m_driveline->GetNumDrivenAxles());

    switch (m_driveType) {
        case DrivelineType::RWD:
            driven_susp_indexes[0] = 1;
            driven_susp_indexes[1] = 0;
            break;
        case DrivelineType::SIMPLE:
            driven_susp_indexes[0] = 0;
            driven_susp_indexes[1] = 1;
            break;
    }

    m_driveline->Initialize(m_chassis->GetBody(), m_suspensions, driven_susp_indexes);

    // Initialize the four brakes
    m_brakes[0]->Initialize(m_suspensions[0]->GetRevolute(LEFT));
    m_brakes[1]->Initialize(m_suspensions[0]->GetRevolute(RIGHT));
    m_brakes[2]->Initialize(m_suspensions[1]->GetRevolute(LEFT));
    m_brakes[3]->Initialize(m_suspensions[1]->GetRevolute(RIGHT));
}

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono

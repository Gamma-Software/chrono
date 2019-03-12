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
// Wrapper classes for modeling an entire Tract vehicle assembly
// (including the vehicle itself, the powertrain, and the tires).
//
// =============================================================================

#include "chrono/ChConfig.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/tract/Tract.h"

namespace chrono {
namespace vehicle {
namespace tract {

// -----------------------------------------------------------------------------
Tract::Tract()
    : m_system(NULL),
      m_vehicle(NULL),
      m_powertrain(NULL),
      m_tires({{NULL, NULL, NULL, NULL}}),
      m_contactMethod(ChMaterialSurface::NSC),
      m_chassisCollisionType(ChassisCollisionType::NONE),
      m_fixed(false),
      m_driveType(DrivelineType::RWD),
      m_powertrainType(PowertrainModelType::SIMPLE_MAP),
      m_tireType(TireModelType::RIGID),
      m_vehicle_step_size(-1),
      m_tire_step_size(-1),
      m_initFwdVel(0),
      m_initPos(ChCoordsys<>(ChVector<>(0, 0, 1), QUNIT)),
      m_initOmega({0, 0, 0, 0}),
      m_apply_drag(false) {}

Tract::Tract(ChSystem* system)
    : m_system(system),
      m_vehicle(NULL),
      m_powertrain(NULL),
      m_tires({{NULL, NULL, NULL, NULL}}),
      m_contactMethod(ChMaterialSurface::NSC),
      m_chassisCollisionType(ChassisCollisionType::NONE),
      m_fixed(false),
      m_driveType(DrivelineType::RWD),
      m_powertrainType(PowertrainModelType::SIMPLE_MAP),
      m_tireType(TireModelType::RIGID),
      m_vehicle_step_size(-1),
      m_tire_step_size(-1),
      m_initFwdVel(0),
      m_initPos(ChCoordsys<>(ChVector<>(0, 0, 1), QUNIT)),
      m_initOmega({0, 0, 0, 0}),
      m_apply_drag(false) {}

Tract::~Tract() {
    delete m_vehicle;
    delete m_powertrain;
    delete m_tires[0];
    delete m_tires[1];
    delete m_tires[2];
    delete m_tires[3];
}

// -----------------------------------------------------------------------------
void Tract::SetAerodynamicDrag(double Cd, double area, double air_density) {
    m_Cd = Cd;
    m_area = area;
    m_air_density = air_density;

    m_apply_drag = true;
}

// -----------------------------------------------------------------------------
void Tract::Initialize() {
    // Create and initialize the Tract vehicle
    m_vehicle = CreateVehicle();
    m_vehicle->SetInitWheelAngVel(m_initOmega);
    m_vehicle->Initialize(m_initPos, m_initFwdVel);

    if (m_vehicle_step_size > 0) {
        m_vehicle->SetStepsize(m_vehicle_step_size);
    }

    // If specified, enable aerodynamic drag
    if (m_apply_drag) {
        m_vehicle->GetChassis()->SetAerodynamicDrag(m_Cd, m_area, m_air_density);
    }

    // Create and initialize the powertrain system
    switch (m_powertrainType) {
        case PowertrainModelType::SHAFTS: {
            Tract_Powertrain* ptrain = new Tract_Powertrain("Powertrain");
            m_powertrain = ptrain;
            break;
        }
        case PowertrainModelType::SIMPLE_MAP: {
            Tract_SimpleMapPowertrain* ptrain = new Tract_SimpleMapPowertrain("Powertrain");
            m_powertrain = ptrain;
            break;
        }
        case PowertrainModelType::SIMPLE: {
            Tract_SimplePowertrain* ptrain = new Tract_SimplePowertrain("Powertrain");
            m_powertrain = ptrain;
            break;
        }
    }

    m_powertrain->Initialize(GetChassisBody(), m_vehicle->GetDriveshaft());

    // Create the tires and set parameters depending on type.
    switch (m_tireType) {
        case TireModelType::RIGID:
        case TireModelType::RIGID_MESH: {
            bool use_mesh = (m_tireType == TireModelType::RIGID_MESH);
            Tract_RigidTire* tire_FL = new Tract_RigidTire("FL", use_mesh);
            Tract_RigidTire* tire_FR = new Tract_RigidTire("FR", use_mesh);
            Tract_RigidTire* tire_RL = new Tract_RigidTire("RL", use_mesh);
            Tract_RigidTire* tire_RR = new Tract_RigidTire("RR", use_mesh);

            m_tires[0] = tire_FL;
            m_tires[1] = tire_FR;
            m_tires[2] = tire_RL;
            m_tires[3] = tire_RR;

            break;
        }
        case TireModelType::LUGRE: {
            Tract_LugreTire* tire_FL = new Tract_LugreTire("FL");
            Tract_LugreTire* tire_FR = new Tract_LugreTire("FR");
            Tract_LugreTire* tire_RL = new Tract_LugreTire("RL");
            Tract_LugreTire* tire_RR = new Tract_LugreTire("RR");

            m_tires[0] = tire_FL;
            m_tires[1] = tire_FR;
            m_tires[2] = tire_RL;
            m_tires[3] = tire_RR;

            break;
        }
        case TireModelType::FIALA: {
            Tract_FialaTire* tire_FL = new Tract_FialaTire("FL");
            Tract_FialaTire* tire_FR = new Tract_FialaTire("FR");
            Tract_FialaTire* tire_RL = new Tract_FialaTire("RL");
            Tract_FialaTire* tire_RR = new Tract_FialaTire("RR");

            m_tires[0] = tire_FL;
            m_tires[1] = tire_FR;
            m_tires[2] = tire_RL;
            m_tires[3] = tire_RR;

            break;
        }
        case TireModelType::TMEASY: {
            Tract_TMeasyTire* tire_FL = new Tract_TMeasyTire("FL");
            Tract_TMeasyTire* tire_FR = new Tract_TMeasyTire("FR");
            Tract_TMeasyTire* tire_RL = new Tract_TMeasyTire("RL");
            Tract_TMeasyTire* tire_RR = new Tract_TMeasyTire("RR");

            m_tires[0] = tire_FL;
            m_tires[1] = tire_FR;
            m_tires[2] = tire_RL;
            m_tires[3] = tire_RR;

            break;
        }
        case TireModelType::PAC89: {
            Tract_Pac89Tire* tire_FL = new Tract_Pac89Tire("FL");
            Tract_Pac89Tire* tire_FR = new Tract_Pac89Tire("FR");
            Tract_Pac89Tire* tire_RL = new Tract_Pac89Tire("RL");
            Tract_Pac89Tire* tire_RR = new Tract_Pac89Tire("RR");

            m_tires[0] = tire_FL;
            m_tires[1] = tire_FR;
            m_tires[2] = tire_RL;
            m_tires[3] = tire_RR;

            break;
        }
        case TireModelType::PACEJKA: {
            ChPacejkaTire* tire_FL = new Tract_Pac02Tire("FL");
            ChPacejkaTire* tire_FR = new Tract_Pac02Tire("FR");
            ChPacejkaTire* tire_RL = new Tract_Pac02Tire("RL");
            ChPacejkaTire* tire_RR = new Tract_Pac02Tire("RR");

            tire_FL->SetDrivenWheel(false);
            tire_FR->SetDrivenWheel(false);
            tire_RL->SetDrivenWheel(true);
            tire_RR->SetDrivenWheel(true);

            m_tires[0] = tire_FL;
            m_tires[1] = tire_FR;
            m_tires[2] = tire_RL;
            m_tires[3] = tire_RR;

            break;
        }
        case TireModelType::ANCF: {
            Tract_ANCFTire* tire_FL = new Tract_ANCFTire("FL");
            Tract_ANCFTire* tire_FR = new Tract_ANCFTire("FR");
            Tract_ANCFTire* tire_RL = new Tract_ANCFTire("RL");
            Tract_ANCFTire* tire_RR = new Tract_ANCFTire("RR");

            m_tires[0] = tire_FL;
            m_tires[1] = tire_FR;
            m_tires[2] = tire_RL;
            m_tires[3] = tire_RR;
            break;
        }
        case TireModelType::REISSNER: {
            Tract_ReissnerTire* tire_FL = new Tract_ReissnerTire("FL");
            Tract_ReissnerTire* tire_FR = new Tract_ReissnerTire("FR");
            Tract_ReissnerTire* tire_RL = new Tract_ReissnerTire("RL");
            Tract_ReissnerTire* tire_RR = new Tract_ReissnerTire("RR");

            m_tires[0] = tire_FL;
            m_tires[1] = tire_FR;
            m_tires[2] = tire_RL;
            m_tires[3] = tire_RR;
            break;
        }
        default:
            break;
    }

    // Initialize the tires.
    m_tires[0]->Initialize(m_vehicle->GetWheelBody(FRONT_LEFT), LEFT);
    m_tires[1]->Initialize(m_vehicle->GetWheelBody(FRONT_RIGHT), RIGHT);
    m_tires[2]->Initialize(m_vehicle->GetWheelBody(REAR_LEFT), LEFT);
    m_tires[3]->Initialize(m_vehicle->GetWheelBody(REAR_RIGHT), RIGHT);

    if (m_tire_step_size > 0) {
        m_tires[0]->SetStepsize(m_tire_step_size);
        m_tires[1]->SetStepsize(m_tire_step_size);
        m_tires[2]->SetStepsize(m_tire_step_size);
        m_tires[3]->SetStepsize(m_tire_step_size);
    }

    m_tire_mass = m_tires[0]->ReportMass();
}

// -----------------------------------------------------------------------------
void Tract::SetTireVisualizationType(VisualizationType vis) {
    m_tires[0]->SetVisualizationType(vis);
    m_tires[1]->SetVisualizationType(vis);
    m_tires[2]->SetVisualizationType(vis);
    m_tires[3]->SetVisualizationType(vis);
}

// -----------------------------------------------------------------------------
void Tract::Synchronize(double time,
                        double steering_input,
                        double braking_input,
                        double throttle_input,
                        const ChTerrain& terrain) {
    TerrainForces tire_forces(4);
    WheelState wheel_states[4];

    tire_forces[0] = m_tires[0]->GetTireForce();
    tire_forces[1] = m_tires[1]->GetTireForce();
    tire_forces[2] = m_tires[2]->GetTireForce();
    tire_forces[3] = m_tires[3]->GetTireForce();

    wheel_states[0] = m_vehicle->GetWheelState(FRONT_LEFT);
    wheel_states[1] = m_vehicle->GetWheelState(FRONT_RIGHT);
    wheel_states[2] = m_vehicle->GetWheelState(REAR_LEFT);
    wheel_states[3] = m_vehicle->GetWheelState(REAR_RIGHT);

    double powertrain_torque = m_powertrain->GetOutputTorque();

    double driveshaft_speed = m_vehicle->GetDriveshaftSpeed();

    m_tires[0]->Synchronize(time, wheel_states[0], terrain);
    m_tires[1]->Synchronize(time, wheel_states[1], terrain);
    m_tires[2]->Synchronize(time, wheel_states[2], terrain);
    m_tires[3]->Synchronize(time, wheel_states[3], terrain);

    m_powertrain->Synchronize(time, throttle_input, driveshaft_speed);

    m_vehicle->Synchronize(time, steering_input, braking_input, powertrain_torque, tire_forces);
}

// -----------------------------------------------------------------------------
void Tract::Advance(double step) {
    m_tires[0]->Advance(step);
    m_tires[1]->Advance(step);
    m_tires[2]->Advance(step);
    m_tires[3]->Advance(step);

    m_powertrain->Advance(step);

    m_vehicle->Advance(step);
}

// -----------------------------------------------------------------------------
double Tract::GetTotalMass() const {
    return m_vehicle->GetVehicleMass() + 4 * m_tire_mass;
}

// =============================================================================

Tract_Vehicle* TractModel::CreateVehicle() {
    if (m_system) {
        return new Tract_VehicleReduced(m_system, m_fixed, m_driveType, m_chassisCollisionType);
    }

    return new Tract_VehicleReduced(m_fixed, m_driveType, m_contactMethod, m_chassisCollisionType);
}

}  // end namespace Tract
}  // end namespace vehicle
}  // end namespace chrono

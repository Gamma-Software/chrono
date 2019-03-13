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
// Main driver function for the HMMWV 9-body model.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================


#include "chrono/core/ChStream.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"

#include "chrono_vehicle/driver/ChIrrGuiDriver.h"

#include "chrono_vehicle/output/ChVehicleOutputASCII.h"

#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#include "chrono/core/ChMatrixDynamic.h"
#include "chrono_cosimulation/ChCosimulation.h"
#include "chrono_cosimulation/ChExceptionSocket.h"
#include "chrono_vehicle/utils/ChSpeedController.h"


#include "chrono/physics/ChBodyEasy.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

// =============================================================================

// Initial vehicle location and orientation
ChVector<> initLoc(0, 0, 1.0);
ChQuaternion<> initRot(1, 0, 0, 0);
// ChQuaternion<> initRot(0.866025, 0, 0, 0.5);
// ChQuaternion<> initRot(0.7071068, 0, 0, 0.7071068);
// ChQuaternion<> initRot(0.25882, 0, 0, 0.965926);
// ChQuaternion<> initRot(0, 0, 0, 1);

// Type of powertrain model (SHAFTS, SIMPLE)
PowertrainModelType powertrain_model = PowertrainModelType::SIMPLE;

// Type of tire model (RIGID, RIGID_MESH, TMEASY, PACEJKA, PAC89, FIALA)
TireModelType tire_model = TireModelType::RIGID;

// Rigid terrain dimensions
double terrainHeight = 0;
double terrainLength = 200.0;  // size in X direction
double terrainWidth = 100.0;   // size in Y direction

// Point on chassis tracked by the camera
ChVector<> trackPoint(0.0, 0.0, .75);

// Simulation step sizes
double step_size = 0.005;
double tire_step_size = 0.001;
bool enforce_soft_real_time = true;

// Time interval between two render frames
int FPS = 50;
double render_step_size = 1.0 / FPS;  // FPS = 50

int input_nb = 1;
ChMatrixDynamic<int> data_in(input_nb * 15, 1);
double data;
ChMatrixDynamic<int> data_out(1, 1);

// POV-Ray output
bool povray_output = false;
const std::string out_dir = GetChronoOutputPath() + "HMMWV9";
const std::string pov_dir = out_dir + "/POVRAY";

ChSpeedController m_speedPID;

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";
    chrono::cosimul::ChSocketFramework socket_tools;
    chrono::cosimul::ChCosimulation cosimul_interface(socket_tools,
        input_nb * 15,   // n.input values from Simulink
        1);

    GetLog() << " *** Waiting Simulink to start... *** \n     (load 'data/cosimulation/test_cosimulation.mdl' in "
        "Simulink and press Start...)\n\n";

    int PORTNUM = 50009;
    cosimul_interface.WaitConnection(PORTNUM);


    // --------------
    // Create systems
    // --------------

    // Create the HMMWV vehicle, set parameters, and initialize
    HMMWV_Reduced my_hmmwv;
    my_hmmwv.SetChassisFixed(false);
    my_hmmwv.SetChassisCollisionType(ChassisCollisionType::NONE);
    my_hmmwv.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    my_hmmwv.SetPowertrainType(powertrain_model);
    my_hmmwv.SetDriveType(DrivelineType::RWD);
    my_hmmwv.SetTireType(tire_model);
    my_hmmwv.SetTireStepSize(tire_step_size);
    my_hmmwv.SetVehicleStepSize(step_size);
    my_hmmwv.Initialize();

    m_speedPID.Reset(my_hmmwv.GetVehicle());
    m_speedPID.SetGains(1., 0., 0.);

    my_hmmwv.SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    my_hmmwv.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    my_hmmwv.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    my_hmmwv.SetWheelVisualizationType(VisualizationType::NONE);
    my_hmmwv.SetTireVisualizationType(VisualizationType::PRIMITIVES);



    /*HMMWV_Full trailor(my_hmmwv.GetSystem());
    trailor.SetChassisFixed(false);
    trailor.SetChassisCollisionType(ChassisCollisionType::PRIMITIVES);
    trailor.SetInitPosition(ChCoordsys<>(my_hmmwv.GetChassisBody()->GetPos() - ChVector<>(5., 0., 0), initRot));
    trailor.SetPowertrainType(powertrain_model);
    trailor.SetDriveType(DrivelineType::RWD);
    trailor.SetTireType(tire_model);
    trailor.SetTireStepSize(tire_step_size);
    trailor.SetVehicleStepSize(step_size);
    trailor.Initialize();

    trailor.SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    trailor.SetSuspensionVisualizationType(VisualizationType::NONE);
    trailor.SetSteeringVisualizationType(VisualizationType::NONE);
    trailor.SetWheelVisualizationType(VisualizationType::NONE);
    trailor.SetTireVisualizationType(VisualizationType::PRIMITIVES);

    auto lock_trailer_point = std::make_shared<ChLinkLockSpherical>();
    lock_trailer_point->Initialize(my_hmmwv.GetChassisBody(), trailor.GetChassisBody(), ChCoordsys<>(my_hmmwv.GetChassisBody()->GetPos() - ChVector<>(2., 0., 0.), Q_from_AngZ(0.)));
    my_hmmwv.GetSystem()->AddLink(lock_trailer_point);*/

    /*// Create the point of the trailor
    auto attach_point = std::make_shared<ChBodyEasySphere>(0.1, 1);
    attach_point->SetPos(my_hmmwv.GetChassis()->GetPointLocation(my_hmmwv.GetChassis()->GetBody()->GetPos() + ChVector<>(-2., 0., -1.5)));
    my_hmmwv.GetSystem()->Add(attach_point);

    // Link it to the Chassis
    auto attach_point_link = std::make_shared<ChLinkLockLock>();
    attach_point_link->Initialize(my_hmmwv.GetChassisBody(), attach_point, ChCoordsys<>(attach_point->GetPos()));
    my_hmmwv.GetSystem()->AddLink(attach_point_link);

    // Create a rod connected to the trailor
    auto rod = std::make_shared<ChBodyEasyBox>(0.5, 0.1, 0.1, 1);
    rod->SetPos(attach_point->GetPos() - ChVector<>(0.1 + 0.25, 0., 0.));
    rod->GetMaterialSurfaceNSC()->SetFriction(0.8f);
    my_hmmwv.GetSystem()->Add(rod);

    // Link it to the Attach point
    auto rod_link = std::make_shared<ChLinkLockSpherical>();
    rod_link->Initialize(attach_point, rod, ChCoordsys<>(attach_point->GetPos(), Q_from_AngZ(0.)));
    my_hmmwv.GetSystem()->AddLink(rod_link);

    // Create the Trailor
    auto trailor = std::make_shared<ChBodyEasyBox>(1., 0.7, 0.1, 1, true);
    trailor->SetPos(rod->GetPos() - ChVector<>(0.5/2 + 1./2, 0., 0.));
    my_hmmwv.GetSystem()->Add(trailor);

    // Link it to the rod
    auto trailor_link = std::make_shared<ChLinkLockLock>();
    trailor_link->Initialize(rod, trailor, ChCoordsys<>(rod->GetPos()));
    my_hmmwv.GetSystem()->AddLink(trailor_link);

    auto left_trailor_wheel_body = std::make_shared<ChBodyEasyCylinder>(0.3, 0.1, 1, true);
    left_trailor_wheel_body->SetPos(trailor->GetPos() - ChVector<>(0., 0.7 / 2 + 0.1 / 2, 0.));
    left_trailor_wheel_body->GetMaterialSurfaceNSC()->SetRollingFriction(1.f);
    my_hmmwv.GetSystem()->Add(left_trailor_wheel_body);

    // Link it to the Attach point
    auto left_wheel_link = std::make_shared<ChLinkLockRevolute>();
    left_wheel_link->Initialize(trailor, left_trailor_wheel_body, ChCoordsys<>(trailor->GetPos(), Q_from_AngY(0.)));
    my_hmmwv.GetSystem()->AddLink(left_wheel_link);


    //auto right_trailor_wheel = std::make_shared<ChWheel>("right_trailor");
    auto right_trailor_wheel_body = std::make_shared<ChBodyEasyCylinder>(0.3, 0.1, 1, true);
    right_trailor_wheel_body->SetPos(trailor->GetPos() + ChVector<>(0., 0.7 / 2 + 0.1 / 2, 0.));
    right_trailor_wheel_body->GetMaterialSurfaceNSC()->SetRollingFriction(1.f);
    my_hmmwv.GetSystem()->Add(right_trailor_wheel_body);

    // Link it to the Attach point
    auto right_wheel_link = std::make_shared<ChLinkLockRevolute>();
    right_wheel_link->Initialize(trailor, right_trailor_wheel_body, ChCoordsys<>(trailor->GetPos(), Q_from_AngX(0.)));
    my_hmmwv.GetSystem()->AddLink(right_wheel_link);*/


    // Create the terrain
    RigidTerrain terrain(my_hmmwv.GetSystem());
    auto patch = terrain.AddPatch(ChCoordsys<>(ChVector<>(0, 0, terrainHeight - 5), QUNIT),
                                  ChVector<>(terrainLength, terrainWidth, 10));
    patch->SetContactFrictionCoefficient(0.9f);
    patch->SetContactRestitutionCoefficient(0.01f);
    patch->SetContactMaterialProperties(2e7f, 0.3f);
    patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
    terrain.Initialize();

    // Create the vehicle Irrlicht interface
    ChWheeledVehicleIrrApp app(&my_hmmwv.GetVehicle(), &my_hmmwv.GetPowertrain(), L"HMMWV-9 Demo",
                               irr::core::dimension2d<irr::u32>(1000, 800), irr::ELL_NONE);
    app.SetSkyBox();
    app.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
    app.SetChaseCamera(trackPoint, 6.0, 0.5);
    app.SetTimestep(step_size);
    app.AssetBindAll();
    app.AssetUpdateAll();

    // Create the interactive driver system
    ChIrrGuiDriver driver(app);
    driver.SetInputMode(ChIrrGuiDriver::KEYBOARD);
    driver.Initialize();

    // Set the time response for steering and throttle keyboard inputs.
    double steering_time = 1.0;  // time to go from 0 to +1 (or from 0 to -1)
    double throttle_time = 1.0;  // time to go from 0 to +1
    double braking_time = 0.3;   // time to go from 0 to +1
    driver.SetSteeringDelta(render_step_size / steering_time);
    driver.SetThrottleDelta(render_step_size / throttle_time);
    driver.SetBrakingDelta(render_step_size / braking_time);

    // -----------------
    // Initialize output
    // -----------------

    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    if (povray_output) {
        if (!filesystem::create_directory(filesystem::path(pov_dir))) {
            std::cout << "Error creating directory " << pov_dir << std::endl;
            return 1;
        }
    }

    // Set up vehicle output
    my_hmmwv.GetVehicle().SetChassisOutput(true);
    my_hmmwv.GetVehicle().SetSuspensionOutput(0, true);
    my_hmmwv.GetVehicle().SetSteeringOutput(0, true);
    my_hmmwv.GetVehicle().SetOutput(ChVehicleOutput::ASCII, out_dir, "output", 0.1);

    // Generate JSON information with available output channels
    my_hmmwv.GetVehicle().ExportComponentList(out_dir + "/component_list.json");

    // ---------------
    // Simulation loop
    // ---------------

    // Number of simulation steps between two 3D view render frames
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Initialize simulation frame counter and simulation time
    ChRealtimeStepTimer realtime_timer;
    int step_number = 0;
    int render_frame = 0;
    double time = 0;

    // Driver location in vehicle local frame
    ChVector<> driver_pos = my_hmmwv.GetChassis()->GetLocalDriverCoordsys().pos;

    int histime = 0;
    double m_throttle = 0;
    double m_brake = 0;
    while (app.GetDevice()->run()) {
        // Receive data from Rtmaps
        if (cosimul_interface.ReceiveData(histime, &data_in))
        {
            int iteration = 0;
            bool next_data = false;
            do
            {
                histime = data_in(9 + 15 * iteration);
                data = data_in(13 + 15 * iteration);
                //GetLog() << "--- synchronization at time: " << histime << "\n\n";
                GetLog() << "value " << data/100 << "\n";
                next_data = data_in(14 + 15 * iteration) != 0;
                iteration++;
            } while (next_data);
        }

        time = my_hmmwv.GetSystem()->GetChTime();
        app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
        app.DrawAll();

        // Output POV-Ray data
        if (povray_output && step_number % render_steps == 0) {
            char filename[100];
            sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), render_frame + 1);
            utils::WriteShapesPovray(my_hmmwv.GetSystem(), filename);
            render_frame++;
        }

        // Collect output data from modules (for inter-module communication)
        //double throttle_input = driver.GetThrottle();
        double steering_input = driver.GetSteering();
        //double braking_input = driver.GetBraking();

        //data_out = throttle_input;
        //cosimul_interface.SendData(time, &data_out);

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        terrain.Synchronize(time);
        my_hmmwv.Synchronize(time, steering_input, m_brake, m_throttle, terrain);
        //trailor.Synchronize(time, 0., 0., 0., terrain);
        app.Synchronize(driver.GetInputModeAsString(), steering_input, m_throttle, m_brake);

        // Advance simulation for one timestep for all modules
        double step = enforce_soft_real_time ? realtime_timer.SuggestSimulationStep(step_size) : step_size;
        driver.Advance(step);
        terrain.Advance(step);
        my_hmmwv.Advance(step);
        //trailor.Advance(step);
        double out_speed = m_speedPID.Advance(my_hmmwv.GetVehicle(), data/100, step);
        ChClampValue(out_speed, -1.0, 1.0);

        GetLog() << "current speed" << my_hmmwv.GetVehicle().GetVehicleSpeed() << "\n";
        GetLog() << "outspeed " << out_speed << "\n";

        if (out_speed > 0)
        {
            // Vehicle moving too slow
            m_brake = 0;
            m_throttle = out_speed;
        }
        else if (m_throttle > 0.2)
        {
            // Vehicle moving too fast: reduce throttle
            m_brake = 0;
            m_throttle = 1 + out_speed;
        }
        else
        {
            // Vehicle moving too fast: apply brakes
            m_brake = -out_speed;
            m_throttle = 0;
        }
        app.Advance(step);

        // Increment frame number
        step_number++;

        app.EndScene();
    }

    return 0;
}
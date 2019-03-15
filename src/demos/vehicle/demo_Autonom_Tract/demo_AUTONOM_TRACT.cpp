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

#include "chrono_models/vehicle/tract/Tract.h"


#include "chrono/physics/ChBodyEasy.h"

//#include "chrono_cosimulation/ChCosimulation.h"
//#include "chrono_cosimulation/ChExceptionSocket.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::tract;
//using namespace chrono::cosimul;

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
double terrainWidth = 200.0;   // size in Y direction

// Point on chassis tracked by the camera
ChVector<> trackPoint(0.0, 0.0, .75);

// Simulation step sizes
double step_size = 0.005;
double tire_step_size = 0.001;
bool enforce_soft_real_time = true;

// Time interval between two render frames
int FPS = 50;
double render_step_size = 1.0 / FPS;  // FPS = 50

// POV-Ray output
bool povray_output = false;
const std::string out_dir = GetChronoOutputPath() + "HMMWV9";
const std::string pov_dir = out_dir + "/POVRAY";

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    //ChSocketFramework socket_tools;
    /*ChCosimulation cosimul_interface(socket_tools, 1, 1);
    ChMatrixDynamic<double> data_in(1, 1);
    ChMatrixDynamic<double> data_out(1, 1);
    int PORTNUM = 50009;
    cosimul_interface.WaitConnection(PORTNUM);*/


    // --------------
    // Create systems
    // --------------

    // Create the HMMWV vehicle, set parameters, and initialize
    TractModel tract;
    tract.SetChassisFixed(false);
    tract.SetChassisCollisionType(ChassisCollisionType::NONE);
    tract.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    tract.SetPowertrainType(powertrain_model);
    tract.SetDriveType(DrivelineType::RWD);
    tract.SetTireType(tire_model);
    tract.SetTireStepSize(tire_step_size);
    tract.SetVehicleStepSize(step_size);
    tract.Initialize();

    tract.SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    tract.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    tract.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    tract.SetWheelVisualizationType(VisualizationType::PRIMITIVES);
    tract.SetTireVisualizationType(VisualizationType::PRIMITIVES);


    // Create the terrain
    RigidTerrain terrain(tract.GetSystem());
    auto patch = terrain.AddPatch(ChCoordsys<>(ChVector<>(0, 0, terrainHeight - 5), QUNIT),
                                  ChVector<>(terrainLength, terrainWidth, 10));
    patch->SetContactFrictionCoefficient(0.9f);
    patch->SetContactRestitutionCoefficient(0.01f);
    patch->SetContactMaterialProperties(2e7f, 0.3f);
    patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
    terrain.Initialize();

    // Create the vehicle Irrlicht interface
    ChWheeledVehicleIrrApp app(&tract.GetVehicle(), &tract.GetPowertrain(), L"Tract Demo",
                               irr::core::dimension2d<irr::u32>(1000, 800), irr::ELL_NONE);
    app.SetSkyBox();
    app.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
    app.SetChaseCamera(trackPoint, 6.0, 0.5);
    app.SetTimestep(step_size);
    app.AssetBindAll();
    app.AssetUpdateAll();

    // Create the interactive driver system
    ChIrrGuiDriver driver(app);
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
    tract.GetVehicle().SetChassisOutput(true);
    tract.GetVehicle().SetSuspensionOutput(0, true);
    tract.GetVehicle().SetSteeringOutput(0, true);
    tract.GetVehicle().SetOutput(ChVehicleOutput::ASCII, out_dir, "output", 0.1);

    // Generate JSON information with available output channels
    tract.GetVehicle().ExportComponentList(out_dir + "/component_list.json");

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
    ChVector<> driver_pos = tract.GetChassis()->GetLocalDriverCoordsys().pos;

    while (app.GetDevice()->run()) {
        time = tract.GetSystem()->GetChTime();
        app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
        app.DrawAll();

        // Output POV-Ray data
        if (povray_output && step_number % render_steps == 0) {
            char filename[100];
            sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), render_frame + 1);
            utils::WriteShapesPovray(tract.GetSystem(), filename);
            render_frame++;
        }

        // Collect output data from modules (for inter-module communication)
        double throttle_input = driver.GetThrottle();
        double steering_input = driver.GetSteering();
        double braking_input = driver.GetBraking();

        /*data_out(0) = throttle_input;
        cosimul_interface.SendData(time, &data_out);*/

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        terrain.Synchronize(time);
        tract.Synchronize(time, steering_input, braking_input, throttle_input, terrain);
        app.Synchronize(driver.GetInputModeAsString(), steering_input, throttle_input, braking_input);

        // Advance simulation for one timestep for all modules
        double step = enforce_soft_real_time ? realtime_timer.SuggestSimulationStep(step_size) : step_size;
        driver.Advance(step);
        terrain.Advance(step);
        tract.Advance(step);
        app.Advance(step);

        // Increment frame number
        step_number++;

        app.EndScene();
    }

    return 0;
}
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

#include "chrono_vehicle/utils/ChVehiclePath.h"
#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/driver/ChPathFollowerDriverNavya.h"
#include "chrono_vehicle/output/ChVehicleOutputASCII.h"

#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"

#include "chrono_models/vehicle/tract/Tract.h"


#include "chrono/physics/ChBodyEasy.h"

#include "chrono_cosimulation/ChCosimulation.h"
#include "chrono_cosimulation/ChExceptionSocket.h"

#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_postprocess/ChGnuPlot.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::tract;
using namespace chrono::cosimul;

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
double terrainLength = 400.0;  // size in X direction
double terrainWidth = 5.0;   // size in Y direction

// Point on chassis tracked by the camera
ChVector<> trackPoint(0.0, 0.0, .75);

// Simulation step sizes
double step_size = 0.005;
double tire_step_size = 0.001;
bool enforce_soft_real_time = true;

// Time interval between two render frames
int FPS = 50;
double render_step_size = 1.0 / FPS;  // FPS = 50

// Plot result
const std::string out_dir = GetChronoOutputPath() + "DEMO_GNUPLOT";
std::string datafile = out_dir + "/speed_controller_plot.dat";
ChStreamOutAsciiFile mdatafile(datafile.c_str());

// =============================================================================
int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

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

    VisualizationType type = VisualizationType::PRIMITIVES;
    tract.SetChassisVisualizationType(type);
    tract.SetSuspensionVisualizationType(type);
    tract.SetSteeringVisualizationType(type);
    tract.SetWheelVisualizationType(type);
    tract.SetTireVisualizationType(type);


    // Create the terrain
    RigidTerrain terrain(tract.GetSystem());
    auto patch = terrain.AddPatch(ChCoordsys<>(ChVector<>(terrainLength/2 - 10, 0, 0), QUNIT),
                                  ChVector<>(terrainLength, terrainWidth, 1));
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
    auto path1 = StraightLinePath(ChVector<>(0, 0, 1), ChVector<>(400, 0, 1), 1);
    auto path2 = CirclePath(ChVector<>(1, 2, 0), 3.0, 5.0, true, 1);
    auto path3 = CirclePath(ChVector<>(1, 2, 0), 3.0, 5.0, false, 1);
    auto path4 = DoubleLaneChangePath(ChVector<>(-100, 0, 0.1), 28.93, 3.6105, 25.0, 100.0, true);
    auto path5 = DoubleLaneChangePath(ChVector<>(-100, 0, 0.1), 28.93, 3.6105, 25.0, 100.0, false);
    auto path6 = DoubleLaneChangePath(ChVector<>(-100, 0, 0.1), 13.5, 4.0, 11.0, 100.0, true);

    // Create the velocity profile
    double simu_duration = 0;
    ChFunction_Sequence sequence;
    std::shared_ptr<ChFunction_Const> const_speed_0 =
        std::make_shared<ChFunction_Const>();
    const_speed_0->Set_yconst(0.);
    sequence.InsertFunct(const_speed_0, 1.);
    simu_duration += 1.;
    std::shared_ptr<ChFunction_ConstAcc> acceleration =
        std::make_shared<ChFunction_ConstAcc>();
    acceleration->Set_h(1);
    acceleration->Set_av(0.5);
    acceleration->Set_aw(0.5);
    acceleration->Set_end(2.);
    simu_duration += 2.;
    sequence.InsertFunct(acceleration, acceleration->Get_end());
    std::shared_ptr<ChFunction_Const> const_speed_1 =
        std::make_shared<ChFunction_Const>();
    const_speed_1->Set_yconst(1.);
    sequence.InsertFunct(const_speed_1, 5.);
    simu_duration += 5.;

    ChPathFollowerDriverNavya driver(tract.GetVehicle(), path1, "Line", 10.);
    driver.GetSteeringController().SetLookAheadDistance(5);
    driver.GetSteeringController().SetGains(0.8, 0.2, 0);
    driver.GetSpeedController().SetGains(0.8, 0.2, 0);
    driver.Initialize();

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
    double steering_input = 0., braking_input = 0., throttle_input = 0.;

    while (app.GetDevice()->run() && time < simu_duration){
        time = tract.GetSystem()->GetChTime();
        app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
        app.DrawAll();

        driver.SetDesiredSpeed(sequence.Get_y(time));

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        steering_input = driver.GetSteering();
        throttle_input = driver.GetThrottle();
        braking_input = driver.GetBraking();
        terrain.Synchronize(time);
        tract.Synchronize(time, steering_input, braking_input, throttle_input, terrain);

        app.Synchronize(std::string("Input mode: KEY"), steering_input, throttle_input, braking_input);

        // Advance simulation for one timestep for all modules
        double step = enforce_soft_real_time ? realtime_timer.SuggestSimulationStep(step_size) : step_size;
        driver.Advance(step);
        terrain.Advance(step);
        tract.Advance(step);
        app.Advance(step);

        app.EndScene();


        mdatafile << time << ", " << sequence.Get_y(time)
            << ", " << tract.GetVehicle().GetVehicleSpeed() << "\n";
    }

    std::string filename = out_dir + "/speed_controller_plot.gpl";
    postprocess::ChGnuPlot mplot(filename.c_str());
    mplot.SetGrid();
    mplot.SetLabelX("x");
    mplot.SetLabelY("y");
    mplot.Plot("speed_controller_plot.dat", 1, 2, "Speed target", " with lines lt -1 lw 2");
    mplot.Plot("speed_controller_plot.dat", 1, 3, "Real speed", " with lines lt 2 lw 2");


    return 0;
}
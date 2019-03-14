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
// HMMWV rigid tire subsystem
//
// =============================================================================

#include <algorithm>

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_models/vehicle/tract/Tract_RigidTire.h"

namespace chrono {
namespace vehicle {
namespace tract {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double Tract_RigidTire::m_width = 0.254;

const double Tract_RigidTire::m_mass = 37.6;
const ChVector<> Tract_RigidTire::m_inertia(3.84, 6.69, 3.84);

const std::string Tract_RigidTire::m_meshName = "tract_tire_POV_geom";
const std::string Tract_RigidTire::m_meshFile = "tract/tract_tire.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Tract_RigidTire::Tract_RigidTire(const std::string& name, double radius, bool use_mesh)
    : ChRigidTire(name), m_radius(radius) {
    SetContactFrictionCoefficient(0.9f);
    SetContactRestitutionCoefficient(0.1f);
    SetContactMaterialProperties(2e7f, 0.3f);
    SetContactMaterialCoefficients(2e5f, 40.0f, 2e5f, 20.0f);

    if (use_mesh) {
        SetMeshFilename(GetDataFile("tract/tract_tire.obj"), 0.005);
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Tract_RigidTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        auto trimesh = std::make_shared<geometry::ChTriangleMeshConnected>();
        trimesh->LoadWavefrontMesh(vehicle::GetDataFile(m_meshFile), false, false);
        m_trimesh_shape = std::make_shared<ChTriangleMeshShape>();
        m_trimesh_shape->SetMesh(trimesh);
        m_trimesh_shape->SetName(m_meshName);
        m_trimesh_shape->SetStatic(true);
        m_wheel->AddAsset(m_trimesh_shape);
    } else {
        ChRigidTire::AddVisualizationAssets(vis);
    }
}

void Tract_RigidTire::RemoveVisualizationAssets() {
    ChRigidTire::RemoveVisualizationAssets();

    // Make sure we only remove the assets added by HMMWV_RigidTire::AddVisualizationAssets.
    // This is important for the ChTire object because a wheel may add its own assets
    // to the same body (the spindle/wheel).
    auto it = std::find(m_wheel->GetAssets().begin(), m_wheel->GetAssets().end(), m_trimesh_shape);
    if (it != m_wheel->GetAssets().end())
        m_wheel->GetAssets().erase(it);
}

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono

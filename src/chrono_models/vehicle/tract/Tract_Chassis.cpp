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

#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/tract/Tract_Chassis.h"

namespace chrono {
namespace vehicle {
namespace tract {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------



// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Tract_Chassis::Tract_Chassis(const std::string& name, const ChassisData& data, bool fixed, ChassisCollisionType chassis_collision_type)
    : ChRigidChassis(name, fixed), m_data(data) {
    m_data.inertia.SetElement(0, 0, m_data.inertiaXX.x());
    m_data.inertia.SetElement(1, 1, m_data.inertiaXX.y());
    m_data.inertia.SetElement(2, 2, m_data.inertiaXX.z());

    m_data.inertia.SetElement(0, 1, m_data.inertiaXY.x());
    m_data.inertia.SetElement(0, 2, m_data.inertiaXY.y());
    m_data.inertia.SetElement(1, 2, m_data.inertiaXY.z());
    m_data.inertia.SetElement(1, 0, m_data.inertiaXY.x());
    m_data.inertia.SetElement(2, 0, m_data.inertiaXY.y());
    m_data.inertia.SetElement(2, 1, m_data.inertiaXY.z());

    //// TODO:
    //// A more appropriate contact shape from primitives
    BoxShape box1(ChVector<>(0.0, 0.0, 0.1), ChQuaternion<>(1, 0, 0, 0), ChVector<>(2.0, 1.0, 0.2));

    m_has_primitives = true;
    m_vis_boxes.push_back(box1);

    m_has_mesh = true;
    m_vis_mesh_name = "tract_chassis_POV_geom";
    m_vis_mesh_file = "tract/tract_chassis.obj";

    m_has_collision = (chassis_collision_type != ChassisCollisionType::NONE);
    switch (chassis_collision_type) {
        case ChassisCollisionType::PRIMITIVES:
            m_coll_boxes.push_back(box1);
            break;
        case ChassisCollisionType::MESH:
            m_coll_mesh_names.push_back("tract/tract_chassis_simple.obj");
            break;
        default:
            break;
    }
}

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono

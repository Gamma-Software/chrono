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
// Authors: Radu Serban, Justin Madsen, Daniel Melanz
// =============================================================================
//
// Front and Rear HMMWV suspension subsystems (double A-arm)
//
// These concrete suspension subsystems are defined with respect to right-handed
// frames with X pointing towards the front, Y to the left, and Z up (as imposed
// by the base class ChDoubleWishbone) and origins at the midpoint between the
// lower control arms' connection points to the chassis.
//
// All point locations are provided for the left half of the suspension.
//
// =============================================================================

#pragma once

#include "chrono_vehicle/wheeled_vehicle/suspension/ChRigidSuspension.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace trailor {

/// @addtogroup vehicle_models_hmmwv
/// @{

/// Full double wishbone front suspension for the HMMWV vehicle.
/// The control arms are modeled using rigid bodies.
class CH_MODELS_API Trailor_RigidSuspensionFront : public ChRigidSuspension {
  public:
    Trailor_RigidSuspensionFront(const std::string& name);
    ~Trailor_RigidSuspensionFront();

    virtual double getSpindleMass() const override { return m_spindleMass; }

    virtual double getSpindleRadius() const override { return m_spindleRadius; }
    virtual double getSpindleWidth() const override { return m_spindleWidth; }

    virtual const ChVector<>& getSpindleInertia() const override { return m_spindleInertia; }

    virtual double getAxleInertia() const override { return m_axleInertia; }

  private:
    virtual const ChVector<> getLocation(PointId which) override;

    static const double m_spindleMass;

    static const double m_spindleRadius;
    static const double m_spindleWidth;

    static const ChVector<> m_spindleInertia;

    static const double m_axleInertia;
};

// -----------------------------------------------------------------------------

/// Full double wishbone rear suspension for the HMMWV vehicle.
/// The control arms are modeled using rigid bodies.
class CH_MODELS_API Trailor_RigidSuspensionRear : public ChRigidSuspension {
  public:
    Trailor_RigidSuspensionRear(const std::string& name);
    ~Trailor_RigidSuspensionRear();

    virtual double getSpindleMass() const override { return m_spindleMass; }

    virtual double getSpindleRadius() const override { return m_spindleRadius; }
    virtual double getSpindleWidth() const override { return m_spindleWidth; }

    virtual const ChVector<>& getSpindleInertia() const override { return m_spindleInertia; }

    virtual double getAxleInertia() const override { return m_axleInertia; }

  private:
    virtual const ChVector<> getLocation(PointId which) override;

    static const double m_spindleMass;

    static const double m_spindleRadius;
    static const double m_spindleWidth;

    static const ChVector<> m_spindleInertia;

    static const double m_axleInertia;
};

/// @} vehicle_models_hmmwv

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono

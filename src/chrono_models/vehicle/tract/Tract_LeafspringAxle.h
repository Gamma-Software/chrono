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
// Tract leafspring axle.
//
// =============================================================================

#pragma once

#include "chrono_vehicle/wheeled_vehicle/suspension/ChLeafspringAxle.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace tract {

struct SuspensionData
{
    double axleTubeMass = 124.0;
    double spindleMass = 14.705;
    double axleTubeRadius = 0.0476;
    double spindleRadius = 0.10;
    double spindleWidth = 0.06;

    ChVector<> axleTubeInertia = ChVector<>(22.21, 0.0775, 22.21);
    ChVector<> spindleInertia = ChVector<>(0.04117, 0.07352, 0.04117);

    double springDesignLength = 0.2;
    double springCoefficient = 102643.885771329;
    double springRestLength = 0.2 + 0.0621225507207084;
    double springMinLength = 0.2 - 0.05;
    double springMaxLength = 0.2 + 0.1;
    double damperCoefficient = 16336.2817986669;
    double damperDegressivityCompression = 3.0;
    double damperDegressivityExpansion = 1.0;
    double axleShaftInertia = 0.4;

    double wheelTrack = 1.14;
};

/// @addtogroup vehicle_models_uaz
/// @{

/// Leafspring axle subsystem for the uaz vehicle.

/// @} vehicle_models_uaz
class CH_MODELS_API Tract_LeafspringAxle : public ChLeafspringAxle {
  public:
    Tract_LeafspringAxle(const std::string& name, const SuspensionData& data);
    ~Tract_LeafspringAxle();


  protected:
    virtual const ChVector<> getLocation(PointId which) override;

    virtual double getAxleTubeMass() const override { return m_data.axleTubeMass; }
    virtual double getSpindleMass() const override { return m_data.spindleMass; }

    virtual double getAxleTubeRadius() const override { return m_data.axleTubeRadius; }
    virtual double getSpindleRadius() const override { return m_data.spindleRadius; }
    virtual double getSpindleWidth() const override { return m_data.spindleWidth; }

    virtual const ChVector<> getAxleTubeCOM() const override { return ChVector<>(0,0,0); }

    virtual const ChVector<>& getAxleTubeInertia() const override { return m_data.axleTubeInertia; }
    virtual const ChVector<>& getSpindleInertia() const override { return m_data.spindleInertia; }

    virtual double getAxleInertia() const override { return m_data.axleShaftInertia; }

    virtual double getSpringRestLength() const override { return m_data.springRestLength; }
    /// Return the functor object for spring force.
    virtual ChLinkSpringCB::ForceFunctor* getSpringForceFunctor() const override { return m_springForceCB; }
    /// Return the functor object for shock force.
    virtual ChLinkSpringCB::ForceFunctor* getShockForceFunctor() const override { return m_shockForceCB; }

  private:
    ChLinkSpringCB::ForceFunctor* m_springForceCB;
    ChLinkSpringCB::ForceFunctor* m_shockForceCB;

    SuspensionData m_data;
};

/// @} vehicle_models_uaz

}  // end namespace uaz
}  // end namespace vehicle
}  // end namespace chrono
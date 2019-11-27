// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/feature/imageDescriberCommon.hpp>
#include <aliceVision/image/pixelTypes.hpp>
#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/stl/FlatMap.hpp>
#include <aliceVision/types.hpp>

namespace aliceVision
{
namespace sfmData
{

/**
 * @brief 2D observation of a 3D landmark.
 */
struct Observation
{
    Observation()
        : id_feat(UndefinedIndexT)
    {
        m_RSME = 0;
    }
    Observation(const Vec2& p, IndexT idFeat)
        : x(p)
        , id_feat(idFeat)
    {
        m_RSME = 0;
    }
    /*~Observation()
    { 
        if(m_ObservationResidual != NULL)
        {
            delete m_ObservationResidual;
        }
        
    }*/

    Vec2 x;
    IndexT id_feat;

    bool m_RSMECalculated = false;
    double m_RSME = 0;              // calc by statistics
    Vec2* m_ObservationResidual = new Eigen::Matrix<double, 2, 1, 0>; // cal by statistics

    bool operator==(const Observation& other) const
    {
        return AreVecNearEqual(x, other.x, 1e-6) && id_feat == other.id_feat;
    }

public:
    void SetRSME(Vec2* residual, double* rsme)
    {
        m_ObservationResidual = residual;
        m_RSME = *rsme;
        m_RSMECalculated = true;
    }

	void SetObservations(Vec2& observation) 
	{
		m_ObservationResidual = &observation;
	}
};

/// Observations are indexed by their View_id
typedef stl::flat_map<IndexT, Observation> Observations;

/**
 * @brief Landmark is a 3D point with its 2d observations.
 */
struct Landmark
{
    Landmark() = default;
    explicit Landmark(feature::EImageDescriberType descType)
        : descType(descType)
    {
        m_RSMECalculated = false;
        m_RSME = 0;
    }
    Landmark(const Vec3& pos3d, feature::EImageDescriberType descType = feature::EImageDescriberType::UNINITIALIZED,
             const Observations& observations = Observations(), const image::RGBColor& color = image::WHITE)
        : X(pos3d)
        , descType(descType)
        , observations(observations)
        , rgb(color)
    {
        m_RSMECalculated = false;
        m_RSME = 0;
    }

    Vec3 X;
    feature::EImageDescriberType descType = feature::EImageDescriberType::UNINITIALIZED;
    Observations observations;
    image::RGBColor rgb = image::WHITE; //!> the color associated to the point

    // RMSE
    bool m_RSMECalculated;
    double m_RSME;

    bool operator==(const Landmark& other) const
    {
        return AreVecNearEqual(X, other.X, 1e-3) && AreVecNearEqual(rgb, other.rgb, 1e-3) &&
               observations == other.observations && descType == other.descType;
    }

public:
    void SetRSME(double* rsme)
    {
        m_RSME = *rsme;
        m_RSMECalculated = true;
    }
};

} // namespace sfmData
} // namespace aliceVision

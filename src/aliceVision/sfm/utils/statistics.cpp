// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "statistics.hpp"
#include <aliceVision/sfmData/SfMData.hpp>

namespace aliceVision
{
namespace sfm
{

double RMSE(sfmData::SfMData& sfmData)
{    
   
    // Compute residuals for each observation
    std::vector<double> vec; // for all landmarks all residuals

    std::vector<double> vecLandMark; // stores for every landmark the residuals
    for(sfmData::Landmarks::iterator iterTracks = sfmData.getLandmarks().begin();
        iterTracks != sfmData.getLandmarks().end(); ++iterTracks)
    {
        sfmData::Observations& obs = iterTracks->second.observations;
        for(sfmData::Observations::iterator itObs = obs.begin(); itObs != obs.end(); ++itObs)
        {
            const sfmData::View* view = sfmData.getViews().find(itObs->first)->second.get();
            const geometry::Pose3 pose = sfmData.getPose(*view).getTransform();
            const std::shared_ptr<camera::IntrinsicBase> intrinsic = sfmData.getIntrinsics().at(view->getIntrinsicId());
            Vec2 residual = intrinsic->residual(pose, iterTracks->second.X, itObs->second.x);

            vec.push_back(residual(0));
            vec.push_back(residual(1));

            // generate RSME for observation point
            std::vector<double> resiVec;
            resiVec.push_back(residual(0));
            resiVec.push_back(residual(1));

            // Store landmark array
            vecLandMark.push_back(residual(0));
            vecLandMark.push_back(residual(1));

            *itObs->second.m_ObservationResidual = residual;
            itObs->second.m_RSME = CalculateRMSE(resiVec);
        } // End loop over observation

        iterTracks->second.m_RSME = CalculateRMSE(vecLandMark);

        vecLandMark.clear(); // remove residuals for next landmark
    }                        // end loop for landmarks

    // calculate for pose/view for RMSE

    // store in the hashmap all residuals per view
    IndexT residualCount = 0;
    HashMap<IndexT, std::vector<double>> residuals_per_view;
    for(sfmData::Landmarks::const_iterator iterTracks = sfmData.getLandmarks().begin();
        iterTracks != sfmData.getLandmarks().end(); ++iterTracks)
    {
        const sfmData::Observations& observations = iterTracks->second.observations;
        for(sfmData::Observations::const_iterator itObs = observations.begin(); itObs != observations.end(); ++itObs)
        {
            //if(!itObs->second.m_RSMECalculated)
            //{
            //    continue;
            //}
           
            Vec2 vec2 = *itObs->second.m_ObservationResidual;

            residuals_per_view[itObs->first].push_back(vec2(0));
            residuals_per_view[itObs->first].push_back(vec2(1));
            ++residualCount;
        }
    }

    // iterate over view || generate for every pose and RMSE value
    for(sfmData::Views::const_iterator iterV = sfmData.getViews().begin(); iterV != sfmData.getViews().end(); ++iterV)
    {
        const sfmData::View* v = iterV->second.get();
        const IndexT id_view = v->getViewId();
        
        if(residuals_per_view.find(id_view) != residuals_per_view.end())
        {
            std::vector<double>& residuals = residuals_per_view.at(id_view);
            if(!residuals.empty())
            {
                sfmData::CameraPose& cameraPose = sfmData.getPoses().at(v->getPoseId());
                double RMSE = CalculateRMSE(residuals);

                cameraPose.SetRSME(RMSE);
            }
        }
    }



    const Eigen::Map<Eigen::RowVectorXd> residuals(&vec[0], vec.size());
    const double RMSE = std::sqrt(residuals.squaredNorm() / vec.size());

    return RMSE;
}

} // namespace sfm
} // namespace aliceVision

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
    std::vector<double> residualsAllLandmarks; // for all landmarks all residuals

    std::vector<double> vecLandMark; // stores for every landmark the residuals

    // get landmarks
    sfmData::Landmarks& landmarks = sfmData.getLandmarks();

    //store residuals for each observatoin
    std::vector<double> residualVecObs;
    Vec2 residual; //temporary storing of them

	//Foreach Landmark
#pragma omp parallel for // OpenMP
    for(int i = 0; i < landmarks.size(); i++)
    {
        sfmData::Observations& obs = landmarks[i].observations;

        // foreach Observation
        for(int o = 0; o < obs.size(); o++)
        {
            const sfmData::View* view = &sfmData.getView(obs[i].id_feat);
			const geometry::Pose3 pose = sfmData.getPose(*view).getTransform();
            const std::shared_ptr<camera::IntrinsicBase> intrinsic = sfmData.getIntrinsics().at(view->getIntrinsicId());
            residual = intrinsic->residual(pose, landmarks[i].X, obs[i].x);

            residualsAllLandmarks.push_back(residual(0));
            residualsAllLandmarks.push_back(residual(1));

            // generate RSME for observation point
            residualVecObs.push_back(residual(0));
            residualVecObs.push_back(residual(1));

            // Store landmark array
            vecLandMark.push_back(residual(0));
            vecLandMark.push_back(residual(1));

			obs[i].SetObservations(residual);
            //*itObs->second.m_ObservationResidual = residual;
            obs[i].m_RSME = CalculateRMSE(residualVecObs);

            residualVecObs.clear();
        } // End loop over observation

        //calculate landmark RMSE
		landmarks[i].m_RSME = CalculateRMSE(vecLandMark);

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
            // if(!itObs->second.m_RSMECalculated)
            //{
            //    continue;
            //}

            Vec2 vec2 = *itObs->second.m_ObservationResidual;

            residuals_per_view[itObs->first].push_back(vec2(0));
            residuals_per_view[itObs->first].push_back(vec2(1));
            ++residualCount;
        }
    }

	
	sfmData::Views& views = sfmData.getViews();
    double tempRMSE = 0;

	#pragma omp parallel for // OpenMP
    for(int i = 0; i < views.size(); i++)     // iterate over view || generate for every pose and RMSE value
    {
        const sfmData::View& v = *views.at(i);
        const IndexT id_view = v.getViewId();

        if(residuals_per_view.find(id_view) != residuals_per_view.end())
        {
            std::vector<double>& residuals = residuals_per_view.at(id_view);
            if(!residuals.empty())
            {
                sfmData::CameraPose& cameraPose = sfmData.getPoses().at(v.getPoseId());
                tempRMSE = CalculateRMSE(residuals);

                cameraPose.SetRSME(tempRMSE);
            }
        }
    }

    const Eigen::Map<Eigen::RowVectorXd> residuals(&residualsAllLandmarks[0], residualsAllLandmarks.size());
    const double RMSE = std::sqrt(residuals.squaredNorm() / residualsAllLandmarks.size());
    return RMSE;
}

} // namespace sfm
} // namespace aliceVision

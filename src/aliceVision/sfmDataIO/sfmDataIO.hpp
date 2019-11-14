// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/sfmData/SfMData.hpp>

namespace aliceVision {
namespace sfmDataIO {

enum ESfMData
{
  VIEWS                      = 1,
  EXTRINSICS                 = 2,
  INTRINSICS                 = 4,
  STRUCTURE                  = 8,
  OBSERVATIONS               = 16,
  OBSERVATIONS_WITH_FEATURES = 32,
  CONTROL_POINTS             = 64,
  LANDMARKS_UNCERTAINTY      = 128,
  POSES_UNCERTAINTY          = 256,

  UNCERTAINTY = LANDMARKS_UNCERTAINTY | POSES_UNCERTAINTY,
  ALL_DENSE = VIEWS | EXTRINSICS | INTRINSICS | STRUCTURE | OBSERVATIONS | CONTROL_POINTS,
  ALL = VIEWS | EXTRINSICS | INTRINSICS | STRUCTURE | OBSERVATIONS_WITH_FEATURES | CONTROL_POINTS | UNCERTAINTY
};

/// check that each pose have a valid intrinsic and pose id in the existing View ids
bool ValidIds(const sfmData::SfMData& sfmData, ESfMData partFlag);

/// load SfMData SfM scene from a file
bool Load(sfmData::SfMData& sfmData, const std::string& filename, ESfMData partFlag);

/// save SfMData SfM scene to a file
bool Save(const sfmData::SfMData& sfmData, const std::string& filename, ESfMData partFlag);

bool SaveStatisticFile(const sfmData::SfMData& sfmData, const std::string& filename);

} // namespace sfmDataIO
} // namespace aliceVision

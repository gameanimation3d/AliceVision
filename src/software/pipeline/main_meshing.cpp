// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmData/colorize.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/fuseCut/LargeScale.hpp>
#include <aliceVision/fuseCut/ReconstructionPlan.hpp>
#include <aliceVision/fuseCut/DelaunayGraphCut.hpp>
#include <aliceVision/mesh/meshPostProcessing.hpp>
#include <aliceVision/mvsData/Point3d.hpp>
#include <aliceVision/mvsData/Rgb.hpp>
#include <aliceVision/mvsData/StaticVector.hpp>
#include <aliceVision/mvsUtils/common.hpp>
#include <aliceVision/mvsUtils/MultiViewParams.hpp>
#include <aliceVision/mvsUtils/fileIO.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/Timer.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 3
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace fs = boost::filesystem;
namespace po = boost::program_options;

enum EPartitioningMode
{
    ePartitioningUndefined = 0,
    ePartitioningSingleBlock = 1,
    ePartitioningAuto = 2,
};

EPartitioningMode EPartitioning_stringToEnum(const std::string& s)
{
    if(s == "singleBlock")
        return ePartitioningSingleBlock;
    if(s == "auto")
        return ePartitioningAuto;
    return ePartitioningUndefined;
}

inline std::istream& operator>>(std::istream& in, EPartitioningMode& out_mode)
{
    std::string s;
    in >> s;
    out_mode = EPartitioning_stringToEnum(s);
    return in;
}

enum ERepartitionMode
{
    eRepartitionUndefined = 0,
    eRepartitionMultiResolution = 1,
    eRepartitionRegularGrid = 2,
};

ERepartitionMode ERepartitionMode_stringToEnum(const std::string& s)
{
    if(s == "multiResolution")
        return eRepartitionMultiResolution;
    if(s == "regularGrid")
        return eRepartitionRegularGrid;
    return eRepartitionUndefined;
}

inline std::istream& operator>>(std::istream& in, ERepartitionMode& out_mode)
{
    std::string s;
    in >> s;
    out_mode = ERepartitionMode_stringToEnum(s);
    return in;
}
inline float round_nplaces(float& value, int to)
{
    float places = pow(10.0, to);
    return round(value * places) / places;
}

inline double round_nplaces(const double& value, int to)
{
    double places = pow(10.0, to);
    return round(value * places) / places;
}
inline bool CheckIfPoseTheSame(const Point3d& point3D, const Vec3& landmark)
{
    double not_1 = point3D.x;
    double not_2 = landmark[0];
    
    double round_1 = round_nplaces(point3D.x, 6);
    double round_2 = round_nplaces(landmark[0], 6);
    double difference = round_1 - round_2;

    if(round_nplaces(point3D.x, 6) - round_nplaces(landmark[0], 6) >= 0.00001f)
    {
        return false;
    }

     not_1 = point3D.y;
     not_2 = landmark[1];

     round_1 = round_nplaces(point3D.y, 6);
     round_2 = round_nplaces(landmark[1], 6);
     difference = round_1 - round_2;


    if(round_nplaces(point3D.y, 6) - round_nplaces(landmark[1], 6) >= 0.00001f)
    {
        return false;
    }

    not_1 = point3D.z;
    not_2 = landmark[2];

    round_1 = round_nplaces(point3D.z, 6);
    round_2 = round_nplaces(landmark[2], 6);
    difference = round_1 - round_2;

    if(round_nplaces(point3D.z, 6) - round_nplaces(landmark[2], 6) >= 0.00001f)
    {
        return false;
    }
    return true;
}

bool FindLandmarkForVertexinHashMap(HashMap<std::string, std::vector<sfmData::Landmark>>& landmarkPerX, Point3d& point,
                                    std::string inputKey)
{
    if(landmarkPerX.find(inputKey) != landmarkPerX.end())
    {
        std::vector<sfmData::Landmark>& pointInSameXspace = landmarkPerX.at(inputKey);
        for(int j = 0; j < pointInSameXspace.size(); ++j)
        {
            if(CheckIfPoseTheSame(point, pointInSameXspace[j].X))
            {
                point.id = pointInSameXspace[j].m_RawIndex;
                return true;
            }
        }
        // counter++;
    }

    return false;
}

int CheckIfMatchesAreWorking(sfmData::Landmarks& landmarks, std::vector<Point3d>& data)
{
    int counter = 0;
    HashMap<std::string, std::vector<sfmData::Landmark>> landmarkPerX;
    bool hasBeenFound = false;
    int numberOfDigitsForHashMapKey = 3;
    float valueToChangeIfNotFound = 0.001f;

    float convertedDouble = 0;
    //#pragma omp parallel for
    for(int i = 0; i < landmarks.size(); ++i)
    {
        landmarks[i].m_RawIndex = i;
        convertedDouble = landmarks[i].X[0];
        convertedDouble = round_nplaces(convertedDouble, numberOfDigitsForHashMapKey);
        landmarkPerX[std::to_string(convertedDouble)].push_back(landmarks[i]);

        // landmarkPerX.insert(std::to_string(landmarks[i].X[0]), landmarks[i]);
    }
    std::string hashKey = "";

    //#pragma omp parallel for
    // iterate over vertices
    Point3d point = data[0];
    for(int i = 0; i < data.size(); ++i)
    {
        hasBeenFound = false;
        point = data[i];

        if(i >= 7905 && i < 7907)
        {
            hasBeenFound = false;
        }

        if(!CheckIfPoseTheSame(data[i], landmarks[data[i].id].X))
        {
            hashKey = std::to_string(round_nplaces(data[i].x, numberOfDigitsForHashMapKey));

            hasBeenFound = FindLandmarkForVertexinHashMap(landmarkPerX, data[i], hashKey);

            // Upper positive element of hasmap
            if(!hasBeenFound)
            {
                hashKey =
                    std::to_string(round_nplaces(data[i].x + valueToChangeIfNotFound, numberOfDigitsForHashMapKey));
                hasBeenFound = FindLandmarkForVertexinHashMap(landmarkPerX, data[i], hashKey);
            }

            // negative element of hasmap
            if(!hasBeenFound)
            {
                hashKey =
                    std::to_string(round_nplaces(data[i].x - valueToChangeIfNotFound, numberOfDigitsForHashMapKey));
                hasBeenFound = FindLandmarkForVertexinHashMap(landmarkPerX, data[i], hashKey);
            }

            // file = std::to_string(data[i].x);

            if(!hasBeenFound)
            {
                counter++;
            }
        }
        // ALICEVISION_LOG_INFO(std::to_string(counter) + " vs " + std::to_string(mesh->pts->size()));
    }

    return counter;
}
/// Create a dense SfMData based on reference \p sfmData,
/// using \p vertices as landmarks and \p ptCams as observations
void createDenseSfMData(const sfmData::SfMData& sfmData, const mvsUtils::MultiViewParams& mp,
                        const std::vector<Point3d>& vertices, const StaticVector<StaticVector<int>*>& ptsCams,
                        sfmData::SfMData& outSfmData)
{
    outSfmData = sfmData;
    outSfmData.getLandmarks().clear();

    for(std::size_t i = 0; i < vertices.size(); ++i)
    {
        const Point3d& point = vertices.at(i);
        const Vec3 pt3D(point.x, point.y, point.z);
        sfmData::Landmark landmark(pt3D, feature::EImageDescriberType::UNKNOWN);
        // set landmark observations from ptsCams if any
        if(!ptsCams[i]->empty())
        {
            for(int cam : *(ptsCams[i]))
            {
                const sfmData::View& view = sfmData.getView(mp.getViewId(cam));
                const camera::IntrinsicBase* intrinsicPtr = sfmData.getIntrinsicPtr(view.getIntrinsicId());
                const sfmData::Observation observation(
                    intrinsicPtr->project(sfmData.getPose(view).getTransform(), pt3D, true),
                    UndefinedIndexT); // apply distortion
                landmark.observations[view.getViewId()] = observation;
                // landmark.m_RawIndex = i; //setting old index
            }
        }
        outSfmData.getLandmarks()[i] = landmark;
    }
}

/// Remove all landmarks without observations from \p sfmData.
void removeLandmarksWithoutObservations(sfmData::SfMData& sfmData)
{
    auto& landmarks = sfmData.getLandmarks();
    for(auto it = landmarks.begin(); it != landmarks.end();)
    {
        if(it->second.observations.empty())
            it = landmarks.erase(it);
        else
            ++it;
    }
}

int main(int argc, char* argv[])
{
    system::Timer timer;

    std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
    std::string sfmDataFilename;
    std::string outputMesh;
    std::string outputDensePointCloud;
    std::string depthMapsFolder;
    std::string depthMapsFilterFolder;
    EPartitioningMode partitioningMode = ePartitioningSingleBlock;
    ERepartitionMode repartitionMode = eRepartitionMultiResolution;
    std::size_t estimateSpaceMinObservations = 3;
    float estimateSpaceMinObservationAngle = 10.0f;
    double universePercentile = 0.999;
    int maxPtsPerVoxel = 6000000;
    bool meshingFromDepthMaps = true;
    bool estimateSpaceFromSfM = true;
    bool addLandmarksToTheDensePointCloud = false;
    bool saveRawDensePointCloud = false;
    bool colorizeOutput = false;
    std::string LandmarkMatchingFilePath;

    fuseCut::FuseParams fuseParams;

    sfmDataFilename =
        "M:/Repo/GitRepos/SFMVisualizer_dev_Triangulation/external/SFM/aliceVision/data/cloud_and_poses.sfm";
    outputMesh = "M:/Repo/GitRepos/SFMVisualizer_dev_Triangulation/external/SFM/aliceVision/data/sparseMesh.obj";
    LandmarkMatchingFilePath =
        "M:/Repo/GitRepos/SFMVisualizer_dev_Triangulation/external/SFM/aliceVision/data/landmarkMatches.json";
    colorizeOutput = true;

    // po::options_description allParams("AliceVision meshing");

    // po::options_description requiredParams("Required parameters");
    // requiredParams.add_options()
    //    ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
    //      "SfMData file.")
    //    //("output,o", po::value<std::string>(&outputDensePointCloud)->required(),
    //    //  "Output Dense SfMData file.")
    //    ("outputMesh,o", po::value<std::string>(&outputMesh)->required(),
    //      "Output mesh (OBJ file format).");

    // po::options_description optionalParams("Optional parameters");
    // optionalParams.add_options()
    //    ("depthMapsFolder", po::value<std::string>(&depthMapsFolder),
    //        "Input depth maps folder.")
    //    ("depthMapsFilterFolder", po::value<std::string>(&depthMapsFilterFolder),
    //        "Input filtered depth maps folder.")
    //    ("maxInputPoints", po::value<int>(&fuseParams.maxInputPoints)->default_value(fuseParams.maxInputPoints),
    //        "Max input points loaded from images.")
    //    ("maxPoints", po::value<int>(&fuseParams.maxPoints)->default_value(fuseParams.maxPoints),
    //        "Max points at the end of the depth maps fusion.")
    //    ("maxPointsPerVoxel", po::value<int>(&maxPtsPerVoxel)->default_value(maxPtsPerVoxel),
    //        "Max points per voxel.")
    //    ("minStep", po::value<int>(&fuseParams.minStep)->default_value(fuseParams.minStep),
    //        "The step used to load depth values from depth maps is computed from maxInputPts. Here we define the
    //        minimal value for this step, " "so on small datasets we will not spend too much time at the beginning
    //        loading all depth values.")
    //    ("simFactor", po::value<float>(&fuseParams.simFactor)->default_value(fuseParams.simFactor),
    //        "simFactor")
    //    ("angleFactor", po::value<float>(&fuseParams.angleFactor)->default_value(fuseParams.angleFactor),
    //        "angleFactor")
    //    ("partitioning", po::value<EPartitioningMode>(&partitioningMode)->default_value(partitioningMode),
    //        "Partitioning: 'singleBlock' or 'auto'.")
    //    ("repartition", po::value<ERepartitionMode>(&repartitionMode)->default_value(repartitionMode),
    //        "Repartition: 'multiResolution' or 'regularGrid'.")
    //    ("estimateSpaceFromSfM", po::value<bool>(&estimateSpaceFromSfM)->default_value(estimateSpaceFromSfM),
    //        "Estimate the 3d space from the SfM.")
    //    ("addLandmarksToTheDensePointCloud",
    //    po::value<bool>(&addLandmarksToTheDensePointCloud)->default_value(addLandmarksToTheDensePointCloud),
    //        "Add SfM Landmarks into the dense point cloud (created from depth maps). If only the SfM is provided
    //        in input, SfM landmarks will be used regardless of this option.")
    //    ("colorizeOutput", po::value<bool>(&colorizeOutput)->default_value(colorizeOutput),
    //    "Whether to colorize output dense point cloud and mesh.")
    //    ("landmarkMatchesFile", po::value<std::string>(&LandmarkMatchingFilePath),
    //    "Export Landmark Matching File finding the Landmark in the OBJ by index.");

    // po::options_description advancedParams("Advanced parameters");
    // advancedParams.add_options()
    //    ("universePercentile", po::value<double>(&universePercentile)->default_value(universePercentile),
    //        "universe percentile")
    //    ("estimateSpaceMinObservations",
    //    po::value<std::size_t>(&estimateSpaceMinObservations)->default_value(estimateSpaceMinObservations),
    //        "Minimum number of observations for SfM space estimation.")
    //    ("estimateSpaceMinObservationAngle",
    //    po::value<float>(&estimateSpaceMinObservationAngle)->default_value(estimateSpaceMinObservationAngle),
    //        "Minimum angle between two observations for SfM space estimation.")
    //    ("pixSizeMarginInitCoef",
    //    po::value<double>(&fuseParams.pixSizeMarginInitCoef)->default_value(fuseParams.pixSizeMarginInitCoef),
    //        "pixSizeMarginInitCoef")
    //    ("pixSizeMarginFinalCoef",
    //    po::value<double>(&fuseParams.pixSizeMarginFinalCoef)->default_value(fuseParams.pixSizeMarginFinalCoef),
    //        "pixSizeMarginFinalCoef")
    //    ("voteMarginFactor",
    //    po::value<float>(&fuseParams.voteMarginFactor)->default_value(fuseParams.voteMarginFactor),
    //        "voteMarginFactor")
    //    ("contributeMarginFactor",
    //    po::value<float>(&fuseParams.contributeMarginFactor)->default_value(fuseParams.contributeMarginFactor),
    //        "contributeMarginFactor")
    //    ("simGaussianSizeInit",
    //    po::value<float>(&fuseParams.simGaussianSizeInit)->default_value(fuseParams.simGaussianSizeInit),
    //        "simGaussianSizeInit")
    //    ("simGaussianSize",
    //    po::value<float>(&fuseParams.simGaussianSize)->default_value(fuseParams.simGaussianSize),
    //        "simGaussianSize")
    //    ("minAngleThreshold",
    //    po::value<double>(&fuseParams.minAngleThreshold)->default_value(fuseParams.minAngleThreshold),
    //        "minAngleThreshold")
    //    ("refineFuse", po::value<bool>(&fuseParams.refineFuse)->default_value(fuseParams.refineFuse),
    //        "refineFuse")
    //    ("saveRawDensePointCloud",
    //    po::value<bool>(&saveRawDensePointCloud)->default_value(saveRawDensePointCloud),
    //        "Save dense point cloud before cut and filtering.");

    // po::options_description logParams("Log parameters");
    // logParams.add_options()
    //  ("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
    //    "verbosity level (fatal, error, warning, info, debug, trace).");

    // allParams.add(requiredParams).add(optionalParams).add(advancedParams).add(logParams);

    // po::variables_map vm;

    // try
    //{
    //  po::store(po::parse_command_line(argc, argv, allParams), vm);

    //  if(vm.count("help") || (argc == 1))
    //  {
    //    ALICEVISION_COUT(allParams);
    //    return EXIT_SUCCESS;
    //  }

    //  po::notify(vm);
    //}
    // catch(boost::program_options::required_option& e)
    //{
    //  ALICEVISION_CERR("ERROR: " << e.what() << std::endl);
    //  ALICEVISION_COUT("Usage:\n\n" << allParams);
    //  return EXIT_FAILURE;
    //}
    // catch(boost::program_options::error& e)
    //{
    //  ALICEVISION_CERR("ERROR: " << e.what() << std::endl);
    //  ALICEVISION_COUT("Usage:\n\n" << allParams);
    //  return EXIT_FAILURE;
    //}

    // ALICEVISION_COUT("Program called with the following parameters:");
    // ALICEVISION_COUT(vm);

    // set verbose level
    system::Logger::get()->setLogLevel(verboseLevel);

    if(depthMapsFolder.empty() || depthMapsFilterFolder.empty())
    {
        if(depthMapsFolder.empty() && depthMapsFilterFolder.empty() && repartitionMode == eRepartitionMultiResolution &&
           partitioningMode == ePartitioningSingleBlock)
        {
            meshingFromDepthMaps = false;
            addLandmarksToTheDensePointCloud = true;
        }
        else
        {
            ALICEVISION_LOG_ERROR(
                "Invalid input options:\n"
                "- Meshing from depth maps require --depthMapsFolder and --depthMapsFilterFolder options.\n"
                "- Meshing from SfM require option --partitioning set to 'singleBlock' and option --repartition "
                "set to "
                "'multiResolution'.");
            return EXIT_FAILURE;
        }
    }

    // read the input SfM scene
    sfmData::SfMData sfmData;
    if(!sfmDataIO::Load(sfmData, sfmDataFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilename << "' cannot be read.");
        return EXIT_FAILURE;
    }

    // initialization
    mvsUtils::MultiViewParams mp(sfmData, "", depthMapsFolder, depthMapsFilterFolder, meshingFromDepthMaps);

    mp.userParams.put("LargeScale.universePercentile", universePercentile);

    int ocTreeDim = mp.userParams.get<int>("LargeScale.gridLevel0", 1024);
    const auto baseDir = mp.userParams.get<std::string>("LargeScale.baseDirName", "root01024");

    fs::path outDirectory = fs::path(outputMesh).parent_path();
    if(!fs::is_directory(outDirectory))
        fs::create_directory(outDirectory);

    fs::path tmpDirectory = outDirectory / "tmp";

    ALICEVISION_LOG_WARNING("repartitionMode: " << repartitionMode);
    ALICEVISION_LOG_WARNING("partitioningMode: " << partitioningMode);

    mesh::Mesh* mesh = nullptr;
    StaticVector<StaticVector<int>*>* ptsCams = nullptr;

    switch(repartitionMode)
    {
        case eRepartitionRegularGrid:
        {
            switch(partitioningMode)
            {
                case ePartitioningAuto:
                {
                    ALICEVISION_LOG_INFO("Meshing mode: regular Grid, partitioning: auto.");
                    fuseCut::LargeScale lsbase(&mp, tmpDirectory.string() + "/");
                    lsbase.generateSpace(maxPtsPerVoxel, ocTreeDim, true);
                    std::string voxelsArrayFileName = lsbase.spaceFolderName + "hexahsToReconstruct.bin";
                    StaticVector<Point3d>* voxelsArray = nullptr;
                    if(fs::exists(voxelsArrayFileName))
                    {
                        // If already computed reload it.
                        ALICEVISION_LOG_INFO(
                            "Voxels array already computed, reload from file: " << voxelsArrayFileName);
                        voxelsArray = loadArrayFromFile<Point3d>(voxelsArrayFileName);
                    }
                    else
                    {
                        ALICEVISION_LOG_INFO("Compute voxels array.");
                        fuseCut::ReconstructionPlan rp(lsbase.dimensions, &lsbase.space[0], lsbase.mp,
                                                       lsbase.spaceVoxelsFolderName);
                        voxelsArray = rp.computeReconstructionPlanBinSearch(fuseParams.maxPoints);
                        saveArrayToFile<Point3d>(voxelsArrayFileName, voxelsArray);
                    }
                    fuseCut::reconstructSpaceAccordingToVoxelsArray(voxelsArrayFileName, &lsbase);
                    // Join meshes and ptsCams
                    mesh = fuseCut::joinMeshes(voxelsArrayFileName, &lsbase);
                    ptsCams = fuseCut::loadLargeScalePtsCams(lsbase.getRecsDirs(voxelsArray));
                    break;
                }
                case ePartitioningSingleBlock:
                {
                    ALICEVISION_LOG_INFO("Meshing mode: regular Grid, partitioning: single block.");
                    fuseCut::LargeScale ls0(&mp, tmpDirectory.string() + "/");
                    ls0.generateSpace(maxPtsPerVoxel, ocTreeDim, true);
                    unsigned long ntracks = std::numeric_limits<unsigned long>::max();
                    while(ntracks > fuseParams.maxPoints)
                    {
                        fs::path dirName =
                            outDirectory / ("LargeScaleMaxPts" + mvsUtils::num2strFourDecimal(ocTreeDim));
                        fuseCut::LargeScale* ls = ls0.cloneSpaceIfDoesNotExists(ocTreeDim, dirName.string() + "/");
                        fuseCut::VoxelsGrid vg(ls->dimensions, &ls->space[0], ls->mp, ls->spaceVoxelsFolderName);
                        ntracks = vg.getNTracks();
                        delete ls;
                        ALICEVISION_LOG_INFO("Number of track candidates: " << ntracks);
                        if(ntracks > fuseParams.maxPoints)
                        {
                            ALICEVISION_LOG_INFO("ocTreeDim: " << ocTreeDim);
                            double t = (double)ntracks / (double)fuseParams.maxPoints;
                            ALICEVISION_LOG_INFO("downsample: " << ((t < 2.0) ? "slow" : "fast"));
                            ocTreeDim = (t < 2.0) ? ocTreeDim - 100 : ocTreeDim * 0.5;
                        }
                    }
                    ALICEVISION_LOG_INFO("Number of tracks: " << ntracks);
                    ALICEVISION_LOG_INFO("ocTreeDim: " << ocTreeDim);
                    fs::path dirName = outDirectory / ("LargeScaleMaxPts" + mvsUtils::num2strFourDecimal(ocTreeDim));
                    fuseCut::LargeScale lsbase(&mp, dirName.string() + "/");
                    lsbase.loadSpaceFromFile();
                    fuseCut::ReconstructionPlan rp(lsbase.dimensions, &lsbase.space[0], lsbase.mp,
                                                   lsbase.spaceVoxelsFolderName);

                    StaticVector<int> voxelNeighs;
                    voxelNeighs.resize(rp.voxels->size() / 8);
                    ALICEVISION_LOG_INFO("voxelNeighs.size(): " << voxelNeighs.size());
                    for(int i = 0; i < voxelNeighs.size(); ++i)
                        voxelNeighs[i] = i;

                    Point3d* hexah = &lsbase.space[0];

                    StaticVector<int> cams;
                    if(hexah)
                    {
                        cams = mp.findCamsWhichIntersectsHexahedron(hexah);
                    }
                    else
                    {
                        cams.resize(mp.getNbCameras());
                        for(int i = 0; i < cams.size(); ++i)
                            cams[i] = i;
                    }

                    if(cams.size() < 1)
                        throw std::logic_error("No camera to make the reconstruction");

                    fuseCut::DelaunayGraphCut delaunayGC(&mp);
                    delaunayGC.createDensePointCloudFromPrecomputedDensePoints(hexah, cams, &voxelNeighs,
                                                                               (fuseCut::VoxelsGrid*)&rp);
                    delaunayGC.createGraphCut(hexah, cams, (fuseCut::VoxelsGrid*)&rp, outDirectory.string() + "/",
                                              lsbase.getSpaceCamsTracksDir(), false, lsbase.getSpaceSteps());
                    delaunayGC.graphCutPostProcessing();
                    mesh = delaunayGC.createMesh();
                    ptsCams = delaunayGC.createPtsCams();
                    mesh::meshPostProcessing(mesh, ptsCams, mp, outDirectory.string() + "/", nullptr, hexah);
                    break;
                }
                case ePartitioningUndefined:
                default:
                    throw std::invalid_argument("Partitioning mode is not defined");
            }
            break;
        }
        case eRepartitionMultiResolution:
        {
            switch(partitioningMode)
            {
                case ePartitioningAuto:
                {
                    throw std::invalid_argument(
                        "Meshing mode: 'multiResolution', partitioning: 'auto' is not yet implemented.");
                }
                case ePartitioningSingleBlock:
                {
                    ALICEVISION_LOG_INFO("Meshing mode: multi-resolution, partitioning: single block.");
                    std::array<Point3d, 8> hexah;

                    float minPixSize;
                    fuseCut::Fuser fs(&mp);

                    if(meshingFromDepthMaps && !estimateSpaceFromSfM)
                        fs.divideSpaceFromDepthMaps(&hexah[0], minPixSize);
                    else
                        fs.divideSpaceFromSfM(sfmData, &hexah[0], estimateSpaceMinObservations,
                                              estimateSpaceMinObservationAngle);

                    Voxel dimensions =
                        fs.estimateDimensions(&hexah[0], &hexah[0], 0, ocTreeDim,
                                              (meshingFromDepthMaps && !estimateSpaceFromSfM) ? nullptr : &sfmData);
                    StaticVector<Point3d>* voxels = mvsUtils::computeVoxels(&hexah[0], dimensions);

                    StaticVector<int> voxelNeighs;
                    voxelNeighs.resize(voxels->size() / 8);
                    ALICEVISION_LOG_INFO("voxelNeighs.size(): " << voxelNeighs.size());

                    for(int i = 0; i < voxelNeighs.size(); ++i)
                        voxelNeighs[i] = i;

                    Point3d spaceSteps;
                    {
                        Point3d vx = hexah[1] - hexah[0];
                        Point3d vy = hexah[3] - hexah[0];
                        Point3d vz = hexah[4] - hexah[0];
                        spaceSteps.x = (vx.size() / (double)dimensions.x) / (double)ocTreeDim;
                        spaceSteps.y = (vy.size() / (double)dimensions.y) / (double)ocTreeDim;
                        spaceSteps.z = (vz.size() / (double)dimensions.z) / (double)ocTreeDim;
                    }
                    delete voxels;

                    StaticVector<int> cams;
                    if(meshingFromDepthMaps)
                    {
                        cams = mp.findCamsWhichIntersectsHexahedron(&hexah[0]);
                    }
                    else
                    {
                        cams.resize(mp.getNbCameras());
                        for(int i = 0; i < cams.size(); ++i)
                            cams[i] = i;
                    }

                    if(cams.empty())
                        throw std::logic_error("No camera to make the reconstruction");

                    fuseCut::DelaunayGraphCut delaunayGC(&mp);
                    delaunayGC.createDensePointCloud(&hexah[0], cams,
                                                     addLandmarksToTheDensePointCloud ? &sfmData : nullptr,
                                                     meshingFromDepthMaps ? &fuseParams : nullptr);
                    if(saveRawDensePointCloud)
                    {
                        ALICEVISION_LOG_INFO("Save dense point cloud before cut and filtering.");
                        StaticVector<StaticVector<int>*>* ptsCams = delaunayGC.createPtsCams();
                        sfmData::SfMData densePointCloud;
                        createDenseSfMData(sfmData, mp, delaunayGC._verticesCoords, *ptsCams, densePointCloud);
                        removeLandmarksWithoutObservations(densePointCloud);
                        if(colorizeOutput)
                            sfmData::colorizeTracks(densePointCloud);
                        sfmDataIO::Save(densePointCloud, (outDirectory / "densePointCloud_raw.abc").string(),
                                        sfmDataIO::ESfMData::ALL_DENSE);
                        deleteArrayOfArrays(&ptsCams);
                    }

                    delaunayGC.createGraphCut(&hexah[0], cams, nullptr, outDirectory.string() + "/",
                                              outDirectory.string() + "/SpaceCamsTracks/", false, spaceSteps);
                    delaunayGC.graphCutPostProcessing();
                    mesh = delaunayGC.createMesh();
                    ptsCams = delaunayGC.createPtsCams();
                    mesh::meshPostProcessing(mesh, ptsCams, mp, outDirectory.string() + "/", nullptr, &hexah[0]);

                    break;
                }
                case ePartitioningUndefined:
                default:
                    throw std::invalid_argument("Partitioning mode is not defined");
            }
            break;
        }
        case eRepartitionUndefined:
        default:
            throw std::invalid_argument("Repartition mode is not defined");
    }

    // Generate output files:
    // - dense point-cloud with observations as sfmData
    // - mesh as .obj

    if(mesh == nullptr || mesh->pts->empty() || mesh->tris->empty())
        throw std::runtime_error("No valid mesh was generated.");

    if(ptsCams == nullptr)
        throw std::runtime_error("Points visibilities data has not been initialized.");

    sfmData::SfMData densePointCloud;
    createDenseSfMData(sfmData, mp, mesh->pts->getData(), *ptsCams, densePointCloud);

    deleteArrayOfArrays<int>(&ptsCams);

    if(colorizeOutput)
    {
        sfmData::colorizeTracks(densePointCloud);
        // colorize output mesh before landmarks filtering
        // to have a 1:1 mapping between points and mesh vertices
        const auto& landmarks = densePointCloud.getLandmarks();
        std::vector<rgb>& colors = mesh->colors();
        colors.resize(mesh->pts->size(), {0, 0, 0});
        for(std::size_t i = 0; i < mesh->pts->size(); ++i)
        {
            const auto& c = landmarks.at(i).rgb;
            colors[i] = {c.r(), c.g(), c.b()};
        }
    }

    removeLandmarksWithoutObservations(densePointCloud);
    ALICEVISION_LOG_INFO("Save dense point cloud.");
    sfmDataIO::Save(densePointCloud, outputDensePointCloud, sfmDataIO::ESfMData::ALL_DENSE);

    if(!LandmarkMatchingFilePath.empty())
    {
        ALICEVISION_LOG_INFO("Start Checking if Landmark and Vertex matches are correct.");

        // mesh->saveLandmarkMatchingFile(sfmData.getLandmarks(),densePointCloud.getLandmarks(),
        // LandmarkMatchingFilePath);
        int count = CheckIfMatchesAreWorking(sfmData.getLandmarks(), mesh->pts->getDataWritable());
        ALICEVISION_LOG_INFO(std::to_string(count));

        ALICEVISION_LOG_INFO("Start Save JSON Landmark matches.");
        aliceVision::system::Timer timerLandmarkMatches;

        sfmDataIO::SaveLandmarkVertexMatches(mesh->pts->getData(), LandmarkMatchingFilePath);

        ALICEVISION_LOG_INFO("Export JSON Landmark Matches took " + std::to_string(timerLandmarkMatches.elapsed()) +
                             "s");

        // int counter = 0;
        // const sfmData::Landmarks& landmarks = sfmData.getLandmarks();
        // const std::vector<Point3d>& data = mesh->pts->getData();
        // for(int i = 0; i < data.size(); ++i)
        //{
        //    /*if(data[i].id >= landmarks.size())
        //    {
        //        counter++;
        //        continue;
        //    }*/

        //    if(!CheckIfPoseTheSame(data[i], landmarks.at(data[i].id).X))
        //    {
        //        ALICEVISION_LOG_INFO(std::to_string(data[i].x) + "," + std::to_string(data[i].y) + "," +
        //                             std::to_string(data[i].z) + " vs " +
        //                             std::to_string(landmarks.at(data[i].id).X[0]) + "," +
        //                             std::to_string(landmarks.at(data[i].id).X[1]) + "," +
        //                             std::to_string(landmarks.at(data[i].id).X[2]));
        //        counter++;
        //    }
        //}
        // ALICEVISION_LOG_INFO(std::to_string(counter) + " vs " + std::to_string(mesh->pts->size()));

        // ALICEVISION_LOG_INFO(std::to_string(sfmData.getLandmarks()[mesh->pts->getData()[0].id].X[0]) +
        //                     std::to_string(sfmData.getLandmarks()[mesh->pts->getData()[0].id].X[1]) +
        //                     std::to_string(sfmData.getLandmarks()[mesh->pts->getData()[0].id].X[2])
        //                     + " Landmark| Point " + std::to_string(mesh->pts->getData()[0].x) +
        //                     std::to_string(mesh->pts->getData()[0].y) +
        //                     std::to_string(mesh->pts->getData()[0].z));
    }

    ALICEVISION_LOG_INFO("Save obj mesh file.");
    mesh->saveToObj(outputMesh);

    delete mesh;

    ALICEVISION_LOG_INFO("Task done in (s): " + std::to_string(timer.elapsed()));
    return EXIT_SUCCESS;
}

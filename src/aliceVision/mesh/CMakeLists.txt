# Headers
set(mesh_files_headers
  geoMesh.hpp
  Mesh.hpp
  MeshAnalyze.hpp
  MeshClean.hpp
  MeshEnergyOpt.hpp
  meshPostProcessing.hpp
  meshVisibility.hpp
  Texturing.hpp
  UVAtlas.hpp
)

# Sources
set(mesh_files_sources
  Mesh.cpp
  MeshAnalyze.cpp
  MeshClean.cpp
  MeshEnergyOpt.cpp
  meshPostProcessing.cpp
  meshVisibility.cpp
  Texturing.cpp
  UVAtlas.cpp
)

find_package( PCL 1.8 REQUIRED )

if( PCL_FOUND )
  # [C/C++]>[General]>[Additional Include Directories]
  include_directories( ${PCL_INCLUDE_DIRS} )

  # [C/C++]>[Preprocessor]>[Preprocessor Definitions]
  add_definitions( ${PCL_DEFINITIONS} )
  
  # For Use Not PreCompiled Features 
  #add_definitions( -DPCL_NO_PRECOMPILE )

  # [Linker]>[General]>[Additional Library Directories]
  link_directories( ${PCL_LIBRARY_DIRS} )

  # [Linker]>[Input]>[Additional Dependencies]
  #target_link_libraries(aliceVision_mesh ${PCL_LIBRARIES} )
endif()

alicevision_add_library(aliceVision_mesh
  SOURCES ${mesh_files_headers} ${mesh_files_sources}
  PUBLIC_LINKS
    aliceVision_mvsData
    aliceVision_mvsUtils
    Geogram::geogram
    ${Boost_FILESYSTEM_LIBRARY}
  PRIVATE_LINKS
  	${PCL_LIBRARIES}
    aliceVision_system
)

message(${PCL_LIBRARIES})



# Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.

project(dw_samples_pilotnet_common C CXX)

#-------------------------------------------------------------------------------
# Project files
#-------------------------------------------------------------------------------
set(PUBLIC_DOCS
    
)

set(SOURCES
    PilotNet.hpp
    PilotNet.cpp
)

set(LIBRARIES
    dw_samples_framework
    ${Driveworks_LIBRARIES}
    lodepng
)

#-------------------------------------------------------------------------------
# Final target
#-------------------------------------------------------------------------------
add_library(${PROJECT_NAME} STATIC ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${LIBRARIES})
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER "Samples")

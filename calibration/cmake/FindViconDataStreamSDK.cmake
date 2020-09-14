#############################################################################
#
# Description:
# Try to find Vicon Data Stream SDK
#
# ViconDataStreamSDK_FOUND
# ViconDataStreamSDK_INCLUDE_DIRS
# ViconDataStreamSDK_LIBRARIES
#
# Authors:
# Fabien Spindler
#
#############################################################################

find_library(ViconDataStreamSDK_LIBRARY 
  NAMES ViconDataStreamSDK_CPP
  PATHS
    $ENV{ViconDataStreamSDK_HOME}/bin/Release
    ${ViconDataStreamSDK_HOME}/bin/Release
)

if(ViconDataStreamSDK_LIBRARY)
  set(ViconDataStreamSDK_LIBRARIES ${ViconDataStreamSDK_LIBRARY})
endif()
  
mark_as_advanced(
  ViconDataStreamSDK_LIBRARY
)


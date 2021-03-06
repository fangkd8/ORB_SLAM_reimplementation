cmake_minimum_required( VERSION 2.8 )
project ( VisualOdometry )

#set( CMAKE_CXX_COMPILER "g++" )
#set( CMAKE_BUILD_TYPE "Release" )
#set( CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3" )

## configure build
set( CMAKE_BUILD_TYPE Release )
add_compile_options(-std=c++11)
# Optimization flag, (low) O1 -> O3 (high)
set( CMAKE_CXX_FLAGS_RELEASE "-O3" )
# generate compile commands
set( CMAKE_EXPORT_COMPILE_COMMANDS ON )

#list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )

############### dependencies ######################
# Eigen
include_directories( "/usr/include/eigen3" )
# OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
# Sophus 
find_package( Sophus REQUIRED )
set(Sophus_LIBRARIES libSophus.so)
include_directories( ${Sophus_INCLUDE_DIRS} )
# G2O
LIST( APPEND CMAKE_MODULE_PATH /home/fangkd/Desktop/Prac/g2o-master/cmake_modules )
SET( G2O_ROOT /home/fangkd/Desktop/Prac/g2o-master )
find_package( G2O REQUIRED )
include_directories( ${G2O_INCLUDE_DIRS} )

set( THIRD_PARTY_LIBS 
    ${OpenCV_LIBS}
    ${Sophus_LIBRARIES}
    g2o_core g2o_stuff g2o_types_sba
)
############### source and test ######################
include_directories( ${PROJECT_SOURCE_DIR}/include )
add_subdirectory( src )
add_subdirectory( test )

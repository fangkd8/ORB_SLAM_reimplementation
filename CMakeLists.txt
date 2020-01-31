cmake_minimum_required(VERSION 2.8.3)
set( CMAKE_CXX_FLAGS "-std=c++14" )

project(ORB_SLAM_re)

set(BOOST_ROOT "/usr/include/boost") # either set it here or from the command line  
set(Boost_USE_STATIC_LIBS OFF) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF)
 
find_package(Boost REQUIRED COMPONENTS system)
find_package( OpenCV REQUIRED )
find_package(Ceres REQUIRED)

# Please edit to your own g2o directory
LIST( APPEND CMAKE_MODULE_PATH /home/fangkd/Desktop/Prac/g2o-master/cmake_modules )
SET( G2O_ROOT /home/fangkd/Desktop/Prac/g2o-master )
find_package( G2O REQUIRED )

include_directories( 
  "/usr/include/eigen3"
  ${OpenCV_INCLUDE_DIRS}
)




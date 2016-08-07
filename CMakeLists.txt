PROJECT ( myDemo )
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)
INCLUDE_DIRECTORIES("../bayesopt/include")
INCLUDE_DIRECTORIES("../bayesopt/utils")

FIND_PACKAGE(Boost 1.54.0 REQUIRED COMPONENTS system filesystem program_options)
if (Boost_FOUND)
    INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIR})
endif(Boost_FOUND)

FIND_PACKAGE(Eigen3 REQUIRED)
if (Eigen3_FOUND)
    INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
endif(Eigen3_FOUND)

FILE(GLOB SRCS ${PROJECT_SOURCE_DIR}/bayesopt_synth.cpp
               ${PROJECT_SOURCE_DIR}/src/OptFuncs.cpp
               ${PROJECT_SOURCE_DIR}/src/LogManager.cpp
               ${PROJECT_SOURCE_DIR}/src/RKHS.cpp
               ${PROJECT_SOURCE_DIR}/src/iCubGraspQuality.cpp
               ${PROJECT_SOURCE_DIR}/src/iCubOptimizable.cpp
               ${PROJECT_SOURCE_DIR}/src/OptimizablesManager.cpp
               ${PROJECT_SOURCE_DIR}/src/TGPOptimization.cpp)

set(CMAKE_CXX_FLAGS "-Wall -Wno-unused-local-typedefs")

add_executable(bayesopt_synth ${SRCS})

TARGET_LINK_LIBRARIES(bayesopt_synth ${Boost_LIBRARIES} libbayesopt.a libnlopt.a libjsoncpp.a libGraspStudio.so libsimox-qhull.a libVirtualRobot.so)

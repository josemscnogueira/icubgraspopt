PROJECT ( myDemo )
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

option(SIMOX_SIMULATION_ENVIRONMENT "Build with Simox simulation environment?" OFF)

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)
INCLUDE_DIRECTORIES("../bayesopt/include")
INCLUDE_DIRECTORIES("../bayesopt/utils")

IF(NOT SIMOX_SIMULATION_ENVIRONMENT)
  FIND_PACKAGE(Boost 1.54.0 REQUIRED COMPONENTS system filesystem program_options)
  if (Boost_FOUND)
    INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIR})
  endif()

  FILE(GLOB SRCS ${PROJECT_SOURCE_DIR}/iCubSimulator.cpp
                 ${PROJECT_SOURCE_DIR}/src/OptFuncs.cpp
                 ${PROJECT_SOURCE_DIR}/src/LogManager.cpp
                 ${PROJECT_SOURCE_DIR}/src/RKHS.cpp
                 ${PROJECT_SOURCE_DIR}/src/OptimizablesManager.cpp
                 ${PROJECT_SOURCE_DIR}/src/TGPOptimization.cpp)

  set(CMAKE_CXX_FLAGS "-Wall -Wno-unused-local-typedefs")

  add_executable(myDemo ${SRCS})

  TARGET_LINK_LIBRARIES(myDemo ${Boost_LIBRARIES} libbayesopt.a libnlopt.a libjsoncpp.a)
ELSE()
  FIND_PACKAGE(Simox REQUIRED)

  IF(Simox_USE_COIN_VISUALIZATION)
    FILE(GLOB SRCS ${PROJECT_SOURCE_DIR}/iCubSimulator.cpp
                   ${PROJECT_SOURCE_DIR}/src/OptFuncs.cpp
                   ${PROJECT_SOURCE_DIR}/src/LogManager.cpp
                   ${PROJECT_SOURCE_DIR}/src/RKHS.cpp
                   ${PROJECT_SOURCE_DIR}/src/OptimizablesManager.cpp
                   ${PROJECT_SOURCE_DIR}/src/TGPOptimization.cpp
                   ${PROJECT_SOURCE_DIR}/src/OrientedBoundingBox.cpp
                   ${PROJECT_SOURCE_DIR}/src/iCub.cpp
                   ${PROJECT_SOURCE_DIR}/src/iCubHand.cpp
                   ${PROJECT_SOURCE_DIR}/src/ApproachMovementSpace.cpp
                   ${PROJECT_SOURCE_DIR}/src/iCubOptParameters.cpp
                   ${PROJECT_SOURCE_DIR}/src/iCubOptimizable.cpp
                   ${PROJECT_SOURCE_DIR}/src/Window.cpp)

    FILE(GLOB INCS ${PROJECT_SOURCE_DIR}/include/OptFuncs.hpp
                   ${PROJECT_SOURCE_DIR}/include/LogManager.hpp
                   ${PROJECT_SOURCE_DIR}/include/RKHS.hpp
                   ${PROJECT_SOURCE_DIR}/include/OptimizablesManager.hpp
                   ${PROJECT_SOURCE_DIR}/include/TGPOptimization.hpp
                   ${PROJECT_SOURCE_DIR}/include/OrientedBoundingBox.hpp
                   ${PROJECT_SOURCE_DIR}/include/iCub.hpp
                   ${PROJECT_SOURCE_DIR}/include/iCubOptParameters.hpp
                   ${PROJECT_SOURCE_DIR}/include/iCubHand.hpp
                   ${PROJECT_SOURCE_DIR}/include/ApproachMovementSpace.hpp
                   ${PROJECT_SOURCE_DIR}/include/iCubOptParameters.hpp
                   ${PROJECT_SOURCE_DIR}/include/iCubOptimizable.hpp
                   ${PROJECT_SOURCE_DIR}/include/Window.hpp)

    set(GUI_MOC_HDRS ${PROJECT_SOURCE_DIR}/include/Window.hpp)
    set(GUI_UIS ${PROJECT_SOURCE_DIR}/ui/iCubSimulator.ui)
    set(CMAKE_CXX_FLAGS "-Wall -std=gnu++11 -lpthread -Wno-unused-local-typedefs")

    SimoxQtApplication(${PROJECT_NAME} "${SRCS}" "${INCS}" "${GUI_MOC_HDRS}" "${GUI_UIS}")

    target_link_libraries(myDemo libbayesopt.a libnlopt.a libjsoncpp.a)
  ENDIF()
ENDIF(SIMOX_SIMULATION_ENVIRONMENT)

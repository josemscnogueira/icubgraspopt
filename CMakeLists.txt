PROJECT ( myDemo )

FIND_PACKAGE(Simox REQUIRED)
find_package(PythonLibs REQUIRED)

IF(Simox_USE_COIN_VISUALIZATION)
  include_directories(${PROJECT_SOURCE_DIR}/include)
  include_directories("../bayesopt/include")
  include_directories("../bayesopt/utils")
  include_directories(${PYTHON_INCLUDE_DIRS})

  FILE(GLOB SRCS ${PROJECT_SOURCE_DIR}/iCubSimulator.cpp
                 ${PROJECT_SOURCE_DIR}/src/OptFuncs.cpp
                 ${PROJECT_SOURCE_DIR}/src/LogManager.cpp
                 ${PROJECT_SOURCE_DIR}/src/RKHS.cpp
                 ${PROJECT_SOURCE_DIR}/src/TGPOptimization.cpp
                 ${PROJECT_SOURCE_DIR}/src/iCub.cpp
                 ${PROJECT_SOURCE_DIR}/src/iCubHand.cpp
                 ${PROJECT_SOURCE_DIR}/src/ApproachMovementSpace.cpp
                 ${PROJECT_SOURCE_DIR}/src/OrientedBoundingBox.cpp
                 ${PROJECT_SOURCE_DIR}/src/iCubOptimizable.cpp
                 ${PROJECT_SOURCE_DIR}/src/Window.cpp)

  FILE(GLOB INCS ${PROJECT_SOURCE_DIR}/include/OptFuncs.hpp
                 ${PROJECT_SOURCE_DIR}/include/LogManager.hpp
                 ${PROJECT_SOURCE_DIR}/include/RKHS.hpp
                 ${PROJECT_SOURCE_DIR}/include/TGPOptimization.hpp
                 ${PROJECT_SOURCE_DIR}/include/iCub.hpp
                 ${PROJECT_SOURCE_DIR}/include/iCubOptParameters.hpp
                 ${PROJECT_SOURCE_DIR}/include/iCubHand.hpp
                 ${PROJECT_SOURCE_DIR}/include/ApproachMovementSpace.hpp
                 ${PROJECT_SOURCE_DIR}/include/OrientedBoundingBox.hpp
                 ${PROJECT_SOURCE_DIR}/include/iCubOptParameters.hpp
                 ${PROJECT_SOURCE_DIR}/include/iCubOptimizable.hpp
                 ${PROJECT_SOURCE_DIR}/include/Window.hpp)

  set(GUI_MOC_HDRS ${PROJECT_SOURCE_DIR}/include/Window.hpp)
  set(GUI_UIS ${PROJECT_SOURCE_DIR}/ui/iCubSimulator.ui)
  set(CMAKE_CXX_FLAGS "-Wall -std=c++11 -lpthread -Wno-unused-local-typedefs")

  SimoxQtApplication(${PROJECT_NAME} "${SRCS}" "${INCS}" "${GUI_MOC_HDRS}" "${GUI_UIS}")
ENDIF()

find_package(OpenMP)
if (OPENMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
ENDIF()

target_link_libraries(myDemo libbayesopt.a libnlopt.a libjsoncpp.a ${PYTHON_LIBRARIES})

#                          Implementation of the papers
#
#                "Exact Acceleration of Linear Object Detectors"
#               12th European Conference on Computer Vision, 2012.
#
#             "Deformable Part Models with Individual Part Scaling"
#                  24th British Machine Vision Conference, 2013.
#
#      Copyright (c) 2013 Idiap Research Institute, <http://www.idiap.ch/>
#              Written by Charles Dubout <charles.dubout@idiap.ch>

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

PROJECT(FFLD)

set (CMAKE_CXX_STANDARD 11)

# The build options
OPTION(FFLD_HOGPYRAMID_DOUBLE "Use doubles instead of floats (slower, uses twice more memory, and the increase in precision is not necessarily useful)." OFF)
OPTION(FFLD_HOGPYRAMID_EXTRA_FEATURES "Use extra features (LBP + color) in addition to HOG." OFF)
OPTION(FFLD_MODEL_3D "Allow parts to also deform across scales." OFF)
OPTION(FFLD_MIXTURE_STANDARD_CONVOLUTION "Use standard convolutions instead of the optimized Fourier ones." OFF)

# Select a default build configuration if none was chosen
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: None (CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel." FORCE)
ENDIF()

# Also list the headers so that they are displayed along the .cpp files in the IDE
SET(HEADERS HOGPyramid.h Intersector.h JPEGImage.h LBFGS.h Mixture.h Model.h Object.h Patchwork.h Rectangle.h Scene.h SimpleOpt.h Detector.hpp)
SET(SOURCES HOGPyramid.cpp JPEGImage.cpp LBFGS.cpp Mixture.cpp Model.cpp Object.cpp Patchwork.cpp Rectangle.cpp Scene.cpp Detector.cpp)
ADD_EXECUTABLE(train ${HEADERS} ${SOURCES} train.cpp)
ADD_EXECUTABLE(detect ${HEADERS} ${SOURCES} test.cpp)
ADD_LIBRARY(ffld_detector SHARED ${HEADERS} ${SOURCES} ffld_detector.cpp)
SET_TARGET_PROPERTIES(ffld_detector PROPERTIES PREFIX "")

# Define the options
IF(FFLD_HOGPYRAMID_DOUBLE)
  MESSAGE("Use doubles instead of floats.")
  ADD_DEFINITIONS(-DFFLD_HOGPYRAMID_DOUBLE)
ENDIF()

IF(FFLD_HOGPYRAMID_EXTRA_FEATURES)
  MESSAGE("Use extra features (LBP + color) in addition to HOG.")
  ADD_DEFINITIONS(-DFFLD_HOGPYRAMID_EXTRA_FEATURES)
ENDIF()

IF(FFLD_MODEL_3D)
  MESSAGE("Allow parts to also deform across scales.")
  ADD_DEFINITIONS(-DFFLD_MODEL_3D)
ENDIF()

IF(FFLD_MIXTURE_STANDARD_CONVOLUTION)
  MESSAGE("Use standard convolutions instead of the optimized Fourier ones.")
  ADD_DEFINITIONS(-DFFLD_MIXTURE_STANDARD_CONVOLUTION)
ENDIF()

# There are no CMake Eigen package, so find it ourselves
FIND_PACKAGE(Eigen3 REQUIRED)
# FILE(GLOB EIGEN_ARCHIVE "eigen*")
# FIND_PATH(EIGEN_INCLUDE_DIR Eigen ${EIGEN_ARCHIVE} .)
# IF(NOT EIGEN_INCLUDE_DIR)
#  MESSAGE(FATAL_ERROR "Could not find Eigen.")
#ENDIF()
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})

# There are no CMake FFTW package, so find it ourselves
FIND_PATH(FFTW3_INCLUDE_DIR fftw3.h)
IF(FFLD_HOGPYRAMID_DOUBLE)
  FIND_LIBRARY(FFTW3_LIBRARIES fftw3)
ELSE()
  FIND_LIBRARY(FFTW3_LIBRARIES fftw3f)
ENDIF()
IF(NOT FFTW3_INCLUDE_DIR OR NOT FFTW3_LIBRARIES)
  MESSAGE(FATAL_ERROR "Could not find fftw3.")
ENDIF()
INCLUDE_DIRECTORIES(${FFTW3_INCLUDE_DIR})

TARGET_LINK_LIBRARIES(train ${FFTW3_LIBRARIES})
TARGET_LINK_LIBRARIES(detect ${FFTW3_LIBRARIES})
TARGET_LINK_LIBRARIES(ffld_detector ${FFTW3_LIBRARIES})

FIND_PACKAGE(JPEG REQUIRED)
IF(JPEG_FOUND)
  INCLUDE_DIRECTORIES(${JPEG_INCLUDE_DIR})
  TARGET_LINK_LIBRARIES(train ${JPEG_LIBRARIES})
  TARGET_LINK_LIBRARIES(detect ${JPEG_LIBRARIES})
  TARGET_LINK_LIBRARIES(ffld_detector ${JPEG_LIBRARIES})
ENDIF()

FIND_PACKAGE(LibXml2 REQUIRED)
IF(LIBXML2_FOUND)
  INCLUDE_DIRECTORIES(${LIBXML2_INCLUDE_DIR})
  TARGET_LINK_LIBRARIES(train ${LIBXML2_LIBRARIES})
  TARGET_LINK_LIBRARIES(detect ${LIBXML2_LIBRARIES})
  TARGET_LINK_LIBRARIES(ffld_detector ${LIBXML2_LIBRARIES})
  ADD_DEFINITIONS(${LIBXML2_DEFINITIONS})
ENDIF()

FIND_PACKAGE(PythonLibs REQUIRED)
IF(PYTHONLIBS_FOUND)
    INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_DIR})
    TARGET_LINK_LIBRARIES(ffld_detector ${PYTHON_LIBRARIES})
ENDIF()

# Not required, but stronlgy recommended on multi-core systems
FIND_PACKAGE(OpenMP)
IF(OPENMP_FOUND)
  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_C_FLAGS}")
ENDIF()

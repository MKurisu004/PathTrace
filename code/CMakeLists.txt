CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
PROJECT(PA1 CXX)

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF()

ADD_SUBDIRECTORY(deps/vecmath)

SET(PA1_SOURCES
        src/image.cpp
        src/main.cpp
        src/mesh.cpp
        src/scene_parser.cpp)

SET(PA1_INCLUDES
        include/camera.hpp
        include/group.hpp
        include/hit.hpp
        include/image.hpp
        include/light.hpp
        include/material.hpp
        include/mesh.hpp
        include/object3d.hpp
        include/plane.hpp
        include/ray.hpp
        include/scene_parser.hpp
        include/sphere.hpp
        include/transform.hpp
        include/triangle.hpp
        )

SET(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

ADD_EXECUTABLE(${PROJECT_NAME} ${PA1_SOURCES} ${PA1_INCLUDES})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} vecmath)

TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} PRIVATE include)

if(APPLE)
  set(LIBM_PREFIX "/opt/homebrew/opt/libomp")
  find_path(OMP_INCLUDE_DIR omp.h HINTS ${LIBM_PREFIX}/include)
  find_library(OMP_LIBRARY omp HINTS ${LIBM_PREFIX}/lib)

  if(OMP_INCLUDE_DIR AND OMP_LIBRARY)
    message(STATUS "Found libomp: ${OMP_INCLUDE_DIR}, ${OMP_LIBRARY}")
    target_include_directories(${PROJECT_NAME} PRIVATE ${OMP_INCLUDE_DIR})
    target_compile_options   (${PROJECT_NAME} PRIVATE -Xpreprocessor -fopenmp)
    target_link_libraries    (${PROJECT_NAME} PUBLIC ${OMP_LIBRARY})
  else()
    message(WARNING "libomp not found, OpenMP DISABLED")
  endif()
else()
  find_package(OpenMP REQUIRED)
  target_compile_options(${PROJECT_NAME} PUBLIC ${OpenMP_CXX_FLAGS})
  target_link_libraries (${PROJECT_NAME} PUBLIC OpenMP::OpenMP_CXX)
endif()

if(OpenMP_CXX_FOUND)
  target_link_libraries(${PROJECT_NAME} PUBLIC OpenMP::OpenMP_CXX)
  target_compile_options(${PROJECT_NAME} PUBLIC ${OpenMP_CXX_FLAGS})
  target_link_libraries(${PROJECT_NAME} PUBLIC ${OpenMP_EXE_LINKER_FLAGS})
endif()
# Try to find ORB_SLAM3.
# Set ORB_SLAM3_ROOT_DIR with a CMake argument or environment variable.
# Example:
#   colcon build --cmake-args -DORB_SLAM3_ROOT_DIR=/home/user/ORB_SLAM3

if(NOT ORB_SLAM3_ROOT_DIR)
  if(DEFINED ENV{ORB_SLAM3_ROOT_DIR} AND NOT "$ENV{ORB_SLAM3_ROOT_DIR}" STREQUAL "")
    file(TO_CMAKE_PATH "$ENV{ORB_SLAM3_ROOT_DIR}" ORB_SLAM3_ROOT_DIR)
  else()
    message(FATAL_ERROR
      "ORB_SLAM3_ROOT_DIR is not set.\n"
      "Pass it with: colcon build --cmake-args -DORB_SLAM3_ROOT_DIR=/path/to/ORB_SLAM3\n"
      "Or set the environment variable: export ORB_SLAM3_ROOT_DIR=/path/to/ORB_SLAM3")
  endif()
endif()

set(ORB_SLAM3_ROOT_DIR "${ORB_SLAM3_ROOT_DIR}" CACHE PATH "ORB_SLAM3 root directory")

# message(${ORB_SLAM3_ROOT_DIR})
# message(${ORB_SLAM3_ROOT_DIR}/include)
# message(${ORB_SLAM3_ROOT_DIR}/Thirdparty/DBoW2/DBoW2)

# Find ORB_SLAM3
find_path(ORB_SLAM3_INCLUDE_DIR NAMES System.h
          PATHS ${ORB_SLAM3_ROOT_DIR}/include
          NO_DEFAULT_PATH)

find_library(ORB_SLAM3_LIBRARY NAMES ORB_SLAM3 libORB_SLAM3
             PATHS ${ORB_SLAM3_ROOT_DIR}/lib
             NO_DEFAULT_PATH)

# Find built-in DBoW2
find_path(DBoW2_INCLUDE_DIR NAMES Thirdparty/DBoW2/DBoW2/BowVector.h
          PATHS ${ORB_SLAM3_ROOT_DIR}
          NO_DEFAULT_PATH)

find_library(DBoW2_LIBRARY NAMES DBoW2
             PATHS ${ORB_SLAM3_ROOT_DIR}/Thirdparty/DBoW2/lib
             NO_DEFAULT_PATH)

# Find built-in g2o
find_library(g2o_LIBRARY NAMES g2o
             PATHS ${ORB_SLAM3_ROOT_DIR}/Thirdparty/g2o/lib
             NO_DEFAULT_PATH)



include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set ORB_SLAM3_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(ORB_SLAM3  DEFAULT_MSG
                                  ORB_SLAM3_LIBRARY ORB_SLAM3_INCLUDE_DIR DBoW2_INCLUDE_DIR DBoW2_LIBRARY g2o_LIBRARY)

mark_as_advanced(ORB_SLAM3_INCLUDE_DIR ORB_SLAM3_LIBRARY )

set(ORB_SLAM3_LIBRARIES ${ORB_SLAM3_LIBRARY} ${DBoW2_LIBRARY} ${g2o_LIBRARY})
set(ORB_SLAM3_INCLUDE_DIRS ${ORB_SLAM3_INCLUDE_DIR} ${DBoW2_INCLUDE_DIR})

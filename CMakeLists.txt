cmake_minimum_required(VERSION 3.16)
project(calib_ground VERSION 1.0.0 LANGUAGES CXX)

# set(CMAKE_CUDA_STANDARD 14)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-Wall -fPIC")
add_definitions(-w)

find_package(CUDA REQUIRED)
find_package(OpenCV REQUIRED)
find_package(VISP REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
        include/
        ${EIGEN3_INCLUDE_DIRS}
        ${CUDA_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${VISP_INCLUDE_DIRS}
)

file(GLOB_RECURSE SRCS ${PROJECT_SOURCE_DIR}/src/*/*.cpp)

add_executable(extract_image apps/extract_image_main.cpp ${SRCS})
add_executable(pano_calib apps/pano_calib_main.cpp ${SRCS})
add_executable(get_poses apps/get_poses_main.cpp ${SRCS})
add_executable(calib_cameras apps/calib_cameras_main.cpp ${SRCS})
add_executable(calib_ground apps/calib_ground_main.cpp ${SRCS})
add_executable(calib_intrin apps/calib_intrin_main.cpp ${SRCS})
add_executable(vis_cam apps/vis_cam_main.cpp ${SRCS})
add_executable(vis_ground apps/vis_ground_main.cpp ${SRCS})

# add_executable(extract_image apps/extract_image_main.cpp src/stage/april_tag.cpp src/core/camera_calib.cpp src/stage/undistorter.cpp src/stage/extract_image.cpp)
# add_executable(pano_calib apps/pano_calib_main.cpp src/stage/extract_image.cpp src/core/pano_calib.cpp src/core/camera_calib.cpp src/stage/april_tag.cpp)
# add_executable(get_poses apps/get_poses_main.cpp src/stage/april_tag.cpp src/core/camera_calib.cpp src/stage/extract_image.cpp)
# add_executable(calib_cameras apps/calib_cameras_main.cpp src/stage/tag_poses_set.cpp src/utils/utils.cpp)
# add_executable(calib_ground apps/calib_ground_main.cpp src/stage/tag_poses_set.cpp src/utils/utils.cpp)
# add_executable(calib_intrin apps/calib_intrin_main.cpp src/core/camera_calib.cpp src/stage/april_tag.cpp)
# add_executable(vis_cam apps/vis_cam_main.cpp src/core/viz_painter.cpp)
# add_executable(vis_ground apps/vis_ground_main.cpp src/core/viz_painter.cpp   src/utils/utils.cpp )

# link libraries
target_link_libraries(extract_image ${OpenCV_LIBS} ${VISP_LIBRARIES})
target_link_libraries(pano_calib ${OpenCV_LIBS} ${VISP_LIBRARIES})
target_link_libraries(get_poses ${OpenCV_LIBS} ${VISP_LIBRARIES})
target_link_libraries(calib_cameras ${OpenCV_LIBS} ${VISP_LIBRARIES})
target_link_libraries(calib_ground ${OpenCV_LIBS} ${VISP_LIBRARIES})
target_link_libraries(calib_intrin ${OpenCV_LIBS} ${VISP_LIBRARIES})
target_link_libraries(vis_cam ${OpenCV_LIBS} ${VISP_LIBRARIES})
target_link_libraries(vis_ground ${OpenCV_LIBS} ${VISP_LIBRARIES})

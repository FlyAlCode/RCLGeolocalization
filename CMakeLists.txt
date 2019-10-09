# 声明CMake的最低要求版本
cmake_minimum_required(VERSION 2.8)

PROJECT(road_cross_based_location)
add_compile_options(-std=c++11)
SET(CMAKE_BUILD_TYPE "Release")

# 外部依赖项
# opencv
find_package(OpenCV 3.3.1 EXACT REQUIRED
            PATHS /opt/ros/kinetic/share/OpenCV-3.3.1-dev)
include_directories(${OpenCV_INCLUDE_DIRS})
SET(REQUIRED_LIBRARIES ${REQUIRED_LIBRARIES} ${OpenCV_LIBS})

# nanoflann
find_package(nanoflann)
include_directories(${NANOFLANN_INCLUDE_DIRS})
SET(REQUIRED_LIBRARIES ${REQUIRED_LIBRARIES} nanoflann::nanoflann)

# deal with shapelib
SET(REQUIRED_LIBRARIES ${REQUIRED_LIBRARIES} ${PROJECT_SOURCE_DIR}/third_lib/lib/libshp.so)
include_directories(${PROJECT_SOURCE_DIR}/third_lib/include/  )

# 添加头文件的查找目录
include_directories(${PROJECT_SOURCE_DIR}/include/basic_structure)
include_directories(${PROJECT_SOURCE_DIR}/include/cross_point_detect)
include_directories(${PROJECT_SOURCE_DIR}/include/locator)
include_directories(${PROJECT_SOURCE_DIR}/include/shp)

# 添加参与编译的子目录,会直接跳转到子目录，因此这一句应该放在所有需要继承的变量已经被设置后
ADD_SUBDIRECTORY(src)

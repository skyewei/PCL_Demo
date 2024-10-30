
cmake_minimum_required(VERSION 3.15)

project(filter)

find_package(PCL 1.8 REQUIRED)

# 定义一个可执行目标
add_executable(passfilter)

# 指定可执行目标的源文件
target_sources(passfilter PRIVATE
    main.cpp
    src/pass.cpp
    src/voxelgrid.cpp
    src/statistical_removal.cpp
    src/project_inliers.cpp
    src/extract_indices.cpp
    src/remove_outliers.cpp
)

target_include_directories(passfilter PRIVATE 
    ${PCL_INCLUDE_DIRS} 
    ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(passfilter PRIVATE ${PCL_LIBRARIES})
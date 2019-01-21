cmake_minimum_required(VERSION 2.8)
project(hero_cannon)

#set(CMAKE)
add_compile_options(-std=c++11 -O3)

set(CMAKE_CXX_STANDARD 11)
set(EXECUTABLE_OUTPUT_PATH          ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY  ${PROJECT_SOURCE_DIR}/lib)

set(INC_DIR     /opt/DahuaTech/MVViewer/include
        ${PROJECT_SOURCE_DIR}/include )
set(LINK_DIR    /opt/DahuaTech/MVViewer/lib/GenICam/bin/Linux64_x64
        /opt/DahuaTech/MVViewer/lib
        ${PROJECT_SOURCE_DIR}/lib/cubot
        ${PROJECT_SOURCE_DIR}/lib/dahua_cam_usb)

find_package( OpenCV  REQUIRED )
find_package( Eigen3  REQUIRED )
#find_package( CCTag CONFIG REQUIRED)

include_directories(${Boost_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${INC_DIR} )
link_directories( ${LINK_DIR} )

add_subdirectory(src_cam)  #daHuaCam_node

add_executable(imgProc_SmallGun src_cubot/imgProc.cpp armor/armor_main.cpp armor/bar_points.cpp armor/coordinante_process.cpp src_cubot/Serial_port.cpp)
target_link_libraries(imgProc_SmallGun ${OpenCV_LIBS} ${Eigen3_LIBRARIES} ${Boost_LIBRARY}
        boost_thread boost_system timer imgSubscrible )




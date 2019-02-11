add_executable(pictures_pub pictures_pub.cpp)
target_link_libraries(pictures_pub ${OpenCV_LIBS} boost_system boost_thread pthread rt timer imgPublish)

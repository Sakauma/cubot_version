add_executable(ArmorHitTest ArmorHitTest.cpp my.cpp bar_points.cpp coordinante_process.cpp)
target_link_libraries(ArmorHitTest ${OpenCV_LIBS})

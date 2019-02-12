#add_executable(videoWriter videoWrite.cpp)
#target_link_libraries(videoWriter ${OpenCV_LIBS} ${Eigen3_LIBRARIES} ${Boost_LIBRARY}
#        imgSubscrible boost_thread boost_system timer)

add_executable(pictures_pub pictures_pub.cpp)
target_link_libraries(pictures_pub ${OpenCV_LIBS} boost_system boost_thread pthread rt timer imgPublish)
#
#add_executable(videoPublish videoPublish.cpp)
#target_link_libraries(videoPublish ${OpenCV_LIBS} boost_system boost_thread pthread rt timer imgPublish)

#add_executable(train_svm train_svm.cpp ../armor/pro_file.cpp)
#target_link_libraries(train_svm ${OpenCV_LIBS})



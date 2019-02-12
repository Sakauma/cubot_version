set(daHua_shared_libs   GCBase_gcc421_v3_0   GenApi_gcc421_v3_0       Log_gcc421_v3_0           MathParser_gcc421_v3_0
                        log4cpp_gcc421_v3_0  NodeMapData_gcc421_v3_0  XmlParser_gcc421_v3_0     ImageConvert
                        MVSDK                log4cpp )

add_executable(camDaHua_node camDaHua_node.cpp streamRetrieve.cpp camParaConfig.cpp)
target_link_libraries(camDaHua_node ${OpenCV_LIBS}
                                    ${daHua_shared_libs}
                                    boost_system
                                    boost_thread
                                    pthread
                                    rt
                                    timer
                                    imgPublish)

add_executable(velocity_pose_sensor main.cpp)

target_link_libraries(velocity_pose_sensor pose_distorter ${catkin_LIBRARIES})

add_dependencies(velocity_pose_sensor ${${PROJECT_NAME}_EXPORTED_TARGETS})

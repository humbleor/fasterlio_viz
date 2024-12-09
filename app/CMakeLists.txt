add_executable(run_mapping_online run_mapping_online.cc)
target_link_libraries(run_mapping_online
        ${PROJECT_NAME} gflags
        )
add_executable(run_mapping_offline run_mapping_offline.cc)
target_link_libraries(run_mapping_offline
        ${PROJECT_NAME} gflags
        )

add_executable(run_mapping_newbag run_mapping_newbag.cc)
target_link_libraries(run_mapping_newbag
        ${PROJECT_NAME}
        )

# 添加编译源文件
add_executable(run_mapping_viz run_mapping_viz.cc)
target_link_libraries(run_mapping_viz
        ${PROJECT_NAME}
        )
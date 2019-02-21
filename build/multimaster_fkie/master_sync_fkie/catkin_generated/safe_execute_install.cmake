execute_process(COMMAND "/home/jackson/Development/HARE/build/multimaster_fkie/master_sync_fkie/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/jackson/Development/HARE/build/multimaster_fkie/master_sync_fkie/catkin_generated/python_distutils_install.sh) returned error code ")
endif()

execute_process(COMMAND "/home/phk3601/obj-lst-vis/build/tp3/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/phk3601/obj-lst-vis/build/tp3/catkin_generated/python_distutils_install.sh) returned error code ")
endif()

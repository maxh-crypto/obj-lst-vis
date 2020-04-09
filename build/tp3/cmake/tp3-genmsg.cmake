# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(WARNING "Invoking generate_messages() without having added any message or service file before.
You should either add add_message_files() and/or add_service_files() calls or remove the invocation of generate_messages().")
message(STATUS "tp3: 0 messages, 0 services")

set(MSG_I_FLAGS "-Itp3:/home/tobias/obj-lst-vis/src/tp3/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators

add_custom_target(tp3_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



#
#  langs = 
#



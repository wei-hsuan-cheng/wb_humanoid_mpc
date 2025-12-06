cmake_minimum_required(VERSION 3.10)
project(ocs2_switched_model_msgs)

# Enable modern policy so IN_LIST is available (needed by ament/rcutils deps).
if(POLICY CMP0057)
  cmake_policy(SET CMP0057 NEW)
endif()

set(CATKIN_PACKAGE_DEPENDENCIES
  std_msgs
  ocs2_msgs
)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
  message_generation
)

# Declare the message files to be built
add_message_files(DIRECTORY msg FILES
  gait.msg
  gait_sequence.msg
  scheduled_gait_sequence.msg
)

# Declare the service files to be built
add_service_files(DIRECTORY srv FILES
  trajectory_request.srv
)

# Actually generate the language-specific message and service files
generate_messages(DEPENDENCIES
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

# Declare that this catkin package's runtime dependencies
catkin_package(CATKIN_DEPENDS
  ${CATKIN_PACKAGE_DEPENDENCIES}
  message_runtime
)

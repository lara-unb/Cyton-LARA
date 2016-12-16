# Make sure the compiler can find include files from our Hello library.
include_directories (${DQ_ROBOTICS_SOURCE_DIR}/DQ ${DQ_ROBOTICS_SOURCE_DIR}/kinematics)

# Make sure the linker can find the Hello library once it is built.
link_directories (${DQ_ROBOTICS_BINARY_DIR}/DQ ${DQ_ROBOTICS_BINARY_DIR}/kinematics)

add_library (kinematics kinematics.c)

target_link_libraries (kinematics DQ) 

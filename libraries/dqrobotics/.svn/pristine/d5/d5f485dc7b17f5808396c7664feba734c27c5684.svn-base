include_directories (${DQ_ROBOTICS_SOURCE_DIR}/DQ ${DQ_ROBOTICS_SOURCE_DIR}/kinematics)
link_directories (${DQ_ROBOTICS_BINARY_DIR}/DQ ${DQ_ROBOTICS_BINARY_DIR}/kinematics)

add_library(DQ dq.c)
add_library(operand operand.c dq.c )
target_link_libraries (operand kinematics gmatrix gmatrix_linalg) 

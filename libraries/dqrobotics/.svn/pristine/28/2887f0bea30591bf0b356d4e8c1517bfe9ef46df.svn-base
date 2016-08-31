include_directories (${DQ_ROBOTICS_SOURCE_DIR}/DQ ${DQ_ROBOTICS_SOURCE_DIR}/kinematics)

# Make sure the linker can find the DQ library once it is built.
link_directories (${DQ_ROBOTICS_BINARY_DIR}/DQ ${DQ_ROBOTICS_BINARY_DIR}/kinematics)

# Add executable called "dq" that is built from the source files
# "dq_demo.cxx". The extensions are automatically found.
add_executable (dq_demo dq_demo.c)

# Link the executable to the Hello library.
target_link_libraries (dq_demo dq kinematics operand) 

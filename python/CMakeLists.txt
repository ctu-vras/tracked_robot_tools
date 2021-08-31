# find SWIG and Python
find_package(SWIG 3.0.8 REQUIRED)
if (${CMAKE_VERSION} VERSION_GREATER "3.13")
  cmake_policy(SET CMP0078 OLD)  # use the legacy swig target names
endif()
if (${CMAKE_VERSION} VERSION_GREATER_EQUAL "3.12")
  cmake_policy(SET CMP0086 OLD)
endif()

include(${SWIG_USE_FILE})
find_package(PythonInterp ${PYTHON_VERSION} REQUIRED)
find_package(PythonLibs ${PYTHON_VERSION} REQUIRED)

# output the support files to the same directory as the generated library
set(CMAKE_SWIG_OUTDIR ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION})
# but do not put there the temporary .cxx files etc.
set(SWIG_OUTFILE_DIR ${CMAKE_CURRENT_BINARY_DIR})

set_property(SOURCE kinematics.i PROPERTY CPLUSPLUS ON)
swig_add_library(kinematics LANGUAGE python SOURCES kinematics.i)
target_include_directories(${SWIG_MODULE_kinematics_REAL_NAME} PRIVATE ${PYTHON_INCLUDE_DIRS})
swig_link_libraries(kinematics PRIVATE tracked_robot_kinematics ${PYTHON_LIBRARIES})

set_target_properties(${SWIG_MODULE_kinematics_REAL_NAME} PROPERTIES
  ARCHIVE_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
  LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
  RUNTIME_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

install(TARGETS ${SWIG_MODULE_kinematics_REAL_NAME}
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

# SWIG_SUPPORT_FILES is available only since CMake 3.12
get_property(support_files TARGET ${SWIG_MODULE_kinematics_REAL_NAME} PROPERTY SWIG_SUPPORT_FILES)
if(NOT support_files)
  set(support_files ${CMAKE_SWIG_OUTDIR}/kinematics.py)
endif()
install(FILES ${support_files}
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)
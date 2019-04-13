cmake_minimum_required( VERSION 2.8 )
set(PROJ_NAME PointCloud)
set(OpenCV_DIR "C:/Users/Arata/Downloads/opencv/build")

# Create Project
project( PointCloud )
add_executable( ${PROJ_NAME} main.cpp )

# Set StartUp Project (Option)
# (This setting is able to enable by using CMake 3.6.0 RC1 or later.)
set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT "PointCloud" )

# Find Packages
find_package( PCL 1.8 REQUIRED )
find_package( OpenCV REQUIRED)

if( PCL_FOUND )
  # [C/C++]>[General]>[Additional Include Directories]
  include_directories( ${PCL_INCLUDE_DIRS} )

  # [C/C++]>[Preprocessor]>[Preprocessor Definitions]
  add_definitions( ${PCL_DEFINITIONS} )
  
  # For Use Not PreCompiled Features 
  #add_definitions( -DPCL_NO_PRECOMPILE )

  # [Linker]>[General]>[Additional Library Directories]
  link_directories( ${PCL_LIBRARY_DIRS} )

  # [Linker]>[Input]>[Additional Dependencies]
  target_link_libraries( ${PROJ_NAME} ${PCL_LIBRARIES} )
endif()

if( OPENCV_FOUND )
  include_directories( ${OpenCV_INCLUDE_DIRS} )
  
  target_link_libraries( ${PROJ_NAME} ${OpenCV_LIBS} )
endif()
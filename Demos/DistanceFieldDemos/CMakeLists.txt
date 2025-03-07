set(SIMULATION_LINK_LIBRARIES AntTweakBar glfw PositionBasedDynamics Simulation Utils)
set(SIMULATION_DEPENDENCIES AntTweakBar glfw PositionBasedDynamics Simulation Utils CopyPBDShaders CopyPBDModels)

if(WIN32)
  set(SIMULATION_LINK_LIBRARIES opengl32.lib glu32.lib ${SIMULATION_LINK_LIBRARIES})
  #set(SIMULATION_DEPENDENCIES ${SIMULATION_DEPENDENCIES})
else()
  set(OpenGL_GL_PREFERENCE GLVND)
  find_package(OpenGL REQUIRED)
  include_directories( ${OPENGL_INCLUDE_DIRS})

  set(SIMULATION_LINK_LIBRARIES
    ${SIMULATION_LINK_LIBRARIES}
    ${OPENGL_LIBRARIES}
  )
endif()

############################################################
# GenericParameters
############################################################
include_directories(${GenericParameters_INCLUDE_DIR})
if(TARGET Ext_GenericParameters)
	set(SIMULATION_DEPENDENCIES ${SIMULATION_DEPENDENCIES} Ext_GenericParameters)
endif()


add_executable(ClothCollisionDemo
	  ClothCollisionDemo.cpp

	  ../Common/TweakBarParameters.cpp
	  ../Common/TweakBarParameters.h
	  ../Common/DemoBase.cpp
	  ../Common/DemoBase.h
	  
	  ${VIS_FILES}          
	  ${PROJECT_PATH}/Common/Common.h

	  CMakeLists.txt
)

set_target_properties(ClothCollisionDemo PROPERTIES FOLDER "Demos")
set_target_properties(ClothCollisionDemo PROPERTIES DEBUG_POSTFIX ${PBD_BINARY_DEBUG_POSTFIX})
set_target_properties(ClothCollisionDemo PROPERTIES RELWITHDEBINFO_POSTFIX ${PBD_BINARY_RELWITHDEBINFO_POSTFIX})
set_target_properties(ClothCollisionDemo PROPERTIES MINSIZEREL_POSTFIX ${PBD_BINARY_MINSIZEREL_POSTFIX})
add_dependencies(ClothCollisionDemo ${SIMULATION_DEPENDENCIES})
target_link_libraries(ClothCollisionDemo ${SIMULATION_LINK_LIBRARIES})


add_executable(DeformableCollisionDemo
	  DeformableCollisionDemo.cpp

	  ../Common/TweakBarParameters.cpp
	  ../Common/TweakBarParameters.h
	  ../Common/DemoBase.cpp
	  ../Common/DemoBase.h
	  
	  ${VIS_FILES}          
	  ${PROJECT_PATH}/Common/Common.h

	  CMakeLists.txt
)

set_target_properties(DeformableCollisionDemo PROPERTIES FOLDER "Demos")
set_target_properties(DeformableCollisionDemo PROPERTIES DEBUG_POSTFIX ${PBD_BINARY_DEBUG_POSTFIX})
set_target_properties(DeformableCollisionDemo PROPERTIES RELWITHDEBINFO_POSTFIX ${PBD_BINARY_RELWITHDEBINFO_POSTFIX})
set_target_properties(DeformableCollisionDemo PROPERTIES MINSIZEREL_POSTFIX ${PBD_BINARY_MINSIZEREL_POSTFIX})
add_dependencies(DeformableCollisionDemo ${SIMULATION_DEPENDENCIES})
target_link_libraries(DeformableCollisionDemo ${SIMULATION_LINK_LIBRARIES})


add_executable(RigidBodyCollisionDemo
	  RigidBodyCollisionDemo.cpp

	  ../Common/TweakBarParameters.cpp
	  ../Common/TweakBarParameters.h
	  ../Common/DemoBase.cpp
	  ../Common/DemoBase.h
	  
	  ${VIS_FILES}          
	  ${PROJECT_PATH}/Common/Common.h

	  CMakeLists.txt
)

set_target_properties(RigidBodyCollisionDemo PROPERTIES FOLDER "Demos")
set_target_properties(RigidBodyCollisionDemo PROPERTIES DEBUG_POSTFIX ${PBD_BINARY_DEBUG_POSTFIX})
set_target_properties(RigidBodyCollisionDemo PROPERTIES RELWITHDEBINFO_POSTFIX ${PBD_BINARY_RELWITHDEBINFO_POSTFIX})
set_target_properties(RigidBodyCollisionDemo PROPERTIES MINSIZEREL_POSTFIX ${PBD_BINARY_MINSIZEREL_POSTFIX})
add_dependencies(RigidBodyCollisionDemo ${SIMULATION_DEPENDENCIES})
target_link_libraries(RigidBodyCollisionDemo ${SIMULATION_LINK_LIBRARIES})


add_definitions(-DTW_NO_LIB_PRAGMA -DTW_STATIC -DFREEGLUT_LIB_PRAGMAS=0)
find_package( Eigen3 REQUIRED )
include_directories( ${EIGEN3_INCLUDE_DIR} )
include_directories(${PROJECT_PATH}/extern/freeglut/include)
include_directories(${PROJECT_PATH}/extern/glew/include)


VIS_SOURCE_GROUPS()

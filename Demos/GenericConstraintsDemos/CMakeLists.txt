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

add_executable(GenericParticleConstraintsDemo
	  GenericParticleConstraintsDemo.cpp
	  
	  GenericConstraintsModel.cpp
	  GenericConstraintsModel.h	  
	  GenericConstraints.cpp
	  GenericConstraints.h

	  ../Common/TweakBarParameters.cpp
	  ../Common/TweakBarParameters.h
	  ../Common/DemoBase.cpp
	  ../Common/DemoBase.h
	  
	  ${VIS_FILES}          
	  ${PROJECT_PATH}/Common/Common.h
	  
	  CMakeLists.txt
)

add_executable(GenericRigidBodyConstraintsDemo
	  GenericRigidBodyConstraintsDemo.cpp
	  
	  GenericConstraintsModel.cpp
	  GenericConstraintsModel.h	  
	  GenericConstraints.cpp
	  GenericConstraints.h

	  ../Common/TweakBarParameters.cpp
	  ../Common/TweakBarParameters.h
	  ../Common/DemoBase.cpp
	  ../Common/DemoBase.h
	  
	  ${VIS_FILES}          
	  ${PROJECT_PATH}/Common/Common.h
	  
	  CMakeLists.txt
)

add_definitions(-DTW_NO_LIB_PRAGMA -DTW_STATIC -DFREEGLUT_LIB_PRAGMAS=0)

find_package( Eigen3 REQUIRED )
include_directories( ${EIGEN3_INCLUDE_DIR} )
include_directories(${PROJECT_PATH}/extern/freeglut/include)
include_directories(${PROJECT_PATH}/extern/glew/include)

set_target_properties(GenericParticleConstraintsDemo PROPERTIES FOLDER "Demos")
set_target_properties(GenericParticleConstraintsDemo PROPERTIES DEBUG_POSTFIX ${PBD_BINARY_DEBUG_POSTFIX})
set_target_properties(GenericParticleConstraintsDemo PROPERTIES RELWITHDEBINFO_POSTFIX ${PBD_BINARY_RELWITHDEBINFO_POSTFIX})
set_target_properties(GenericParticleConstraintsDemo PROPERTIES MINSIZEREL_POSTFIX ${PBD_BINARY_MINSIZEREL_POSTFIX})
add_dependencies(GenericParticleConstraintsDemo ${SIMULATION_DEPENDENCIES})
target_link_libraries(GenericParticleConstraintsDemo ${SIMULATION_LINK_LIBRARIES})

set_target_properties(GenericRigidBodyConstraintsDemo PROPERTIES FOLDER "Demos")
set_target_properties(GenericRigidBodyConstraintsDemo PROPERTIES DEBUG_POSTFIX ${PBD_BINARY_DEBUG_POSTFIX})
set_target_properties(GenericRigidBodyConstraintsDemo PROPERTIES RELWITHDEBINFO_POSTFIX ${PBD_BINARY_RELWITHDEBINFO_POSTFIX})
set_target_properties(GenericRigidBodyConstraintsDemo PROPERTIES MINSIZEREL_POSTFIX ${PBD_BINARY_MINSIZEREL_POSTFIX})
add_dependencies(GenericRigidBodyConstraintsDemo ${SIMULATION_DEPENDENCIES})
target_link_libraries(GenericRigidBodyConstraintsDemo ${SIMULATION_LINK_LIBRARIES})

VIS_SOURCE_GROUPS()

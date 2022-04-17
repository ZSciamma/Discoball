
if(NOT "/home/accts/zas8/Documents/temp/PositionBasedDynamics/Demos/GenericParameters/src/Ext_GenericParameters-stamp/Ext_GenericParameters-gitinfo.txt" IS_NEWER_THAN "/home/accts/zas8/Documents/temp/PositionBasedDynamics/Demos/GenericParameters/src/Ext_GenericParameters-stamp/Ext_GenericParameters-gitclone-lastrun.txt")
  message(STATUS "Avoiding repeated git clone, stamp file is up to date: '/home/accts/zas8/Documents/temp/PositionBasedDynamics/Demos/GenericParameters/src/Ext_GenericParameters-stamp/Ext_GenericParameters-gitclone-lastrun.txt'")
  return()
endif()

execute_process(
  COMMAND ${CMAKE_COMMAND} -E rm -rf "/home/accts/zas8/Documents/temp/PositionBasedDynamics/Demos/GenericParameters/src/Ext_GenericParameters"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to remove directory: '/home/accts/zas8/Documents/temp/PositionBasedDynamics/Demos/GenericParameters/src/Ext_GenericParameters'")
endif()

# try the clone 3 times in case there is an odd git clone issue
set(error_code 1)
set(number_of_tries 0)
while(error_code AND number_of_tries LESS 3)
  execute_process(
    COMMAND "/usr/bin/git"  clone --no-checkout --config "advice.detachedHead=false" "https://github.com/InteractiveComputerGraphics/GenericParameters.git" "Ext_GenericParameters"
    WORKING_DIRECTORY "/home/accts/zas8/Documents/temp/PositionBasedDynamics/Demos/GenericParameters/src"
    RESULT_VARIABLE error_code
    )
  math(EXPR number_of_tries "${number_of_tries} + 1")
endwhile()
if(number_of_tries GREATER 1)
  message(STATUS "Had to git clone more than once:
          ${number_of_tries} times.")
endif()
if(error_code)
  message(FATAL_ERROR "Failed to clone repository: 'https://github.com/InteractiveComputerGraphics/GenericParameters.git'")
endif()

execute_process(
  COMMAND "/usr/bin/git"  checkout 9f1570aad21017c0116adf6a42bc4a19547df6ec --
  WORKING_DIRECTORY "/home/accts/zas8/Documents/temp/PositionBasedDynamics/Demos/GenericParameters/src/Ext_GenericParameters"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to checkout tag: '9f1570aad21017c0116adf6a42bc4a19547df6ec'")
endif()

set(init_submodules TRUE)
if(init_submodules)
  execute_process(
    COMMAND "/usr/bin/git"  submodule update --recursive --init 
    WORKING_DIRECTORY "/home/accts/zas8/Documents/temp/PositionBasedDynamics/Demos/GenericParameters/src/Ext_GenericParameters"
    RESULT_VARIABLE error_code
    )
endif()
if(error_code)
  message(FATAL_ERROR "Failed to update submodules in: '/home/accts/zas8/Documents/temp/PositionBasedDynamics/Demos/GenericParameters/src/Ext_GenericParameters'")
endif()

# Complete success, update the script-last-run stamp file:
#
execute_process(
  COMMAND ${CMAKE_COMMAND} -E copy
    "/home/accts/zas8/Documents/temp/PositionBasedDynamics/Demos/GenericParameters/src/Ext_GenericParameters-stamp/Ext_GenericParameters-gitinfo.txt"
    "/home/accts/zas8/Documents/temp/PositionBasedDynamics/Demos/GenericParameters/src/Ext_GenericParameters-stamp/Ext_GenericParameters-gitclone-lastrun.txt"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to copy script-last-run stamp file: '/home/accts/zas8/Documents/temp/PositionBasedDynamics/Demos/GenericParameters/src/Ext_GenericParameters-stamp/Ext_GenericParameters-gitclone-lastrun.txt'")
endif()


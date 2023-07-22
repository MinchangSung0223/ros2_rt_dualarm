###
# Set minimum version of CMake. Since command 'project' use
# VERSION sub-option we need at least 3.0.
# Note: If you use 2.6 or 2.4, God kills a kitten. Seriously.
cmake_minimum_required(VERSION 3.8)

# THIS HAS TO COME BEFORE THE PROJECT LINE
#set(CMAKE_C_COMPILER "gcc")
#set(CMAKE_CXX_COMPILER "gcc")

####
# Set variables:
#   * PROJECT_NAME
#   * PROJECT_VERSION
project(ros2_rt_dualarm VERSION 2.0.0 LANGUAGES C CXX)

# Use MACOSX_RPATH by default on OS X. This was added in CMake 2.8.12 and
# became default in CMake 3.0. Explicitly setting this policy is necessary to
# suppress a warning in CMake 3.0 and above.
if(POLICY CMP0042)
    cmake_policy(SET CMP0042 NEW)
endif()

# Simplify variable reference and escape sequence evaluation. This was added in
# CMake 3.1. Explicitly setting this policy is necessary to suppress a warning
# in CMake 3.1 and above.
if(POLICY CMP0053)
    cmake_policy(SET CMP0053 NEW)
endif()

cmake_policy(SET CMP0022 NEW)
if (POLICY CMP0048)
    cmake_policy(SET CMP0048 NEW)
endif()

if (POLICY CMP0025)
    cmake_policy(SET CMP0025 NEW)
endif()

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
include(FindPkgConfig)
include(modules/CMakeFunctions.cmake)

set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR})
set(PROJ_INSTALL_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/bin/inc)
set(PROJ_INSTALL_BIN_DIR ${PROJECT_SOURCE_DIR}/bin)
set(PROJ_INSTALL_LIB_DIR ${PROJECT_SOURCE_DIR}/lib)

set(CMAKE_DEBUG_POSTFIX "d")
set(CMAKE_MODULE_PATH  ${CMAKE_CURRENT_SOURCE_DIR}/modules)
set(BUILD_SHARED_LIBS TRUE)


set(PROJECT_NAME_ROS2_SIM ${PROJECT_NAME}_ros2_sim)
set(PROJECT_NAME_BULLET_SIM ${PROJECT_NAME}_bullet_sim)
set(PROJECT_NAME_TRAJ_TEST ${PROJECT_NAME}_traj_test)
set(PROJECT_NAME_TRAJ_RUCKIG_TEST ${PROJECT_NAME}_traj_ruckig_test)


set(Etherlab_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib/etherlab)
set(BULLET3_HEADER_DIR ${PROJECT_SOURCE_DIR}/include/bullet3)
set(SIM_ROBOT_HEADER_DIR ${PROJECT_SOURCE_DIR}/include/SimRobot)

#===============================================================================
# Build options
#===============================================================================
#option(BUILD_DOC "Build Documentation" off)
option(RTIndy7_VERBOSE "Whether print detailed information in CMake process" ON)
option(RTIndy7_SHARED_LIBS "Build shared libraries" ON)
#===============================================================================
# Build type settings
#===============================================================================
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build, options are: Debug | Release | RelWithDebInfo | MinSizeRel" FORCE)
endif()
string(TOUPPER ${CMAKE_BUILD_TYPE} CMAKE_BUILD_TYPE_UPPERCASE)

set(BUILD_TYPE_DEBUG TRUE)
set(BUILD_TYPE_RELEASE TRUE)
set(BUILD_TYPE_RELWITHDEBINFO FALSE)
set(BUILD_TYPE_MINSIZEREL FALSE)

if("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "DEBUG")
    set(BUILD_TYPE_DEBUG TRUE)
elseif("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "RELEASE")
    set(BUILD_TYPE_RELEASE TRUE)
elseif("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "RELWITHDEBINFO")
    set(BUILD_TYPE_RELWITHDEBINFO TRUE)
elseif("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "MINSIZEREL")
    set(BUILD_TYPE_MINSIZEREL TRUE)
else()
    message(WARNING "CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} unknown. Valid options are: Debug | Release | RelWithDebInfo | MinSizeRel")
endif()

#===============================================================================
# Compiler flags
#===============================================================================
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag(-std=c++17 CXX17_SUPPORT)
if(CXX17_SUPPORT)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS OFF)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
else()
    check_cxx_compiler_flag(-std=c++14 CXX14_SUPPORT)
    set(CMAKE_CXX_STANDARD 14)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS OFF)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14" )
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -fPIC" )
execute_process(
        COMMAND ${CMAKE_CXX_COMPILER} -dumpfullversion -dumpversion OUTPUT_VARIABLE GCC_VERSION)
if(GCC_VERSION VERSION_LESS 5.3.1)
    message(FATAL_ERROR "The installed g++ version is ${GCC_VERSION}. ${PROJECT_NAME} requires g++ 5.3.1 or greater.")
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "-g -fno-omit-frame-pointer -fno-inline-functions -fno-inline-functions-called-once -fno-optimize-sibling-calls")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELEASE} ${CMAKE_CXX_FLAGS_DEBUG}")
set(CMAKE_CXX_FLAGS_PROFILE "${CMAKE_CXX_FLAGS_DEBUG} -pg")
set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--no-undefined")


find_package(jsoncpp REQUIRED)
find_package(ruckig REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)


set(ECAT_HEADERS_DIR ${PROJECT_SOURCE_DIR}/include/EcatSystem)
set(INTP_HEADERS_DIR ${PROJECT_SOURCE_DIR}/include/Interpolator)
set(MR_HEADERS_DIR ${PROJECT_SOURCE_DIR}/include/MR)
set(NET_HEADERS_DIR ${PROJECT_SOURCE_DIR}/include/Network)
set(ROBOT_HEADERS_DIR ${PROJECT_SOURCE_DIR}/include/Robot)
set(FT_HEADERS_DIR ${PROJECT_SOURCE_DIR}/include/FTread)
set(CANLIB_HEADERS_DIR ${PROJECT_SOURCE_DIR}/include/canlib)
include_directories(${PROJ_INSTALL_INCLUDE_DIR})
include_directories(
        ${BULLET3_HEADER_DIR}        
        ${ECAT_HEADERS_DIR}
        ${INTP_HEADERS_DIR}
        ${MR_HEADERS_DIR}
        ${NET_HEADERS_DIR}
        ${ROBOT_HEADERS_DIR}
        ${SIM_ROBOT_HEADER_DIR}
        ${FT_HEADERS_DIR}
        ${CANLIB_HEADERS_DIR}
)
# 하위 디렉토리 경로를 리스트로 정의
set(SUBDIRECTORIES
    "include/EcatSystem"
    "include/Interpolator"
    "include/MR"
    "include/Network"
    "include/Robot"
    "include/bullet3"
    "include/SimRobot"
    "include/FTread"
)
# for문을 사용하여 각 하위 디렉토리를 추가
foreach(SUBDIR ${SUBDIRECTORIES})
    add_subdirectory(${SUBDIR})
endforeach()

add_executable(${PROJECT_NAME}     src/RTIndy7Client.cpp)
add_executable(${PROJECT_NAME_ROS2_SIM}    src/TaskSpaceControl/RTIndy7_ros2_sim.cpp)
add_executable(${PROJECT_NAME_BULLET_SIM}        src/BulletSim/RTIndy7_bullet_sim.cpp)
add_executable(${PROJECT_NAME_TRAJ_TEST}        src/TrajectoryTest/RTIndy7_traj_test.cpp)
add_executable(${PROJECT_NAME_TRAJ_RUCKIG_TEST}        src/TrajectoryTest/RTIndy7_traj_ruckig_test.cpp)
set(PROJECT_NAME_LIST
    ${PROJECT_NAME}
    ${PROJECT_NAME_ROS2_SIM}   
    ${PROJECT_NAME_BULLET_SIM} 
    ${PROJECT_NAME_TRAJ_TEST}    
    ${PROJECT_NAME_TRAJ_RUCKIG_TEST}    
)
set(COMMON_LIBRARIES
    sub::ECAT
    sub::Interpolator
    sub::MR
    sub::NETWORK
    sub::Robot
    sub::FTread
    dl
    sub::SimRobot
    pybullet_cpp
    jsoncpp_lib
    ruckig
)
foreach(PROJECT ${PROJECT_NAME_LIST})
    target_link_libraries(${PROJECT} ${COMMON_LIBRARIES})
endforeach()
list(APPEND INCLUDE_DIRS_FOR_DYNAMIC_VIEW ${jSONCPP_INCLUDE_DIRS})
# xenomai
find_package(Xenomai 3.1.1 REQUIRED NATIVE)
message(STATUS "Xenomai found with theses skins: ${Xenomai_SKINS}")
if(Xenomai_FOUND)
    foreach(PROJECT ${PROJECT_NAME_LIST})
    target_include_directories(${PROJECT} PUBLIC ${KERNELHEADERS_DIR})
        foreach(comp_lists ${Xenomai_FOUND_COMP})
            target_include_directories(${PROJECT} PUBLIC ${Xenomai_${comp_lists}_INCLUDE_DIRS})
            target_link_libraries(${PROJECT} ${Xenomai_${comp_lists}_DL_FLAGS})
            set_target_properties(${PROJECT} PROPERTIES COMPILE_FLAGS ${Xenomai_${comp_lists}_DEFINITIONS})
        endforeach(comp_lists ${Xenomai_SKINS})
    endforeach()    
endif(Xenomai_FOUND)

#===============================================================================
# Print build summary
#===============================================================================
if(RTIndy7_VERBOSE)
    message(STATUS "")
    message(STATUS "[ Build summary ]")
    message(STATUS "CMAKE_GENERATOR  : ${CMAKE_GENERATOR}")
    message(STATUS "Compiler ID      : ${CMAKE_CXX_COMPILER_ID}")
    message(STATUS "Compiler version : ${CMAKE_CXX_COMPILER_VERSION}")
    message(STATUS "Build type       : ${CMAKE_BUILD_TYPE}")
    message(STATUS "BUILD_SHARED_LIBS: ${BUILD_SHARED_LIBS}")
    message(STATUS "Install path     : ${CMAKE_INSTALL_PREFIX}")
    message(STATUS "CXX_FLAGS        : ${CMAKE_CXX_FLAGS}")
    if(${CMAKE_BUILD_TYPE_UPPERCASE} STREQUAL "RELEASE")
        message(STATUS "CXX_FLAGS_RELEASE: ${CMAKE_CXX_FLAGS_RELEASE}")
    elseif(${CMAKE_BUILD_TYPE_UPPERCASE} STREQUAL "DEBUG")
        message(STATUS "CXX_FLAGS_DEBUG  : ${CMAKE_CXX_FLAGS_DEBUG}")
    elseif(${CMAKE_BUILD_TYPE_UPPERCASE} STREQUAL "RELWITHDEBINFO")
        message(STATUS "CXX_FLAGS_RELWITHDEBINFO: ${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")
    elseif(${CMAKE_BUILD_TYPE_UPPERCASE} STREQUAL "PROFILE")
        message(STATUS "CXX_FLAGS_PROFILE: ${CMAKE_CXX_FLAGS_PROFILE}")
    endif()
    message(STATUS "CMAKE_SOURCE_DIR : ${CMAKE_SOURCE_DIR}")
    message(STATUS "CMAKE_BINARY_DIR : ${CMAKE_BINARY_DIR}")
    message(STATUS "")
endif(RTIndy7_VERBOSE)

set(ament_target_dependencies_list 
    "robot_state_publisher"
    "sensor_msgs"
    "std_msgs"
    "geometry_msgs"
    "nav_msgs"
    "jsoncpp"
)
foreach(PROJECT ${PROJECT_NAME_LIST})
    ament_target_dependencies(${PROJECT} ${ament_target_dependencies_list})
endforeach()
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD    COMMAND ${CMAKE_COMMAND} -E copy_if_different        ${CMAKE_CURRENT_SOURCE_DIR}/info/MR_info.json lib/${PROJECT_NAME})
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD    COMMAND ${CMAKE_COMMAND} -E copy_if_different        ${CMAKE_CURRENT_SOURCE_DIR}/info/MR_info.json share/${PROJECT_NAME})
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD        COMMAND ${CMAKE_COMMAND} -E copy_if_different            ${CMAKE_CURRENT_SOURCE_DIR}/info/MR_info.json ${CMAKE_CURRENT_SOURCE_DIR}/bin/)        

foreach(PROJECT ${PROJECT_NAME_LIST})
    install(TARGETS ${PROJECT}    DESTINATION lib/${PROJECT_NAME})
    install(TARGETS ${PROJECT_NAME_BULLET_SIM}    DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/bin/)   
endforeach()
install(DIRECTORY  ${CMAKE_CURRENT_SOURCE_DIR}/urdf/    DESTINATION share/${PROJECT_NAME})
install(DIRECTORY  ${CMAKE_CURRENT_SOURCE_DIR}/meshes/visual/    DESTINATION share/${PROJECT_NAME}/meshes/visual/)
install(DIRECTORY  ${CMAKE_CURRENT_SOURCE_DIR}/meshes/collision/    DESTINATION share/${PROJECT_NAME}/meshes/collision/)
install(DIRECTORY  ${CMAKE_CURRENT_SOURCE_DIR}/launch/    DESTINATION share/${PROJECT_NAME})
install(DIRECTORY  ${CMAKE_CURRENT_SOURCE_DIR}/info/   DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/../../)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()


ament_package()

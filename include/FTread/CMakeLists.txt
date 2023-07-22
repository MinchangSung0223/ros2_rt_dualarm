cmake_minimum_required(VERSION 3.8)
project(FTread LANGUAGES CXX)

set(CMAKE_DEBUG_POSTFIX "d")
file(GLOB SOURCE_FILES "*.cpp" "*.c" "*.h")
link_directories(
        "${CMAKE_CURRENT_SOURCE_DIR}/lib/canlib"
      )
add_library(${PROJECT_NAME} SHARED STATIC
            ${SOURCE_FILES}
            )

add_library(sub::FTread ALIAS ${PROJECT_NAME})
set(CANLIB_HEADERS_DIR ${PROJECT_SOURCE_DIR}/include/canlib)
include_directories(${PROJ_INSTALL_INCLUDE_DIR})
include_directories(

        ${CANLIB_HEADERS_DIR}
)
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)
target_include_directories(${PROJECT_NAME}
        INTERFACE ${PROJECT_SOURCE_DIR})
        find_package(Eigen3 3.0 REQUIRED NO_MODULE)
        target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen canlib)
install(TARGETS ${PROJECT_MANE}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)
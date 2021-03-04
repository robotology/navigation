project(keyboardControl)

file(GLOB folder_source *.cpp)
file(GLOB folder_header *.h)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

find_package(YARP 3.3.101 COMPONENTS os sig dev math gsl rosmsg idl_tools REQUIRED)
 find_package(SDL2 CONFIG REQUIRED)

include_directories(${ICUB_INCLUDE_DIRS} ${SDL2_INCLUDE_DIRS} )

add_executable(${PROJECT_NAME} ${folder_source} ${folder_header})

target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} ${SDL2_LIBRARIES} SDL2::SDL2 SDL2::SDL2main )

install(TARGETS ${PROJECT_NAME} DESTINATION bin)

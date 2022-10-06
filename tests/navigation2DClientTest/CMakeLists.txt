project(navigation2DClientTest)

file(GLOB folder_source *.cpp)
file(GLOB folder_header *.h)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

include_directories(${ICUB_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} ${folder_source} ${folder_header})

target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} navigation_lib)

set_property(TARGET navigation2DClientTest PROPERTY FOLDER "Tests")

install(TARGETS ${PROJECT_NAME} DESTINATION bin)

cmake_minimum_required(VERSION 3.0)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# find YARP
find_package(YARP REQUIRED)
find_package( OpenCV REQUIRED )
find_package(YARPBTModules)


# set up our program
set(ASSISTIVE_REHAB_INC         $ENV{ROBOT_CODE}/assistive-rehab/lib/include)
set(YARP_CV                     $ENV{ROBOT_CODE}/yarp/src/libYARP_cv/include)
set(ASSISTIVE_REHAB_SRC         $ENV{ROBOT_CODE}/assistive-rehab/lib/src/dtw.cpp
                                $ENV{ROBOT_CODE}/assistive-rehab/lib/src/helpers.cpp
                                $ENV{ROBOT_CODE}/assistive-rehab/lib/src/skeleton.cpp)

if(YARPBTModules_FOUND)
    message("YARPBTModules FOUND!")
    add_definitions( -DTICK_SERVER)
else()
    message("YARPBTModules NOT found!")
endif()


include_directories (${ASSISTIVE_REHAB_INC} ${YARP_CV})

add_executable(follower
                ./src/Follower.h
                ./src/Follower.cpp
                ./src/FollowerModule.h
                ./src/FollowerModule.cpp
                ./src/TargetRetriver.h
                ./src/Ball3DPointRetriver.h
                ./src/Person3DPointRetriver.h
                ./src/TargetRestriver.cpp
                ./src/Ball3DPointRetriver.cpp
                ./src/Person3DPointRetriver.cpp
                ./src/SimFramePainter.h
                ./src/SimFramePainter.cpp
                ./src/main.cpp
                ${ASSISTIVE_REHAB_SRC})
# link with YARP libraries
target_link_libraries(follower ${YARP_LIBRARIES} ${OpenCV_LIBS} YARPBTModules )

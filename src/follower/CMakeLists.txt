cmake_minimum_required(VERSION 3.0)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# find YARP
find_package(YARP REQUIRED cv)
find_package( OpenCV REQUIRED )
find_package(YARPBTModules)
find_package(AssistiveRehab QUIET)

if(AssistiveRehab_FOUND)
    add_definitions( -DASSISTIVE_REHAB_AVAILABLE)
    message("AssistiveRehab found, adding skeleton retriever module")
else()
    message("AssistiveRehab not found, follower will not have skeleton retiever module")
endif()

# set up our program
set(ASSISTIVE_REHAB_INC         $ENV{ROBOT_CODE}/assistive-rehab/lib/include)
set(YARP_CV                     $ENV{ROBOT_CODE}/yarp/src/libYARP_cv/include)

if(AssistiveRehab_FOUND)
set(ASSISTIVE_REHAB_SRC         $ENV{ROBOT_CODE}/assistive-rehab/lib/src/dtw.cpp
                                $ENV{ROBOT_CODE}/assistive-rehab/lib/src/helpers.cpp
                                $ENV{ROBOT_CODE}/assistive-rehab/lib/src/skeleton.cpp)
endif()

if(YARPBTModules_FOUND)
    message("YARPBTModules FOUND!")
    add_definitions( -DTICK_SERVER)
else()
    message("YARPBTModules NOT found!")
endif()


include_directories (${ASSISTIVE_REHAB_INC} ${YARP_CV})

set(FOLLOWER_FILES ./src/Follower.h
                   ./src/Follower.cpp
                   ./src/FollowerModule.h
                   ./src/FollowerModule.cpp
                   ./src/TargetRetriever.h
                   ./src/Ball3DPointRetriever.h
                   ./src/TargetRetriever.cpp
                   ./src/Ball3DPointRetriever.cpp
                   ./src/SimFramePainter.h
                   ./src/SimFramePainter.cpp
                   ./src/main.cpp
                   ./src/GazeController.h
                   ./src/GazeController.cpp
                   ./src/NavigationController.h
                   ./src/NavigationController.cpp
                   ./src/ObstacleAvoidance.h
                   ./src/ObstacleAvoidance.cpp)
                   
set(ASSISTIVE_REHAB_FOR_MY_PROJ   ./src/Person3DPointRetriever.h
                                  ./src/Person3DPointRetriever.cpp)
                                  
if(AssistiveRehab_FOUND)
    add_executable(follower ${FOLLOWER_FILES}
                            ${ASSISTIVE_REHAB_FOR_MY_PROJ})
else()
    add_executable(follower ${FOLLOWER_FILES})
endif()

message("AssistiveRehab_LIBRARIES ${AssistiveRehab_LIBRARIES}")

# link with YARP libraries
target_link_libraries(follower YARP::YARP_OS 
                               YARP::YARP_sig
                               YARP::YARP_math 
                               YARP::YARP_dev
                               ${OpenCV_LIBS}
                               ${YARP_LIBRARIES})

if(YARPBTModules_FOUND)
    target_link_libraries(follower YARPBTModules)
endif()
 
if(AssistiveRehab_FOUND)
    target_link_libraries(follower ${AssistiveRehab_LIBRARIES})
endif()

install(TARGETS follower DESTINATION bin)

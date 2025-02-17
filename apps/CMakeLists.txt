cmake_minimum_required(VERSION 3.6)

project(libcamera-still)
add_executable(libcamera-still libcamera_still.cpp)
target_link_libraries(libcamera-still libcamera_app outputs images)

project(libcamera-vid)
add_executable(libcamera-vid libcamera_vid.cpp)
target_link_libraries(libcamera-vid libcamera_app encoders outputs)

project(libcamera-hello)
add_executable(libcamera-hello libcamera_hello.cpp)
target_link_libraries(libcamera-hello libcamera_app)

project(libcamera-raw)
add_executable(libcamera-raw libcamera_raw.cpp)
target_link_libraries(libcamera-raw libcamera_app encoders outputs)

project(libcamera-jpeg)
add_executable(libcamera-jpeg libcamera_jpeg.cpp)
target_link_libraries(libcamera-jpeg libcamera_app images)

project(libcamera-motion)
add_executable(libcamera-motion libcamera_motion.cpp)
target_link_libraries(libcamera-motion libcamera_app encoders outputs images)

project(libcamera-motionvid)
add_executable(libcamera-motionvid libcamera_motionvid.cpp)
target_link_libraries(libcamera-motionvid libcamera_app encoders outputs images)

set(EXECUTABLES libcamera-still libcamera-vid libcamera-hello libcamera-raw libcamera-jpeg libcamera-motion libcamera-motionvid)

if (ENABLE_TFLITE)
    project(libcamera-detect)
    add_executable(libcamera-detect libcamera_detect.cpp)
    target_link_libraries(libcamera-detect libcamera_app images)
    set(EXECUTABLES ${EXECUTABLES} libcamera-detect)
endif()

set(EXECUTABLE_OUTPUT_PATH  ${CMAKE_BINARY_DIR})
install(TARGETS ${EXECUTABLES} RUNTIME DESTINATION bin)

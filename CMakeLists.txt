cmake_minimum_required(VERSION 3.1)
project(pong)

find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

set(CMAKE_CXX_COMPILER clang++)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -std=c++11")

set(SOURCE_FILES
    src/main.cpp
    include/utils/lexical_cast.h
    include/utils/fps_counter.h
    include/utils/make_unique.h
    include/utils/scoped_timer.h
    include/utils/logger.h
    src/utils/fps_counter.cpp
    src/utils/scoped_timer.cpp
    src/utils/logger.cpp
    )

include_directories(include)

add_executable(pong ${SOURCE_FILES} include/utils/message_queue.h include/utils/spin_lock.h include/image.h include/bounded_value.h include/motion_detector.h include/window.h include/percent_point.h)
target_link_libraries(pong ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT})
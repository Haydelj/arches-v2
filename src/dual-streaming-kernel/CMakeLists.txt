cmake_minimum_required(VERSION 3.14)

set(PROJECT_NAME "Dual-streaming-kernel")

file(GLOB FILES *.hpp *.cpp)
add_executable(${PROJECT_NAME} ${FILES})
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/dual-streaming-kernel)
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/include)
set_target_properties(${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_BINARY_DIR})
set_target_properties(${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_BINARY_DIR})
set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER ${PROJECT_NAME})

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/riscv)
    file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/riscv DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
endif()

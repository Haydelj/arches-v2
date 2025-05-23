cmake_minimum_required(VERSION 3.14)
add_compile_definitions(UNICODE _UNICODE)

set(PROJECT_NAME "arches-v2")

file(GLOB_RECURSE ALL_INCLUDE CONFIGURE_DEPENDS "*.hpp" "*.h")
file(GLOB_RECURSE ALL_SOURCES CONFIGURE_DEPENDS "*.cpp" "*.cc")
file(GLOB_RECURSE USIMM_CONFIG CONFIGURE_DEPENDS "*.vi" "*.cfg")
file(GLOB_RECURSE RAMU_CONFIG CONFIGURE_DEPENDS "*.yaml")

# Exclude the Trax Unit Folder
foreach(Source IN LISTS ALL_INCLUDE ALL_SOURCES USIMM_CONFIG RAMU_CONFIG)
  get_filename_component(SourceDir ${Source} DIRECTORY)
  string(FIND ${SourceDir} "/units/temp" IsExcludedFolder)
  if(${IsExcludedFolder} EQUAL -1)
    source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${Source})
  endif()
endforeach()

# Exclude the Trax Folder
set(EXCLUDED_FOLDER "/units/temp")
set(NEW_INCLUDE "")
set(NEW_SOURCE "")
foreach(Source IN LISTS ALL_INCLUDE)
  get_filename_component(SourceDir ${Source} DIRECTORY)

  string(FIND ${SourceDir} ${EXCLUDED_FOLDER} IsExcludedFolder)
  if(${IsExcludedFolder} EQUAL -1)
    list(APPEND NEW_INCLUDE ${Source})
  endif()

endforeach()

foreach(Source IN LISTS ALL_SOURCES)
  get_filename_component(SourceDir ${Source} DIRECTORY)

  string(FIND ${SourceDir} ${EXCLUDED_FOLDER} IsExcludedFolder)
  if(${IsExcludedFolder} EQUAL -1)
    list(APPEND NEW_SOURCES ${Source})
  endif()
endforeach()


add_executable(${PROJECT_NAME} ${NEW_INCLUDE} ${NEW_SOURCES} ${USIMM_CONFIG} ${RAMU_CONFIG})

# add_executable(${PROJECT_NAME} ${ALL_INCLUDE} ${ALL_SOURCES})
# source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${ALL_INCLUDE} ${ALL_SOURCES})

target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/external)
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/src/arches-v2)
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/src)
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT DISABLE)

set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME ${PROJECT_NAME})
target_link_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/libraries/tbb)
target_link_libraries(${PROJECT_NAME} PRIVATE tbb12.lib)
target_link_libraries(${PROJECT_NAME} PRIVATE Ramulator)
#set_target_properties(${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_BINARY_DIR})
#set_target_properties(${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_BINARY_DIR})
set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER ${PROJECT_NAME})

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/units/usimm/config-files)
    file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/units/usimm/config-files DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/units/ramulator/config-files)
    file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/units/ramulator/config-files DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
endif()

file(GLOB TBB_DLLS ${PROJECT_SOURCE_DIR}/libraries/tbb/*)
set(RELEASE_DIR ${CMAKE_BINARY_DIR}/Release)
# Check if the destination directory exists, and if not, create it
file(MAKE_DIRECTORY ${RELEASE_DIR})
file(COPY ${TBB_DLLS} DESTINATION ${RELEASE_DIR})
file(COPY ${TBB_DLLS} DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
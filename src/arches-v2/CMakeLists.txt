cmake_minimum_required(VERSION 3.14)
add_compile_definitions(UNICODE _UNICODE)

set(PROJECT_NAME "Arches-v2")


file(GLOB_RECURSE ALL_INCLUDE CONFIGURE_DEPENDS "*.hpp" "*.h")
file(GLOB_RECURSE ALL_SOURCES CONFIGURE_DEPENDS "*.cpp" "*.cc")
file(GLOB_RECURSE USIMM_CONFIG CONFIGURE_DEPENDS "*.vi" "*.cfg")

# Exclude the Trax Unit Folder
foreach(Source IN LISTS ALL_INCLUDE ALL_SOURCES USIMM_CONFIG)
  get_filename_component(SourceDir ${Source} DIRECTORY)
  string(FIND ${SourceDir} "/units/trax" IsExcludedFolder)
  if(${IsExcludedFolder} EQUAL -1)
    source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${Source})
  endif()
endforeach()

# Exclude the Trax Folder
set(EXCLUDED_FOLDER "/units/trax")
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


add_executable(${PROJECT_NAME} ${NEW_INCLUDE} ${NEW_SOURCES} ${USIMM_CONFIG})

# add_executable(${PROJECT_NAME} ${ALL_INCLUDE} ${ALL_SOURCES})
# source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${ALL_INCLUDE} ${ALL_SOURCES})


target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/src/arches-v2)
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/src)
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT DISABLE)

set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME ${PROJECT_NAME})
target_link_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/libraries/tbb)
target_link_libraries(${PROJECT_NAME} tbb12.lib)
set_target_properties(${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_BINARY_DIR})
set_target_properties(${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_BINARY_DIR})
set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER ${PROJECT_NAME})

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/units/usimm/config-files)
    file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/units/usimm/config-files DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
endif()

file(GLOB TBB_DLLS ${PROJECT_SOURCE_DIR}/libraries/tbb/*)
file(COPY ${TBB_DLLS} DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
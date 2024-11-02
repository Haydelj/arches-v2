add_subdirectory(ramulator2 EXCLUDE_FROM_ALL)

# cmake for ramulator
function( build_source_group )
	file(GLOB_RECURSE files ${CMAKE_CURRENT_SOURCE_DIR}/*.* )
	list(REMOVE_ITEM files "CMakeLists.txt")
	source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} PREFIX "[src]" FILES ${files})
endfunction()

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

file(GLOB_RECURSE ALL_INCLUDE CONFIGURE_DEPENDS "*.hpp" "*.h")
file(GLOB_RECURSE ALL_SOURCES CONFIGURE_DEPENDS "*.cpp" "*.cc")

set(EXCLUDED_FOLDERS "/dram_controller/impl/plugin" "/example/impl" "/resources/")
set(NEW_INCLUDE "")
set(NEW_SOURCE "")
foreach(Source IN LISTS ALL_INCLUDE)
  set(should_exclude FALSE)
  foreach(EXCLUDED_FOLDER ${EXCLUDED_FOLDERS})
    get_filename_component(SourceDir ${Source} DIRECTORY)
    string(FIND ${SourceDir} ${EXCLUDED_FOLDER} IsExcludedFolder)
    if(${IsExcludedFolder} GREATER -1)
      set(should_exclude TRUE)
      break()
    endif()
  endforeach()

  if (NOT should_exclude)
    list(APPEND NEW_INCLUDE ${Source})
  endif()
endforeach()

foreach(Source IN LISTS ALL_SOURCES)
  set(should_exclude FALSE)
  foreach(EXCLUDED_FOLDER ${EXCLUDED_FOLDERS})
    get_filename_component(SourceDir ${Source} DIRECTORY)
    string(FIND ${SourceDir} ${EXCLUDED_FOLDER} IsExcludedFolder)
    if(${IsExcludedFolder} GREATER -1)
      set(should_exclude TRUE)
      break()
    endif()
  endforeach()

  if (NOT should_exclude)
    list(APPEND NEW_SOURCES ${Source})
  endif()
endforeach()

list(REMOVE_ITEM NEW_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/ramulator2/src/base/config.cpp")

add_library(Ramulator STATIC ${NEW_INCLUDE} ${NEW_SOURCES})
target_include_directories(Ramulator PUBLIC ${PROJECT_SOURCE_DIR}/external/ramulator2/src)
target_include_directories(Ramulator PUBLIC ${argparse_SOURCE_DIR}/include)
target_include_directories(Ramulator PUBLIC ${yaml-cpp_SOURCE_DIR}/include)
target_include_directories(Ramulator PUBLIC ${spdlog_SOURCE_DIR}/include)
target_link_libraries(
  Ramulator 
  PUBLIC yaml-cpp
  PUBLIC spdlog
)
build_source_group()
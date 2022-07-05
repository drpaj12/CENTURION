cmake_minimum_required(VERSION 2.8.9)
project(CENTURION)

# We require c++14 support
#
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF) #No compiler specific extensions

# options
#option(GA_LAPAGOS_ENABLE_DEBUG_LOGGING "Enable debug logging" OFF)

#
# Warning flags
#
set(WARN_FLAGS "") #The actual warning flags to be applied

set(WARN_FLAGS_TO_CHECK  #The flags to check if the compiler supports
        #GCC-like
        "-Wall"                         #Most warnings, typically good
        "-Wextra"                       #Extra warning, usually good
        "-Wpedantic"                    #Ensure ISO compliance (i.e. no non-standard extensions)
        "-Wcast-qual"                   #Warn if cast removes qualifier (e.g. const char* -> char*)
        "-Wcast-align"                  #Warn if a cast causes memory alignment changes
        "-Wshadow"                      #Warn if local variable shadows another variable
        "-Wformat=2"                    #Sanity checks for printf-like formatting
        "-Wno-format-nonliteral"        # But don't worry about non-literal formtting (i.e. run-time printf format strings)
        "-Wlogical-op"                  #Checks for logical op when bit-wise expected
        "-Wmissing-declarations"        #Warn if a global function is defined with no declaration
        "-Wmissing-include-dirs"        #Warn if a user include directory is missing
        "-Wredundant-decls"             #Warn if there are overlapping declarations
        "-Wswitch-default"              #Warn if a switch has no default
        "-Wundef"                       #Warn if #if() preprocessor refers to an undefined directive
        "-Wunused"                      #Warn about unused variables/parameters
        "-Wunused-variable"             #Warn about variables that are not used
        "-Wunused-parameter"            #Warn about function parameters which are unused
        "-Wdisabled-optimization"       #Warn when optimizations are skipped (usually due to large/complex code)
        "-Wnoexcept"                    #Warn when functions should be noexcept (i.e. compiler know it doesn't throw)
        "-Woverloaded-virtual"          #Warn when a function declaration overrides a virtual method
        "-Wctor-dtor-privacy"           #Warn about inaccessible constructors/destructors
        "-Wnon-virtual-dtor"            #Warn about missing virtual destructors
        "-Wduplicated-cond"             #Warn about identical conditions in if-else chains
        "-Wduplicated-branches"         #Warn when different branches of an if-else chain are equivalent
        "-Wnull-dereference"            #Warn about null pointer dereference execution paths
        "-Wuninitialized"               #Warn about unitialized values
        "-Winit-self"                   #Warn about self-initialization
        "-Wcatch-value=3"               #Warn when catch statements don't catch by reference
        "-Wextra-semi"                  #Warn about redudnant semicolons
        )

#
# Set final flags
#
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${WARN_FLAGS} ${SANITIZE_FLAGS} ${PROFILING_FLAGS} ${COVERAGE_FLAGS} ${LOGGING_FLAGS} ${COLORED_COMPILE}")
message(STATUS "CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")

# makes it so the BIN is installed locally
set(CMAKE_INSTALL_PREFIX .) 

#Bring the headers
include_directories(SRC)
include_directories(LIBS/libargparse/src)
include_directories(/home/robot/SIMULATION_WORK/CENTURION/LIBS/libxml2-2.9.9/LOCAL/include/libxml2)

#However, the file(GLOB...) allows for wildcard additions:
file(GLOB_RECURSE SOURCES 	SRC/*.cpp
				SRC/*.c
)
file(GLOB_RECURSE HEADERS 	SRC/INCLUDE/*.h)

#Create the executable
#add_executable(centurion ${SOURCES} ${HEADERS} ${LIB_HEADERS}) 
add_executable(centurion ${SOURCES} ${HEADERS}) 

# Libraries needed
find_library(ARGPARSE NAMES libargparse argparse HINTS LIBS/libargparse PATH_SUFFIXES libargparse)
target_link_libraries(centurion ${ARGPARSE})
find_library(LIBXML2 NAMES libxml2 xml2 HINTS /home/robot/SIMULATION_WORK/CENTURION/LIBS/libxml2-2.9.9/LOCAL/lib PATH_SUFFIXES libxml2)
target_link_libraries(centurion ${LIBXML2})
target_link_libraries(centurion m)

# Add a top-level "tags" target which includes all files in both
# the build and source versions of src/*.
set_source_files_properties(tags PROPERTIES GENERATED true)
add_custom_target(tags
    COMMAND ctags -R --c++-kinds=+p --fields=+iaS --extra=+q
        ${SOURCES} ${HEADERS} ${LIB_HEADERS}
)

# ...but only make it a dependency of the project if the ctags program
# is available, else it will fail to build on Windows.
find_program(CTAGS_PATH ctags)
if(CTAGS_PATH)
    message(STATUS "Found ctags: ${CTAGS_PATH}")
    add_dependencies(centurion tags)
else()
    message(STATUS "No ctags")
endif(CTAGS_PATH)

# CLEAN instructions
# clean up cmake files by a "make clean"
add_custom_target(clean-cmake-files
   COMMAND ${CMAKE_COMMAND} -P clean-all.cmake
)

# clean-all.cmake
set(cmake_generated ${CMAKE_BINARY_DIR}/CMakeCache.txt
                    ${CMAKE_BINARY_DIR}/cmake_install.cmake
                    ${CMAKE_BINARY_DIR}/Makefile
                    ${CMAKE_BINARY_DIR}/CMakeFiles
)

foreach(file ${cmake_generated})

  if (EXISTS ${file})
     file(REMOVE_RECURSE ${file})
  endif()

endforeach(file)

# INSTALL instructions
install(DIRECTORY DESTINATION SANDBOX)

install(TARGETS centurion DESTINATION BIN)
install(TARGETS centurion DESTINATION SANDBOX)

# moves the current .gdbinit to SANDBOX for playing with
# install(FILES DEBUG/.gdbinit DESTINATION SANDBOX)

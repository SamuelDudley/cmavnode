cmake_minimum_required(VERSION 2.8)
project("cmavnode" C CXX)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING
        "Choose the type of build, options are: Debug, Release."
        FORCE)
endif(NOT CMAKE_BUILD_TYPE)

configure_file(log.conf log.conf COPYONLY)

set(Boost_USE_STATIC_LIBS OFF) ### OFF is required for shared library

### python
find_package( Boost COMPONENTS python REQUIRED )
find_package( PythonLibs 2.7 REQUIRED )
###

find_package( Boost 1.40 COMPONENTS program_options thread system REQUIRED )
find_package( Threads )

INCLUDE_DIRECTORIES(
${Boost_INCLUDE_DIR}
${PYTHON_INCLUDE_DIRS}
)

file(GLOB cmavnode_SRC
    "src/*.cpp"
    )

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

set(CMAKE_CXX_FLAGS_RELEASE "-DNDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG " -ggdb")

## shared library for python import
add_library(${PROJECT_NAME} SHARED ${cmavnode_SRC})
target_compile_options(${PROJECT_NAME} PRIVATE -std=c++11 -ggdb -DMUASMAV -DELPP_NO_DEFAULT_LOG_FILE -DELP_STACKTRACE_ON_CRASH -DELPP_THREAD_SAFE )
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${Boost_LIBRARIES} ${PYTHON_LIBRARIES} -lpthread -lreadline -lconfig++ -lboost_python -lboost_system)

ADD_LIBRARY(${PROJECT_NAME}py SHARED src/wrapper.cpp)
target_compile_options(${PROJECT_NAME}py PRIVATE -std=c++11 -ggdb -DMUASMAV -DELPP_NO_DEFAULT_LOG_FILE -DELP_STACKTRACE_ON_CRASH -DELPP_THREAD_SAFE )
TARGET_LINK_LIBRARIES(${PROJECT_NAME}py ${PROJECT_NAME} ${Boost_LIBRARIES} ${PYTHON_LIBRARIES} -lpthread -lreadline -lconfig++ -lboost_python -lboost_system)


## actual executable
#add_executable(${PROJECT_NAME} ${cmavnode_SRC})
#target_compile_options(${PROJECT_NAME} PRIVATE -std=c++11 -ggdb -DMUASMAV -DELPP_NO_DEFAULT_LOG_FILE -DELP_STACKTRACE_ON_CRASH -DELPP_THREAD_SAFE )
#TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${Boost_LIBRARIES} ${PYTHON_LIBRARIES} -lpthread -lreadline -lconfig++ -lboost_python -lboost_system)

#install(TARGETS ${PROJECT_NAME}2 DESTINATION /opt/cmavnode)
#install(DIRECTORY DESTINATION /opt/${PROJECT_NAME}/log)
#install(FILES log.conf DESTINATION /opt/${PROJECT_NAME})

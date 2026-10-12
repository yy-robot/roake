#-------------------------------------------------------------------------------
# example executables setup

project(librokae-examples CXX)

# Here puts required example source files
set(EXAMPLES
    sdk_example
    move_example
    path_record
    rl_project
    study_example
    vision_pick
    smooth_move_test
    gripper_do_example
    ksl
    ksl_test
    Y_study_exp
    )

# static link
find_package (Threads REQUIRED) #threading
set(THREADS_PREFER_PTHREAD_FLAG ON)

foreach(example ${EXAMPLES})
    add_executable(${example} cpp/${example}.cpp)
    target_include_directories(${example} PUBLIC
        ${SHARED_INCLUDEDIR}
        ${EXTERNAL_INCLUDEDIR})
    target_link_libraries(${example} ${ROKAE_LIB} Threads::Threads)
    if(MSVC)
        target_compile_definitions(${example} PRIVATE _USE_MATH_DEFINES)
        target_compile_options(${example} PRIVATE /utf-8)
    endif() # for M_PI in cmath
endforeach()

if(WIN32)
    target_link_libraries(vision_pick ws2_32)
endif()

# check for xMateModel library option
if(UNIX AND XCORE_USE_XMATE_MODEL)
    set(EXAMPLES_USE_XMATEMODEL
        xmatemodel_er3_er7p
        )
    foreach(example ${EXAMPLES_USE_XMATEMODEL})
        add_executable(${example} cpp/${example}.cpp)
        target_include_directories(${example} PUBLIC
            ${SHARED_INCLUDEDIR}
            ${EXTERNAL_INCLUDEDIR})
        target_link_libraries(${example} ${ROKAE_LIB} Threads::Threads)
        target_compile_definitions(${example} PUBLIC XMATEMODEL_LIB_SUPPORTED)
    endforeach()
endif()

# we use static library
remove_definitions(ROKAE_DLL_BUILD)

# ------------------------------------------------------------------------------
add_subdirectory(cpp/rt)

#-------------------------------------------------------------------------------
# Installation

include(GNUInstallDirs)
install(TARGETS ${EXAMPLES}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )

#-------------------------------------------------------------------------------
# RT mode example executables setup

# Here puts required example source files
set(RT_EXAMPLES
    cartesian_impedance_control
    move_commands
    )

# static link
find_package (Threads REQUIRED) #threading
set(THREADS_PREFER_PTHREAD_FLAG ON)

foreach(example ${RT_EXAMPLES})
    add_executable(${example} ${example}.cpp ${librokae-examples_SOURCE_DIR}/cpp/print_helper.hpp)
    target_include_directories(${example} PUBLIC
        ${SHARED_INCLUDEDIR}
        ${EXTERNAL_INCLUDEDIR})
    target_link_libraries(${example} ${ROKAE_LIB} Threads::Threads)
    if(MSVC)
        add_compile_definitions(_USE_MATH_DEFINES)
    endif() # for M_PI in cmath
endforeach()

# we use static library
remove_definitions(ROKAE_DLL_BUILD)

# check for xMateModel library option

if(UNIX AND XCORE_USE_XMATE_MODEL)
    set(EXAMPLES_USE_XMATEMODEL
        torque_control
        follow_joint_position
        )
    foreach(example ${EXAMPLES_USE_XMATEMODEL})
        add_executable(${example} ${example}.cpp ${librokae-examples_SOURCE_DIR}/cpp/print_helper.hpp)
        target_include_directories(${example} PUBLIC
            ${SHARED_INCLUDEDIR}
            ${EXTERNAL_INCLUDEDIR})
        target_link_libraries(${example} ${ROKAE_LIB} Threads::Threads)
        target_compile_definitions(${example} PUBLIC XMATEMODEL_LIB_SUPPORTED)
    endforeach()
endif()

#-------------------------------------------------------------------------------
# Installation

include(GNUInstallDirs)
install(TARGETS ${EXAMPLES}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )

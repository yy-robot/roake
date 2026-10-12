if(WIN32)
    if(${CMAKE_SIZEOF_VOID_P} EQUAL 8) #64bit
	    set(ROKAE_LIB "${CMAKE_CURRENT_SOURCE_DIR}/Windows/cpp/${CMAKE_BUILD_TYPE}/64bit/RokaeSDK.lib")
	elseif(${CMAKE_SIZEOF_VOID_P} EQUAL 4) #32bit
	    set(ROKAE_LIB "${CMAKE_CURRENT_SOURCE_DIR}/Windows/cpp/${CMAKE_BUILD_TYPE}/32bit/RokaeSDK.lib")
	endif()
elseif(UNIX)
    set(ROKAE_LIB "${CMAKE_CURRENT_SOURCE_DIR}/Linux/cpp/x86_64/libRokaeSDK.a")
endif()

set(ROKAE_LIB ${ROKAE_LIB} CACHE INTERNAL "Rokae static library file")
	
# Public headers
set(SHARED_INCLUDEDIR ${CMAKE_CURRENT_SOURCE_DIR} PARENT_SCOPE)

# Here puts required headers
set(SHARED_HEADERS
    base.h
    robot.h
    model.h
    data_types.h
    exception.h
    planner.h
    motion_control_rt.h
    utility.h
    )

list(TRANSFORM SHARED_HEADERS PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/rokae/")
set(SHARED_HEADERS ${SHARED_HEADERS} CACHE INTERNAL "Shared headers with examples")
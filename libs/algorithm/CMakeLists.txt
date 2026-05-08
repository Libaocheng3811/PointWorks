file(GLOB AlgoSrcs CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
file(GLOB AlgoHdrs CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")

add_library(pw_algorithm STATIC ${AlgoSrcs} ${AlgoHdrs})

target_include_directories(pw_algorithm PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(pw_algorithm
    PUBLIC
        pw_core
    PRIVATE
        ${PCL_LIBRARIES}
        CSF_Lib
        OpenMP::OpenMP_CXX
)

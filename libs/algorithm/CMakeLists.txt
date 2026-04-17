file(GLOB AlgoSrcs CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
file(GLOB AlgoHdrs CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")

add_library(ct_algorithm STATIC ${AlgoSrcs} ${AlgoHdrs})

target_include_directories(ct_algorithm PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(ct_algorithm
    PUBLIC
        ct_core
    PRIVATE
        ${PCL_LIBRARIES}
        ${VTK_LIBRARIES}
        CSF_Lib
        OpenMP::OpenMP_CXX
)

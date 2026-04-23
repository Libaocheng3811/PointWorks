file(GLOB Python_Srcs CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/bindings/*.cpp"
)
file(GLOB Python_Hdrs CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/bindings/*.h"
)

add_library(ct_python OBJECT ${Python_Srcs} ${Python_Hdrs})

target_include_directories(ct_python PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(ct_python
    PUBLIC
    Qt${QT_VERSION_MAJOR}::Core
    pybind11::embed
    Python3::Python
    PRIVATE
    ct_core
    ct_algorithm
)

file(GLOB Python_Srcs CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/bindings/*.cpp"
)
file(GLOB Python_Hdrs CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/bindings/*.h"
)

add_library(pw_python OBJECT ${Python_Srcs} ${Python_Hdrs})

target_include_directories(pw_python PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(pw_python
    PUBLIC
    Qt${QT_VERSION_MAJOR}::Core
    pybind11::embed
    Python3::Python
    PRIVATE
    pw_core
    pw_algorithm
)

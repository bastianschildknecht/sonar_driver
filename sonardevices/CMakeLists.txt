add_library(sonardevices sonardevices.cxx)

target_include_directories(sonardevices PUBLIC $(CMAKE_CURRENT_SOURCE_DIR))
target_link_libraries(sonardevices LINK_PUBLIC ezsocket)
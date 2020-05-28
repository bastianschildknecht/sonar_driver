add_library(sonardevices sonardevices.cxx)

target_link_libraries(sonardevices LINK_PUBLIC ezsocket)
target_include_directories(sonardevices PUBLIC $(CMAKE_CURRENT_SOURCE_DIR))
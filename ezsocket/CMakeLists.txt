add_library(ezsocket ezsocket.cxx socketWorker.cxx)

target_include_directories(ezsocket PUBLIC $(CMAKE_CURRENT_SOURCE_DIR))
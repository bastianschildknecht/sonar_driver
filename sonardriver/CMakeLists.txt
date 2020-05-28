add_executable(sonarDriver sonardriver.cxx)

if(WIN32)
  target_link_libraries(sonarDriver LINK_PUBLIC ezsocket sonardevices wsock32 ws2_32)
else()
  target_link_libraries(sonarDriver LINK_PUBLIC ezsocket sonardevices)
endif()
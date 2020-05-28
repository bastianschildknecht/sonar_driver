cmake_minimum_required(VERSION "3.16.3")

project(SonarDriver VERSION 0.1.0 LANGUAGES C CXX)

add_subdirectory(ezsocket)
add_subdirectory(sonardevices)
add_subdirectory(sonardriver)
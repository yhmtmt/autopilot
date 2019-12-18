add_library(autopilot SHARED f_autopilot.cpp)

target_include_directories(autopilot PUBLIC ${PROJECT_SOURCE_DIR}/include)
install(TARGETS autopilot DESTINATION lib)

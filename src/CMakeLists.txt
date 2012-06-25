ADD_EXECUTABLE(kitti2pcd kitti2pcd.cpp)
TARGET_LINK_LIBRARIES(kitti2pcd ${Boost_LIBRARIES} ${PCL_LIBRARIES})

ADD_EXECUTABLE(kittitrackletextract kittitrackletextract.cpp ${CMAKE_SOURCE_DIR}/include/tracklets.h)
TARGET_LINK_LIBRARIES(kittitrackletextract ${Boost_LIBRARIES} ${PCL_LIBRARIES})

INSTALL(TARGETS kitti2pcd kittitrackletextract
  RUNTIME DESTINATION bin COMPONENT app
  LIBRARY DESTINATION lib COMPONENT runtime
  ARCHIVE DESTINATION lib COMPONENT runtime
  PUBLIC_HEADER DESTINATION include COMPONENT dev
  FRAMEWORK DESTINATION "/Library/Frameworks"
)

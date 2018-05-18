cmake_minimum_required(VERSION 2.8)
project( Filters-VO )
find_package( OpenCV 3.2 REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )

file(GLOB Circle
    "src/*.h"
    "src/circle.cpp"
)
add_executable( circle ${Circle} )
target_link_libraries( circle ${OpenCV_LIBS} )

file(GLOB SearchMovement
    "src/*.h"
    "src/searchMovement.cpp"
)
add_executable( searchMovement ${SearchMovement} )
target_link_libraries( searchMovement ${OpenCV_LIBS} )

file(GLOB MotionTracking
    "src/*.h"
    "src/motionTracking.cpp"
)
add_executable( motionTracking ${MotionTracking} )
target_link_libraries( motionTracking ${OpenCV_LIBS} )

file(GLOB ArucoTeste
    "src/*.h"
    "src/aruco.cpp"
)
add_executable( aruco ${ArucoTeste} )
target_link_libraries( aruco ${OpenCV_LIBS} )

file(GLOB CameraCalibration
    "src/*.h"
    "src/calibration.cpp"
)
add_executable( calibration ${CameraCalibration} )
target_link_libraries( calibration ${OpenCV_LIBS} )

file(GLOB LandingAruco
    "src/*.h"
    "src/landing-aruco.cpp"
)
add_executable( landing-aruco ${LandingAruco} )
target_link_libraries( landing-aruco ${OpenCV_LIBS} )


cmake_minimum_required(VERSION 2.8)
project( Filter )
find_package( OpenCV 3.2 REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )

file(GLOB circle
    "src/*.h"
    "src/circle.cpp"
)
add_executable( c ${circle} )
target_link_libraries( c ${OpenCV_LIBS} )

file(GLOB SearchMovement
    "src/*.h"
    "src/SearchMovement.cpp"
)
add_executable( sm ${SearchMovement} )
target_link_libraries( sm ${OpenCV_LIBS} )

file(GLOB MotionTracking
    "src/*.h"
    "src/MotionTracking.cpp"
)
add_executable( mt ${MotionTracking} )
target_link_libraries( mt ${OpenCV_LIBS} )

file(GLOB ArucoMarkes
    "src/*.h"
    "src/aruco.cpp"
)
add_executable( aruco ${ArucoMarkes} )
target_link_libraries( aruco ${OpenCV_LIBS} )

file(GLOB CameraCalibration
    "src/*.h"
    "src/cameraCalibration.cpp"
)
add_executable( calibration ${CameraCalibration} )
target_link_libraries( calibration ${OpenCV_LIBS} )

file(GLOB LandingAruco
    "src/*.h"
    "src/landing-aruco.cpp"
)
add_executable( land-aruco ${LandingAruco} )
target_link_libraries( land-aruco ${OpenCV_LIBS} )

file(GLOB ArucoDictionary
    "src/*.h"
    "src/aruco_dictionary.cpp"
)
add_executable( aruco-dictionary ${ArucoDictionary} )
target_link_libraries( aruco-dictionary ${OpenCV_LIBS} )


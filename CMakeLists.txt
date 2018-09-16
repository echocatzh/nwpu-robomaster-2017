project(rmvision)
cmake_minimum_required(VERSION 2.8)

add_compile_options(--std=c++11)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED )
include_directories(${Boost_INCLUDE_DIRS}/ ${PROJECT_SOURCE_DIR}/)

add_library(RMVideoCapture RMVideoCapture.cpp)
target_link_libraries(RMVideoCapture ${OpenCV_LIBS})

add_library(transform transform.cpp)
target_link_libraries(transform  ${OpenCV_LIBS})

add_library(armordetector ArmorDetector.cpp)
target_link_libraries(armordetector ${OpenCV_LIBS})

add_executable(rmvision main.cpp)
target_link_libraries(rmvision transform RMVideoCapture armordetector  SerialInterface)

add_executable(calibrated calibration.cpp)
target_link_libraries(calibrated ${OpenCV_LIBS} RMVideoCapture)

include_directories(${Boost_INCLUDE_DIRS})
add_library(serialnode SHARED SerialNode.cpp transport_serial.cpp wmj_link.cpp)
add_library(SerialInterface SHARED SerialInterface.cpp)
target_link_libraries(serialnode ${Boost_LIBRARIES})
target_link_libraries(SerialInterface serialnode)
target_link_libraries(SerialInterface boost_thread)
target_link_libraries(SerialInterface boost_system)

add_executable(testPort testPort.cpp)
target_link_libraries(testPort SerialInterface)

add_executable(testTf testTf.cpp)
target_link_libraries(testTf transform)

add_executable(testArmor ArmorDetectTest.cpp)
target_link_libraries(testArmor transform RMVideoCapture armordetector  SerialInterface)

add_executable(calibrated_photo calibrated_photo.cpp)
target_link_libraries(calibrated_photo ${OpenCV_LIBS} RMVideoCapture)
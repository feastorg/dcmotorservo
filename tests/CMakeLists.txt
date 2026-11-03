cmake_minimum_required(VERSION 3.16)
project(DCMotorServoTests CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
# br3ttb/PID 1.2.1, the version the PlatformIO registry serves.
FetchContent_Declare(pid
  GIT_REPOSITORY https://github.com/br3ttb/Arduino-PID-Library.git
  GIT_TAG 524a4268fc01e6ea397e7fc5b5d820741e9b662f)
FetchContent_MakeAvailable(pid)

add_library(dcmotorservo STATIC
  ../src/DCMotorServo.cpp
  ../src/DCMotorTacho.cpp
  ${pid_SOURCE_DIR}/PID_v1.cpp
  fakes/Arduino.cpp)
target_include_directories(dcmotorservo PUBLIC ../src fakes ${pid_SOURCE_DIR})
target_compile_definitions(dcmotorservo PUBLIC ARDUINO=100)
target_compile_options(dcmotorservo PRIVATE -Wall -Wextra)

# The library again, with abs() taking a 16-bit int as on the megaavr core.
add_library(dcmotorservo_int16_abs STATIC
  ../src/DCMotorServo.cpp
  ../src/DCMotorTacho.cpp
  ${pid_SOURCE_DIR}/PID_v1.cpp
  fakes/Arduino.cpp)
target_include_directories(dcmotorservo_int16_abs PUBLIC ../src fakes ${pid_SOURCE_DIR})
target_compile_definitions(dcmotorservo_int16_abs PUBLIC ARDUINO=100 FAKE_INT16_ABS)
target_compile_options(dcmotorservo_int16_abs PRIVATE -Wall -Wextra)

enable_testing()
foreach(name test_servo_smoke test_servo_accuracy test_servo_stop test_servo_deadband test_servo_drive test_tacho_resync test_tacho_measure_at_zero test_tacho_speed_loop)
  add_executable(${name} ${name}.cpp)
  target_link_libraries(${name} PRIVATE dcmotorservo)
  add_test(NAME ${name} COMMAND ${name})
endforeach()

add_executable(test_servo_int16_abs test_servo_int16_abs.cpp)
target_link_libraries(test_servo_int16_abs PRIVATE dcmotorservo_int16_abs)
add_test(NAME test_servo_int16_abs COMMAND test_servo_int16_abs)

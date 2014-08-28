rock_library(imu_inemo
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(imu_inemo_bin Main.cpp
    DEPS imu_inemo)


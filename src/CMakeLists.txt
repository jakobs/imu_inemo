rock_library(imu_inemo
    SOURCES Driver.cpp
    HEADERS Driver.hpp
    DEPS_PKGCONFIG iodrivers_base)

rock_executable(imu_inemo_bin Main.cpp
    DEPS imu_inemo)

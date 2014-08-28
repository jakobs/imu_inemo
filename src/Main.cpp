#include <iostream>
#include <imu_inemo/Driver.hpp>

#include <string>

int main(int argc, char** argv)
{
    if( argc < 2 )
    {
	std::cout << "usage: imu_inemo_bin <device>" << std::endl;
	exit(0);
    }
    std::string device_name( argv[1] );

    imu_inemo::Driver driver;
    driver.openSerial( device_name, 115200 ); 
    driver.connect();

    sleep( 2 );

    driver.disconnect();
    driver.close();
    return 0;
}

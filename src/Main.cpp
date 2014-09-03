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

    std::cout << driver.getAvailableLibraries() << std::endl;;
    std::cout << driver.getAvailableSensors() << std::endl;;

    imu_inemo::output_mode mode;
    mode.setMAG();
    mode.setACC();
    mode.setAHRS();
    mode.setFrequency( imu_inemo::output_mode::HZ_100 );

    driver.setOutputMode( mode );
    driver.startAcquisition();

    imu_inemo::sensor_data data;
    for(int i=0; i<10; i++)
    {
	driver.getSensorData( data );
	std::cout 
	    << data.mag[0] << " "  
	    << data.mag[1] << " "  
	    << data.mag[2] << " -  " 
	    << data.acc[0] << " "  
	    << data.acc[1] << " "  
	    << data.acc[2]
	    << std::endl;
    }

    driver.stopAcquisition();

    driver.disconnect();
    driver.close();
    return 0;
}

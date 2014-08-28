#include <boost/test/unit_test.hpp>
#include <imu_inemo/Driver.hpp>

using namespace imu_inemo;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    imu_inemo::Driver driver;
}

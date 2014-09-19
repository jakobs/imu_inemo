#ifndef IMU_INEMO_DRIVER_HPP__
#define IMU_INEMO_DRIVER_HPP__

#include <iodrivers_base/Driver.hpp>
#include <boost/cstdint.hpp>
#include <base/Logging.hpp>
#include <base/Eigen.hpp>

#include <iostream>

namespace imu_inemo
{

template<int n> struct fixed_int { typedef boost::uint64_t type; };
template<> struct fixed_int<8> { typedef boost::uint64_t type; };
template<> struct fixed_int<4> { typedef boost::uint32_t type; };
template<> struct fixed_int<2> { typedef boost::uint16_t type; };
template<> struct fixed_int<1> { typedef boost::uint8_t type; };

struct Message
{
    static const int HEADER_SIZE = 3;

    boost::uint8_t frame_control;
    boost::uint8_t length;
    boost::uint8_t message_id;
    boost::uint8_t payload[61];

    enum frame_type
    {
	CONTROL = 0,
	DATA = 1,
	ACK = 2,
	NACK = 3
    };

    enum frame_version
    {
	VERSION1 = 0
    };

    enum qos_priority
    {
	QOS_NORMAL = 0,
	QOS_MEDIUM = 1,
	QOS_HIGH = 2,
	RFU = 3
    };

    Message()
    {}

    Message( frame_type type, bool ack, bool lfmf, 
	    frame_version version, qos_priority qos, 
	    uint8_t length, uint8_t message_id )
    {
	init( type, ack, lfmf, version, qos, length, message_id );
    }

    /** 
     * @brief initialize message header
     *
     * @param frame_type - type of frame
     * @param ack - true if the frame requires acknowledgement
     * @param lfmf - true if the frame is fragmented and more are to follow
     * @param version - version of the protocol
     * @param qos - priority setting for the frame
     * @param length - length of the payload in bytes (<=61)
     * @param message_id - message id for the payload data
     */
    void init( frame_type type, bool ack, bool lfmf, 
	    frame_version version, qos_priority qos, 
	    uint8_t length, uint8_t message_id )
    {
	frame_control = 
	    (type << 6) | (ack << 5) | 
	    (lfmf << 4) | (version << 2) | 
	    (qos << 0);

	this->length = length;
	this->message_id = message_id;
    }

    int getPacketLength() const
    {
	return length + HEADER_SIZE - 1;
    }
    int getPayloadLength() const
    {
	return length - 1;
    }

    frame_type getFrameType() const
    {
	return static_cast<frame_type>((frame_control >> 6) & 0x03);
    }

    bool getAck() const
    {
	return (frame_control >> 5) & 0x01;
    }

    bool getLfMf() const
    {
	return (frame_control >> 4) & 0x01;
    }

    boost::uint8_t& operator[]( size_t index )
    {
	// a message length of 1 means no payload
	// this is probably because the message_id is counted as payload
	if( index+1 >= length ) 
	    throw std::runtime_error("Message access out of bounds");

	return payload[index];
    }

    template <class T>
    T get( size_t index )
    {
	typename fixed_int<sizeof(T)>::type res = 0;
	for( int i=0; i<sizeof(T); i++ )
	    res |= operator[](index+sizeof(T)-1-i) << (i*8);
	return *reinterpret_cast<T*>(&res);
    }
};

enum message_id
{
    iNEMO_Connect = 0x00,
    iNEMO_Disconnect = 0x01,
    iNEMO_Reset_Board = 0x02,
    iNEMO_Enter_DFU_Mode = 0x03,
    iNEMO_Trace = 0x07,
    iNEMO_Led_Control = 0x08,

    iNEMO_Get_Device_Mode = 0x10,
    iNEMO_Get_MCU_ID = 0x12,
    iNEMO_Get_FW_Version = 0x13,
    iNEMO_Get_HW_Version = 0x14,
    iNEMO_Identify = 0x15,
    iNEMO_Get_AHRS_Library = 0x17,
    iNEMO_Get_Libraries = 0x18,
    iNEMO_Get_Available_Sensors = 0x19,

    iNEMO_Set_Sensor_Parameter = 0x20,
    iNEMO_Get_Sensor_Parameter = 0x21,

    iNEMO_Restore_Default_Parameter = 0x22,
    iNEMO_Save_to_Flash = 0x23,
    iNEMO_Load_from_Flash = 0x24,

    iNEMO_Set_Output_Mode = 0x50,
    iNEMO_Get_Output_Mode = 0x51,
    iNEMO_Start_Acquisition = 0x52,
    iNEMO_Acquisition_Data = 0x52,
    iNEMO_Stop_Acquisition = 0x53,
    iNEMO_Get_Acq_Data = 0x54,
};

struct error_code
{
    boost::uint8_t value;

    explicit error_code( boost::uint8_t value )
	: value( value ) {}

    enum codes
    {
	iNEMO_Forbidden 	      = 0x00,
	iNEMO_Unsupported_command     = 0x01, 
	iNEMO_Out_of_range_value      = 0x02, 
	iNEMO_Not_executable_command  = 0x03, 
	iNEMO_Wrong_syntax            = 0x04, 
	iNEMO_Not_connected	      = 0x05, 
    };

    std::string toString()
    {
	if( value == iNEMO_Forbidden )
	    return "Forbidden";
	if( value == iNEMO_Unsupported_command )
	    return "Unsupported command";
	if( value == iNEMO_Out_of_range_value )
	    return "Out-of-range value";
	if( value == iNEMO_Not_executable_command )
	    return "Not executable command";
	if( value == iNEMO_Wrong_syntax  )
	    return "Wrong syntax";
	if( value == iNEMO_Not_connected )
	    return "Not Connected";
	
	else return "Unknown Error.";
    }
};


struct available_libraries
{
    boost::uint8_t value;

    explicit available_libraries( boost::uint8_t value )
	: value( value ) {}

    bool hasFAT() const { return value & 1 << 4; }
    bool hasTrace() const { return value & 1 << 3; }
    bool hasAltimeter() const { return value & 1 << 2; }
    bool hasCompass() const { return value & 1 << 1; }
    bool hasAHRS() const { return value & 1 << 0; }

    friend std::ostream& operator<< (std::ostream& os, const available_libraries& lib)
    {
	os << "Available libraries: ";
	if( lib.hasFAT() )
	    os << "FAT ";
	if( lib.hasTrace() )
	    os << "Trace ";
	if( lib.hasAltimeter() )
	    os << "Altimeter ";
	if( lib.hasCompass() )
	    os << "Compass ";
	if( lib.hasAHRS() )
	    os << "AHRS ";
	return os;
    };
};

struct available_sensors
{
    boost::uint8_t value;

    explicit available_sensors( boost::uint8_t value )
	: value( value ) {}

    bool hasACC() const { return value & 1 << 4; }
    bool hasGYRO() const { return value & 1 << 3; }
    bool hasMAG() const { return value & 1 << 2; }
    bool hasPRESS() const { return value & 1 << 1; }
    bool hasTEMP() const { return value & 1 << 0; }

    friend std::ostream& operator<< (std::ostream& os, const available_sensors& lib)
    {
	os << "Available sensors: ";
	if( lib.hasACC() )
	    os << "ACC ";
	if( lib.hasGYRO() )
	    os << "Gyro ";
	if( lib.hasMAG() )
	    os << "MAG ";
	if( lib.hasPRESS() )
	    os << "PRESS ";
	if( lib.hasTEMP() )
	    os << "TEMP ";
	return os;
    };
};

struct output_mode
{
    boost::uint8_t feat1;
    boost::uint8_t feat2;
    boost::uint16_t sample_count;

    enum frequency
    {
	HZ_1 = 0,
	HZ_10 = 1,
	HZ_25 = 2,
	HZ_50 = 3,
	HZ_30 = 4,
	HZ_100 = 5,
	HZ_400 = 6,
	SYNC = 7
    };

    output_mode()
	: feat1(0), feat2(0), sample_count(0)
    {}

    void setAHRS() { feat1 |= 1 << 7; }
    bool hasAHRS() { return 1 & feat1 >> 7; }
    void setCompass() { feat1 |= 1 << 6; }
    bool hasCompass() { return 1 & feat1 >> 6; }

    void setRawData() { feat1 |= 1 << 5; }
    bool hasRawData() { return 1 & feat1 >> 5; }

    void setACC() { feat1 |= 1 << 4; }
    bool hasACC() { return 1 & feat1 >> 4; }
    void setGYRO() { feat1 |= 1 << 3; }
    bool hasGYRO() { return 1 & feat1 >> 3; }
    void setMAG() { feat1 |= 1 << 2; }
    bool hasMAG() { return 1 & feat1 >> 2; }
    void setPRESS() { feat1 |= 1 << 1; }
    bool hasPRESS() { return 1 & feat1 >> 1; }
    void setTEMP() { feat1 |= 1 << 0; }
    bool hasTEMP() { return 1 & feat1 >> 0; }

    void setAskData() { feat2 |= 1 << 6; }
    bool hasAskData() { return 1 & feat2 >> 6; }
    void setFrequency( frequency freq ) { feat2 |= freq << 3; }
    frequency getFrequency() { return static_cast<frequency>(7 & feat2 >> 3); }
};

struct sensor_data
{
    boost::uint16_t frame_counter;
    boost::int16_t acc[3];
    boost::int16_t gyro[3];
    boost::int16_t mag[3];
    boost::int32_t press;
    boost::int16_t temp;
    float rpy[3];
    float quat[4];
    float compass[4];

    base::Quaterniond getOrientation() const
    {
	return base::Quaterniond( quat[0], quat[1], quat[2], quat[3] );
    }

    base::Vector3d getAccelerometer() const
    {
	// value is in mg, convert to g
	return Eigen::Map<const Eigen::Matrix<boost::int16_t,3,1> >(acc).cast<double>() / 1e3;
    }

    base::Vector3d getGyro() const
    {
	// value is in degree per second, convert to rad/s
	return Eigen::Map<const Eigen::Matrix<boost::int16_t,3,1> >(gyro).cast<double>() / 180.0 * M_PI;
    }

    base::Vector3d getMagnetometer() const
    {
	// value is in mGauss, convert to Gauss
	return Eigen::Map<const Eigen::Matrix<boost::int16_t,3,1> >(mag).cast<double>() / 1e3;
    }

    void setData( Message& msg, output_mode mode )
    {
	// check if the message is of the correct type first
	if( msg.message_id != iNEMO_Acquisition_Data )
	    throw std::runtime_error( "Wrong message type to read sensor data." );

	// calculate the offsets
	int offset = 0;
	frame_counter = msg.get<boost::uint16_t>( offset ); 
	offset += 2;
	if( mode.hasACC() ) 
	{
	    for( int i=0; i<3; i++ )
	    {
		acc[i] = msg.get<boost::int16_t>( offset ); 
		offset += 2;
	    }
	}
	if( mode.hasGYRO() ) 
	{
	    for( int i=0; i<3; i++ )
	    {
		gyro[i] = msg.get<boost::int16_t>( offset ); 
		offset += 2;
	    }
	}
	if( mode.hasMAG() ) 
	{
	    for( int i=0; i<3; i++ )
	    {
		mag[i] = msg.get<boost::int16_t>( offset ); 
		offset += 2;
	    }
	}
	if( mode.hasPRESS() ) 
	{
	    press = msg.get<boost::int32_t>( offset );
	    offset += 4;
	}
	if( mode.hasTEMP() ) 
	{
	    temp = msg.get<boost::int16_t>( offset );
	    offset += 2;
	}
	if( mode.hasAHRS() ) 
	{
	    for( int i=0; i<3; i++ )
	    {
		rpy[i] = msg.get<float>( offset ); 
		offset += 4;
	    }
	    for( int i=0; i<4; i++ )
	    {
		quat[i] = msg.get<float>( offset ); 
		offset += 4;
	    }
	}
	if( mode.hasCompass() ) 
	{
	    for( int i=0; i<4; i++ )
	    {
		compass[i] = msg.get<float>( offset ); 
		offset += 4;
	    }
	}

	if( offset != msg.getPayloadLength() )
	{
	    LOG_ERROR_S << "Sensor packet length mismatch. "
		<< " Expected " << offset << " bytes, but message has "
		<< msg.getPayloadLength() << " bytes ";
	}
    }
};

class Driver : public iodrivers_base::Driver
{
public:
    static const int MAX_PACKAGE_SIZE = 64;
    Message response;
    output_mode mode;

    Driver() :
	iodrivers_base::Driver( MAX_PACKAGE_SIZE )
    {
    }

    void connect()
    {
	sendCommand( iNEMO_Connect );
	stopAcquisition();
    };

    void disconnect()
    {
	sendCommand( iNEMO_Disconnect );
    };

    available_libraries getAvailableLibraries()
    {
	sendCommand( iNEMO_Get_Libraries );
	return available_libraries(response[0]);
    };

    available_sensors getAvailableSensors()
    {
	sendCommand( iNEMO_Get_Available_Sensors );
	return available_sensors(response[0]);
    };

    void setOutputMode( const output_mode& mode )
    {
	this->mode = mode;
	Message msg( Message::CONTROL, true, 0, 
		Message::VERSION1, Message::QOS_NORMAL, 5, 
		iNEMO_Set_Output_Mode );
	*reinterpret_cast<output_mode*>(msg.payload) = mode;
	sendMessage( msg );
    }

    void startAcquisition()
    {
	sendCommand( iNEMO_Start_Acquisition );
    }

    void stopAcquisition()
    {
	sendCommand( iNEMO_Stop_Acquisition );
    }

    bool getSensorData( sensor_data& data )
    {
	LOG_DEBUG_S << "Read Sensor Data"; 
	if( readPacket() )
	{
	    data.setData( response, mode );
	    return true;
	}
	else
	    return false;
    }

    void sendCommand( message_id id )
    {
	Message msg( Message::CONTROL, true, 0, 
		Message::VERSION1, Message::QOS_NORMAL, 1, 
		id );
	sendMessage( msg );
    }

    bool readPacket()
    {
	return iodrivers_base::Driver::readPacket( 
		reinterpret_cast<boost::uint8_t*>(&response), MAX_PACKAGE_SIZE );
    }

    void sendMessage( const Message& msg )
    {
	LOG_DEBUG_S << "Sending message with id 0x" 
	    << std::hex << (int)msg.message_id << " and length " 
	    << msg.getPacketLength();
	// send out the packet
	writePacket( reinterpret_cast<const boost::uint8_t*>(&msg), msg.getPacketLength() );

	if( msg.getAck() )
	{
	    // packet requires an acknowledgement
	    while( readPacket() )
	    {
		if( response.message_id != msg.message_id )
		{
		    LOG_DEBUG_S << "Got message id 0x" << std::hex << (int)response.message_id
			<< " but was expecting 0x" << std::hex << (int)msg.message_id; 
		    continue;
		}

		if( response.getFrameType() == Message::ACK )
		{
		    LOG_DEBUG_S << "Got ACK response for message id 0x" << std::hex << (int)msg.message_id;
		    return;
		}
		else if( response.getFrameType() == Message::NACK ) 
		{
		    error_code error( response[0] );
		    LOG_DEBUG_S << "Message responded with error code: " << error.toString();
		    throw std::runtime_error( "Message responded with error code: " + error.toString() );
		}
		else
		{
		    throw std::runtime_error( "Not a valid response packet." );
		}
	    }
	}
    }

    int extractPacket(uint8_t const* buffer, size_t buffer_size) const
    {
	if( buffer_size >= Message::HEADER_SIZE )
	{
	    const Message *msg( reinterpret_cast<const Message*>(buffer) );
	    size_t length = msg->getPacketLength();
	    if( length > MAX_PACKAGE_SIZE || length == 0 )
	    {
		// this is really bad, since the stream is out of sync
		// report this, and then flush the entire buffer, since
		// we don't have a way to resync
		LOG_WARN_S << "Got bad message with length of " << length << std::endl;
		return -buffer_size;
	    }
	    if( length <= buffer_size )
		return length;
	    // packet seems to be partial for know
	}

	// there is no particular way of identifying a packet start
	return 0;
    }
};
}

#endif

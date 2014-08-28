#ifndef IMU_INEMO_DRIVER_HPP__
#define IMU_INEMO_DRIVER_HPP__

#include <iodrivers_base/Driver.hpp>
#include <boost/cstdint.hpp>
#include <base/Logging.hpp>

#include <iostream>

namespace imu_inemo
{

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

enum error_code
{
    iNEMO_Forbidden 	      = 0x00,
    iNEMO_Unsupported_command     = 0x01, 
    iNEMO_Out_of_range_value      = 0x02, 
    iNEMO_Not_executable_command  = 0x03, 
    iNEMO_Wrong_syntax            = 0x04, 
    iNEMO_Not_connected	      = 0x05, 
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


class Driver : public iodrivers_base::Driver
{
public:
    static const int MAX_PACKAGE_SIZE = 64;
    Message response;

    Driver() :
	iodrivers_base::Driver( MAX_PACKAGE_SIZE )
    {
	setReadTimeout( base::Time::fromMilliseconds( 200 ) );
    }

    void connect()
    {
	Message msg( Message::CONTROL, true, 0, Message::VERSION1, Message::QOS_NORMAL, 1, iNEMO_Connect );
	sendMessage( msg );
    };

    void disconnect()
    {
	Message msg( Message::CONTROL, true, 0, Message::VERSION1, Message::QOS_NORMAL, 1, iNEMO_Disconnect );
	sendMessage( msg );
    };

    available_libraries getAvailableLibraries()
    {
	Message msg( Message::CONTROL, true, 0, Message::VERSION1, Message::QOS_NORMAL, 1, iNEMO_Get_Libraries );
	sendMessage( msg );
	return available_libraries(response[0]);
    };

    available_sensors getAvailableSensors()
    {
	Message msg( Message::CONTROL, true, 0, Message::VERSION1, Message::QOS_NORMAL, 1, iNEMO_Get_Available_Sensors );
	sendMessage( msg );
	return available_sensors(response[0]);
    };

    void sendMessage( const Message& msg )
    {
	// send out the packet
	writePacket( reinterpret_cast<const boost::uint8_t*>(&msg), msg.getPacketLength() );

	if( msg.getAck() )
	{
	    // packet requires an acknowledgement
	    if( readPacket( reinterpret_cast<boost::uint8_t*>(&response), MAX_PACKAGE_SIZE ) )
	    {
		if( response.message_id != msg.message_id )
		    throw std::runtime_error( "Response message_id did not match packet." );

		if( response.getFrameType() == Message::ACK )
		{
		    return;
		}
		else if( response.getFrameType() == Message::NACK ) 
		{
		    uint8_t error = response[0];
		    throw std::runtime_error( "Message responded with error code " + error );
		}
		else
		{
		    throw std::runtime_error( "Not a valid response packet." );
		}
	    }
	    else
	    {
		throw std::runtime_error( "Did not get a response for packet which requires ACK." );
	    }
	}
    }

    int extractPacket(uint8_t const* buffer, size_t buffer_size) const
    {
	if( buffer_size >= Message::HEADER_SIZE )
	{
	    const Message *msg( reinterpret_cast<const Message*>(buffer) );
	    int length = msg->getPacketLength();
	    if( length > MAX_PACKAGE_SIZE || length == 0 )
	    {
		// this is really bad, since the stream is out of sync
		// report this, and then flush the entire buffer, since
		// we don't have a way to resync
		LOG_WARN_S << "Got bad message with length of " << length << std::endl;
		return -buffer_size;
	    }
	    return length;
	}
	

	// there is no particular way of identifying a packet start
	return 0;
    }
};
}

#endif

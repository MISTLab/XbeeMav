/* CommunicationManager.h -- Communication Manager class for XBee:
			     Handles all communications with other ROS nodes
			     and the serial port --     	             */
/* ------------------------------------------------------------------------- */
/* September 20, 2016 -- @Copyright Aymen Soussia. All rights reserved.      */
/*                                  (aymen.soussia@gmail.com)                */


#pragma once

#include <inttypes.h>
#include<thread>

#include<mavros_msgs/CommandCode.h>
#include<mavros_msgs/CommandInt.h>
#include<mavros_msgs/Mavlink.h>
#include <ros/ros.h>

#include"SerialDevice.h"


namespace Mist
{


namespace Xbee
{


//*****************************************************************************
struct Waypoint_S
{
	unsigned int latitude;
	unsigned int longitude;
	double altitude;
	unsigned int staytime;
	unsigned int heading;
};


//*****************************************************************************
class CommunicationManager
{
public:
	CommunicationManager();
	~CommunicationManager();

	enum class DRONE_TYPE {MASTER, SLAVE};
	enum class RUNNING_MODE {SWARM, SOLO};

	bool Init(const std::string& device, const std::size_t baud_rate);
	void Run(DRONE_TYPE drone_type, RUNNING_MODE running_mode);

private:

	const unsigned char START_DLIMITER;
	const std::size_t LOOP_RATE;

	void Run_In_Solo_Mode(DRONE_TYPE drone_type);
	void Run_In_Swarm_Mode();
	void Generate_Transmit_Request_Frame(
			const char* const message,
			std::string* frame,
			const unsigned char frame_ID = 
			static_cast<unsigned char>(0x01),
			const std::string& destination_adssress = "000000000000FFFF",
			const std::string& short_destination_adress = "FFFF",
			const std::string& broadcast_radius = "00",
			const std::string& options = "00");
	void Check_In_Messages_and_Transfer_To_Topics();
	void Display_Init_Communication_Failure();
	void Convert_HEX_To_Bytes(const std::string& HEX_data,
			std::string* converted_data);
	void Calculate_and_Append_Checksum(std::string* frame);
	void Add_Length_and_Start_Delimiter(std::string* frame);
	void Send_Mavlink_Message_Callback(
			const mavros_msgs::Mavlink::ConstPtr& mavlink_msg);
	void Display_Drone_Type_and_Running_Mode(DRONE_TYPE drone_type,
			RUNNING_MODE running_mode);
	bool Serve_Flight_Controller(mavros_msgs::CommandInt::
		Request& request, mavros_msgs::CommandInt::Response& response);
	void Check_In_Messages_and_Transfer_To_Server();
	unsigned short Caculate_Checksum(std::string* frame);
	Mist::Xbee::SerialDevice serial_device_;
	Thread_Safe_Deque* in_messages_;
	ros::NodeHandle node_handle_;
	ros::Subscriber mavlink_subscriber_;	
	ros::Publisher mavlink_publisher_;
	ros::ServiceClient mav_dji_client_;
	ros::ServiceServer mav_dji_server_;
        /*Vector msgs*/
	std::map< int, std::shared_ptr<std::string> > multi_msgs;
	std::vector<uint16_t> multi_msgs_available;
	uint16_t cur_checksum;
	//uint16_t multi_msg_size;
};


}


}

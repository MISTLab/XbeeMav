/* CommunicationManager.cpp -- Communication Manager class for XBee:
			       Handles all communications with other ROS nodes
			       and the serial port --     	                             */
/* ------------------------------------------------------------------------- */
/* September 20, 2016 -- @Copyright Aymen Soussia. All rights reserved.      */
/*                                  (aymen.soussia@gmail.com)                */


#include "CommunicationManager.h"


namespace Mist
{


namespace Xbee
{


//*****************************************************************************
CommunicationManager::CommunicationManager():
	START_DLIMITER(static_cast<unsigned char>(0x7E)),
	LOOP_RATE(10) /* 10 fps */
{
}


//*****************************************************************************
CommunicationManager::~CommunicationManager()
{
}


//*****************************************************************************
bool CommunicationManager::Init(
		const std::string& device, const std::size_t baud_rate)
{
	if (serial_device_.Init(device, baud_rate))
	{
		in_messages_ = serial_device_.Get_In_Messages_Pointer();
		return true;
	}
	else
	{
		Display_Init_Communication_Failure();
		return false;
	}
}



//*****************************************************************************
void CommunicationManager::Run(DRONE_TYPE drone_type,
		RUNNING_MODE running_mode)
{
	std::thread thread_service(&SerialDevice::Run_Service, &serial_device_);

	Display_Drone_Type_and_Running_Mode(drone_type, running_mode);

	if (RUNNING_MODE::SWARM == running_mode)
		Run_In_Swarm_Mode();
	else
		Run_In_Solo_Mode(drone_type);

	serial_device_.Stop_Service();
  	thread_service.join();
}


//*****************************************************************************
void CommunicationManager::Run_In_Solo_Mode(DRONE_TYPE drone_type)
{
	std::string service_name;
	bool success = false;

	if (DRONE_TYPE::MASTER == drone_type)
	{
		if (node_handle_.getParam("Xbee_In_From_Controller", service_name))
		{
			mav_dji_server_ = node_handle_.advertiseService(service_name.c_str(), 		&CommunicationManager::Serve_Flight_Controller, this);
			success = true;
		}
		else
			std::cout << "Failed to Get Service Name: param 'Xbee_In_From_Controller' Not Found." << std::endl;
	}
	else
 	{
		if (node_handle_.getParam("Xbee_Out_To_Controller", service_name))
		{
			mav_dji_client_ = node_handle_.serviceClient<mavros_msgs::CommandInt>(service_name.c_str());
			success = true;
		}
		else
			std::cout << "Failed to Get Service Name: param 'Xbee_Out_To_Controller' Not Found." << std::endl;
	}

	if (success)
	{
		ros::Rate loop_rate(LOOP_RATE);

		while (ros::ok())
		{
			Check_In_Messages_and_Transfer_To_Server();
			ros::spinOnce();
	    		loop_rate.sleep();
		}
	}

}


//*****************************************************************************
void CommunicationManager::Run_In_Swarm_Mode()
{
	std::string out_messages_topic;
	std::string in_messages_topic;
	bool success_1 = false;
	bool success_2 = false;

	if (node_handle_.getParam("Xbee_In_From_Buzz", out_messages_topic))
	{
		mavlink_subscriber_ = node_handle_.subscribe(out_messages_topic.c_str(), 1000,
			&CommunicationManager::Send_Mavlink_Message_Callback, this);
		success_1 = true;
	}
	else
		std::cout << "Failed to Get Topic Name: param 'Xbee_In_From_Buzz' Not Found." << std::endl;

	if (node_handle_.getParam("Xbee_Out_To_Buzz", in_messages_topic))
	{
		mavlink_publisher_ = node_handle_.advertise<mavros_msgs::Mavlink>(
			in_messages_topic.c_str(), 1000);
		success_2 = true;
	}
	else
		std::cout << "Failed to Get Topic Name: param 'Xbee_Out_To_Buzz' Not Found." << std::endl;

	if (success_1 && success_2)
	{
	
		ros::Rate loop_rate(LOOP_RATE);

		while (ros::ok())
		{
			Check_In_Messages_and_Transfer_To_Topics();
			ros::spinOnce();
	    		loop_rate.sleep();
		}
	}	
}


//*****************************************************************************
inline bool CommunicationManager::Serve_Flight_Controller(mavros_msgs::CommandInt::
		Request& request, mavros_msgs::CommandInt::Response& response)
{
	const std::size_t MAX_BUFFER_SIZE = 255;
	unsigned int command = 0;
	char temporary_buffer[MAX_BUFFER_SIZE];
	std::string frame;

	switch(request.command)
	{
		case mavros_msgs::CommandCode::NAV_TAKEOFF:
		{
			command = static_cast<unsigned int>(mavros_msgs::CommandCode::NAV_TAKEOFF);
			sprintf(temporary_buffer, "%u ", command);
			response.success = true;
			break;
		}
		case mavros_msgs::CommandCode::NAV_LAND:
		{
			command = static_cast<unsigned int>(mavros_msgs::CommandCode::NAV_LAND);
			sprintf(temporary_buffer, "%u ", command);
			response.success = true;
			break;	
		}
		case mavros_msgs::CommandCode::NAV_RETURN_TO_LAUNCH:
		{
			command = static_cast<unsigned int>(mavros_msgs::CommandCode::NAV_RETURN_TO_LAUNCH);
			sprintf(temporary_buffer, "%u ", command);
			response.success = true;
			break;	
		}
		case mavros_msgs::CommandCode::NAV_WAYPOINT:
		{
			command = static_cast<unsigned int>(mavros_msgs::CommandCode::NAV_WAYPOINT);
			sprintf(temporary_buffer, "%u %u %u %lf %u %u ",
					command, request.x, request.y, request.z, 0, 1); // TO DO change 0 and 1
			response.success = true;
			break;	
		}
		case mavros_msgs::CommandCode::CMD_MISSION_START:
		{
			command = static_cast<unsigned int>(mavros_msgs::CommandCode::CMD_MISSION_START);
			sprintf(temporary_buffer, "%u ", command);
			response.success = true;
			break;
		}
		default:
			response.success = false;		
	}

	if (command != 0)
	{
		Generate_Transmit_Request_Frame(temporary_buffer, &frame);
		serial_device_.Send_Frame(frame);
	}
	
	return true;
}


//*****************************************************************************
void CommunicationManager::Display_Init_Communication_Failure()
{
	std::cout << "Failed to Init Communication With XBee." << std::endl;
	std::cout << "Please Check The Following Parameters:" << std::endl;
	std::cout << "   1) Device (e.g. /dev/ttyUSB0 for Linux. " << std::endl;
	std::cout << "   2) Baud Rate." << std::endl;
	std::cout << "   3) Press Reset Button on The XBee." << std::endl;
	std::cout << "   4) Connect and Disconnect The XBee." << std::endl;
	std::cout << "   5) Update The XBee Firmware." << std::endl;
	std::cout << "   6) Reinstall The FTDI Driver." << std::endl;
}


//*****************************************************************************
inline void CommunicationManager::Generate_Transmit_Request_Frame(
		const char* const message,
		std::string * frame,
		const unsigned char frame_ID,
		const std::string& destination_adssress,
		const std::string& short_destination_adress,
		const std::string& broadcast_radius,
		const std::string& options)
{
	const unsigned char FAME_TYPE = static_cast<unsigned char>(0x10); /* Transmit Request Frame */
	std::string frame_parameters;
	std::string bytes_frame_parameters;

	frame_parameters.append(destination_adssress);
	frame_parameters.append(short_destination_adress);
	frame_parameters.append(broadcast_radius);
	frame_parameters.append(options);

	Convert_HEX_To_Bytes(frame_parameters, &bytes_frame_parameters);

	frame->push_back(FAME_TYPE);
	frame->push_back(frame_ID);
	frame->append(bytes_frame_parameters);
	frame->append(message, std::strlen(message));

	Calculate_and_Append_Checksum(frame);
	Add_Length_and_Start_Delimiter(frame);
}


//*****************************************************************************
inline void CommunicationManager::Convert_HEX_To_Bytes(
		const std::string& HEX_data, std::string* converted_data)
{
	const unsigned short TWO_BYTES = 2;
	char temporary_buffer[TWO_BYTES];
	unsigned char temporary_char;
	int temporary_HEX;

	for (unsigned short i = 0; i <= HEX_data.size() - TWO_BYTES;
			i += TWO_BYTES)
	{
		memcpy(temporary_buffer, HEX_data.c_str() + i, TWO_BYTES);
		sscanf(temporary_buffer, "%02X", &temporary_HEX);
		temporary_char = static_cast<unsigned char>(temporary_HEX);
		converted_data->push_back(temporary_char);
	}
}


//*****************************************************************************
inline void CommunicationManager::Calculate_and_Append_Checksum(
		std::string* frame)
{
	unsigned short bytes_sum = 0;
	unsigned lowest_8_bits = 0;
	unsigned short checksum = 0;
	unsigned char checksum_byte;

	for (unsigned short i = 0; i < frame->size(); i++)
		bytes_sum += static_cast<unsigned short>(frame->at(i));

	lowest_8_bits = bytes_sum & 0xFF;
	checksum = 0xFF - lowest_8_bits;
	checksum_byte = static_cast<unsigned char>(checksum);
	frame->push_back(checksum_byte);
}


//*****************************************************************************
inline void CommunicationManager::Add_Length_and_Start_Delimiter(
		std::string* frame)
{
	const unsigned short FIVE_BYTES = 5;
	char temporary_buffer[FIVE_BYTES];
	unsigned char temporary_char;
	int Hex_frame_length_1;
	int Hex_frame_length_2;
	std::string header;
	int frame_length = frame->size() - 1; /* frame_length = frame_size - checksum byte */

	header.push_back(START_DLIMITER);
	sprintf(temporary_buffer, "%04X", frame_length);
	sscanf(temporary_buffer, "%02X%02X", &Hex_frame_length_1, 
			&Hex_frame_length_2);
	temporary_char = static_cast<unsigned char>(Hex_frame_length_1);
	header.push_back(temporary_char);
	temporary_char = static_cast<unsigned char>(Hex_frame_length_2);
	header.push_back(temporary_char);
	frame->insert(0, header);
}


//*****************************************************************************
inline void CommunicationManager::Check_In_Messages_and_Transfer_To_Topics()
{
	std::size_t size_in_messages = in_messages_->Get_Size();
	
	if (size_in_messages > 0)
	{
		uint64_t current_int64 = 0;

		for (std::size_t j = 0; j < size_in_messages; j++)
		{
			std::shared_ptr<std::string> in_message =
					in_messages_->Pop_Front();
			mavros_msgs::Mavlink mavlink_msg;
			
			for (std::size_t i = 0; i < in_message->size(); i++)
			{
				if (' ' == in_message->at(i) || 0 == i)
				{
					sscanf(in_message->c_str() + i, "%" PRIu64 " ",
							&current_int64);
					mavlink_msg.payload64.push_back(current_int64);
				}
		
			}
			
			mavlink_publisher_.publish(mavlink_msg);
		}
		
	}
}


//*****************************************************************************
inline void CommunicationManager::Check_In_Messages_and_Transfer_To_Server()
{
	std::size_t size_in_messages = in_messages_->Get_Size();
	
	if (size_in_messages > 0)
	{
		for (std::size_t j = 0; j < size_in_messages; j++)
		{
			std::shared_ptr<std::string> in_message =
					in_messages_->Pop_Front();
			mavros_msgs:: CommandInt command_msg;
			unsigned int command = 0;

			sscanf(in_message->c_str(), "%u", &command);

			if (static_cast<unsigned int>(mavros_msgs::CommandCode::NAV_WAYPOINT) == command)
			{
				Waypoint_S new_waypoint;
				sscanf(in_message->c_str(), "%u %u %u %lf %u %u ",
					&command, &new_waypoint.latitude,
					&new_waypoint.longitude, &new_waypoint.altitude,
					&new_waypoint.staytime, &new_waypoint.heading);
				command_msg.request.x = new_waypoint.latitude;
				command_msg.request.y = new_waypoint.longitude;
				command_msg.request.z = new_waypoint.altitude;
				// TO DO add staytime and heading;
			}

			command_msg.request.command = command;
			
			if (mav_dji_client_.call(command_msg))
				ROS_INFO("XBeeMav: Command Successfully Tranferred to Dji Mav.");
			else
				ROS_INFO("XBeeMav: Faild to Tranfer Command.");
			
		}
	}
}


//*****************************************************************************
inline void CommunicationManager::Send_Mavlink_Message_Callback(
		const mavros_msgs::Mavlink::ConstPtr& mavlink_msg)
{
	const unsigned short MAX_BUFFER_SIZE = 211; /* 20 (length(uint64_t)) * 10 (max int number) + 10 (spaces) + 1 */
	const unsigned short MAX_NBR_OF_INT64 = 10;
	char temporary_buffer[MAX_BUFFER_SIZE];
	std::string frame;
	int converted_bytes = 0;

	/* Check the payload is not too long. Otherwise ignore it. */
	if (mavlink_msg->payload64.size() <= MAX_NBR_OF_INT64)
	{
		for (unsigned short i = 0; i < mavlink_msg->payload64.size(); i++)
		{
			converted_bytes += sprintf(
				temporary_buffer + converted_bytes, "%" PRIu64 " ",
				(uint64_t)mavlink_msg->payload64.at(i));
		
		}
	
		Generate_Transmit_Request_Frame(temporary_buffer, &frame);
		serial_device_.Send_Frame(frame);
	}
}


//*****************************************************************************
void CommunicationManager::Display_Drone_Type_and_Running_Mode(
		DRONE_TYPE drone_type, RUNNING_MODE running_mode)
{
	if (DRONE_TYPE::MASTER == drone_type)
		std::cout << "Drone: MASTER" << std::endl;
	else
		std::cout << "Drone: SLAVE" << std::endl;

	if (RUNNING_MODE::SOLO == running_mode)
		std::cout << "XBeeMav Running in SOLO Mode..." << std::endl;
	else
		std::cout << "XBeeMav Running in SWARM Mode..." << std::endl;
}


}


}

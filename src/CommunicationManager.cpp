/* CommunicationManager.cpp -- Communication Manager class for XBee:
			       Handles all communications with other ROS nodes
			       and the serial port --     	                             */
/* ------------------------------------------------------------------------- */
/* September 20, 2016 -- @Copyright Aymen Soussia. All rights reserved.      */
/*                                  (aymen.soussia@gmail.com)                */


#include "CommunicationManager.h"


uint16_t* u64_cvt_u16(uint64_t u64){
   uint16_t* out = new uint16_t[4];
   uint32_t int32_1 = u64 & 0xFFFFFFFF;
   uint32_t int32_2 = (u64 & 0xFFFFFFFF00000000 ) >> 32;
    out[0] = int32_1 & 0xFFFF;
    out[1] = (int32_1 & (0xFFFF0000) ) >> 16;
    out[2] = int32_2 & 0xFFFF;
    out[3] = (int32_2 & (0xFFFF0000) ) >> 16;
   //cout << " values " <<out[0] <<"  "<<out[1] <<"  "<<out[2] <<"  "<<out[3] <<"  ";
return out;

}

uint16_t get_deviceid(){
/* Get hostname */
   char hstnm[30];
   gethostname(hstnm, 30);
   /* Make numeric id from hostname */
   /* NOTE: here we assume that the hostname is in the format M100X */
   int id = strtol(hstnm + 4, NULL, 10);
	//fprintf(stdout, "Robot id from get rid buzz util:  %i\n",id);
return (uint16_t)id;
}

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

	/*Get no of devices*/
	node_handle_.getParam("No_of_dev", no_of_dev);
	/*Get device Id feom host name*/
	device_id=get_deviceid();
	std::cout << "Device Id" <<device_id << std::endl;

	if (success_1 && success_2)
	{
	
		ros::Rate loop_rate(LOOP_RATE);
		counter=0;
		while (ros::ok())
		{
			Check_In_Messages_and_Transfer_To_Topics();
			Send_multi_msg();
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
		//Generate_Transmit_Request_Frame(temporary_buffer, &frame);
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
		int tot,
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
	frame->append(message, tot);

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

unsigned short CommunicationManager::Caculate_Checksum(std::string* frame)
{

	unsigned short bytes_sum = 0;
	unsigned lowest_8_bits = 0;
	unsigned short checksum = 0;
	//unsigned char checksum_byte;

	for (unsigned short i = 0; i < frame->size(); i++)
		bytes_sum += static_cast<unsigned short>(frame->at(i));

	lowest_8_bits = bytes_sum & 0xFF;
	checksum = 0xFF - lowest_8_bits;
	
return checksum;
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
	uint16_t* header;
	if (size_in_messages > 0)
	{
		/*if(!multi_msgs_receive.empty()) steps++;
		  if(steps>500){ 
			steps=0;
			multi_msgs_receive.clear();
			receiver_cur_checksum=0;
		}*/
		/*T0 overcome mesages after the stop transmission of multi packet*/
		if(counter!=0){
		 multi_msgs_receive.clear();
		 counter--;
		}
		uint64_t current_int64 = 0;
		for (std::size_t j = 0; j < size_in_messages; j++)
		{
			std::shared_ptr<std::string> in_message =
					in_messages_->Pop_Front();
			mavros_msgs::Mavlink mavlink_msg;
			int tot = 0;
			//uint64_t header=0;
			/*Copy the header*/
			memcpy(&current_int64,in_message->c_str(),sizeof(uint64_t));
			tot+=sizeof(uint64_t);
			/*sscanf(in_message->c_str(), "%" PRIu64 " ",
							&current_int64);*/
			//std::cout<<in_message<< std::endl;
			header = u64_cvt_u16(current_int64);
			//std::cout << "Received header" <<header[0]<<"  "<<header[1]<<"  "<<header[2]<<"  "<<header[3]<<"  "<< std::endl;	
			/*Check header for msgs or ack msg */
			if(header[0]==(uint16_t)MESSAGE_CONSTANT){
				if(header[3]==1 && header[1]>1 && header[2]==1){
					/*copy msg size*/
					uint16_t tmp_size=0;
					memcpy(&tmp_size,in_message->c_str()+tot,sizeof(uint16_t));
					tot+=sizeof(uint16_t);
					//std::cout<<"received size in bytes: "<<tmp_size << std::endl;
					//int uint64_size=tmp_size/sizeof(uint64_t);
					uint64_t message_obt[tmp_size];
					/*Copy obt msg*/
					memcpy(message_obt,in_message->c_str()+tot,tmp_size*sizeof(uint64_t));
					tot+=tmp_size*sizeof(uint64_t);
					//std::cout<<"tot size : "<<tot<< std::endl;
					for (uint16_t i = 0; i < tmp_size; i++)
					{
						mavlink_msg.payload64.push_back(message_obt[i]);
						/*if (' ' == in_message->at(i) || 0 == i)
						{
							sscanf(in_message->c_str() + i, "%" PRIu64 " ",
									&current_int64);
							mavlink_msg.payload64.push_back(current_int64);
						}*/
		
					}
					//std::cout << "Single packet message received size"<<mavlink_msg.payload64.size()<< std::endl;
					mavlink_publisher_.publish(mavlink_msg);
					//delete[] header;
				}
				else if (header[3]>1 && header[1]>1){
					
					/*multimsg received send ack msg*/
					char temporary_buffer[20];
					std::string frame;
					std::cout << "Multi msg Received header " <<header[0]<<"  "<<header[1]<<"  "<<header[2]<<"  "<<header[3]<<"  "<< std::endl;
					if (header[2]==1){
					//std::cout << "first message" << std::endl;
					multi_msgs_receive.clear();
					multi_msgs_receive.insert(make_pair(header[2], in_message));
					receiver_cur_checksum=header[1];
					//counter=1;
					tot = sizeof(uint64_t);
					uint64_t ack_msg =  (uint64_t)ACK_MESSAGE_CONSTANT | ((uint64_t)header[1] << 16) | ((uint64_t)header[2] << 32) |((uint64_t) device_id << 48) ;
					//sprintf(temporary_buffer,  "%" PRIu64 " ",(uint64_t)ack_msg);
					memcpy(temporary_buffer, &ack_msg,sizeof(uint64_t));
					Generate_Transmit_Request_Frame(temporary_buffer, &frame,tot);
					serial_device_.Send_Frame(frame);
					}
					else if (header[1]==receiver_cur_checksum) {
						tot =sizeof(uint64_t);
						uint64_t ack_msg =  (uint64_t)ACK_MESSAGE_CONSTANT | ((uint64_t)header[1] << 16) | ((uint64_t)header[2] << 32) |((uint64_t) device_id << 48) ;
						memcpy(temporary_buffer, &ack_msg,sizeof(uint64_t));
						Generate_Transmit_Request_Frame(temporary_buffer, &frame,tot);
						serial_device_.Send_Frame(frame);
						/*tmp*/								
						//uint64_t tmp_printer;
						//sscanf(frame.c_str(), "%" PRIu64 " ",&tmp_printer);
						//uint16_t* tmp_printer_16 =u64_cvt_u16(tmp_printer);
						//std::cout << "Send ACK for " <<ACK_MESSAGE_CONSTANT<<"  "<<header[1] <<"  "<<header[2]<<"  "<<device_id<<"  "<< std::endl;
						//delete[] tmp_printer_16;
						/*tmp*/
						std::map< std::size_t, std::shared_ptr<std::string> >::iterator it = multi_msgs_receive.find(header[2]);
						if(it!=multi_msgs_receive.end()){
							multi_msgs_receive.erase(it);
							multi_msgs_receive.insert(make_pair(header[2], in_message));
						}
						else{
							multi_msgs_receive.insert(make_pair(header[2], in_message));
							//counter++;
						}
						std::cout << "multi msg counter " <<multi_msgs_receive.size() << std::endl;					
						/*If the total size of msg reached transfer to topic*/
						if(multi_msgs_receive.size()==header[3]){
						
							for(uint16_t i =1; i<=header[3];i++){
								it = multi_msgs_receive.find(i);
								/*Escape the header*/
								tot =sizeof(uint64_t);
								/*copy msg size*/
								uint16_t tmp_size=0;
								memcpy(&tmp_size,it->second->c_str()+tot,sizeof(uint16_t));
								tot+=sizeof(uint16_t);
								//std::cout<<"multi publisher received size in bytes: "<<tmp_size << std::endl;
								//int uint64_size=tmp_size/sizeof(uint64_t);
								uint64_t message_obt[tmp_size];
								/*Copy obt msg*/
								memcpy(message_obt,it->second->c_str()+tot,tmp_size*sizeof(uint64_t));
								tot+=tmp_size*sizeof(uint64_t);
								//std::cout<<"tot size : "<<tot<< std::endl;
								for (uint16_t i = 0; i < tmp_size; i++)
								{
									mavlink_msg.payload64.push_back(message_obt[i]);
									
		
								}
								//uint64_t previous_int64=0;
								//std::cout<<"Transfering to topic chunk no. :"<<it->first << "Size of current map" <<it->second->size()<< std::endl;
								//std::cout << "received Frame:"<<(void *) it->second->c_str() << std::endl;	
								//std::cout<<"Size of map : "<< multi_msgs.size()<< std::endl;
								/*	for (std::size_t j = 1; j < it->second->size()-1; j++)
									{
									
				
										if (' ' == it->second->at(j) || 0 == j)
										{*/
								//current_int64=0
								//sscanf(it->second->c_str() + j, "%" PRIu64 " ",
								//		&current_int64); 
								/*Copy obt msg*/
								
								//memcpy(current_int64, it->second->c_str()+tot, tmp_size*sizeof(uint64_t));
								//std::cout << "received Frame:" << current_int64 << std::endl;
								//if(previous_int64 != current_int64){
								//mavlink_msg.payload64.push_back(current_int64);
								//previous_int64=current_int64;
								//}
								/*		}
		
									} */
							}
						
						std::cout << "one multi message published in topic with size :" <<mavlink_msg.payload64.size() << std::endl;
						mavlink_publisher_.publish(mavlink_msg);
						multi_msgs_receive.clear();
						receiver_cur_checksum=0;
						counter=10;
						}
						steps=0;
					
					}
				}
			}
			else if(header[0]==(uint16_t)ACK_MESSAGE_CONSTANT){
				//std::cout << "ACK Received header " <<header[0]<<"  "<<header[1]<<"  "<<header[2]<<"  "<<header[3]<<"  "<< std::endl;
				//std::cout << "size of ack map before adding" << ack_received_dict.size()<< std::endl;
				/*Ack message about a message packet find wheather that matches with your current expectation*/
				if(header[1]==Sender_cur_checksum && header[2]== (sending_chunk_no+1)){
					std::map< uint16_t, uint16_t >::iterator it = ack_received_dict.find(header[3]);
					if(it!=ack_received_dict.end()){
						ack_received_dict.erase(it);
						ack_received_dict.insert(std::make_pair((uint16_t)header[3], (uint16_t)ACK_MESSAGE_CONSTANT));
						}
					else{
						ack_received_dict.insert( std::make_pair( (uint16_t)header[3], (uint16_t)ACK_MESSAGE_CONSTANT ) );
					}					
									
				}
				//std::cout << "ACK added and size of ack map " << ack_received_dict.size()<< std::endl;
				
			}
			delete[] header;

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
	const unsigned short MAX_NBR_OF_INT64 = 24;
	char temporary_buffer[MAX_BUFFER_SIZE];
	std::string frame;
	int converted_bytes = 0;
	if((uint64_t)mavlink_msg->payload64.at(0)==(uint64_t)XBEE_STOP_TRANSMISSION && mavlink_msg->payload64.size() == 1){
		std::cout << "clearing multi msg queue after request from buzz"<< std::endl;
		multi_msgs_send_dict.clear();
		ack_received_dict.clear();
		sending_chunk_no=0;
		/*Multi message packets stoped tell rosbuzz this*/
		mavros_msgs::Mavlink mavlink_msg;
		mavlink_msg.payload64.push_back(XBEE_MESSAGE_CONSTANT);
		mavlink_publisher_.publish(mavlink_msg);
	}
	else if(mavlink_msg->payload64.size() > MAX_NBR_OF_INT64 && !( multi_msgs_send_dict.empty() ) ){
		std::cout << "Sending previous multi message not complete yet, so dropping message"<<std::endl;
	}
	else{
		char temporary_buffer_check[6000];
		/* create a whole check sum to handel multi chunk msg*/
		for(std::size_t i=0; i<mavlink_msg->payload64.size(); i++)
		{
			converted_bytes += sprintf(
				temporary_buffer_check + converted_bytes, "%" PRIu64 " ",
				(uint64_t)mavlink_msg->payload64.at(i));
		
		}
		
		frame.append(temporary_buffer_check, std::strlen(temporary_buffer_check));
		uint16_t check_sum = (uint16_t)Caculate_Checksum(&frame);
		uint16_t number=1;
		uint16_t total =ceil((double)((double)mavlink_msg->payload64.size()/(double)MAX_NBR_OF_INT64));
		//std::cout <<"Payload size" <<mavlink_msg->payload64.size() << std::endl;
		/*Create a header for the msgs*/
		uint64_t header = (uint64_t)MESSAGE_CONSTANT | ((uint64_t)check_sum << 16) | ((uint64_t)number << 32) |((uint64_t) total << 48) ;
		frame="";
		memset(temporary_buffer, 0, MAX_BUFFER_SIZE);
		uint16_t* header_16 = u64_cvt_u16(header);
		/*buffer byte counter*/
		int tot=0;
		std::cout << "Sent header" <<header_16[0]<<"  "<<header_16[1]<<"  "<<header_16[2]<<"  "<<header_16[3]<<"  "<< std::endl;
		delete[] header_16;
		/*Single frame message*/
		if(mavlink_msg->payload64.size() <= MAX_NBR_OF_INT64){

			uint64_t message_obt[mavlink_msg->payload64.size()];
			uint8_t* cpy_buff = (uint8_t*)malloc( sizeof(uint64_t)+sizeof(uint16_t) + ( sizeof(uint64_t)*mavlink_msg->payload64.size() ) );
			memset(cpy_buff, 0,sizeof(uint64_t) + ( sizeof(uint64_t)*mavlink_msg->payload64.size() ));
			
			for (std::size_t i =0; i<mavlink_msg->payload64.size(); i++)
			{
				message_obt[i] =(uint64_t)mavlink_msg->payload64[i];
				
			}
			/*Copy the header*/
			memcpy(cpy_buff,&header,sizeof(uint64_t));
			tot+=sizeof(uint64_t);			
			/*copy msg size*/
			uint16_t tmp_size=(uint16_t)mavlink_msg->payload64.size();
			memcpy(cpy_buff+tot,&tmp_size,sizeof(uint64_t));
			tot+=sizeof(uint16_t);
			std::cout<<"tmp size in sender"<<tmp_size<< std::endl;
			/*Copy obt msg*/
			memcpy(cpy_buff+tot,message_obt,( sizeof(uint64_t) )*tmp_size);
			tot+=( sizeof(uint64_t) )*tmp_size;
			/*Copy the data to char buff*/
			memcpy((void*)temporary_buffer,(void*)cpy_buff,tot);
			delete[] cpy_buff;
			//std::cout << "Single packet message sent size"<<mavlink_msg->payload64.size()<<"  Tot size: "<< tot<< std::endl;
			Generate_Transmit_Request_Frame(temporary_buffer, &frame,tot);
			serial_device_.Send_Frame(frame);
			
		}
		else{
			/*clear all the related parameter and get ready to send multi msg*/
			sending_chunk_no=0;
			Sender_cur_checksum = check_sum;
			ack_received_dict.clear();
			multi_msgs_send_dict.clear();
			/*Copy the msgs into a 64bit array*/
			uint64_t message_obt[mavlink_msg->payload64.size()];
			/*buffer for easy handel operation*/
			uint8_t* cpy_buff = (uint8_t*)malloc( sizeof(uint64_t)+sizeof(uint16_t) + ( sizeof(uint64_t)*MAX_NBR_OF_INT64 ) );
			memset(cpy_buff, 0,sizeof(uint64_t)+sizeof(uint16_t) + ( sizeof(uint64_t)*MAX_NBR_OF_INT64 ));
			for (std::size_t i =0; i<mavlink_msg->payload64.size(); i++)
			{
				message_obt[i] =(uint64_t)mavlink_msg->payload64[i];
				
			}
			//std::cout << "put header in dict" <<header_16[0]<<"  "<<header_16[1]<<"  "<<header_16[2]<<"  "<<header_16[3]<<"  "<< std::endl;
					
			/*copy msg size*/
			uint16_t tmp_size=(uint16_t)MAX_NBR_OF_INT64;
			uint16_t uint64_counter=0;
			/*Multi message frame received, split them into chunks and store them in dict*/
			for (uint16_t i =1; i<total; i++)
			{
					/*Copy the header*/
					memcpy(cpy_buff,&header,sizeof(uint64_t));
					tot+=sizeof(uint64_t);			
					memcpy(cpy_buff+tot,&tmp_size,sizeof(uint64_t));
					tot+=sizeof(uint16_t);
					/*Copy obt msg*/
					memcpy(cpy_buff+tot, message_obt+uint64_counter, ( sizeof(uint64_t) )*tmp_size);
					uint64_counter+=tmp_size;
					tot+=( sizeof(uint64_t) )*tmp_size;
					/*Copy the data to char buff*/
					memcpy((void*)temporary_buffer,(void*)cpy_buff,tot);
					/*cnt++;
					converted_bytes += sprintf(
					temporary_buffer+converted_bytes, "%" PRIu64 " ",
					(uint64_t)mavlink_msg->payload64.at(i));
					*/
					//std::cout << "Sent Frame in (uint64):"<<mavlink_msg->payload64.at(i) << std::endl;
					//std::cout << "Sent Frame in string"<<temporary_buffer<<std::endl;
				//if(cnt==MAX_NBR_OF_INT64)
				//{	
					//std::cout << "Multi frame sent no:"<<number << std::endl;					
					Generate_Transmit_Request_Frame(temporary_buffer, &frame,tot);
					//std::S
					multi_msgs_send_dict.push_back(frame);	
					//serial_device_.Send_Frame(frame);
					/*Sleep for some time in order not to confuse Xbee, a try to reduce errors*/
					//usleep(1000);
					//std::cout << "Frame:"<<frame << std::endl;
					//std::cout << "size of frame:"<<std::strlen(temporary_buffer)<< std::endl;
					tot =0;					
					number++;
					frame = "";
					memset(cpy_buff, 0,sizeof(uint64_t)+sizeof(uint16_t) + ( sizeof(uint64_t)*MAX_NBR_OF_INT64 ));
					//std::cout << "total:" <<total << std::endl;
					header=0;
					header = (uint64_t) MESSAGE_CONSTANT | ((uint64_t)check_sum << 16) | ((uint64_t)number << 32) |((uint64_t) total << 48) ;
					header_16 = u64_cvt_u16(header);				
					std::cout << "put header in dict" <<header_16[0]<<"  "<<header_16[1]<<"  "<<header_16[2]<<"  "<<header_16[3]<<"  "<< std::endl;
					memset(temporary_buffer, 0, MAX_BUFFER_SIZE);
					/*converted_bytes = sprintf(
							temporary_buffer, "%" PRIu64 " ",
							(uint64_t)header);*/
					
				//}
						
			}
			delete[] header_16;
			if(uint64_counter!=mavlink_msg->payload64.size()){
				tmp_size=mavlink_msg->payload64.size() - uint64_counter;
				/*Copy the header*/
				memcpy(cpy_buff,&header,sizeof(uint64_t));
				tot+=sizeof(uint64_t);			
				memcpy(cpy_buff+tot,&tmp_size,sizeof(uint64_t));
				tot+=sizeof(uint16_t);
				/*Copy obt msg*/
				memcpy(cpy_buff+tot, message_obt+uint64_counter, ( sizeof(uint64_t) )*tmp_size);
				uint64_counter+=tmp_size;
				tot+=( sizeof(uint64_t) )*tmp_size;
				/*Copy the data to char buff*/
				memcpy((void*)temporary_buffer,(void*)cpy_buff,tot);
				Generate_Transmit_Request_Frame(temporary_buffer, &frame,tot);
				multi_msgs_send_dict.push_back(frame);
			}
			delete[] cpy_buff;
			//std::cout << " Received size: " <<mavlink_msg->payload64.size() << std::endl;
			//std::cout << "total size of multi msg dict mavlink size:" <<multi_msgs_send_dict.size() << std::endl;
			//std::cout << "uint64_counter size:" <<uint64_counter << std::endl;
			/*Send the first message chunk*/
			//serial_device_.Send_Frame(multi_msgs_send_dict[0]);
		}
	}
	/*check for chunk to send, if multi chunk message present*/
	/*first msg*/
/*	if(sending_chunk_no==0 && !( multi_msgs_send_dict.empty() ) ){
		serial_device_.Send_Frame(multi_msgs_send_dict.at(sending_chunk_no));
	}
	else */ 
	
	//}
}

void CommunicationManager::Send_multi_msg(){

	if( !( multi_msgs_send_dict.empty() ) ){
		/*If the sent message chunk not the last message then send else clear the dict*/
		if( (uint16_t)(multi_msgs_send_dict.size() ) - 1 == sending_chunk_no && (uint16_t)ack_received_dict.size() == (uint16_t)(no_of_dev)-1){
			std::cout << "clearing multi msg queue and telling buzz"<< std::endl;
			multi_msgs_send_dict.clear();
			sending_chunk_no=0;
			/*Multi message packet sent tell rosbuzz this*/
			mavros_msgs::Mavlink mavlink_msg;
			mavlink_msg.payload64.push_back(XBEE_MESSAGE_CONSTANT);
			mavlink_publisher_.publish(mavlink_msg);

		}	

		else{
			std::cout << "current size of ack in sender " <<ack_received_dict.size()<<"No. of devices"<<(uint16_t) no_of_dev<< std::endl;
			if((uint16_t)ack_received_dict.size() == (uint16_t) (no_of_dev)-1){
				sending_chunk_no++;
				ack_received_dict.clear();
				//std::cout << "sending next msg"<< std::endl;
			}
			std::cout << "Sent frame no. " <<sending_chunk_no+1 << std::endl;
			//uint64_t tmp_printer;
			//std::cout<<"Current msg in string "<<multi_msgs_send_dict.at(sending_chunk_no)<< std::endl;
			//sscanf(multi_msgs_send_dict.at(sending_chunk_no).c_str(), "%" PRIu64 " ",&tmp_printer);
			//uint16_t* tmp_printer_16 =u64_cvt_u16(tmp_printer);
			//std::cout << "Send header" <<tmp_printer_16[0]<<"  "<<tmp_printer_16[1]<<"  "<<tmp_printer_16[2]<<"  "<<tmp_printer_16[3]<<"  "<< std::endl;
			//delete[] tmp_printer_16;

			serial_device_.Send_Frame(multi_msgs_send_dict.at(sending_chunk_no));
		}


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

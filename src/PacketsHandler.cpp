/* PacketsHandler.cpp -- Packets Handler class for XBee:
			     Serialize, deserialize, fragment and reassemly mavlink
			     messages (packets and std messages)  --                     */
/* ------------------------------------------------------------------------- */
/* February 06, 2017 -- @Copyright Aymen Soussia. All rights reserved.       */
/*                                  (aymen.soussia@gmail.com)                */


#include"PacketsHandler.h"


namespace Mist
{


namespace Xbee
{


//*****************************************************************************
PacketsHandler::PacketsHandler():
	MAX_PACEKT_SIZE(64000),
	XBEE_NETWORK_MTU(250),
	FRAGMENT_HEADER_SIZE(6),
	MAX_TIME_TO_SEND_PACKET(30000),
	START_DLIMITER(static_cast<unsigned char>(0x7E)),
	device_64_bits_address_("12345678"),
	loaded_SL_(false),
	loaded_SH_(false),
	optimum_MT_NBR_(3),
	delay_interframes_(100 * 1000)
{
}


//*****************************************************************************
PacketsHandler::~PacketsHandler()
{
}


//*****************************************************************************
bool PacketsHandler::Init(SerialDevice* serial_device,
		Thread_Safe_Deque* in_packets)
{
	serial_device_ = serial_device;
	
	if (!Load_Database_Addresses())
		return false;
		
	Send_SL_and_SH_Commands();
	in_packets_ = in_packets;
	
	return true;
}


//*****************************************************************************
bool PacketsHandler::Init_Device_ID()
{
	if (Get_Local_Address())
		return true;
	else
		return false;
}


//*****************************************************************************
void PacketsHandler::Run()
{	
	quit_.store(false);
	
	while (!quit_.load())
	{
		Process_Out_Standard_Messages();
		Process_Out_Packets();
	}
}


//*****************************************************************************
void PacketsHandler::Handle_Mavlink_Message(const mavros_msgs::Mavlink::ConstPtr& mavlink_msg)
{
	std::shared_ptr<std::string> serialized_packet =
			std::make_shared<std::string>();
	
	Serialize_Mavlink_Message(mavlink_msg, serialized_packet);
	
	if (serialized_packet->size() > XBEE_NETWORK_MTU && serialized_packet->size() <= MAX_PACEKT_SIZE)
	{
		std::shared_ptr<std::vector<std::shared_ptr<std::string>>> fragmented_packet =
		std::make_shared<std::vector<std::shared_ptr<std::string>>>();
			
		std::size_t offset = 0;
		std::size_t NBR_of_bytes = 0;
		std::size_t NBR_of_fragments = std::ceil(
			static_cast<float>(serialized_packet->size()) / XBEE_NETWORK_MTU);
		
		for (uint8_t i = 0; i < NBR_of_fragments; i++)
		{
			fragmented_packet->push_back(std::make_shared<std::string>());
			NBR_of_bytes = std::min(XBEE_NETWORK_MTU, serialized_packet->size() - offset);
			Insert_Fragment_Header(false, fragmented_packet->at(i), mavlink_msg->msgid, i, offset);
			fragmented_packet->at(i)->append(serialized_packet->c_str() + offset, NBR_of_bytes);
			offset += NBR_of_bytes;
		}
		
		out_packets_.Push_Back({mavlink_msg->msgid, fragmented_packet});
	}
	else if (serialized_packet->size() < XBEE_NETWORK_MTU)
	{
		serialized_packet->insert(0, 1, 'S');
		out_std_messages_.Push_Back(serialized_packet);
	}
}


//*****************************************************************************
void PacketsHandler::Process_Fragment(std::shared_ptr<std::string> fragment)
{
	uint8_t packet_ID = fragment->at(1);
	uint8_t node_8_bits_address = fragment->at(2);
	uint8_t fragment_ID = fragment->at(3);
	uint16_t offset = static_cast<uint16_t>(
			static_cast<uint16_t>(static_cast<unsigned char>(fragment->at(4))) << 8 |
			static_cast<uint16_t>(static_cast<unsigned char>(fragment->at(5))));
	
	assembly_map_it_ = packets_assembly_map_.find(node_8_bits_address);
	
	if (assembly_map_it_ != packets_assembly_map_.end())
	{
		if (assembly_map_it_->second.packet_ID_ == packet_ID)
		{
			std::set<uint8_t>::iterator it = assembly_map_it_->second.received_fragments_IDs_.find(fragment_ID);
			
			if (it == assembly_map_it_->second.received_fragments_IDs_.end())
			{
				if (assembly_map_it_->second.received_fragments_IDs_.size() == 0)
					assembly_map_it_->second.time_since_creation_ = std::clock();
				
				Insert_Fragment_In_Packet_Buffer(&assembly_map_it_->second.packet_buffer_, fragment->c_str(), offset, fragment->size());
				assembly_map_it_->second.received_fragments_IDs_.insert(fragment_ID);
			}
		}
		else
		{
			assembly_map_it_->second = {};
			assembly_map_it_->second.packet_ID_ = packet_ID;
			Insert_Fragment_In_Packet_Buffer(&assembly_map_it_->second.packet_buffer_, fragment->c_str(), offset, fragment->size());
			assembly_map_it_->second.received_fragments_IDs_.insert(fragment_ID);
			assembly_map_it_->second.time_since_creation_ = std::clock();
		}
	}
	else
	{
		Add_New_Node_To_Network(node_8_bits_address);
		assembly_map_it_ = packets_assembly_map_.find(node_8_bits_address);
		assembly_map_it_->second.packet_ID_ = packet_ID;
		Insert_Fragment_In_Packet_Buffer(&assembly_map_it_->second.packet_buffer_, fragment->c_str(), offset, fragment->size());
		assembly_map_it_->second.received_fragments_IDs_.insert(fragment_ID);
		assembly_map_it_->second.time_since_creation_ = std::clock();
	}
}


//*****************************************************************************
void PacketsHandler::Insert_Fragment_In_Packet_Buffer(std::string* buffer, const char* fragment, const uint16_t offset, const std::size_t length)
{
	if (offset >= buffer->size())
		buffer->append(fragment + FRAGMENT_HEADER_SIZE, length - FRAGMENT_HEADER_SIZE);
	else
		buffer->insert(offset, fragment + FRAGMENT_HEADER_SIZE, length - FRAGMENT_HEADER_SIZE);
}


//*****************************************************************************
void PacketsHandler::Process_Ping_Or_Acknowledgement(std::shared_ptr<std::string> frame)
{
	uint8_t packet_ID = frame->at(12);
	uint8_t node_8_bits_address = frame->at(13);
	
	if (frame->at(11) == 'A')
	{
		mutex_.lock();
		
		connected_network_nodes_it_ = connected_network_nodes_.find(node_8_bits_address);
	
		if (connected_network_nodes_it_ == connected_network_nodes_.end())
		{
			mutex_.unlock();
			Add_New_Node_To_Network(node_8_bits_address);
			mutex_.lock();
			connected_network_nodes_it_ = connected_network_nodes_.find(node_8_bits_address);
		}

		if (packet_ID == current_processed_packet_ID_)
		{
			if (frame->size() < 15)
  				connected_network_nodes_it_->second = true;
			else
			{
				for (uint8_t i = 14; i < frame->size(); i++)
					fragments_indexes_to_transmit_.insert(frame->at(i));
			}
		}
		
		mutex_.unlock();
	}
	else  if (frame->at(11) == 'P')
	{
		assembly_map_it_ = packets_assembly_map_.find(node_8_bits_address);
		
		if (assembly_map_it_ == packets_assembly_map_.end())
		{
			Add_New_Node_To_Network(node_8_bits_address);
			assembly_map_it_ = packets_assembly_map_.find(node_8_bits_address);
			assembly_map_it_->second.packet_ID_ = packet_ID;
			assembly_map_it_->second.time_since_creation_ = std::clock();
		}
		
		if (assembly_map_it_->second.packet_ID_ == packet_ID)
		{
			std::string Acknowledgement = "A";
			Acknowledgement.push_back(packet_ID);
			Acknowledgement.push_back(device_address_);
			uint8_t packet_size = frame->at(14);
			
			if (assembly_map_it_->second.received_fragments_IDs_.size() == packet_size)
			{
				in_packets_->Push_Back(std::make_shared<std::string>(assembly_map_it_->second.packet_buffer_));
				assembly_map_it_->second.packet_buffer_.clear();
				assembly_map_it_->second.received_fragments_IDs_.clear();
				assembly_map_it_->second.time_since_creation_ = 0;
			}
			else
			{
				std::set<uint8_t>::iterator it = assembly_map_it_->second.received_fragments_IDs_.begin();
				uint8_t j = 0;
				
				while (j <= packet_size - 1)
				{
					if (j != *it)
						Acknowledgement.push_back(j);
					else if (it != std::prev(assembly_map_it_->second.received_fragments_IDs_.end()))
						it++;

					j++;
				}
			}
			
			std::string Ack_frame;
			Generate_Transmit_Request_Frame(Acknowledgement.c_str(), &Ack_frame, Acknowledgement.size());
			serial_device_->Send_Frame(Ack_frame);
			usleep(delay_interframes_);
			
		}
		else
		{
			assembly_map_it_->second = {};
			assembly_map_it_->second.packet_ID_ = packet_ID;
		}
	}
}


//*****************************************************************************
void PacketsHandler::Add_New_Node_To_Network(const uint8_t new_node_address)
{
	std::set<uint8_t> empty_set;
	packets_assembly_map_.insert(std::pair<uint8_t, Reassembly_Packet_S>(new_node_address, {}));
	
	std::lock_guard<std::mutex> guard(mutex_);
	connected_network_nodes_.insert(std::pair<uint8_t, bool>(new_node_address, false));
}


//*****************************************************************************
void PacketsHandler::Process_Command_Response(const char* command_response)
{
	if (command_response[0] == 'N' && command_response[1] == 'D')
	{
		uint64_t new_node_address = 0;
		std::lock_guard<std::mutex> guard(mutex_);
		
		if (command_response[2] == static_cast<unsigned char>(0))
		{
			new_node_address = static_cast<uint64_t>(
			static_cast<unsigned char>(command_response[5]) << 56 |
			static_cast<unsigned char>(command_response[6]) << 48 |
			static_cast<unsigned char>(command_response[7]) << 40 |
			static_cast<unsigned char>(command_response[8]) << 32 |
			static_cast<unsigned char>(command_response[9]) << 24 |
			static_cast<unsigned char>(command_response[10]) << 16 |
			static_cast<unsigned char>(command_response[11]) << 8 |
			static_cast<unsigned char>(command_response[12]));
		}
		
		database_addresses_it_ = database_addresses_.find(new_node_address);
		
		if (database_addresses_it_ != database_addresses_.end())
			device_address_ = database_addresses_it_->second;
		else
			std::cout << "Remote Node Not in Database" << std::endl; 
	}
	else if (command_response[0] == 'S' && command_response[1] == 'H')
	{
		if (command_response[2] == static_cast<unsigned char>(0))
		{
			loaded_SH_ = true;
			
			for (std::size_t i = 0; i < 4; i++)
				device_64_bits_address_[i] = command_response[3 + i];
		}
	}
	else if (command_response[0] == 'S' && command_response[1] == 'L')
	{
		if (command_response[2] == static_cast<unsigned char>(0))
		{
			loaded_SL_ = true;
			
			for (std::size_t i = 0; i < 4; i++)
				device_64_bits_address_[4 + i] = command_response[3 + i];
		}
	}
}


//*****************************************************************************
void PacketsHandler::Quit()
{
	quit_.store(true);
}


//*****************************************************************************
void PacketsHandler::Serialize_Mavlink_Message(const mavros_msgs::Mavlink::ConstPtr&
		mavlink_msg, std::shared_ptr<std::string> serialized_packet)
{
	serialized_packet->push_back(mavlink_msg->sysid);
	serialized_packet->push_back(mavlink_msg->msgid);
	
	std::string bytes="12345678";
	
	for (std::size_t j = 0; j < mavlink_msg->payload64.size(); j++)
	{
		for (std::size_t i = 0; i < 8; i++)
			bytes[7 - i] = (mavlink_msg->payload64.at(j) >> (i * 8));
			
	 	serialized_packet->append(bytes);
	}
}


//*****************************************************************************
void PacketsHandler::Insert_Fragment_Header(bool single_fragment,
		std::shared_ptr<std::string> fragment, const uint8_t packet_ID,
		const uint8_t fragment_ID, const uint16_t offset) 
{	
	if (!single_fragment)
	{
		fragment->push_back('F');
		fragment->push_back(packet_ID);
		fragment->push_back(device_address_);
		fragment->push_back(fragment_ID);
		fragment->push_back(offset >> (1 * 8));
		fragment->push_back(offset >> (0 * 8));
	}
	else
		fragment->push_back('S');
}


//*****************************************************************************
void PacketsHandler::Delete_Packets_With_Time_Out()
{	
	for(auto& iterator: packets_assembly_map_)
	{
		if (std::clock_t() - iterator.second.time_since_creation_ > MAX_TIME_TO_SEND_PACKET && iterator.second.time_since_creation_ != 0)
			iterator.second = {};
	}
}


//*****************************************************************************
void PacketsHandler::Process_Out_Standard_Messages()
{
	std::size_t deque_size = out_std_messages_.Get_Size();
	
	if (deque_size > 0)
	{
		std::string frame;
		std::shared_ptr<std::string> out_message;
		
		for (std::size_t i = 0; i < deque_size; i++)
		{
			frame.clear();
			out_message = out_std_messages_.Pop_Front();
			
			Generate_Transmit_Request_Frame(out_message->c_str(), &frame, out_message->size());
			serial_device_->Send_Frame(frame);
			usleep(delay_interframes_);
		}
	}
}


//*****************************************************************************
void PacketsHandler::Process_Out_Packets()
{
	std::size_t deque_size = out_packets_.Get_Size();
	
	if (deque_size > 0)
	{
		Out_Packet_S out_packet;
		
		for (std::size_t i = 0; i < deque_size; i++)
		{
			Process_Out_Standard_Messages();
			out_packet = out_packets_.Pop_Front();

			Send_Packet(out_packet);
		}
	}
}


//*****************************************************************************
void PacketsHandler::Send_Packet(const Out_Packet_S& packet)
{
	std::size_t NBR_of_transmission = 0; 
	std::vector<std::string> frames;
	
	Init_Network_Nodes_For_New_Transmission(packet.packet_ID_, &frames, packet.packet_buffer_);
	current_processed_packet_ID_ = packet.packet_ID_;
	
	std::clock_t start_time = std::clock();
		
	while (std::clock() - start_time <= MAX_TIME_TO_SEND_PACKET && !Check_Packet_Transmitted_To_All_Nodes())
	{
		NBR_of_transmission++;
		Transmit_Fragments(frames);
		Send_End_Of_Packet_Ping(packet.packet_ID_, packet.packet_buffer_->size());
		usleep(500 * 1000);
	}
	
	Adjust_Optimum_MT_Number(std::clock() - start_time, NBR_of_transmission);
}


//*****************************************************************************
void PacketsHandler::Send_End_Of_Packet_Ping(const uint8_t packet_ID, const uint8_t total_NBR_of_fragments)
{
	std::string ping_message = "P";
	std::string ping_frame;
	
	ping_message.push_back(packet_ID);
	ping_message.push_back(device_address_);
	ping_message.push_back(total_NBR_of_fragments);
	
	Generate_Transmit_Request_Frame(ping_message.c_str(), &ping_frame, ping_message.size());
	serial_device_->Send_Frame(ping_frame);
	usleep(delay_interframes_);
}


//*****************************************************************************
bool PacketsHandler::Load_Database_Addresses()
{
	const std::string FILE_PATH = "/home/vivek/catkin_ws/src/xbee/Resources/database.xml";
	
	
	if (!boost::filesystem::exists(FILE_PATH))
	{	
		std::cout << "database.xml Not Found." << std::endl;
		return false;
	}
	
	ptree pt;
	boost::property_tree::read_xml(FILE_PATH, pt);
	std::string short_address;
	std::string address_64_bits;
	unsigned int short_address_int;
	uint64_t address_64_bits_int;
	
	BOOST_FOREACH(ptree::value_type const&v, pt.get_child("Addresses"))
	{
		if (v.first == "Device")
		{
			short_address = v.second.get<std::string>("<xmlattr>.Address");
			address_64_bits = v.second.data();
			
			if (sscanf(short_address.c_str(), "%3u", &short_address_int) < 0)
			{
				std::cout << "Short Address Error. Please Check database.xml For Possible Errors." << std::endl;
				return false;
			}
			
			if (sscanf(address_64_bits.c_str(), "%" SCNx64, &address_64_bits_int) < 0)
			{
				std::cout << "64 bits Address Error. Please Check database.xml For Possible Errors." << std::endl;
				return false;
			}
			
			database_addresses_.insert(std::pair<uint64_t, uint8_t>(address_64_bits_int, static_cast<uint8_t>(short_address_int)));
		}
	}
	
	return true;
}


//*****************************************************************************
bool PacketsHandler::Get_Local_Address()
{
	const useconds_t ONE_SECOND = 1*1000*1000; /* 1s = 1 * 10⁶ microseconds. */
	usleep(ONE_SECOND);
	
	if (loaded_SH_ && loaded_SL_)
	{
		uint64_t local_64_bits_address = (
		static_cast<uint64_t>(static_cast<unsigned char>(device_64_bits_address_[0])) << 56 |
		static_cast<uint64_t>(static_cast<unsigned char>(device_64_bits_address_[1])) << 48 |
		static_cast<uint64_t>(static_cast<unsigned char>(device_64_bits_address_[2])) << 40 |
		static_cast<uint64_t>(static_cast<unsigned char>(device_64_bits_address_[3])) << 32 |
		static_cast<uint64_t>(static_cast<unsigned char>(device_64_bits_address_[4])) << 24 |
		static_cast<uint64_t>(static_cast<unsigned char>(device_64_bits_address_[5])) << 16 |
		static_cast<uint64_t>(static_cast<unsigned char>(device_64_bits_address_[6])) << 8 |
		static_cast<uint64_t>(static_cast<unsigned char>(device_64_bits_address_[7])));
		
		database_addresses_it_ = database_addresses_.find(local_64_bits_address);
		
		if (database_addresses_it_ != database_addresses_.end())
		{
			device_address_ = database_addresses_it_->second;
			std::cout << "Loaded Short Device Address :  " << static_cast<int>(device_address_) << std::endl;
			return true;
		}
		else
		{
			std::cout << "Local Address Not Found In Database. Please Add The Xbee Address to database.xml." << std::endl;
			return false;
		}
	}
	else
	{
		Send_SL_and_SH_Commands();
		return false;
	}	
}


//*****************************************************************************
bool PacketsHandler::Check_Packet_Transmitted_To_All_Nodes()
{
	std::lock_guard<std::mutex> guard(mutex_);
	
	if (connected_network_nodes_.size() == 0)
		return false;
	
	for (auto it : connected_network_nodes_)
	{
		if (!it.second)
			return false;
	}
	
	return true;
}


//*****************************************************************************
void PacketsHandler::Init_Network_Nodes_For_New_Transmission(const uint8_t packet_ID, std::vector<std::string>* frames, std::shared_ptr<std::vector<std::shared_ptr<std::string>>> packet)
{
	std::lock_guard<std::mutex> guard(mutex_);
	fragments_indexes_to_transmit_.clear();
	
	for (auto& it : connected_network_nodes_)
		it.second = false;
	
	current_processed_packet_ID_ = packet_ID;
	
	for (uint8_t i = 0; i < packet->size(); i++)
	{
		frames->push_back("");
		fragments_indexes_to_transmit_.insert(i);
		Generate_Transmit_Request_Frame(packet->at(i)->c_str(), &frames->at(i), packet->at(i)->size());
	}
}


//*****************************************************************************
void PacketsHandler::Transmit_Fragments(const std::vector<std::string>& frames)
{
	std::lock_guard<std::mutex> guard(mutex_);
	
	for (auto index: fragments_indexes_to_transmit_)
	{
		serial_device_->Send_Frame(frames.at(index));
		usleep(delay_interframes_);
	}
	
	fragments_indexes_to_transmit_.clear();		
}
		

//*****************************************************************************
void PacketsHandler::Adjust_Optimum_MT_Number(const std::clock_t elapsed_time,
		const std::size_t NBR_of_transmission)
{	
	if (NBR_of_transmission > 1 && elapsed_time < MAX_TIME_TO_SEND_PACKET)
		delay_interframes_ += 5000;
	else if (NBR_of_transmission == 1 && delay_interframes_ >= 5000)
		delay_interframes_ -= 5000;
}


//*****************************************************************************
void PacketsHandler::Send_SL_and_SH_Commands()
{
	std::string command;
	std::string frame;
	
	command = "SL";
	Generate_AT_Command(command.c_str(), &frame);
	serial_device_->Send_Frame(frame);
	
	command = "SH";
	frame = "";
	Generate_AT_Command(command.c_str(), &frame);
	serial_device_->Send_Frame(frame);
}


//*****************************************************************************
void PacketsHandler::Deserialize_Mavlink_Message(const char * bytes,
			mavros_msgs::Mavlink* mavlink_msg, const std::size_t msg_size)
{
	mavlink_msg->sysid = bytes[0];
	mavlink_msg->msgid = bytes[1];
	uint64_t current_int64 = 0;
	
	for (std::size_t i = 2; i < msg_size; i += 8)
	{
		current_int64 = (
		static_cast<uint64_t>(static_cast<unsigned char>(bytes[i])) << 56 |
		static_cast<uint64_t>(static_cast<unsigned char>(bytes[i + 1])) << 48 |
		static_cast<uint64_t>(static_cast<unsigned char>(bytes[i + 2])) << 40 |
		static_cast<uint64_t>(static_cast<unsigned char>(bytes[i + 3])) << 32 |
		static_cast<uint64_t>(static_cast<unsigned char>(bytes[i + 4])) << 24 |
		static_cast<uint64_t>(static_cast<unsigned char>(bytes[i + 5])) << 16 |
		static_cast<uint64_t>(static_cast<unsigned char>(bytes[i + 6])) << 8 |
		static_cast<uint64_t>(static_cast<unsigned char>(bytes[i + 7])));
		
		mavlink_msg->payload64.push_back(current_int64);
	}
}


//*****************************************************************************
inline void PacketsHandler::Generate_Transmit_Request_Frame(
		const char* message,
		std::string * frame,
		const std::size_t message_size,
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
	frame->append(message, message_size);

	Calculate_and_Append_Checksum(frame);
	Add_Length_and_Start_Delimiter(frame);
}


//*****************************************************************************
inline void PacketsHandler::Add_Length_and_Start_Delimiter(
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
inline void PacketsHandler::Calculate_and_Append_Checksum(
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
inline void PacketsHandler::Convert_HEX_To_Bytes(
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
void PacketsHandler::Generate_AT_Command(const char* command,
			std::string* frame,
			const unsigned char frame_ID)
{
	const unsigned char FAME_TYPE = static_cast<unsigned char>(0x09);/* AT Command Frame */
	std::string temporary_parameter_value;

	frame->push_back(FAME_TYPE);
	frame->push_back(frame_ID);
	frame->append(command);
	
	Calculate_and_Append_Checksum(frame);
	Add_Length_and_Start_Delimiter(frame);
}

uint8_t PacketsHandler::get_device_id(){
return device_address_;
}

}


}

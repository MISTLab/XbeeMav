/* PacketsHandler.h-- Packets Handler class for XBee:
			     Serialize, deserialize, fragment and reassemly mavlink
			     messages                             --                     */
/* ------------------------------------------------------------------------- */
/* February 06, 2017 -- @Copyright Aymen Soussia. All rights reserved.       */
/*                                  (aymen.soussia@gmail.com)                */


#pragma once

#include<atomic>
#include<inttypes.h>
#include<mutex>
#include<map>
#include<set>
#include<string>
#include<unistd.h>
#include<vector>

#include<boost/property_tree/ptree.hpp>
#include<boost/property_tree/xml_parser.hpp>
#include<boost/foreach.hpp>
#include<boost/filesystem.hpp>
#include<mavros_msgs/Mavlink.h>

#include"MultithreadingDeque.hpp"
#include"SerialDevice.h"


//*****************************************************************************
using boost::property_tree::ptree;


namespace Mist
{


namespace Xbee
{


//*****************************************************************************
class PacketsHandler
{
public:
	PacketsHandler();
	~PacketsHandler();
	
	bool Init(SerialDevice* serial_device, Thread_Safe_Deque* in_packets);
	bool Init_Device_ID();
	void Run();
	void Handle_Mavlink_Message(const mavros_msgs::Mavlink::ConstPtr& mavlink_msg);
	void Process_Fragment(std::shared_ptr<std::string> fragment);
	void Process_Ping_Or_Acknowledgement(std::shared_ptr<std::string> frame);
	void Process_Command_Response(const char* command_response);
	void Quit();
	void Delete_Packets_With_Time_Out();
	void Deserialize_Mavlink_Message(const char * bytes,
			mavros_msgs::Mavlink* mavlink_msg, const std::size_t msg_size);
	
	
	
private:
	const std::size_t MAX_PACEKT_SIZE; /* MAX packet size in bytes = 63750 bytes */
	const std::size_t XBEE_NETWORK_MTU; /* Maximum Transmission Unit of Xbee netwrok = 256 bytes (max payload) - 6 bytes (Header size of each fragment) = 250 bytes */
	const std::size_t FRAGMENT_HEADER_SIZE; /* Header size of each fragment = 6 bytes */
	const std::clock_t MAX_TIME_TO_SEND_PACKET; /* Maximum time before dropping a packet = 30 seconds*/
	const unsigned char START_DLIMITER;
	
	struct Reassembly_Packet_S
	{
		uint8_t packet_ID_;
		std::string packet_buffer_; // TO DO make it shared ptr
		std::set<uint8_t> received_fragments_IDs_;
		std::clock_t time_since_creation_; // TO DO use it to delete packets with time out
	};
	
	void Insert_Fragment_In_Packet_Buffer(std::string* buffer,
			const char* fragment, const uint16_t offset, const std::size_t length);
	void Add_New_Node_To_Network(const uint8_t new_node_address);
	void Serialize_Mavlink_Message(const mavros_msgs::Mavlink::ConstPtr&
		mavlink_msg, std::shared_ptr<std::string> serialized_packet);
	void Insert_Fragment_Header(bool single_fragment,
		std::shared_ptr<std::string> fragment, const uint8_t packet_ID,
		const uint8_t fragment_ID, const uint16_t offset);
	void Process_Out_Standard_Messages();
	void Process_Out_Packets();
	void Send_Packet(const Out_Packet_S& packet);
	void Send_End_Of_Packet_Ping(const uint8_t packet_ID, const uint8_t total_NBR_of_fragments);
	bool Load_Database_Addresses();
	bool Get_Local_Address();
	bool Check_Packet_Transmitted_To_All_Nodes();
	void Init_Network_Nodes_For_New_Transmission(const uint8_t packet_ID,
			std::vector<std::string>* frames,
			std::shared_ptr<std::vector<std::shared_ptr<std::string>>> packet);
	void Transmit_Fragments(const std::vector<std::string>& frames);
	void Adjust_Optimum_MT_Number(const std::clock_t elapsed_time,
		const std::size_t NBR_of_transmission);
	void Send_SL_and_SH_Commands();
	void Generate_Transmit_Request_Frame(
			const char* message,
			std::string* frame,
			const std::size_t message_size,
			const unsigned char frame_ID = 
			static_cast<unsigned char>(0x01),
			const std::string& destination_adssress = "000000000000FFFF",
			const std::string& short_destination_adress = "FFFF",
			const std::string& broadcast_radius = "00",
			const std::string& options = "00");
	void Convert_HEX_To_Bytes(const std::string& HEX_data,
			std::string* converted_data);
	void Calculate_and_Append_Checksum(std::string* frame);
	void Add_Length_and_Start_Delimiter(std::string* frame);
	void Generate_AT_Command(const char* command,
			std::string* frame,
			const unsigned char frame_ID = 
			static_cast<unsigned char>(0x01));
	
	struct On_Line_Node_S
	{
		bool received_hole_packet_;
		std::string address_64_bits_;
	};
	
	std::set<uint8_t> fragments_indexes_to_transmit_;
	SerialDevice* serial_device_;
	std::atomic<bool> quit_;
	Thread_Safe_Deque out_std_messages_;
	Thread_Safe_Deque_Of_Vectors out_packets_;
	Thread_Safe_Deque* in_packets_;
	std::map<uint8_t, bool> connected_network_nodes_;
	std::map<uint8_t, bool>::iterator connected_network_nodes_it_;
	std::map<uint8_t, Reassembly_Packet_S> packets_assembly_map_;
	std::map<uint8_t, Reassembly_Packet_S>::iterator assembly_map_it_;
	std::map<uint64_t, uint8_t> database_addresses_;
	std::map<uint64_t, uint8_t>::iterator database_addresses_it_;
	std::mutex mutex_;
	uint8_t device_address_;
	std::string device_64_bits_address_;
	bool loaded_SL_;
	bool loaded_SH_;
	uint8_t current_processed_packet_ID_;
	std::size_t optimum_MT_NBR_;
	// TO DO & after auto !?
	useconds_t delay_interframes_;
};


}


}

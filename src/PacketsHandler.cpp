/* PacketsHandler.cpp -- Packets Handler class for XBee:
           Serialize, deserialize, fragment and reassemly mavlink
           messages (packets and std messages)  --                     */
/* ------------------------------------------------------------------------- */
/* February 06, 2017 -- @Copyright Aymen Soussia. All rights reserved.       */
/*                                  (aymen.soussia@gmail.com)                */
/* Revision
 * Date: July 1st, 2017
 * Author: Pierre-Yves Breches
 */
/* ------------------------------------------------------------------------- */
/* TODO
 *  * Split PacketsHandler class (the last two could be instantiated by the first one)
 *    * class PacketsHandler
 *    * class PacketsLossHandler
 *    * class APIRssiHandler
 */


#include"PacketsHandler.h"


namespace Mist
{


namespace Xbee
{

const uint8_t PacketsHandler::ALL_IDS = 0xFF;
const uint16_t PacketsHandler::PACKET_LOSS_UNAVAILABLE = 0xFFFF;

//*****************************************************************************
PacketsHandler::PacketsHandler():
  MAX_PACEKT_SIZE(64000),
  XBEE_NETWORK_MTU(250),
  FRAGMENT_HEADER_SIZE(6),
  MAX_TIME_TO_SEND_PACKET(30000), // TO DO change it to packet time out
  RSSI_COMMAND("DB"),
  RSSI_FILTER_GAIN(0.5),
  PACKET_LOSS_IDENTIFIER("L"),
  MAX_PACKET_LOSS_MSG_ID(100),
  PACKET_LOSS_FILTER_GAIN(0.7),
  device_64_bits_address_("12345678"),
  loaded_SL_(false),
  loaded_SH_(false),
  rssi_float_(0),
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

  // TO DO node dicov

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
    out_packets_.Push_Back({static_cast<uint8_t>(mavlink_msg->msgid & 0xFF),
                            fragmented_packet});
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
  packet_loss_it_ = packet_loss_map_.find(node_8_bits_address);

  if (packet_loss_it_ != packet_loss_map_.end())
    packet_loss_it_->second.received_packets_ = packet_loss_it_->second.received_packets_ + 1;

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
void PacketsHandler::Insert_Fragment_In_Packet_Buffer(std::string* buffer, const char* fragment, const uint16_t offset, const std::size_t length) // TO DO delete length
{
  if (offset >= buffer->size())
    buffer->append(fragment + FRAGMENT_HEADER_SIZE, length - FRAGMENT_HEADER_SIZE);
  else
    buffer->insert(offset, fragment + FRAGMENT_HEADER_SIZE, length - FRAGMENT_HEADER_SIZE);
}


//*****************************************************************************
void PacketsHandler::Process_Ping_Or_Acknowledgement(std::shared_ptr<std::string> frame) // TO DO change useless std::shared_ptr<std::string> frame args to ->c_str()
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
        // TO DO add test if the packet was already transmitted to rosbuzz dont transmit ack only if
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

    if (command_response[2] == static_cast<unsigned char>(0)) // TO DO check this
    {
      new_node_address = get64BitsAddress(command_response, 5);
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
  else if (command_response[0] == 'D' && command_response[1] == 'B')
  {
     // TODO implemetation of a handler for error value responses (outside valid range)
     rssi_float_ = filterIIR(static_cast<float>(static_cast<uint16_t>(command_response[3])),
                              rssi_float_,
                              RSSI_FILTER_GAIN);
  }
  else if (command_response[0] == 'R' && command_response[1] == 'S')
  {
    processAPIRssi(command_response);
  }
  else
  {
    // nothing to do
  }
}

//*****************************************************************************
void PacketsHandler::processAPIRssi(const char* command_response)
{
  uint64_t node_address = get64BitsAddress(command_response, 2);

  database_addresses_it_ = database_addresses_.find(node_address);
  if (database_addresses_it_ != database_addresses_.end())
  {
    rssi_result_map_it_ = rssi_result_map_.find(database_addresses_it_->second);
    if(rssi_result_map_it_ == rssi_result_map_.end())
    {
      RSSI_Result result ={ucharToUint16(command_response[10], command_response[11]),
                           ucharToUint16(command_response[12], command_response[13]),
                           ucharToUint16(command_response[14], command_response[15]),
                           ucharToUint16(command_response[16], command_response[17]),
                           static_cast<uint8_t>(command_response[18]),
                           static_cast<uint8_t>(command_response[19]),
                           static_cast<uint8_t>(command_response[20]),
                           static_cast<uint8_t>(command_response[21]),
                           static_cast<uint8_t>(command_response[22]),
                           NEW_VALUE
                         };
      rssi_result_map_[database_addresses_it_->second] = result;
    }
    else
    {
      rssi_result_map_it_->second.payload_size = ucharToUint16(command_response[10], command_response[11]);
      rssi_result_map_it_->second.iterations = ucharToUint16(command_response[12], command_response[13]);
      rssi_result_map_it_->second.success = ucharToUint16(command_response[14], command_response[15]);
      rssi_result_map_it_->second.retries = ucharToUint16(command_response[16], command_response[17]);
      rssi_result_map_it_->second.result = static_cast<uint8_t>(command_response[18]);
      rssi_result_map_it_->second.rr = static_cast<uint8_t>(command_response[19]);
      rssi_result_map_it_->second.max_rssi = static_cast<uint8_t>(command_response[20]);
      rssi_result_map_it_->second.min_rssi = static_cast<uint8_t>(command_response[21]);
      rssi_result_map_it_->second.avg_rssi = static_cast<uint8_t>(command_response[22]);
      rssi_result_map_it_->second.status = NEW_VALUE;
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
    const uint8_t fragment_ID, const uint16_t offset) // TO DO change this function
{
  if (!single_fragment) // TO DO delete
  {
    fragment->push_back('F');
    fragment->push_back(packet_ID);
    fragment->push_back(device_address_);
    fragment->push_back(fragment_ID);
    fragment->push_back(offset >> (1 * 8));
    fragment->push_back(offset >> (0 * 8));
  }
  else // TO DO delete
    fragment->push_back('S'); // TO DO delete
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
    if(deque_size>10){printf("deque full");}


    for (std::size_t i = 0; i < deque_size; i++)
    {
      frame.clear();
      out_message = out_std_messages_.Pop_Front();

      Generate_Transmit_Request_Frame(out_message->c_str(), &frame, out_message->size());
      serial_device_->Send_Frame(frame);

      for (auto& it:packet_loss_map_)
      {
        it.second.sent_packets_ = it.second.sent_packets_ + 1;
      }

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

    // TO DO usleep(flow_control_time)
    // TO DO clear fragments_IDs_to_transmit_set
    Send_End_Of_Packet_Ping(packet.packet_ID_, packet.packet_buffer_->size());
    usleep(500 * 1000);
    // TO DO sleep after ping
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
  const std::string FILE_PATH = DATABASE_PATH;


  if (!boost::filesystem::exists(FILE_PATH))
  {
    std::cout << "database.xml Not Found with path: "
              << FILE_PATH << std::endl;
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
      database_addresses_inv_.insert(std::pair<uint8_t, uint64_t>(static_cast<uint8_t>(short_address_int), address_64_bits_int));
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
      local_64_bits_address_ = local_64_bits_address;
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
void PacketsHandler::Init_Network_Nodes_For_New_Transmission(const uint8_t packet_ID,
                                                             std::vector<std::string>* frames,
                                                             std::shared_ptr<std::vector<std::shared_ptr<std::string>>> packet)
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
    for (auto& it:packet_loss_map_)
    {
      it.second.sent_packets_ = it.second.sent_packets_ + 1;
    }

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
    current_int64 = get64BitsAddress(bytes, i);

    mavlink_msg->payload64.push_back(current_int64);
  }

  packet_loss_it_ = packet_loss_map_.find(mavlink_msg->sysid);

  if (packet_loss_it_ != packet_loss_map_.end())
  {
    packet_loss_it_->second.received_packets_ = packet_loss_it_->second.received_packets_ + 1;
  }
  else
  {
    database_addresses_inv_it_ = database_addresses_inv_.find(mavlink_msg->sysid);
    if (database_addresses_inv_it_ != database_addresses_inv_.end())
    {
      uint64_t long_address = database_addresses_inv_it_->second;
      packet_loss_map_.insert(std::pair<uint8_t, On_Line_Node_S>(mavlink_msg->sysid,
                                                                 {long_address, 0.0, 0.0, 0, 0, 0, 0}));
    }
  }
}

//*****************************************************************************
uint8_t PacketsHandler::getDeviceId(){
  return device_address_;
}

//*****************************************************************************
float PacketsHandler::getSignalStrength()
{
  return rssi_float_;
}

//*****************************************************************************
float PacketsHandler::getAPISignalStrength(uint8_t short_node_id)
{
  float result = 0;
  int nb_node = 0;
  if(short_node_id == ALL_IDS)
  {
    for (auto& it:rssi_result_map_)
    {
      if(it.second.status == NEW_VALUE &&
         it.second.result == 0x00)
      {
        nb_node++;
        result += static_cast<float>(it.second.avg_rssi);
        it.second.status = OLD_VALUE;
      }
    }
    if(nb_node != 0)
    {
      result =  static_cast<float>(result / nb_node);
    }
  }
  else
  {
    rssi_result_map_it_ = rssi_result_map_.find(short_node_id);
    if(rssi_result_map_it_ != rssi_result_map_.end())
    {
      if(rssi_result_map_it_->second.status == NEW_VALUE &&
         rssi_result_map_it_->second.result == 0x00)
      {
        result = static_cast<float>(rssi_result_map_it_->second.avg_rssi);
        rssi_result_map_it_->second.status = OLD_VALUE;
      }
    }
  }
  return result;
}

//*****************************************************************************
float PacketsHandler::getRawPacketLoss(uint8_t short_node_id)
{
  if(short_node_id == ALL_IDS)
  {
    int count = 0;
    float result = 0;
    for(const auto& it:packet_loss_map_)
    {
      result += it.second.packet_loss_raw_;
      count++;
    }
    if(count != 0)
    {
      return result / static_cast<float>(count);
    }
    else {return PACKET_LOSS_UNAVAILABLE;}
  }
  else
  {
    packet_loss_it_ = packet_loss_map_.find(short_node_id);

    if (packet_loss_it_ != packet_loss_map_.end())
    {
      return packet_loss_map_[short_node_id].packet_loss_raw_;
    }
    else {return PACKET_LOSS_UNAVAILABLE;}
  }
}

//*****************************************************************************
float PacketsHandler::getPacketLoss(uint8_t short_node_id)
{
  if(short_node_id == ALL_IDS)
  {
    int count = 0;
    float result = 0;
    for(const auto& it:packet_loss_map_)
    {
      result += it.second.packet_loss_filtered_;
      count++;
    }
    if(count != 0)
    {
      return result / static_cast<float>(count);
    }
    else {return PACKET_LOSS_UNAVAILABLE;}
  }
  else
  {
    packet_loss_it_ = packet_loss_map_.find(short_node_id);

    if (packet_loss_it_ != packet_loss_map_.end())
    {
      return packet_loss_map_[short_node_id].packet_loss_filtered_;
    }
    else {return PACKET_LOSS_UNAVAILABLE;}
  }
}

//*****************************************************************************
void PacketsHandler::triggerRssiUpdate()
/* Description: function sending the AT commad DB to the Xbee module.
 *
 ------------------------------------------------------------------ */
{
  std::string frame;
  Generate_AT_Command(RSSI_COMMAND.c_str(), &frame);
  serial_device_->Send_Frame(frame);
}

//*****************************************************************************
uint8_t PacketsHandler::triggerAPIRssiUpdate(uint16_t rssi_payload_size,
                                             uint16_t rssi_iterations,
                                             uint8_t target_id)
/* Description: Function triggering the link testing function described
 *    pages 29-30 in Ressources/XbeeModule_Datasheet.pdf
 *    The packet_loss_map_ is used to know which nodes are connected
 *    instead of the connected_network_nodes_ since the latest is only
 *    updated when a Ping is sent.
 ------------------------------------------------------------------ */
{
  uint8_t sent_request = 0;

  if(target_id == ALL_IDS)
  {
    for (const auto& it:packet_loss_map_)
    {
      std::string frame;
      generateLinkTestingFrame(&frame, rssi_payload_size,
                               rssi_iterations,
                               device_64_bits_address_,
                               database_addresses_inv_[it.first]);
      serial_device_->Send_Frame(frame);
      sent_request++;

      rssi_result_map_it_ = rssi_result_map_.find(it.first);
      if(rssi_result_map_it_ == rssi_result_map_.end())
      {
        RSSI_Result result ={rssi_payload_size, rssi_iterations, 0, 0, 0,
                             0, 0, 0, 0, TRIGGERED};
        rssi_result_map_[database_addresses_it_->second] = result;
      }
      else
      {
        rssi_result_map_it_->second.payload_size = rssi_payload_size;
        rssi_result_map_it_->second.iterations = rssi_iterations;
        rssi_result_map_it_->second.avg_rssi = TRIGGERED;
      }

    }
  }
  else
  {
    packet_loss_it_ = packet_loss_map_.find(target_id);
    if(packet_loss_it_ != packet_loss_map_.end())
    {
      std::string frame;
      generateLinkTestingFrame(&frame, rssi_payload_size,
                               rssi_iterations,
                               device_64_bits_address_,
                               database_addresses_inv_[packet_loss_it_->first]);
      serial_device_->Send_Frame(frame);
      sent_request++;

      rssi_result_map_it_ = rssi_result_map_.find(target_id);
      if(rssi_result_map_it_ == rssi_result_map_.end())
      {
        RSSI_Result result ={rssi_payload_size, rssi_iterations, 0, 0, 0,
                             0, 0, 0, 0, TRIGGERED};
        rssi_result_map_[database_addresses_it_->second] = result;
      }
      else
      {
        rssi_result_map_it_->second.payload_size = rssi_payload_size;
        rssi_result_map_it_->second.iterations = rssi_iterations;
        rssi_result_map_it_->second.avg_rssi = TRIGGERED;
      }
    }
  }

  return sent_request;
}

//*****************************************************************************
float PacketsHandler::computePercentage(const int16_t numerator, const int16_t denumerator) const
{
  return static_cast<float>((numerator * 100.0) / (denumerator * 1.0));
}

//*****************************************************************************
float PacketsHandler::filterIIR(const float new_val, const float old_val, const float gain) const
{
  return (new_val * (1.0 - gain)) + (old_val * gain);
}

//*****************************************************************************
void PacketsHandler::processPacketLoss(const char* packet_loss)
{
  uint8_t source_ID = static_cast<uint8_t>(packet_loss[1]);
  uint16_t received_packet_from_me = static_cast<uint16_t>(
    static_cast<uint16_t>(static_cast<unsigned char>(packet_loss[2])) << 8 |
    static_cast<uint16_t>(static_cast<unsigned char>(packet_loss[3])));
  uint8_t packet_loss_msg_id = static_cast<uint8_t>(packet_loss[4]);

  packet_loss_it_ = packet_loss_map_.find(source_ID);

  if (packet_loss_it_ != packet_loss_map_.end())
  {
    if (packet_loss_msg_id == packet_loss_it_->second.packet_loss_received_id_)
    {
      packet_loss_it_->second.packet_loss_received_id_ = (packet_loss_it_->second.packet_loss_received_id_ + 1) % MAX_PACKET_LOSS_MSG_ID;
      updatePacketLoss(packet_loss_it_->second, received_packet_from_me);
    }
    else
    {
      // Missed a packet(s) loss msg, thus we resynchronize the id
      packet_loss_it_->second.packet_loss_received_id_ = (packet_loss_msg_id + 1) % MAX_PACKET_LOSS_MSG_ID;
      printf("Resynchronized %i\n", packet_loss_it_->second.packet_loss_received_id_);
    }
  }
  else
  {
    database_addresses_inv_it_ = database_addresses_inv_.find(source_ID);
    if (database_addresses_inv_it_ != database_addresses_inv_.end())
    {
      uint64_t long_address = database_addresses_inv_it_->second;
      packet_loss_map_.insert(std::pair<uint8_t, On_Line_Node_S>(source_ID,
                                                                {long_address, 0.0, 0.0, 0, 0, 0, 0}));
    }
  }
}

//*****************************************************************************
void PacketsHandler::updatePacketLoss(On_Line_Node_S& node, const uint16_t received_packet)
/* Description: Update the packet loss of the node: node.
 * The information received is condered valid only if:
 *   - The number of sent packets is different from 0
 *   - The number of received packets is inferior to sent_packets + 2
 *
 * The + 2 allows the system to take into consideration the messages sent after
 * the packet loss information messages was sent.
 ------------------------------------------------------------------ */
{
  if(node.sent_packets_ !=  0) // Division by 0
  {
    if(node.sent_packets_ + 2 >= received_packet)
    {
      node.packet_loss_raw_ = computePercentage((static_cast<int16_t>(node.sent_packets_) - static_cast<int16_t>(received_packet)),
                                                node.sent_packets_);
      node.packet_loss_filtered_ = filterIIR(node.packet_loss_raw_,
                                             node.packet_loss_filtered_,
                                             PACKET_LOSS_FILTER_GAIN);
    }
    else
    {
      printf("Impossible received packet number: %i for %i sent (coming from %lu)",
             static_cast<int>(received_packet),
             static_cast<int>(node.sent_packets_),
             static_cast<long>(node.device_64_bits_address_));
    }
    node.sent_packets_ = 0;

  }
  else
  {
    // Do nothing
  }
}

//*****************************************************************************
void PacketsHandler::sendPacketLoss()
{
  for (auto& it:packet_loss_map_)
  {
    std::string packet_loss_message = PACKET_LOSS_IDENTIFIER;
    std::string packet_loss_frame;

    packet_loss_message.push_back(device_address_);
    packet_loss_message.push_back((it.second.received_packets_ & 0xFF00) >> 8);
    packet_loss_message.push_back(it.second.received_packets_ & 0x00FF);
    packet_loss_message.push_back(it.second.packet_loss_sent_id_);

    it.second.received_packets_ = 0;
    it.second.packet_loss_sent_id_ = (it.second.packet_loss_sent_id_ + 1) % MAX_PACKET_LOSS_MSG_ID;

    // MSG format   L       source id     received packets    packet loss msg id
    //            1 byte     8 bits           16 bits                8 bits
    Generate_Transmit_Request_Frame(packet_loss_message.c_str(),
                                    &packet_loss_frame,
                                    packet_loss_message.size(),
                                    static_cast<unsigned char>(0x01),
                                    int_to_hex(it.second.device_64_bits_address_, 16),
                                    int_to_hex(static_cast<int>(it.first), 4));

    serial_device_->Send_Frame(packet_loss_frame);

  }
}

uint16_t PacketsHandler::ucharToUint16(unsigned char msb, unsigned char lsb)
/* Description: return the numerical value of the corresponding
 *   binary number msb lsb
 ------------------------------------------------------------------ */
 {
   return (static_cast<uint16_t>(msb)<<8) + static_cast<uint16_t>(lsb);
 }

 inline uint64_t PacketsHandler::get64BitsAddress(const char* bytes,
                                                  const int offset)
 {
    return (
      static_cast<uint64_t>(static_cast<unsigned char>(bytes[offset])) << 56 |
      static_cast<uint64_t>(static_cast<unsigned char>(bytes[offset + 1])) << 48 |
      static_cast<uint64_t>(static_cast<unsigned char>(bytes[offset + 2])) << 40 |
      static_cast<uint64_t>(static_cast<unsigned char>(bytes[offset + 3])) << 32 |
      static_cast<uint64_t>(static_cast<unsigned char>(bytes[offset + 4])) << 24 |
      static_cast<uint64_t>(static_cast<unsigned char>(bytes[offset + 5])) << 16 |
      static_cast<uint64_t>(static_cast<unsigned char>(bytes[offset + 6])) << 8 |
      static_cast<uint64_t>(static_cast<unsigned char>(bytes[offset + 7])));
 }

}


}

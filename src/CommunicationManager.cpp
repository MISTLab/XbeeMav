/* CommunicationManager.cpp -- Communication Manager class for XBee:
             Handles all communications with other ROS nodes
             and the serial port --                                    */
/* ------------------------------------------------------------------------- */
/* September 20, 2016 -- @Copyright Aymen Soussia. All rights reserved.      */
/*                                  (aymen.soussia@gmail.com)                */
/* ------------------------------------------------------------------------- */
/* Revision
 * Date: July 1st, 2017
 * Author: Pierre-Yves Breches
 */
/* ------------------------------------------------------------------------- */
#include "CommunicationManager.h"


namespace Mist
{

namespace Xbee
{

//*****************************************************************************
CommunicationManager::CommunicationManager() :
  START_DLIMITER(static_cast<unsigned char>(0x7E)),
  LOOP_RATE(10), /* 10 fps */
  DEFAULT_RATE_DIVIDER_RSSI(5),
  DEFAULT_RATE_DIVIDER_PACKET_LOSS(20),
  DEFAULT_RSSI_PAYLOAD_SIZE(10),
  DEFAULT_RSSI_ITERATIONS(2)
{
}


//*****************************************************************************
CommunicationManager::~CommunicationManager()
{
}


//*****************************************************************************
bool CommunicationManager::Init(const std::string& device,
	                              const std::size_t baud_rate)
{
  if (serial_device_.Init(device, baud_rate))
  {
    serial_device_.Set_In_Messages_Pointers(&in_std_messages_, &in_fragments_,
                                            &in_Acks_and_Pings_, &command_responses_,
                                            &in_packet_loss_);

    service_thread_ = std::make_shared<std::thread>(std::thread(&SerialDevice::Run_Service, &serial_device_));

    if (!packets_handler_.Init(&serial_device_, &in_packets_))
      return false;

    std::clock_t elapsed_time = std::clock();
    bool device_ID_loaded = false;

    while (std::clock() - elapsed_time <= 300000)
    {
      Process_Command_Responses();

      if (packets_handler_.Init_Device_ID())
      {
        device_ID_loaded = true;
        break;
      }
    }

    if (!device_ID_loaded)
      return false;
  }
  else
  {
    Display_Init_Communication_Failure();
    return false;
  }

  return true;
}



//*****************************************************************************
void CommunicationManager::Run(DRONE_TYPE drone_type,
                               RUNNING_MODE running_mode)
{
  std::thread thread_packets_handler(&PacketsHandler::Run, &packets_handler_);

  Display_Drone_Type_and_Running_Mode(drone_type, running_mode);

  if (RUNNING_MODE::SWARM == running_mode)
    Run_In_Swarm_Mode();
  else
    Run_In_Solo_Mode(drone_type);

  serial_device_.Stop_Service();
  serial_device_.Close_Serial_Port();
  packets_handler_.Quit();
  service_thread_->join();
  thread_packets_handler.join();
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
      mav_dji_server_ = node_handle_.advertiseService(service_name.c_str(), &CommunicationManager::Serve_Flight_Controller, this);
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
      ros::spinOnce();
      loop_rate.sleep();
    }
  }

}


//*****************************************************************************
void CommunicationManager::Run_In_Swarm_Mode()
{

  if (getRosParams())
  {

    ros::Rate loop_rate(LOOP_RATE);

    while (ros::ok())
    {
      triggerRssiUpdate();
      Process_In_Standard_Messages();
      Process_In_Fragments();
      Process_In_Acks_and_Pings();
      Process_In_Packets();
      Process_Packet_Loss();
      Process_Command_Responses();
      packets_handler_.Delete_Packets_With_Time_Out();
      ros::spinOnce();
      loop_rate.sleep();
    }
  }
}


//*****************************************************************************
inline bool CommunicationManager::Serve_Flight_Controller(mavros_msgs::CommandInt::
                                                          Request& request, mavros_msgs::CommandInt::Response& response) // TODO to be cleaned
{
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
inline void CommunicationManager::Send_Mavlink_Message_Callback(
  const mavros_msgs::Mavlink::ConstPtr& mavlink_msg)
{
  packets_handler_.Handle_Mavlink_Message(mavlink_msg);
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


//*****************************************************************************
void CommunicationManager::Process_In_Standard_Messages()
{
  std::size_t in_messages_size = in_std_messages_.Get_Size();

  if (in_messages_size > 0)
  {
    for (std::size_t j = 0; j < in_messages_size; j++)
    {
      std::shared_ptr<std::string> in_message =
        in_std_messages_.Pop_Front();
      mavros_msgs::Mavlink mavlink_msg;

      packets_handler_.Deserialize_Mavlink_Message(in_message->c_str(), &mavlink_msg, in_message->size());
      mavlink_publisher_.publish(mavlink_msg);
    }
  }
}


//*****************************************************************************
void CommunicationManager::Process_In_Acks_and_Pings()
{
  std::size_t in_messages_size = in_Acks_and_Pings_.Get_Size();

  if (in_messages_size > 0)
  {
    for (std::size_t j = 0; j < in_messages_size; j++)
    {
      std::shared_ptr<std::string> in_message =
        in_Acks_and_Pings_.Pop_Front();

      packets_handler_.Process_Ping_Or_Acknowledgement(in_message);
    }
  }
}


//*****************************************************************************
void CommunicationManager::Process_In_Fragments()
{
  std::size_t in_messages_size = in_fragments_.Get_Size();

  if (in_messages_size > 0)
  {
    for (std::size_t j = 0; j < in_messages_size; j++)
    {
      std::shared_ptr<std::string> in_message =
        in_fragments_.Pop_Front();

      packets_handler_.Process_Fragment(in_message);
    }
  }
}


//*****************************************************************************
void CommunicationManager::Process_In_Packets()
  /* This function publish MAVROS msgs comming the xbee network.
   */
{
  std::size_t in_messages_size = in_packets_.Get_Size();

  if (in_messages_size > 0)
  {
    for (std::size_t j = 0; j < in_messages_size; j++)
    {
      std::shared_ptr<std::string> in_message =
        in_packets_.Pop_Front();
      mavros_msgs::Mavlink mavlink_msg;

      packets_handler_.Deserialize_Mavlink_Message(in_message->c_str(), &mavlink_msg, in_message->size());
      mavlink_publisher_.publish(mavlink_msg);
    }
  }
}


//*****************************************************************************
void CommunicationManager::Process_Command_Responses()
	/* This function forwards the command responses to the packethandler to be
	 * processed.
	 */
{
  std::size_t in_messages_size = command_responses_.Get_Size();

  if (in_messages_size > 0)
  {
    for (std::size_t j = 0; j < in_messages_size; j++)
    {
      std::shared_ptr<std::string> in_message =
        command_responses_.Pop_Front();

      packets_handler_.Process_Command_Response(in_message->c_str());
    }
  }
}

//*****************************************************************************
bool CommunicationManager::Get_Param (mavros_msgs::ParamGet::Request& req,
                                      mavros_msgs::ParamGet::Response& res)
  /* This function processes the requests sent to the xbee_status rosservice
   * The response success is set to true if the param_id of the request is known
   * Return: true
   */
	//TODO these ids need to be defined as constants
{
  mavros_msgs::ParamValue val;
  res.success = true;

  if(req.param_id == "id")
  {
    val.integer=packets_handler_.getDeviceId();
  }
  /////////////////////////////////////////////////////////
  // RSSI Section
  else if(req.param_id == "rssi")
  {
    val.real=packets_handler_.getSignalStrength();
  }
  else if(req.param_id == "trig_rssi_api_avg")
  {
    if(packets_handler_.triggerAPIRssiUpdate(rssi_payload_size_,
                                             rssi_iterations_,
                                             PacketsHandler::ALL_IDS) == 0)
    {
      res.success = false;
    }
  }
  else if (safeSubStr(req.param_id, 14) == "trig_rssi_api_")
  {
    int short_id = std::strtol(req.param_id.substr(14).c_str(), NULL, 10);
    if(packets_handler_.triggerAPIRssiUpdate(rssi_payload_size_,
                                             rssi_iterations_,
                                             static_cast<uint8_t>(short_id)) == 0)
    {
      res.success = false;
    }
  }
  else if (req.param_id == "get_rssi_api_avg")
  {
    val.real = packets_handler_.getAPISignalStrength(PacketsHandler::ALL_IDS);
    if(val.real == 0){res.success = false;}
  }
  else if (safeSubStr(req.param_id, 13) == "get_rssi_api_")
  {
    int short_id = std::strtol(req.param_id.substr(13).c_str(), NULL, 10);
    val.integer = short_id;
    val.real = packets_handler_.getAPISignalStrength(static_cast<uint8_t>(short_id));
    if(val.real == 0){res.success = false;}
  }
  /////////////////////////////////////////////////////////
  // Packet loss Section
  else if (req.param_id == "pl_raw_avg")
  {
    val.real = packets_handler_.getRawPacketLoss(PacketsHandler::ALL_IDS);
    if(val.real == PacketsHandler::PACKET_LOSS_UNAVAILABLE){res.success = false;}
  }
  else if (safeSubStr(req.param_id, 7) == "pl_raw_")
  {
    int short_id = std::strtol(req.param_id.substr(7).c_str(), NULL, 10);
    val.integer = short_id;
    val.real = packets_handler_.getRawPacketLoss(static_cast<uint8_t>(short_id));
    if(val.real == PacketsHandler::PACKET_LOSS_UNAVAILABLE){res.success = false;}
  }
  else if (req.param_id == "pl_filtered_avg")
  {
    val.real = packets_handler_.getPacketLoss(PacketsHandler::ALL_IDS);
    if(val.real == PacketsHandler::PACKET_LOSS_UNAVAILABLE){res.success = false;}
  }
  else if (safeSubStr(req.param_id, 12) == "pl_filtered_")
  {
    int short_id = std::strtol(req.param_id.substr(12).c_str(), NULL, 10);
    val.integer = short_id;
    val.real = packets_handler_.getPacketLoss(static_cast<uint8_t>(short_id));
    if(val.real == PacketsHandler::PACKET_LOSS_UNAVAILABLE){res.success = false;}
  }
  else
  {
    res.success = false;
  }

  res.value = val;
  return true;
}

//*****************************************************************************
bool CommunicationManager::getRosParams()
/* This function queries all the ROS parameters.
 * Return False if a mandatory parameter could not be queried.
 */
{
  std::string out_messages_topic;
  std::string in_messages_topic;
  bool success_get_param_in_topic = false;
  bool success_get_param_out_topic = false;
  StatusSrv_ = node_handle_.advertiseService("/xbee_status", &CommunicationManager::Get_Param, this);

  if (node_handle_.getParam("Xbee_In_From_Buzz", out_messages_topic))
  {
    mavlink_subscriber_ = node_handle_.subscribe(out_messages_topic.c_str(), 1000,
                                                 &CommunicationManager::Send_Mavlink_Message_Callback, this);
    success_get_param_in_topic = true;
  }
  else
  {
    std::cout << "Failed to Get Topic Name: param 'Xbee_In_From_Buzz' Not Found." << std::endl;
  }

  if (node_handle_.getParam("Xbee_Out_To_Buzz", in_messages_topic))
  {
    mavlink_publisher_ = node_handle_.advertise<mavros_msgs::Mavlink>(
      in_messages_topic.c_str(), 1000);
    success_get_param_out_topic = true;
  }
  else
  {
    std::cout << "Failed to Get Topic Name: param 'Xbee_Out_To_Buzz' Not Found." << std::endl;
  }

  rate_divider_rssi_ = static_cast<uint8_t>(getIntParam("rate_divider_rssi", DEFAULT_RSSI_ITERATIONS));
  rate_divider_packet_loss_ = static_cast<uint8_t>(getIntParam("rate_divider_packet_loss_", DEFAULT_RSSI_ITERATIONS));
  rssi_payload_size_ = static_cast<uint16_t>(getIntParam("rssi_payload_size", DEFAULT_RSSI_ITERATIONS));
  rssi_iterations_ = static_cast<uint16_t>(getIntParam("rssi_iterations", DEFAULT_RSSI_ITERATIONS));

  return success_get_param_in_topic && success_get_param_out_topic;
}

//*****************************************************************************
int  CommunicationManager::getIntParam(std::string name, int default_value)
{
  int temp_param;
  if(node_handle_.getParam(name, temp_param))
  {
    return temp_param;
  }
  else
  {
    return default_value;
  }
}

//*****************************************************************************
void CommunicationManager::triggerRssiUpdate()
{
	/* This trigger the update of the rssi parameter (Signal strength) every
	 * rate_divider_rssi_ cycles
	 */
  static uint8_t rssi_update_count = 0;

  rssi_update_count++;
  if(rssi_update_count >= rate_divider_rssi_){
    packets_handler_.triggerRssiUpdate();
    rssi_update_count = 0;
  }
}

//*****************************************************************************
void CommunicationManager::Process_Packet_Loss()
{
	/* This function passes the packet loss messages to the packethandler to be
	 * processed.
	 * It also sends the packet loss messages to the other every
	 * rate_divider_packet_loss_ cycles
	 */
  std::size_t in_messages_size = in_packet_loss_.Get_Size();

  if (in_messages_size > 0)
  {
    for (std::size_t j = 0; j < in_messages_size; j++)
    {
      std::shared_ptr<std::string> in_message =
          in_packet_loss_.Pop_Front();

      packets_handler_.processPacketLoss(in_message->c_str());
    }
  }

  packet_loss_timer_++;
  if (packet_loss_timer_ > rate_divider_packet_loss_)
  {
    packets_handler_.sendPacketLoss();
    packet_loss_timer_ = 0;
  }
}

//*****************************************************************************
std::string CommunicationManager::safeSubStr(const std::string strg,
                                             const unsigned int index_max) const
/* Verify if the operation substr(0, index_max) is possible on the given string
 * and return the substring if successful.
 * Error value: empty string;
 */
{
  if(strg.size() > index_max) {
    return strg.substr(0,index_max);
  }
  return std::string("");

}

}

}

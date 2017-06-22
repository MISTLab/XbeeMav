/* CommunicationManager.h -- Communication Manager class for XBee:
                             Handles all communications with other ROS nodes
                             and the serial port -- */
/* ------------------------------------------------------------------------- */
/* September 20, 2016 -- @Copyright Aymen Soussia. All rights reserved.      */
/*                                  (aymen.soussia@gmail.com)                */

#pragma once

#include "PacketsHandler.h"
#include "SerialDevice.h"
#include <inttypes.h>
#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/CommandInt.h>
#include <mavros_msgs/Mavlink.h>
#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/ParamValue.h>
#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <thread>

namespace Mist {

namespace Xbee {

//*****************************************************************************
struct Waypoint_S {
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

    enum class DRONE_TYPE { MASTER, SLAVE };
    enum class RUNNING_MODE { SWARM, SOLO };

    bool Init(const std::string &device, const std::size_t baud_rate);
    void Run(DRONE_TYPE drone_type, RUNNING_MODE running_mode);

  private:
    const unsigned char START_DLIMITER;
    const std::size_t LOOP_RATE;
    const uint8_t DEFAULT_RATE_DIVIDER_RSSI;
    const uint8_t DEFAULT_RATE_DIVIDER_PACKET_LOSS;
    const uint16_t DEFAULT_RSSI_PAYLOAD_SIZE;
    const uint16_t DEFAULT_RSSI_ITERATIONS;

    void Run_In_Solo_Mode(DRONE_TYPE drone_type);
    void Run_In_Swarm_Mode();
    void Display_Init_Communication_Failure();
    void Send_Mavlink_Message_Callback(
        const mavros_msgs::Mavlink::ConstPtr &mavlink_msg);
    void Display_Drone_Type_and_Running_Mode(DRONE_TYPE drone_type,
                                             RUNNING_MODE running_mode);
    bool Serve_Flight_Controller(mavros_msgs::CommandInt::Request &request,
                                 mavros_msgs::CommandInt::Response &response);
    void Check_In_Messages_and_Transfer_To_Server();
    void Process_In_Standard_Messages();
    void Process_In_Acks_and_Pings();
    void Process_In_Fragments();
    void Process_In_Packets();
    void Process_Command_Responses();
    void Process_Packet_Loss();
    bool Get_Param(mavros_msgs::ParamGet::Request &req,
                   mavros_msgs::ParamGet::Response &res);
    bool getRosParams();
    int getIntParam(std::string name, int default_value);
    void triggerRssiUpdate();
    std::string safeSubStr(const std::string strg,
                           const unsigned int index_max) const;

    Mist::Xbee::SerialDevice serial_device_;
    Mist::Xbee::PacketsHandler packets_handler_;
    Thread_Safe_Deque in_std_messages_;
    Thread_Safe_Deque in_fragments_;
    Thread_Safe_Deque in_Acks_and_Pings_;
    Thread_Safe_Deque command_responses_;
    Thread_Safe_Deque in_packets_;
    Thread_Safe_Deque in_packet_loss_;
    ros::NodeHandle node_handle_;
    ros::Subscriber mavlink_subscriber_;
    ros::Publisher mavlink_publisher_;
    ros::ServiceClient mav_dji_client_;
    ros::ServiceServer mav_dji_server_;
    ros::ServiceServer StatusSrv_;
    std_msgs::UInt8 device_id_out;
    std::shared_ptr<std::thread> service_thread_; // TO DO delete !?
    std::uint16_t packet_loss_timer_;
    uint8_t rate_divider_rssi_;
    uint8_t rate_divider_packet_loss_;
    uint16_t rssi_payload_size_;
    uint16_t rssi_iterations_;
};
}
}

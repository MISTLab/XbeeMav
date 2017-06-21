/* ----------------------------------------------------------------
 * File: frame_generators.h
 * Created on: 21/06/2017
 * Author: Pierre-Yves Breches
 * Description: This file contains functions used for the the creation of xbee
 *   frame
 * Copyright Humanitas Solutions. All rights reserved.
 ------------------------------------------------------------------ */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <iomanip>
#include <sstream>

namespace Mist
{

namespace Xbee
{

  void Generate_Transmit_Request_Frame(
    const char* message,
    std::string* frame,
    const std::size_t message_size,
    const unsigned char frame_ID = static_cast<unsigned char>(0x01),
    const std::string& destination_adssress = "000000000000FFFF",
    const std::string& short_destination_adress = "FFFF",
    const std::string& broadcast_radius = "00",
    const std::string& options = "00");

  void generateLinkTestingFrame(
     std::string* frame,
     uint16_t rssi_payload_size,
     uint16_t rssi_iterations,
     std::string device_64_bits_address,
     uint64_t target_64_bits_address);

  void Generate_AT_Command(const char* command,
                          std::string* frame,
                          const unsigned char frame_ID =
                            static_cast<unsigned char>(0x01));

  void Convert_HEX_To_Bytes(const std::string& HEX_data,
                            std::string* converted_data);
  void Calculate_and_Append_Checksum(std::string* frame);
  void Add_Length_and_Start_Delimiter(std::string* frame);

  template< typename T >
  std::string int_to_hex(const T i, int size)
  {
    std::stringstream stream;
    stream << std::setfill ('0') << std::setw(size)
           << std::hex << i;
    return stream.str();
  }
}

}

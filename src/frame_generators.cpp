/* ----------------------------------------------------------------
 * File: frame_generators.cpp
 * Created on: 21/06/2017
 * Author: Pierre-Yves Breches
 * Description: This file contains functions used for the the creation of xbee
 *   frame
 * Copyright Humanitas Solutions. All rights reserved.
 ------------------------------------------------------------------ */

#include "frame_generators.h"

namespace Mist
{

namespace Xbee
{
  void Generate_Transmit_Request_Frame(
      const char* message,
      std::string * frame,
      const std::size_t message_size,
      const unsigned char frame_ID,
      const std::string& destination_adssress,
      const std::string& short_destination_adress,
      const std::string& broadcast_radius,
      const std::string& options)
    /* Description: Generate a link testing frame as presented page 73-74
     *  in Ressources/XbeeModule_Datasheet.pdf
     ------------------------------------------------------------------ */
  {
    const unsigned char FRAME_TYPE = static_cast<unsigned char>(0x10); /* Transmit Request Frame */
    std::string frame_parameters;
    std::string bytes_frame_parameters;

    frame_parameters.append(destination_adssress);
    frame_parameters.append(short_destination_adress);
    frame_parameters.append(broadcast_radius);
    frame_parameters.append(options);

    Convert_HEX_To_Bytes(frame_parameters, &bytes_frame_parameters);

    frame->push_back(FRAME_TYPE);
    frame->push_back(frame_ID);
    frame->append(bytes_frame_parameters);
    frame->append(message, message_size);

    Calculate_and_Append_Checksum(frame);
    Add_Length_and_Start_Delimiter(frame);
  }

  void generateLinkTestingFrame(std::string* frame,
                                uint16_t rssi_payload_size,
                                uint16_t rssi_iterations,
                                std::string device_64_bits_address,
                                uint64_t target_64_bits_address)
    /* Description: Generate a link testing frame as presented page 29-30
     *  in Ressources/XbeeModule_Datasheet.pdf
     ------------------------------------------------------------------ */
  {
    static const unsigned char FRAME_TYPE = static_cast<unsigned char>(0x11);
    static const unsigned char FRAME_ID = static_cast<unsigned char>(0x01);
    static const std::string link_testing_params = "FFFEE6E60014C1050000";

    std::string frame_parameters;
    std::string bytes_frame_parameters;

    frame_parameters = link_testing_params;
    frame_parameters += int_to_hex(target_64_bits_address, 16);
    frame_parameters += int_to_hex(rssi_payload_size, 4); // packet size for testing
    frame_parameters += int_to_hex(rssi_iterations, 4); // number of iterations

    Convert_HEX_To_Bytes(frame_parameters, &bytes_frame_parameters);

    frame->push_back(FRAME_TYPE);
    frame->push_back(FRAME_ID);
    frame->append(device_64_bits_address);
    frame->append(bytes_frame_parameters);

    Calculate_and_Append_Checksum(frame);
    Add_Length_and_Start_Delimiter(frame);
  }


  void Generate_AT_Command(const char* command,
                           std::string* frame,
                           const unsigned char frame_ID)
    /* Description: Generate an AT command frame as decribed page 73
     *  in Ressources/XbeeModule_Datasheet.pdf
     ------------------------------------------------------------------ */
  {
   const unsigned char FRAME_TYPE = static_cast<unsigned char>(0x09);/* AT Command Frame */
   std::string temporary_parameter_value;

   frame->push_back(FRAME_TYPE);
   frame->push_back(frame_ID);
   frame->append(command);

   Calculate_and_Append_Checksum(frame);
   Add_Length_and_Start_Delimiter(frame);
  }

  void Convert_HEX_To_Bytes(const std::string& HEX_data,
                            std::string* converted_data)
     /* Description:
      *
      ------------------------------------------------------------------ */
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

  void Calculate_and_Append_Checksum(std::string* frame)
  /* Description:
   *
   ------------------------------------------------------------------ */
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


  void Add_Length_and_Start_Delimiter(std::string* frame)
  /* Description:
   *
   ------------------------------------------------------------------ */
  {
    const unsigned short FIVE_BYTES = 5;
    const unsigned char START_DLIMITER = static_cast<unsigned char>(0x7E);
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

}

}

/* ----------------------------------------------------------------
 * File: XBeeSetup.cpp
 * Date: 02/06/2017
 * Author: Pierre-Yves Breches
 * Description: Implementation of the xbeeSetup function to
 * configure the xbee module
 * Copyright Humanitas Solutions. All rights reserved.
   ------------------------------------------------------------------ */

#include "XBeeSetup.h"

bool setupXBee(const std::string &device_port, const unsigned int baud_rate) {
  XBeeModule xbee_module;
  XMLConfigParser config_parser;

  if (xbee_module.Init_Port(device_port, baud_rate)) {
    std::thread th_service(&XBeeModule::Run_Service, &xbee_module);

    while (!xbee_module.Is_Connected() &&
           !xbee_module.Check_Time_Out_Exceeded()) {
      continue;
    }

    if (xbee_module.Is_Connected()) {
      std::cout << "Connected to XBee." << std::endl;
      std::cout << "Loading Config File..." << std::endl;

      if (config_parser.Load_Config()) {
        std::cout << "Config Loaded Successfully." << std::endl;
        std::cout << "Transferring Data..." << std::endl;
        std::vector<XBee_Parameter_S> *config_parameters =
            config_parser.Get_Loaded_Parameters();

        for (std::size_t i = 0; i < config_parameters->size(); i++) {
          std::string current_command;
          xbee_module.Format_AT_Command(config_parameters->at(i),
                                        &current_command);
          xbee_module.Send_Data(current_command);
        }

        std::string write_command = "ATWR \r";
        xbee_module.Send_Data(write_command);
      }
    }

    th_service.join();

    std::cout << "Exiting AT Command Mode..." << std::endl;

    if (xbee_module.Is_Connected())
    {
      xbee_module.Exit_AT_Command_Mode();

      if (config_parser.Is_Config_Loaded_Successfully())
      {
        std::cout << "XBee Configured Successfully." << std::endl;
        return true;
      }
      else
      {
        std::cout << "XBee Configuration Failed." << std::endl;
      }
    }
    else
    {
      std::cout << "XBee Configuration Failed." << std::endl;
    }
  }
  return false;
}

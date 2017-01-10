/* main.cpp --                 				                     */
/* ------------------------------------------------------------------------- */
/* November 30, 2016 -- @Copyright Aymen Soussia. All rights reserved.       */
/*                                  (aymen.soussia@gmail.com)                */


#include<thread>

#include"XBeeModule.h"
#include"XMLConfigParser.h"


//*****************************************************************************
void Setup_XBee(const std::string& device_port, const unsigned int baud_rate)
{
	XBeeModule xbee_module;
	XMLConfigParser config_parser;

	if (xbee_module.Init_Port(device_port, baud_rate))
	{
		std::thread th_service(&XBeeModule::Run_Service, &xbee_module);

		while (!xbee_module.Is_Connected() && !xbee_module.Check_Time_Out_Exceeded())
		{
			continue;
		}

		if (xbee_module.Is_Connected())
		{
			std::cout << "Connected to XBee." << std::endl;
			std::cout << "Loading Config File..." << std::endl;

			if (config_parser.Load_Config())
			{
				std::cout << "Config Loaded Successfully." << std::endl;
				std::cout << "Transferring Data..." << std::endl;
				std::vector<XBee_Parameter_S>* config_parameters = config_parser.Get_Loaded_Parameters();

				for (std::size_t i = 0; i < config_parameters->size(); i++)
				{
					std::string current_command;
					xbee_module.Format_AT_Command(config_parameters->at(i), &current_command);
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
				std::cout << "XBee Configured Successfully." << std::endl;
			else
				std::cout << "XBee Configuration Failed." << std::endl;
		}
		else
		{
			std::cout << "XBee Configuration Failed." << std::endl;
		}
	}
}


//*****************************************************************************
int main(int argc, char*argv[])
{
	try
	{
		std::string device_port;
		unsigned int baud_rate = 0;
		const unsigned int DEFAULT_BAUD_RATE = 9600;
		const std::string DEFAULT_DEVICE_PORT = "/dev/ttyUSB0";

		if (argc  < 1)
			device_port = DEFAULT_DEVICE_PORT;
		else
			device_port.append(argv[1]);

		if (argc < 2)
			baud_rate = DEFAULT_BAUD_RATE;
		else
			sscanf(argv[2], "%u", &baud_rate);

		Setup_XBee(device_port, baud_rate);
	}
	catch (const std::exception& e)
	{
		std::cout << "Error: " <<  e.what() << std::endl;
		std::cout << "Please Try One of The Following Options:" << std::endl;
		std::cout << "  1) Change The Baud Rate. Make Sure It Matches The Current Baud Rate Used by The XBee (By default BD = 9600 bps)." << std::endl;
		std::cout << "  2) Change The Specified Device (e.g. COM1 for Windows or /dev/tty/USB0 for Linux)." << std::endl;
		std::cout << "  3) Disconnect and Connect The XBee." << std::endl;
		std::cout << "  4) Press The Reset Button on The XBee." << std::endl;
		std::cout << "  5) Update The Firmware With XCTU." << std::endl;
		std::cout << "XBee Configuration Failed." << std::endl;

	}
	catch (...)
	{
		std::cout << " Unexpected Error." << std::endl;
		std::cout << "XBee Configuration Failed." << std::endl;
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}

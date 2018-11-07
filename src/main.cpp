/* main.cpp --                 				                     */
/* ------------------------------------------------------------------------- */
/* November 30, 2016 -- @Copyright Aymen Soussia. All rights reserved.       */
/*                                  (aymen.soussia@gmail.com)                */

#include "XBeeSetup.h"


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

    //setupXBee(device_port, baud_rate);
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

/* xbee.cpp -- XBeeMav ROS node main --                                      */
/* ------------------------------------------------------------------------- */
/* September 20, 2016 -- @Copyright Aymen Soussia. All rights reserved.      */
/*                                  (aymen.soussia@gmail.com)                */

#include "XBeeSetup.h"
#include "CommunicationManager.h"
#include <climits>

//*****************************************************************************
int main(int argc, char *argv[]) {

  try {
    ros::init(argc, argv, "xbee");

    Mist::Xbee::CommunicationManager communication_manager;
    std::string device;
    double baud_rate;
    Mist::Xbee::CommunicationManager::DRONE_TYPE drone_type =
        Mist::Xbee::CommunicationManager::DRONE_TYPE::SLAVE;
    Mist::Xbee::CommunicationManager::RUNNING_MODE running_mode =
        Mist::Xbee::CommunicationManager::RUNNING_MODE::SOLO;

    if (argc > 1) {
      if (!strcmp(argv[1], "master"))
        drone_type = Mist::Xbee::CommunicationManager::DRONE_TYPE::MASTER;

      if (argc > 2) {
        if (!strcmp(argv[2], "swarm"))
          running_mode = Mist::Xbee::CommunicationManager::RUNNING_MODE::SWARM;
      }
    }

    ros::NodeHandle node_handle;

    if (!node_handle.getParam("USB_port", device))
    {
      std::cout << "Failed to Get Topic Name: param 'USB_port' Not Found." << std::endl;
      return EXIT_FAILURE;
    }

    if (!node_handle.getParam("Baud_rate", baud_rate))
    {
      std::cout << "Failed to Get Topic Name: param 'Baud_rate' Not Found." << std::endl;
      return EXIT_FAILURE;
    }

    //setupXBee(device, static_cast<std::size_t>(baud_rate));

    std::cout << "Going to init !!!!" << std::endl;

    if (communication_manager.Init(device, static_cast<std::size_t>(baud_rate)))
    {
      communication_manager.Run(drone_type, running_mode);
    }
  }
  catch (const std::exception &e)
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
    std::cout << "Unexpected Fatal Error." << std::endl;
    std::cout << "Communication With XBee is Lost." << std::endl;
    return EXIT_FAILURE;
  }

  std::cout << std::endl;
  std::cout << "XBeeMav Exited." << std::endl;
  return EXIT_SUCCESS;
}

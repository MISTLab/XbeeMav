/* xbee.cpp -- XBeeMav ROS node main --                                      */
/* ------------------------------------------------------------------------- */
/* September 20, 2016 -- @Copyright Aymen Soussia. All rights reserved.      */
/*                                  (aymen.soussia@gmail.com)                */


#include "CommunicationManager.h"


//*****************************************************************************
int main(int argc, char* argv[])
{

  try
  {
	ros::init(argc, argv, "xbee");

	Mist::Xbee::CommunicationManager communication_manager;
	const std::string& device = "/dev/ttyUSB0"; // TO DO can be introduced as command.
<<<<<<< HEAD
	const std::size_t baud_rate = 115200; // TO DO Can be introduced as command.
=======
	const std::size_t baud_rate = 230400; // TO DO Can be introduced as command.
>>>>>>> a16cf8b196cb6b63ef52ea26b8cb9a8e861d84d1
	Mist::Xbee::CommunicationManager::DRONE_TYPE drone_type = 
			Mist::Xbee::CommunicationManager::DRONE_TYPE::SLAVE;
	Mist::Xbee::CommunicationManager::RUNNING_MODE running_mode =
			Mist::Xbee::CommunicationManager::RUNNING_MODE::SOLO;

	if (argc > 1)
	{
		if (!strcmp(argv[1], "master"))
			drone_type = Mist::Xbee::CommunicationManager::DRONE_TYPE::MASTER;

		if (argc > 2)
		{
			if (!strcmp(argv[2], "swarm"))
				running_mode = Mist::Xbee::CommunicationManager::RUNNING_MODE::SWARM;
		}
	}
<<<<<<< HEAD
		

=======
	
>>>>>>> a16cf8b196cb6b63ef52ea26b8cb9a8e861d84d1
	if (communication_manager.Init(device, baud_rate))
		communication_manager.Run(drone_type, running_mode);
  }
  catch (const std::exception& e)
  {
	std::cout << "Error Occured:" << std::endl;
	std::cout << e.what() << std::endl;
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

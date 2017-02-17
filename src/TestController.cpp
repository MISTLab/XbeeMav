/* FlightController.cpp -- Basic Flight Controller ROS node --               */
/* ------------------------------------------------------------------------- */
/* September 20, 2016 -- @Copyright Aymen Soussia. All rights reserved.      */
/*                                  (aymen.soussia@gmail.com)                */


#include<climits>
#include<cstdlib> 
#include<ctime> 
#include<iostream>

#include<mavros_msgs/CommandCode.h>
#include<mavros_msgs/CommandInt.h>
#include<mavros_msgs/Mavlink.h>
#include <ros/ros.h>


enum DRONE_TYPE {MASTER, SLAVE};
const std::size_t LOOP_RATE = 10; /* (10 fps) */


//*****************************************************************************
float Get_Random_Float(float min, float max)
{
	return min + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(max - min)));
}


//*****************************************************************************
unsigned long int Get_Random_Unsigned_Int(unsigned long int min, unsigned long int max)
{
	return rand() % (max - min) + min;
}


//*****************************************************************************
bool Fill_Command_Message(mavros_msgs::CommandInt* cmd_msg, const unsigned int command_ID)
{
	switch (command_ID)
	{
		case 7:
			cmd_msg->request.command = mavros_msgs::CommandCode::NAV_TAKEOFF;
			return true;
		case 8:
			cmd_msg->request.command = mavros_msgs::CommandCode::NAV_LAND;
			return true;
		case 9:
			cmd_msg->request.command = mavros_msgs::CommandCode::NAV_RETURN_TO_LAUNCH;
			return true;
		case 21:
			cmd_msg->request.command = mavros_msgs::CommandCode::NAV_WAYPOINT;
			cmd_msg->request.x = Get_Random_Unsigned_Int(0, ULONG_MAX);
			cmd_msg->request.y = Get_Random_Unsigned_Int(0, ULONG_MAX);
			cmd_msg->request.z = Get_Random_Float(0, FLT_MAX);
			return true;
		case 22:
			cmd_msg->request.command = mavros_msgs::CommandCode::CMD_MISSION_START;
			return true;
		case 23:
			cmd_msg->request.command = mavros_msgs::CommandCode::CMD_MISSION_START;
			cmd_msg->request.param1 = 666;
			return true;
		default:
			return false;
	}
}


//*****************************************************************************
void Init_Random_Seed()
{
	srand (time(NULL));
}


//*****************************************************************************
bool Serve_XBee(mavros_msgs::CommandInt::Request& request,
		 mavros_msgs::CommandInt::Response& response)
{
	switch(request.command)
	{
		case mavros_msgs::CommandCode::NAV_TAKEOFF:
			response.success = true;
			std::cout << "Received TakeOff Command" << std::endl;
			return true;
		case mavros_msgs::CommandCode::NAV_LAND:
			response.success = true;
			std::cout << "Received Land Command" << std::endl;
			return true;	
		case mavros_msgs::CommandCode::NAV_RETURN_TO_LAUNCH:
			response.success = true;
			std::cout << "Received Return To Launch Command" << std::endl;
			return true;	
		case mavros_msgs::CommandCode::NAV_WAYPOINT:
			std::cout << "Received New Waypoint Command" << std::endl;
			std::cout << "	Latitude  " << request.x << std::endl;
			std::cout << "	Longitude  " <<  request.y << std::endl;
			std::cout << "	Altitude  " <<  request.z << std::endl;
			std::cout << "	Staytime  " <<  std::endl;
			std::cout << "	Heading  " <<  std::endl;
			response.success = true;
			return true;	
		case mavros_msgs::CommandCode::CMD_MISSION_START:
			std::cout << "Received Mission Start Command" << std::endl;
			response.success = true;
			return true;
		default:
			std::cout << "Unknown Command" << std::endl;
			response.success = false;
			return false;		
	}

}


//*****************************************************************************
void Run_As_Master_Drone()
{
  

  Init_Random_Seed();
  
  const std::size_t MAX_BUFFER_SIZE = 64;
  char line[MAX_BUFFER_SIZE];
  unsigned int command_ID = 0;
  const unsigned int MIN_COMMAND_ID = 1;
  const unsigned int MAX_COMMAND_ID = 32;
  ros::NodeHandle node_handle;
  ros::ServiceClient mav_dji_client_ = node_handle.serviceClient<mavros_msgs::CommandInt>("xbee_cmd");

  std::cout << "Enter a Command ID Between 1 and 32" << std::endl;

  ros::Rate loop_rate(LOOP_RATE);

  while (ros::ok() && std::cin.getline(line, MAX_BUFFER_SIZE))
  {
    sscanf(line, "%u", &command_ID);

    if (command_ID >= MIN_COMMAND_ID && command_ID <= MAX_COMMAND_ID)
    {
	mavros_msgs::CommandInt new_command;
	Fill_Command_Message(&new_command, command_ID);

	if (mav_dji_client_.call(new_command))
		std::cout << "Sent Command." << std::endl;
	else
		std::cout << "Failed to Send Command." << std::endl;
    }
    else
    {
    	if (command_ID < MIN_COMMAND_ID)
    		std::cout << "Introduced Command is Smaller Than 1" << std::endl;
        else if (command_ID > MAX_COMMAND_ID)
		std::cout << "Introduced Command is Greater Than 32" << std::endl;
    }
  

    ros::spinOnce();
    loop_rate.sleep();
    std::cout << "Enter a Command ID Between 1 and 32" << std::endl;
  }
}




//*****************************************************************************
void Run_As_Slave_Drone()
{
  ros::NodeHandle node_handle;
  ros::ServiceServer service = node_handle.advertiseService("mav_dji_cmd", Serve_XBee);

	ros::Rate loop_rate(LOOP_RATE);
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
}


//*****************************************************************************
int main(int argc, char** argv)
{
  DRONE_TYPE drone_type = SLAVE;

  if (argc > 1)
  {
	if (!strcmp(argv[1], "master"))
		drone_type = MASTER;
  }

  if (SLAVE == drone_type)
	std::cout << "Drone: Slave" << std::endl;
  else
	std::cout << "Drone: Master" << std::endl;


  std::cout << "Flight Controller Running in SOLO Mode..." << std::endl;
  
  ros::init(argc, argv, "flight_controller");

  if (MASTER == drone_type)
	Run_As_Master_Drone();
  else
	Run_As_Slave_Drone();


  std::cout << "Flight Controller Exited." << std::endl;

  return 0;
}

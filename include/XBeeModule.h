/* XBeeModule.h -- Xbee Module class provides functions to communicate 
                   with Xbee  --                                             */
/* ------------------------------------------------------------------------- */
/* November 30, 2016 -- @Copyright Aymen Soussia. All rights reserved.       */
/*                                  (aymen.soussia@gmail.com)                */


#pragma once

#include<deque>
#include<iostream>
#include<unistd.h>

#include<boost/asio.hpp>
#include<boost/bind.hpp>

#include"XBeeParameter.h"


//*****************************************************************************
using namespace boost;


//*****************************************************************************
typedef std::deque < std::string > commands_queue;


//*****************************************************************************
class XBeeModule
{
public:
	XBeeModule();
	~XBeeModule();

	enum {MAX_MSG_LENGTH = 8};

	bool Init_Port(const std::string& device_port, const unsigned int baud_rate);
	void Run_Service();
	void Send_Data(const std::string& command);
	void Exit_AT_Command_Mode();
	bool Is_Connected();
	bool Check_Time_Out_Exceeded();
	void Format_AT_Command(const XBee_Parameter_S& parameter, std::string* command);

private:

	const unsigned short TIME_OUT;

	bool Open_Port(const std::string& device_port);
	void Start_Receive();
	void Handle_Receive(const system::error_code& error, size_t bytes_transferred);
	void Set_AT_Command_Mode();
	void Check_Time_Out(const system::error_code& error);
	void Do_Write();
	void Handle_Write(const system::error_code& error);

	asio::io_service io_service_;
	asio::serial_port serial_port_;
	asio::deadline_timer timer_;
	char receive_buffer_[MAX_MSG_LENGTH];
	bool connected_to_XBee_;
	bool time_out_exceeded_;
	commands_queue commands_sequence_;
};


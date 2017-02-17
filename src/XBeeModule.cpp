/* XBeeModule.cpp -- Xbee Module class provides functions to communicate 
                     with Xbee	--                                           */
/* ------------------------------------------------------------------------- */
/* November 30, 2016 -- @Copyright Aymen Soussia. All rights reserved.       */
/*                                  (aymen.soussia@gmail.com)                */


#include "XBeeModule.h"


//*****************************************************************************
XBeeModule::XBeeModule():
	TIME_OUT(7),
	serial_port_(io_service_),
	timer_(serial_port_.get_io_service()),
	connected_to_XBee_(false),
	time_out_exceeded_(false)
{
}


//*****************************************************************************
XBeeModule::~XBeeModule()
{
}


//*****************************************************************************
bool XBeeModule::Init_Port(const std::string& device_port, const unsigned int baud_rate)
{
	if (Open_Port(device_port))
	{
		serial_port_.set_option(boost::asio::serial_port::baud_rate(baud_rate));
		Start_Receive();
		Set_AT_Command_Mode();
		timer_.expires_from_now(posix_time::seconds(TIME_OUT));
		timer_.async_wait(bind(&XBeeModule::Check_Time_Out, this,
			boost::asio::placeholders::error));
		return true;
	}
	else
		return false;
}


//*****************************************************************************
bool XBeeModule::Open_Port(const std::string& device_port)
{
	serial_port_.open(device_port);

	if (serial_port_.is_open())
	{
		std::cout << "Serial Port Open..." << std::endl;
		return true;
	}
	else
	{
		std::cout << "Failed to Open The Serial Port." << std::endl;
		std::cout << "Please Check The Introduced Serial Port is Correct." << std::endl;
		return false;
	}
}


//*****************************************************************************
void XBeeModule::Start_Receive()
{
	const std::size_t THREE_BYTES = 3;

	boost::asio::async_read(serial_port_,
		boost::asio::buffer(receive_buffer_, THREE_BYTES),
		bind(&XBeeModule::Handle_Receive, this,
			asio::placeholders::error,
			asio::placeholders::bytes_transferred));
}


//*****************************************************************************
void XBeeModule::Handle_Receive(const boost::system::error_code& error, size_t bytes_transferred)
{
	if (error)
	{
		std::cout << "An Error Occured: " << error << std::endl;
	}

	if (0 == strncmp(receive_buffer_, "OK\r", bytes_transferred) && !connected_to_XBee_)
	{
		connected_to_XBee_ = true;
	}
}


//*****************************************************************************
void XBeeModule::Set_AT_Command_Mode()
{
	const useconds_t ONE_SECOND = 1*1000*1000; /* 1s = 1 * 10⁶ microseconds. */
	char AT_command_sequence[] = "+++";
	const unsigned short THREE_BYTES = 3;

	usleep(ONE_SECOND);
	serial_port_.write_some(asio::buffer(AT_command_sequence, THREE_BYTES));
	usleep(ONE_SECOND);
}


//*****************************************************************************
void XBeeModule::Check_Time_Out(const system::error_code& error)
{
	if (error)
		return;
	else
	{
		if (!connected_to_XBee_)
		{
			std::cout << "Time Out: The XBee Module is Not Responding." << std::endl;
			std::cout << "Please Try One of The Following Options:" << std::endl;
			std::cout << "  1) Change The Baud Rate. Make Sure It Matches The Baud Rate Used by The XBee." << std::endl;
			std::cout << "  2) Disconnect and Connect The XBee." << std::endl;
			std::cout << "  3) Press The Reset Button on The XBee." << std::endl;
			std::cout << "  4) Update The Firmware With XCTU." << std::endl;

			serial_port_.close();
			time_out_exceeded_ = true;
		}
	}
}


//*****************************************************************************
void XBeeModule::Run_Service()
{
	io_service_.run();
}


//*****************************************************************************
void XBeeModule::Send_Data(const std::string& command)
{
	io_service_.post(
		[this, command]()
	{
		bool write_in_progress = !commands_sequence_.empty();
		commands_sequence_.push_back(command);
		if (!write_in_progress)
			Do_Write();
	});
}


//*****************************************************************************
void XBeeModule::Exit_AT_Command_Mode()
{
	const unsigned short FIVE_BYTES = 5;
	serial_port_.write_some(boost::asio::buffer("ATCN\r", FIVE_BYTES));

	io_service_.post([this]() {io_service_.stop(); });
	io_service_.post([this]() {serial_port_.close(); });
}


//*****************************************************************************
bool XBeeModule::Is_Connected()
{
	return connected_to_XBee_;
}


//*****************************************************************************
bool XBeeModule::Check_Time_Out_Exceeded()
{
	return time_out_exceeded_;
}


//*****************************************************************************
void XBeeModule::Do_Write()
{
	boost::asio::async_write(serial_port_,
		boost::asio::buffer(commands_sequence_.front().data(), commands_sequence_.front().size()),
		boost::bind(&XBeeModule::Handle_Write, this,
			asio::placeholders::error));
}


//*****************************************************************************
void XBeeModule::Handle_Write(const system::error_code& error)
{
	if (!error)
	{
		commands_sequence_.pop_front();
		if (!commands_sequence_.empty())
			Do_Write();
	}
}


//*****************************************************************************
void XBeeModule::Format_AT_Command(const XBee_Parameter_S& parameter, std::string* command)
{
	std::string current_command = "AT";
	current_command.append(parameter.command_);
	current_command.append(" ");
	current_command.append(parameter.value_);
	current_command.append("\r");
	command->append(current_command);
}

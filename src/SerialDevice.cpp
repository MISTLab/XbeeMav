/* SerialDevice.cpp -- Serial Device class to handle serial communications
                       with XBee --                                          */
/* ------------------------------------------------------------------------- */
/* September 20, 2016 -- @Copyright Aymen Soussia. All rights reserved.      */
/*                                  (aymen.soussia@gmail.com)                */


#include "SerialDevice.h"


namespace Mist
{


namespace Xbee
{


//*****************************************************************************
SerialDevice::SerialDevice():
	serial_port_(io_service_)
{
}


//*****************************************************************************
SerialDevice::~SerialDevice()
{
}


//*****************************************************************************
bool SerialDevice::Init(
		const std::string & device, const std::size_t baud_rate)
{
	serial_port_.open(device);
	
	if (serial_port_.is_open())
	{
		Set_Port_Options(baud_rate);
		Init_Frame_Type_Keys();
<<<<<<< HEAD
		Read_Frame_Header();
=======
>>>>>>> a16cf8b196cb6b63ef52ea26b8cb9a8e861d84d1
		return true;
	}
	else
	{
		std::cout << "Port Failed To Open." << std::endl;
		return false;
	}
}


//*****************************************************************************
void SerialDevice::Set_Port_Options(const std::size_t baud_rate)
{
	serial_port_.set_option(boost::asio::serial_port::baud_rate(baud_rate));
	serial_port_.set_option(boost::asio::serial_port::character_size(8));
	serial_port_.set_option(boost::asio::serial_port::parity(
			boost::asio::serial_port::serial_port_base::parity::none));
	serial_port_.set_option(boost::asio::serial_port::stop_bits(
			boost::asio::serial_port::serial_port_base::stop_bits::one));
	serial_port_.set_option(boost::asio::serial_port::flow_control(
			boost::asio::serial_port::serial_port_base::flow_control::none));
}


//*****************************************************************************
void SerialDevice::Send_Frame(const std::string& frame)
{
	io_service_.post(
		[this, frame]()
	{
		bool write_in_progress = !out_messages_.empty();
		out_messages_.push_back(frame);
		
		if (!write_in_progress)
			Write_Frame();
	});
}


//*****************************************************************************
void SerialDevice::Run_Service()
{
<<<<<<< HEAD
=======
	Read_Frame_Header();
>>>>>>> a16cf8b196cb6b63ef52ea26b8cb9a8e861d84d1
	io_service_.run();
}


//*****************************************************************************
void SerialDevice::Stop_Service()
{
	io_service_.post([this]() {io_service_.stop(); });
<<<<<<< HEAD
=======
}


//*****************************************************************************
void SerialDevice::Close_Serial_Port()
{
>>>>>>> a16cf8b196cb6b63ef52ea26b8cb9a8e861d84d1
	io_service_.post([this]() {serial_port_.close(); });
}


//*****************************************************************************
<<<<<<< HEAD
Thread_Safe_Deque * SerialDevice::Get_In_Messages_Pointer()
{
	return &in_messages_;
=======
void SerialDevice::Set_In_Messages_Pointers(Thread_Safe_Deque* in_std_messages,
			Thread_Safe_Deque* in_fragments,
			Thread_Safe_Deque* in_Acks_and_Pings,
			Thread_Safe_Deque* command_responses)
{
	in_std_messages_ = in_std_messages;
	in_fragments_ = in_fragments;
	in_Acks_and_Pings_ = in_Acks_and_Pings;
	command_responses_ = command_responses;
>>>>>>> a16cf8b196cb6b63ef52ea26b8cb9a8e861d84d1
}


//*****************************************************************************
void SerialDevice::Init_Frame_Type_Keys()
{
	sscanf("08", "%02X", &FRAME_TYPE_KEYS[AT_COMMAND]);
	sscanf("09", "%02X", &FRAME_TYPE_KEYS[AT_COMMAND_QUEUE_REGISTER_VALUE]);
	sscanf("10", "%02X", &FRAME_TYPE_KEYS[TRANSMIT_REQUEST]);
	sscanf("11", "%02X",
			&FRAME_TYPE_KEYS[EXPLICIT_ADDRESSING_COMMAND_FRAME]);
	sscanf("17", "%02X", &FRAME_TYPE_KEYS[REMOTE_AT_COMMAND]);
	sscanf("88", "%02X", &FRAME_TYPE_KEYS[AT_COMMAND_RESPONSE]);
	sscanf("8A", "%02X", &FRAME_TYPE_KEYS[MODEM_STATUS]);
	sscanf("8B", "%02X", &FRAME_TYPE_KEYS[TRANSMIT_STATUS]);
	sscanf("8D", "%02X", &FRAME_TYPE_KEYS[ROUTE_INFORMATION_PACKET]);
	sscanf("8E", "%02X", &FRAME_TYPE_KEYS[AGGREGATE_ADDRESSING_UPDATE]);
	sscanf("90", "%02X", &FRAME_TYPE_KEYS[RECEIVE_PACKET]);
	sscanf("91", "%02X", &FRAME_TYPE_KEYS[EXPLICIT_RX_INDICATOR]);
	sscanf("92", "%02X", &FRAME_TYPE_KEYS[IO_DATA_SAMPLE_RX_INDICATOR]);
	sscanf("95", "%02X", &FRAME_TYPE_KEYS[NODE_IDENTIFICATION_INDICATOR]);
	sscanf("97", "%02X", &FRAME_TYPE_KEYS[REMOTE_AT_COMMAND_RESPONSE]);
}


//*****************************************************************************
void SerialDevice::Read_Frame_Header()
{
	boost::asio::async_read(serial_port_,
		boost::asio::buffer(current_frame_.Get_Frame_Data(),
			Xbee::Frame::FRAME_HEADER_LENGTH),
		[this](boost::system::error_code error, std::size_t)
	{
		if (!error)
		{
			int start_delimiter_position = 
				current_frame_.Get_Start_Delimiter_Position();
			
			if (start_delimiter_position >= 0)
			{
				if (0 == start_delimiter_position)
					current_frame_.Decode_Frame_Header();
				else
				{
					/* The header is corrupted. */
					boost::asio::async_read(serial_port_,
						boost::asio::buffer(current_frame_.Get_Frame_Data(),
						start_delimiter_position),
						[this](boost::system::error_code error, std::size_t)
					{
						current_frame_.Rearrange_Corrupted_Header();
						current_frame_.Decode_Frame_Header();
					});
				}

				Read_Frame_Body();
			}
			else
<<<<<<< HEAD
				/* The header is totally corrupted, read another header. */
				Read_Frame_Header();
=======
			{
				/* The header is totally corrupted, read another header. */
				Read_Frame_Header();
			}
>>>>>>> a16cf8b196cb6b63ef52ea26b8cb9a8e861d84d1
		}
		else
		{
			std::cout << "Error Occured:" << std::endl;
			std::cout << error << std::endl;
			std::cout << "Communication With XBee is Lost." << std::endl;
			serial_port_.close();
		}
	});
}


//*****************************************************************************
void SerialDevice::Read_Frame_Body()
{
	boost::asio::async_read(serial_port_,
		boost::asio::buffer(current_frame_.Get_Frame_Body(),
			current_frame_.Get_Frame_Body_Length()),
		[this](boost::system::error_code error, std::size_t)
	{
		if (!error)
		{
			if (current_frame_.Get_Frame_Type() == 
					FRAME_TYPE_KEYS[RECEIVE_PACKET])
			{
<<<<<<< HEAD
				const unsigned short ELEVEN_BYTES = 11;
				const unsigned short TWELVE_BYTES = 12;

				std::shared_ptr<std::string> in_message =
					std::make_shared<std::string>();
				in_message->append(current_frame_.Get_Frame_Body()
					+ ELEVEN_BYTES,
					current_frame_.Get_Frame_Body_Length() - TWELVE_BYTES);
				in_messages_.Push_Pack(in_message);
=======
				char msg_type = current_frame_.Get_Message_Type();
				std::shared_ptr<std::string> in_message =
					std::make_shared<std::string>();
				
				if (msg_type == 'F')
				{
					in_message->append(current_frame_.Get_Frame_Body()
					+ 11,
					current_frame_.Get_Frame_Body_Length() - 12);
					in_fragments_->Push_Back(in_message);
				}
					
				else if (msg_type == 'A' || msg_type == 'P')
				{
					in_message->append(current_frame_.Get_Frame_Body(),
					current_frame_.Get_Frame_Body_Length() - 1);
					in_Acks_and_Pings_->Push_Back(in_message);
				}
					
				else if (msg_type == 'S')
				{
					in_message->append(current_frame_.Get_Frame_Body() + 12,
					current_frame_.Get_Frame_Body_Length() - 13);
					in_std_messages_->Push_Back(in_message);
				}
			}
			else if (current_frame_.Get_Frame_Type() == 
					FRAME_TYPE_KEYS[AT_COMMAND_RESPONSE])
			{	
				std::shared_ptr<std::string> in_message =
					std::make_shared<std::string>();
				
				in_message->append(current_frame_.Get_Frame_Body() + 1,
					current_frame_.Get_Frame_Body_Length() - 2);
				
				command_responses_->Push_Back(in_message);
>>>>>>> a16cf8b196cb6b63ef52ea26b8cb9a8e861d84d1
			}

			Read_Frame_Header();
		}
		else
		{
			std::cout << "Error Occured:" << std::endl;
			std::cout << error << std::endl;
			std::cout << "Communication With XBee is Lost." << std::endl;
			serial_port_.close();
		}
	});
}


//*****************************************************************************
void SerialDevice::Write_Frame()
{
	boost::asio::async_write(serial_port_,
		boost::asio::buffer(out_messages_.front().data(),
			out_messages_.front().size()),
		[this](boost::system::error_code error,
			std::size_t transferred_bytes)
	{
		if (!error)
		{
			out_messages_.pop_front();
			if (!out_messages_.empty())
				Write_Frame();
		}
	});
}


<<<<<<< HEAD
=======
//*****************************************************************************
bool SerialDevice::Is_IO_Service_Stopped()
{
	return io_service_.stopped();
}


//*****************************************************************************
void SerialDevice::Reset_IO_Service()
{
	io_service_.reset();
}


>>>>>>> a16cf8b196cb6b63ef52ea26b8cb9a8e861d84d1
}


}

/* XBeeFrame.cpp -- XBee Frame class --                                      */
/* ------------------------------------------------------------------------- */
/* September 20, 2016 -- @Copyright Aymen Soussia. All rights reserved.      */
/*                                  (aymen.soussia@gmail.com)                */


#include "XBeeFrame.h"


namespace Mist
{


namespace Xbee
{


//*****************************************************************************
Frame::Frame():
	frame_body_length_(0),
	frame_type_(0),
	start_delimiter_position_(0)
{
}


//*****************************************************************************
Frame::~Frame()
{
}


//*****************************************************************************
const char * Frame::Get_Frame_Data() const
{
	return data_buffer_;
}


//*****************************************************************************
char * Frame::Get_Frame_Data()
{
	return data_buffer_;
}


//*****************************************************************************
std::size_t Frame::Get_Frame_Length() const
{
	return FRAME_HEADER_LENGTH + frame_body_length_;
}


//*****************************************************************************
const char * Frame::Get_Frame_Body() const
{
	return data_buffer_ + FRAME_HEADER_LENGTH;
}


//*****************************************************************************
char * Frame::Get_Frame_Body()
{
	return data_buffer_ + FRAME_HEADER_LENGTH;
}


//*****************************************************************************
std::size_t Frame::Get_Frame_Body_Length() const
{
	return frame_body_length_;
}


//*****************************************************************************
bool Frame::Decode_Frame_Header()
{
	const unsigned short THREE_BYTES = 3;
	const unsigned short EIGHT_BYTES = 8;
	const unsigned short FRAME_LENGTH_OFFSET = 1;
	int frame_length_1 = 0;
	int frame_length_2 = 0;
	unsigned int frame_length = 0;
	unsigned char temporary_buffer[THREE_BYTES];
	char header_buffer[EIGHT_BYTES];

	memcpy(temporary_buffer, data_buffer_ + FRAME_LENGTH_OFFSET, THREE_BYTES);

	frame_length_1 = static_cast<int>(temporary_buffer[0]);
	frame_length_2 = static_cast<int>(temporary_buffer[1]);
	frame_type_ = static_cast<int>(temporary_buffer[2]);

	sprintf(header_buffer, "%02X%02X%02X", frame_length_1,
		frame_length_2, frame_type_);
	sscanf(header_buffer, "%04X%02X", &frame_length, &frame_type_);
	frame_body_length_ = frame_length;

	if (frame_body_length_ > MAX_FRAME_BODY_LENGTH)
		frame_body_length_ = 0; /* The message header is corrupted. Ignore the total frame */

	return true;
}


//*****************************************************************************
std::size_t Frame::Get_Frame_Type() const
{
	return frame_type_;
}


//*****************************************************************************
int Frame::Get_Start_Delimiter_Position()
{
	const unsigned char START_DELIMITER = static_cast<unsigned char>(0x7E);

	for (int i = 0; i < FRAME_HEADER_LENGTH; i++)
	{
		if (START_DELIMITER == data_buffer_[i])
		{
			start_delimiter_position_ = i;
			return i;
		}
	}

	return -1;
}


//*****************************************************************************
void Frame::Rearrange_Corrupted_Header()
{
	char temporary_char;
	
	for (unsigned short i = 0; i  < start_delimiter_position_; i++)
	{
		temporary_char = data_buffer_[i];
		data_buffer_[i] = data_buffer_[start_delimiter_position_ + i];
		data_buffer_[start_delimiter_position_ + i] = temporary_char;
	}
}


<<<<<<< HEAD
=======
//*****************************************************************************
char Frame::Get_Message_Type()
{
	return data_buffer_[15];
}


>>>>>>> a16cf8b196cb6b63ef52ea26b8cb9a8e861d84d1
}


}

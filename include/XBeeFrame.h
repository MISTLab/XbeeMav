/* XBeeFrame.cpp -- XBee Frame class --                                      */
/* ------------------------------------------------------------------------- */
/* September 20, 2016 -- @Copyright Aymen Soussia. All rights reserved.      */
/*                                  (aymen.soussia@gmail.com)                */


#pragma once

#include<cstdio>
#include<cstdlib>
#include<cstring>


namespace Mist
{


namespace Xbee
{


//*****************************************************************************
class Frame
{
public:
	Frame();
	~Frame();

	enum {FRAME_HEADER_LENGTH = 4};
	enum {MAX_FRAME_BODY_LENGTH = 280}; // TO DO check value

	const char* Get_Frame_Data() const;
	char* Get_Frame_Data();
	std::size_t Get_Frame_Length() const;
	const char* Get_Frame_Body() const;
	char* Get_Frame_Body();
	std::size_t Get_Frame_Body_Length() const;
	bool Decode_Frame_Header();
	std::size_t Get_Frame_Type() const;
	int Get_Start_Delimiter_Position();
	void Rearrange_Corrupted_Header();


private:

	char data_buffer_[FRAME_HEADER_LENGTH + MAX_FRAME_BODY_LENGTH];
	std::size_t frame_body_length_;
	unsigned int frame_type_;
	int start_delimiter_position_;
};


}


}

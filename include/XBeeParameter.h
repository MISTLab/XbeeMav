/* XBee_Parameter.h -- Xbee command struct  --                               */
/* ------------------------------------------------------------------------- */
/* November 30, 2016 -- @Copyright Aymen Soussia. All rights reserved.       */
/*                                  (aymen.soussia@gmail.com)                */


#pragma once

#include<string>


//*****************************************************************************
struct XBee_Parameter_S
{
	std::string command_;
	std::string value_;
};

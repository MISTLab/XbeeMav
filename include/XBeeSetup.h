/* ----------------------------------------------------------------
 * File: XBeeSetup.cpp
 * Date: 02/06/2017
 * Author: Pierre-Yves Breches
 * Description: Implementation of the xbeeSetup function to
 * configure the xbee module
 * Copyright Humanitas Solutions. All rights reserved.
 ------------------------------------------------------------------ */
#include<thread>

#include"XBeeModule.h"
#include"XMLConfigParser.h"

bool setupXBee(const std::string& device_port, const unsigned int baud_rate);

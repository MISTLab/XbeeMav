/* XMLConfigParser.h -- XML Config Parser class  --                          */
/* ------------------------------------------------------------------------- */
/* November 30, 2016 -- @Copyright Aymen Soussia. All rights reserved.       */
/*                                  (aymen.soussia@gmail.com)                */


#pragma once

#include<iostream>

#include<boost/property_tree/ptree.hpp>
#include<boost/property_tree/xml_parser.hpp>
#include<boost/foreach.hpp>
#include<boost/filesystem.hpp>

#include"XBeeParameter.h"


//*****************************************************************************
using boost::property_tree::ptree;


//*****************************************************************************
class XMLConfigParser
{
public:
	XMLConfigParser();
	~XMLConfigParser();

	bool Load_Config();
	std::vector<XBee_Parameter_S>* Get_Loaded_Parameters();
	bool Is_Config_Loaded_Successfully();

private:

	bool Check_Config_File_Exists(const std::string& file_name);

	std::vector<XBee_Parameter_S> xbee_parameters_;
	bool config_loaded_successfully_;
};


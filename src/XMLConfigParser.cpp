/* XMLConfigParser.cpp -- XML Config Parser class  --                        */
/* ------------------------------------------------------------------------- */
/* November 30, 2016 -- @Copyright Aymen Soussia. All rights reserved.       */
/*                                  (aymen.soussia@gmail.com)                */


#include "XMLConfigParser.h"



//*****************************************************************************
XMLConfigParser::XMLConfigParser():
	config_loaded_successfully_(false)
{
}


//*****************************************************************************
XMLConfigParser::~XMLConfigParser()
{
}


//*****************************************************************************
bool XMLConfigParser::Load_Config()
{
	const std::string FILE_NAME = XBEE_CONFIG_PATH;

	if (Check_Config_File_Exists(FILE_NAME))
	{
		ptree pt;
		boost::property_tree::read_xml(FILE_NAME, pt);

		config_loaded_successfully_ = true;

		BOOST_FOREACH(ptree::value_type const&v, pt.get_child("XBeeConfig.Settings"))
		{
			if (v.first == "Parameter")
			{
				XBee_Parameter_S new_xbee_parameter;
				new_xbee_parameter.command_ = v.second.get<std::string>("<xmlattr>.Command");
				new_xbee_parameter.value_ = v.second.data();
				xbee_parameters_.push_back(new_xbee_parameter);
			}
		}

		return true;
	}
	else
	{
		std::cout << "Error: Config File Not Found." << std::endl;
		return false;
	}
}


//*****************************************************************************
std::vector<XBee_Parameter_S>* XMLConfigParser::Get_Loaded_Parameters()
{
	return &xbee_parameters_;
}


//*****************************************************************************
bool XMLConfigParser::Check_Config_File_Exists(const std::string& file_name)
{
	return boost::filesystem::exists(file_name);
}


bool XMLConfigParser::Is_Config_Loaded_Successfully()
{
	return config_loaded_successfully_;
}

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

#include <iostream>
#include <sstream>
#include <fstream>


// Global variables
std::vector<double> currentArmState;
std::vector<double> currentCamState;

// Callbacks
void armStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	if ( currentArmState.size() != msg->position.size())
	{
		currentArmState.clear();
		currentArmState.resize(msg->position.size());
	}

	for ( size_t i=0; i<currentArmState.size(); ++i )
		currentArmState[i] = msg->position[i];
}

void cameraStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	if ( currentCamState.size() != msg->position.size())
	{
		currentCamState.clear();
		currentCamState.resize(msg->position.size());
	}

	for ( size_t i=0; i<currentCamState.size(); ++i )
		currentCamState[i] = msg->position[i];
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "raw_saver");

	ros::NodeHandle n;
	ros::Subscriber arm_state = n.subscribe<sensor_msgs::JointState>("/arm/joint_states", 1, armStateCallback);
	ros::Subscriber cam_state = n.subscribe<sensor_msgs::JointState>("/torso/joint_states", 1, cameraStateCallback);

	std::string storagePath = "raw_jointstate_saver/Output";
	std::string path_file = storagePath + "/RawJointStates.txt";

	std::stringstream str("mkdir -p " + storagePath);
	int result = system(str.str().c_str());

	if ( result != 0 )
	{
		std::cout << "Error, could not create storage path!" << std::endl;
		return -1;
	}

	while (ros::ok())
	{
		char c = '0';

		std::cout << "Press 'e' to exit, any other key will append the current state to the text file." << std::endl;
		std::cin >> c;
		std::cin.ignore();
		std::cout << std::endl;

		if ( c == 'e' ) // Exit program
			break;

		ros::spinOnce(); // Get new data from callbacks

		// Append current states to text file
		std::fstream file_output;
		std::string filecontent = "";
		bool bWriteToFile = false;

		// First read current data from file
		file_output.open(path_file.c_str(), std::ios::in );
		if ( file_output.is_open() )
		{
			while ( !file_output.eof() )
			{
				std::string line;
				std::getline(file_output,line);
				filecontent += line;
				if ( !file_output.eof() )
					filecontent += "\n";
			}

			file_output.close(); // Data has been read, now close
		}
		else
		{
			std::cout << "Warning, cannot open file for reading! If file didn't exist, it will be created now." << std::endl;
		}

		bool bIsEmpty = filecontent.empty();

		// Clear text file and write whole data to it
		file_output.open(path_file.c_str(), std::ios::out | std::ios::trunc);
		if ( !file_output.is_open() )
		{
			std::cout << "Error, cannot open file! Printing all data to screen to prevent loosing it:" << std::endl;
			if ( !bIsEmpty )
				std::cout << filecontent << std::endl;
			continue;
		}

		if ( bIsEmpty )
			filecontent = "ArmJointStates: []\n\nCameraJointStates: []";

		if ( currentArmState.size() > 0 )
		{
			std::stringstream newData("");
			if ( !bIsEmpty )
				newData << ",\n";

			for ( size_t i=0; i<currentArmState.size(); ++i )
				newData << currentArmState[i] << (i == currentArmState.size()-1 ? "]" : ", ");

			size_t idx = filecontent.find("]");

			if ( idx != std::string::npos )
			{
				filecontent.replace(idx,1,newData.str());
				bWriteToFile = true;
			}
			else
			{
				std::cout << "Output file is corrupted, please delete it!" << std::endl;
				return -1;
			}
		}
		if ( currentCamState.size() > 0 )
		{
			std::stringstream newData("");
			if ( !bIsEmpty )
				newData << ",\n";

			for ( size_t i=0; i<currentCamState.size(); ++i )
				newData << currentCamState[i] << (i == currentCamState.size()-1 ? "]" : ", ");

			size_t idx = filecontent.rfind("]");

			if ( idx != std::string::npos )
			{
				filecontent.replace(idx,1,newData.str());
				bWriteToFile = true;
			}
			else
			{
				std::cout << "Output file is corrupted, please delete it!" << std::endl;
				return -1;
			}
		}

		if ( bWriteToFile )
			file_output << filecontent;

		file_output.close();

		std::cout << "arm: [";
		for (size_t i=0; i<currentArmState.size(); ++i)
			std::cout << currentArmState[i] << (i==currentArmState.size()-1 ? "]\n" : ", ");
		std::cout << "torso: [";
			for (size_t i=0; i<currentCamState.size(); ++i)
				std::cout << currentCamState[i] << (i==currentCamState.size()-1 ? "]\n" : ", ");
		std::cout << std::endl;
	}

	return 0;
}




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
	std::vector< std::vector<double> > armStatesSaved;
	std::vector< std::vector<double> > camStatesSaved;

	std::stringstream str("mkdir -p " + storagePath);
	system(str.str().c_str());

	while (ros::ok())
	{
		char c = '0';

		std::cout << "Press 'e' to exit and save, any other input will store the current state." << std::endl;
		std::cin >> c;
		std::cout << std::endl;

		if ( c == 'e' ) // Save storage array to txt file and exit
		{
			std::fstream file_output;
			std::string path_file = storagePath + "/RawJointStates.txt";
			std::stringstream data;

			data << "ArmJointStates: [";
			for ( size_t i = 0; i<armStatesSaved.size(); ++i )
			{
				for ( size_t j = 0; j<armStatesSaved[i].size(); ++j )
				{
					data << armStatesSaved[i][j];

					if ( j < armStatesSaved[i].size()-1 )
						data << ", ";
					else if ( i < armStatesSaved.size()-1 ) // Don't add a comma on very last element
						data << ",\n";
				}
			}
			data << "]\n\nCameraJointStates: [";
			for ( size_t i = 0; i<camStatesSaved.size(); ++i )
			{
				for ( size_t j = 0; j<camStatesSaved[i].size(); ++j )
				{
					data << camStatesSaved[i][j];

					if ( j < camStatesSaved[i].size()-1 )
						data << ", ";
					else if ( i < camStatesSaved.size()-1 ) // Don't add a comma on very last element
						data << ",\n";
				}
			}
			data << "]\n\n";

			file_output.open(path_file.c_str(), std::ios::out);
			if ( file_output.is_open() )
				file_output << data.str();
			file_output.close();

			break;
		}

		ros::spinOnce();

		// Append current states to storage vector
		if ( currentArmState.size() > 0 )
			armStatesSaved.push_back(currentArmState);
		if ( currentCamState.size() > 0 )
			camStatesSaved.push_back(currentCamState);

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




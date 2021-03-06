/****************************************************************
 *
 * Copyright (c) 2015
 *
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: squirrel
 * ROS stack name: squirrel_calibration
 * ROS package name: robotino_calibration
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Marc Riedlinger, email:marc.riedlinger@ipa.fraunhofer.de
 *
 * Date of creation: October 2016
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/


#include <robotino_calibration/robot_calibration.h>
#include <robotino_calibration/transformation_utilities.h>

//Exception
#include <exception>

#include <sstream>
#include <robotino_calibration/time_utilities.h>


// ToDo: Create custom exception classes for exception handling
// ToDo: Save snapshots one by one instead of just once at the end -> ability to restore current routine after crash
// ToDo: Rename package to libextrinsic_calibration, also in CMakeList (but without lib tag there)


RobotCalibration::RobotCalibration(ros::NodeHandle nh, CalibrationInterface* interface, const bool load_data_from_disk) :
	node_handle_(nh), transform_listener_(nh), calibrated_(false), calibration_interface_(interface), load_data_from_disk_(load_data_from_disk), transform_discard_timeout_(1.0)
{
	// load parameters
	std::cout << std::endl << "========== RobotCalibration Parameters ==========" << std::endl;

	if (calibration_interface_ == 0) // Throw exception, as we need an calibration interface in order to function properly!
	{
		ROS_FATAL("RobotCalibration::RobotCalibration - Calibration interface has not been created.");
		throw std::exception();
	}

	node_handle_.param("optimization_iterations", optimization_iterations_, 1000);
	if ( optimization_iterations_ <= 0 )
	{
		std::cout << "Invalid optimization_iterations value: " << optimization_iterations_ << " -> Setting value to 1000." << std::endl;
		optimization_iterations_ = 1000;
	}
	std::cout << "optimization_iterations: " << optimization_iterations_ << std::endl;

	node_handle_.param<std::string>("calibration_storage_path", calibration_storage_path_, "/calibration");
	std::cout << "calibration_storage_path: " << calibration_storage_path_ << std::endl;

	// setup file name for storing calibration data that can be used for offline calibration
	calib_data_file_name_ = calibration_interface_->getFileName("offline_data", false);
	if ( calib_data_file_name_.empty() )
		calib_data_file_name_ = "offline_data";
	calib_data_file_name_ += ".txt";

	// create folders that will contain calibration result and snapshot files for offline calibration
	file_utilities::createStorageFolder(calibration_storage_path_);
	calib_data_folder_ = "/calibration_data";
	file_utilities::createStorageFolder((calibration_storage_path_+calib_data_folder_));

	if ( !load_data_from_disk_ )
	{
		node_handle_.param("transform_discard_timeout", transform_discard_timeout_, 1.0);
		transform_discard_timeout_ = fmax(transform_discard_timeout_, 0.1);
		std::cout << "transform_discard_timeout: " << transform_discard_timeout_ << std::endl;

		// hack to fix tf::waitForTransform throwing error that transforms do not exist when now() == 0 at startup
		ROS_INFO("RobotCalibration::RobotCalibration - Waiting for TF listener to initialize...");
		const double start_time = time_utilities::getSystemTimeSec();
		while ( time_utilities::getTimeElapsedSec(start_time) < 10.f )
		{
			if ( ros::Time::now().isValid() )
				break;

			ros::Duration(0.2f).sleep();
		}
		ROS_INFO("RobotCalibration::RobotCalibration - End waiting for TF to initialize.");

		// load gaps including its initial values
		std::vector<std::string> uncertainties_list;
		calibration_interface_->getUncertainties(uncertainties_list);

		if ( uncertainties_list.empty() )
		{
			ROS_WARN("RobotCalibration::RobotCalibration - Uncertainties list is empty... nothing to do.");
			return;
		}

		if ( uncertainties_list.size() % 6 != 0 )
			ROS_WARN("RobotCalibration::RobotCalibration - Size of uncertainsties_list is not a factor of 6: [parent frame, child frame, last parent-branch frame, last child-branch frame, parent marker, child marker]");

		// create calibration setups, check for errors
		for ( int i=0; i<uncertainties_list.size(); i+=6 )
		{
			std::string parent = uncertainties_list[i];  // parent frame of uncertainty
			std::string child = uncertainties_list[i+1];  // child frame of uncertainty
			std::string last_parent_branch_frame = uncertainties_list[i+2];
			std::string last_child_branch_frame = uncertainties_list[i+3];

			bool success = transform_listener_.waitForTransform(parent, child, ros::Time(0), ros::Duration(1.0));
			if ( !success )
			{
				ROS_ERROR("RobotCalibration::RobotCalibration - Transform from parent frame %s to child frame %s does not exist, skipping.", parent.c_str(), child.c_str());
				continue;
			}

			success = transform_listener_.waitForTransform(parent, last_parent_branch_frame, ros::Time(0), ros::Duration(1.0));
			if ( !success )
			{
				ROS_ERROR("RobotCalibration::RobotCalibration - Transform from parent frame %s to last parent-branch frame %s does not exist, skipping.", parent.c_str(), last_parent_branch_frame.c_str());
				continue;
			}

			success = transform_listener_.waitForTransform(child, last_child_branch_frame, ros::Time(0), ros::Duration(1.0));
			if ( !success )
			{
				ROS_ERROR("RobotCalibration::RobotCalibration - Transform from child frame %s to last child-branch frame %s does not exist, skipping.", child.c_str(), last_child_branch_frame.c_str());
				continue;
			}

			std::string actual_parent = "";
			transform_listener_.getParent(child, ros::Time(0), actual_parent);
			if ( actual_parent.compare(parent) == 0 )  // compare actual parent of child with parent user has input
			{
				// get origin frame of parent and child branch chain
				// uncertainties that have "similar" parent and child branches will be merged into one calibration setup
				std::string origin = "";
				if ( getOrigin(last_parent_branch_frame, last_child_branch_frame, origin) )
				{
					std::string parent_marker = uncertainties_list[i+4];
					std::string child_marker = uncertainties_list[i+5];

					bool found = false;
					for ( int j=0; j<calibration_setups_.size(); ++j )  // merge entries that have the origin
					{
						// check if uncertainty lies on parent or child branch of any existing setup, if so, merge, otherwise create a new calibration setup
						if ( isPartOfCalibrationSetup(parent, child, origin, calibration_setups_[j]) )
						{
							feedCalibrationSetup(calibration_setups_[j], parent, child, parent_marker, child_marker);  // merge
							found = true;
							break;
						}
					}

					if ( !found )  // no match has been found -> create new calibration setup if possible
					{
						std::vector<std::string> parent_branch_reversed;
						std::vector<std::string> child_branch_reversed;

						bool success = getBackChain(last_parent_branch_frame, origin, parent_branch_reversed);
						success &= getBackChain(last_child_branch_frame, origin, child_branch_reversed);

						if ( !success )
						{
							ROS_ERROR("RobotCalibration::RobotCalibration - Could not build parent- or child-branch, skipping uncertainty from %s to %s.", parent.c_str(), child.c_str());
							continue;
						}

						CalibrationSetup setup;
						setup.origin_ = origin;
						getForwardChain(parent_branch_reversed, setup.parent_branch_);
						getForwardChain(child_branch_reversed, setup.child_branch_);

						if ( setup.parent_branch_.size() == 0 || setup.child_branch_.size() == 0 )
							ROS_WARN("RobotCalibration::RobotCalibration - Parent- or child-branch is empty. Proceeding...");

						feedCalibrationSetup(setup, parent, child, parent_marker, child_marker);
						calibration_setups_.push_back(setup);
					}
				}
				else
					ROS_WARN("RobotCalibration::RobotCalibration - No mutual origin found, skipping uncertainty from %s to %s.", parent.c_str(), child.c_str());
			}
			else
				ROS_WARN("RobotCalibration::RobotCalibration - Given parent frame does not match actual parent (%s) in tf tree. Given parent: %s, given child: %s", actual_parent.c_str(), uncertainties_list[i].c_str(), child.c_str());
		}

		// remove all corrupted entries in calibration setups vector
		for ( int i=0; i<calibration_setups_.size(); ++i )
		{
			if ( calibration_setups_[i].uncertainties_list_.size() == 0 )  // remove corrupted setups
			{
				ROS_WARN("RobotCalibration::RobotCalibration - No transform to calibrate found in calibration setup -> removing setup.");
				calibration_setups_.erase(calibration_setups_.begin()+i);
				--i;
				continue;
			}

			for ( int j=0; j<calibration_setups_[i].uncertainties_list_.size(); ++j )  // check for corrupted transforms
			{
				if ( calibration_setups_[i].uncertainties_list_[j].parent_markers_.size() == 0 ||
						calibration_setups_[i].uncertainties_list_[j].child_markers_.size() == 0 )
				{
					ROS_WARN("RobotCalibration::RobotCalibration - Empty parent or child_markers for %s to %s -> removing transform", calibration_setups_[i].uncertainties_list_[j].parent_.c_str(), calibration_setups_[i].uncertainties_list_[j].child_.c_str());
					calibration_setups_[i].uncertainties_list_.erase(calibration_setups_[i].uncertainties_list_.begin()+j);
					--j;
				}
			}

			if ( calibration_setups_[i].uncertainties_list_.size() == 0 )  // check again
			{
				ROS_WARN("RobotCalibration::RobotCalibration - No transform to calibrate found in calibration setup -> removing setup.");
				calibration_setups_.erase(calibration_setups_.begin()+i);
				--i;
			}
		}

		// organize/sort calibration data
		// the uncertainties_list_ vector for each setup is not useful for snapshotting tf data, as it is unsorted. Sort it now to make the process feasible.
		for ( int i=0; i<calibration_setups_.size(); ++i )
		{
			// find natural order (as they appear in tf tree) of uncertainties within the parent branch or child branch
			std::vector<CalibrationInfo> parent_branch_uncertainties;  // parent branch uncertainties ordered in the way they appear in tf tree
			std::vector<CalibrationInfo> child_branch_uncertainties;  // same for child branch uncertainties

			bool parent_branch = true;
			sortUncertainties(parent_branch, calibration_setups_[i], parent_branch_uncertainties);
			sortUncertainties(!parent_branch, calibration_setups_[i], child_branch_uncertainties);

			// truncate parent- and child-branch so that they end at their last uncertainty, the rest can be retrieved via tf
			truncateBranch(calibration_setups_[i].parent_branch_, parent_branch_uncertainties);
			truncateBranch(calibration_setups_[i].child_branch_, child_branch_uncertainties);
		}
	}
	else
	{
		std::cout << std::endl << "Loading calibration setups from disk..." << std::endl << std::endl;
		bool result = file_utilities::loadCalibrationSetups(calibration_setups_, (calibration_storage_path_+calib_data_folder_), calib_data_file_name_);
		if ( !result )
		{
			ROS_ERROR("RobotCalibration::RobotCalibration - Can't load calibration setups from disk: Exiting.");
			throw std::exception();
		}
	}

	if ( calibration_setups_.size() == 0 )
	{
		ROS_WARN("RobotCalibration::RobotCalibration - No calibration setup entry found: Exiting.");
		throw std::exception();
	}

	std::cout << "calibration setups generated: " << calibration_setups_.size() << std::endl;

	for ( int i=0; i<calibration_setups_.size(); ++i )
	{
		std::cout << std::endl << "Calibration Setup " << (i+1) << ":" << std::endl;
		std::cout << "\tOrigin:\t" << calibration_setups_[i].origin_ << std::endl;

		std::string branch = "";
		for ( int j=0; j<calibration_setups_[i].parent_branch_.size(); ++j )  // parent branch
		{
			if ( j < calibration_setups_[i].parent_branch_.size()-1 )
				branch += (calibration_setups_[i].parent_branch_[j] + ";");
			else
				branch += calibration_setups_[i].parent_branch_[j];
		}
		std::cout << "\tParent branch:\t" << branch << std::endl;
		branch = "";
		for ( int j=0; j<calibration_setups_[i].child_branch_.size(); ++j )  // child branch
		{
			if ( j < calibration_setups_[i].child_branch_.size()-1 )
				branch += (calibration_setups_[i].child_branch_[j] + ";");
			else
				branch += calibration_setups_[i].child_branch_[j];
		}
		std::cout << "\tChild branch:\t" << branch << std::endl;

		for ( int j=0; j<calibration_setups_[i].uncertainties_list_.size(); ++j )
		{
			const CalibrationInfo &uncertainty = calibration_setups_[i].uncertainties_list_[j];
			std::cout << "\tUncertainty " << (j+1) << ":" << std::endl;
			std::cout << "\t\tFrom <" << uncertainty.parent_ << "> to <" << uncertainty.child_ << ">" << std::endl;

			std::string parent_markers = "";
			std::string child_markers = "";
			for ( int l=0; l<uncertainty.parent_markers_.size(); ++l )  // parent markers and child markers have same length
			{
				if ( l < uncertainty.parent_markers_.size()-1 )
				{
					parent_markers += (uncertainty.parent_markers_[l] + ";");
					child_markers += (uncertainty.child_markers_[l] + ";");
				}
				else
				{
					parent_markers += uncertainty.parent_markers_[l];
					child_markers += uncertainty.child_markers_[l];
				}
			}
			std::cout << "\t\tParent markers:\t" << parent_markers << std::endl;
			std::cout << "\t\tChild markers:\t" << child_markers << std::endl;
			std::cout << "\t\tParent branch uncertainty: " << (uncertainty.parent_branch_uncertainty_ ? "True" : "False") << std::endl;
			std::stringstream init_trafo("");
			init_trafo << uncertainty.current_trafo_;
			std::string line = "";
			std::string trafo = "";
			while ( std::getline(init_trafo, line) )
			{
				trafo += (line + "\n\t\t\t\t\t");
			}
			trafo = trafo.substr(0, trafo.length()-6);  // remove newline and tabs for last line
			std::cout << "\t\tInitial transform:\t" << trafo << std::endl;
		}

	}
	std::cout << std::endl;
}

RobotCalibration::~RobotCalibration()
{
	if ( calibration_interface_ != 0 )
		delete calibration_interface_;
}

bool RobotCalibration::getOrigin(const std::string last_parent_branch_frame, const std::string last_child_branch_frame, std::string &origin)
{
	if ( last_parent_branch_frame.empty() || last_child_branch_frame.empty() )
	{
		ROS_ERROR("RobotCalibration::getOrigin - Invalid last parent-branch frame or last child-branch frame!");
		return false;
	}

	std::vector<std::string> parent_frames;  // from last_parent_branch_frame to last available frame
	std::string frame = last_parent_branch_frame;

	do
	{
		parent_frames.push_back(frame);  // populate backwards chain
	}
	while ( transform_listener_.getParent(frame, ros::Time(0), frame) );

	// check for occurrence of first frame of child-branch in parent-branch
	frame = last_child_branch_frame;
	do
	{
		for ( int i=0; i<parent_frames.size(); ++i )
		{
			if ( frame.compare(parent_frames[i]) == 0 )  // find frame where parent branch and child branch meets
			{
				origin = frame;
				return true;
			}
		}
	}
	while ( transform_listener_.getParent(frame, ros::Time(0), frame) );  // go back from last_child_branch_frame on and see where it meets with parent_frames

	ROS_WARN("RobotCalibration::getOrigin - No mutual origin found between %s and %s.", last_parent_branch_frame.c_str(), last_child_branch_frame.c_str());
	return false;
}

bool RobotCalibration::isPartOfCalibrationSetup(const std::string parent, const std::string child, const std::string origin, const CalibrationSetup &setup)
{
	if ( origin.compare(setup.origin_) == 0 )
	{
		for ( int i=0; i<setup.parent_branch_.size()-1; i++ )
		{
			if ( parent.compare(setup.parent_branch_[i]) == 0 && child.compare(setup.parent_branch_[i+1]) == 0 )
			{
				return true;
			}
		}

		for ( int i=0; i<setup.child_branch_.size()-1; i++ )
		{
			if ( parent.compare(setup.child_branch_[i]) == 0 && child.compare(setup.child_branch_[i+1]) == 0 )
			{
				return true;
			}
		}
	}

	return false;
}

void RobotCalibration::feedCalibrationSetup(CalibrationSetup &setup, const std::string parent, const std::string child,
		const std::string parent_marker, const std::string child_marker)
{
	// first check if transform already exists, if so extend it
	for ( int i=0; i<setup.uncertainties_list_.size(); ++i )
	{
		if ( setup.uncertainties_list_[i].parent_.compare(parent) == 0 &&
				setup.uncertainties_list_[i].child_.compare(child) == 0 )  // transform already exists, check if we have to extend its marker frames
		{
			for ( int j=0; j<setup.uncertainties_list_[i].parent_markers_.size(); ++j )  // parent_markers and child_markers always have same size
			{
				if ( setup.uncertainties_list_[i].parent_markers_[j].compare(parent_marker) == 0 &&
						setup.uncertainties_list_[i].child_markers_[j].compare(child_marker) == 0 )  // marker frames already exist -> nothing to do here anymore
				{
					ROS_WARN("RobotCalibration::feedCalibrationSetup - There are redundant entries within the uncertainties list." );
					return;
				}
			}

			// add missing marker frames
			setup.uncertainties_list_[i].parent_markers_.push_back(parent_marker);
			setup.uncertainties_list_[i].child_markers_.push_back(child_marker);
			return;
		}
	}

	// add new transform to be calibrated
	CalibrationInfo info;
	bool success = transform_utilities::getTransform(transform_listener_, parent, child, info.current_trafo_);  // init uncertain trafo with what's in tf

	if ( success )
	{
		info.parent_ = parent;
		info.child_ = child;
		info.calibrated_ = false;
		info.parent_markers_.push_back(parent_marker);
		info.child_markers_.push_back(child_marker);
		setup.uncertainties_list_.push_back(info);
	}
	else
		ROS_WARN("RobotCalibration::feedCalibrationSetup - Unable to get transform from %s to %s, skipping uncertainty.", parent.c_str(), child.c_str());
}

bool RobotCalibration::getBackChain(const std::string frame_start, const std::string frame_end, std::vector<std::string> &backchain)
{
	if ( frame_end.compare(frame_start) != 0 )  // do not execute if end frame and start frame are equal
	{
		std::string frame = frame_start;

		do
		{
			backchain.push_back(frame);  // populate backwards chain

			if ( !transform_listener_.getParent(frame, ros::Time(0), frame) )
			{
				ROS_WARN("RobotCalibration::getBackChain - Could not create back chain for frame %s, no parent frame for %s!", frame_start.c_str(), frame.c_str());
				backchain.clear();
				return false;
			}
		}
		while ( frame.compare(frame_end) != 0 );

	}

	backchain.push_back(frame_end);  // add origin as last frame to parent branch
	return true;
}

void RobotCalibration::getForwardChain(const std::vector<std::string> &backchain, std::vector<std::string> &forwardchain)
{
	if ( !forwardchain.empty() )
		forwardchain.clear();

	forwardchain.reserve(backchain.size());
	for ( int i=backchain.size()-1; i>=0; --i )
		forwardchain.push_back(backchain[i]);
}

void RobotCalibration::sortUncertainties(const bool parent_branch, CalibrationSetup &setup, std::vector<CalibrationInfo> &sorted_uncertainties)
{
	std::vector<std::string> &branch = (parent_branch ? setup.parent_branch_ : setup.child_branch_);

	for ( int i=0; i<branch.size()-1; ++i )
	{
		for ( int k=0; k<setup.uncertainties_list_.size(); ++k )
		{
			if ( branch[i].compare(setup.uncertainties_list_[k].parent_) == 0 &&
					branch[i+1].compare(setup.uncertainties_list_[k].child_) == 0 )
			{
				CalibrationInfo &info = setup.uncertainties_list_[k];

				if ( parent_branch )
				{
					info.parent_branch_uncertainty_ = true;
					sorted_uncertainties.push_back(info);  // now put the transforms in order, so that they reflect the order in which they appear in tf
				}
				else
				{
					info.parent_branch_uncertainty_ = false;
					sorted_uncertainties.push_back(info);
				}
			}
		}
	}
}

void RobotCalibration::truncateBranch(std::vector<std::string> &branch, std::vector<CalibrationInfo> &branch_uncertainties)
{
	if ( branch.size() == 0 )
		return;

	// take child frame of last uncertainty on respective branch or origin (and remove branch completely in this case)
	std::string last_uncertainty_child = (branch_uncertainties.size() > 0) ? branch_uncertainties[branch_uncertainties.size()-1].child_ : branch[0];

	for ( int i=0; i<branch.size()-1; ++i )
	{
		if ( branch[i].compare(last_uncertainty_child) == 0 )
		{
			branch.erase(branch.begin()+i+1, branch.end());  // erase everything that comes after last uncertainty child
			break;
		}
	}
}


bool RobotCalibration::startCalibration()
{
	if ( calibration_interface_ == 0 ) // Throw exception, as we need an calibration interface in order to function properly!
	{
		ROS_FATAL("RobotCalibration::startCalibration - Calibration interface has not been created!");
		throw std::exception();
	}

	if ( calibration_setups_.size() == 0 )
		return false;

	if ( !acquireTFData() )  // make snapshots of all relevant tf transforms for every robot configuration
		return false;

	// extrinsic calibration optimization
	for ( int l=0; l<calibration_setups_.size(); ++l )
	{
		int iterations = 0;

		// if there is only one trafo to calibrate, we don't need to optimize over iterations
		if ( calibration_setups_[l].uncertainties_list_.size() > 1 )
			iterations = optimization_iterations_;
		else
			iterations = 1;

		for (int i=0; i<iterations; ++i)
		{
			for ( int j=0; j<calibration_setups_[l].uncertainties_list_.size(); ++j )
			{
				if ( !ros::ok() || !extrinsicCalibration(l, j) )
				{
					ROS_ERROR("RobotCalibration::startCalibration - Calibration has failed!");
					return false;
				}
			}
		}
	}

	RobotCalibration::displayAndSaveCalibrationResult();
	calibrated_ = true;
	return true;
}

bool RobotCalibration::acquireTFData()
{
	if ( !load_data_from_disk_ )
	{
		const int num_configs = calibration_interface_->getConfigurationCount();
		for ( int config_counter = 0; config_counter < num_configs; ++config_counter )
		{
			if ( !ros::ok() )
				return false;

			std::cout << std::endl << "Configuration " << (config_counter+1) << "/" << num_configs << std::endl;

			// try to move robot
			try
			{
				if ( !calibration_interface_->moveRobot(config_counter) )
					continue;
			}
			catch( std::exception &ex )
			{
				return false;
			}

			// wait a moment here to mitigate shaking camera effects and give tf time to update
			ros::Duration(1).sleep();
			calibration_interface_->preSnapshot(config_counter);  // give user possibility to execute code before snapshots take place
			std::cout << "Populating snapshots..." << std::endl;

			// grab transforms for each setup and store them
			std::vector<TFSnapshot> snapshots;
			snapshots.reserve(calibration_setups_.size());
			bool skip_configuration = false;
			for ( int i=0; i<calibration_setups_.size(); ++i )
			{
				TFSnapshot snapshot;
				populateTFSnapshot(calibration_setups_[i], snapshot);

				if ( !ros::ok() || !snapshot.valid_ )
				{
					skip_configuration = true;
					break;
				}
				else
					snapshots.push_back(snapshot);
			}

			if ( !skip_configuration )
			{
				tf_snapshots_.push_back(snapshots);
			}
		}

		// save calibration setups and snapshots to disk for offline calibration
		std::cout << std::endl << "Saving offline data to disk..." << std::endl << std::endl;
		file_utilities::saveCalibrationSetups(calibration_setups_, (calibration_storage_path_+calib_data_folder_), calib_data_file_name_);
		file_utilities::saveSnapshots(tf_snapshots_, (calibration_storage_path_+calib_data_folder_), calib_data_file_name_);
	}
	else
	{
		std::cout << std::endl << "Loading snapshots from disk..." << std::endl << std::endl;
		bool result = file_utilities::loadSnapshots(tf_snapshots_, (calibration_storage_path_+calib_data_folder_), calib_data_file_name_);

		if ( !result )
			return false;
	}

	return true;
}

void RobotCalibration::populateTFSnapshot(const CalibrationSetup &setup, TFSnapshot &snapshot)
{
	// populate parent branch trafos
	snapshot.valid_ = false;
	for ( int i=0; i<setup.parent_branch_.size()-1; ++i )
	{
		cv::Mat trafo;
		if ( transform_utilities::getTransform(transform_listener_, setup.parent_branch_[i], setup.parent_branch_[i+1], trafo, transform_discard_timeout_, true) )
		{
			TFInfo info;
			info.parent_ = setup.parent_branch_[i];
			info.child_ = setup.parent_branch_[i+1];
			info.transform_ = trafo.clone();
			snapshot.parent_branch_.push_back(info);
		}
		else
		{
			ROS_ERROR("RobotCalibration::populateTFSnapshot - Could not build parent branch, skipping to snapshot current configuration!");
			return;
		}
	}

	// populate child branch trafos
	for ( int i=0; i<setup.child_branch_.size()-1; ++i )
	{
		cv::Mat trafo;
		if ( transform_utilities::getTransform(transform_listener_, setup.child_branch_[i], setup.child_branch_[i+1], trafo, transform_discard_timeout_, true) )
		{
			TFInfo info;
			info.parent_ = setup.child_branch_[i];
			info.child_ = setup.child_branch_[i+1];
			info.transform_ = trafo.clone();
			snapshot.child_branch_.push_back(info);
		}
		else
		{
			ROS_ERROR("RobotCalibration::populateTFSnapshot - Could not build parent branch, skipping to snapshot current configuration!");
			return;
		}
	}

	// populate trafos to parent and child markers for each uncertainty
	for ( int i=0; i<setup.uncertainties_list_.size(); ++i )
	{
		std::string last_branch_frame;  // child markers are always on the branch on which uncertainty is as well
		std::string last_otherbranch_frame;  // parent markers are always on other branch

		// depending on which branch uncertainty lies, end frames are different
		if ( setup.uncertainties_list_[i].parent_branch_uncertainty_ )
		{
			last_branch_frame = (setup.parent_branch_.size() > 0) ? setup.parent_branch_[setup.parent_branch_.size()-1] : setup.origin_;
			last_otherbranch_frame = (setup.child_branch_.size() > 0) ? setup.child_branch_[setup.child_branch_.size()-1] : setup.origin_;
		}
		else
		{
			last_branch_frame = (setup.child_branch_.size() > 0) ? setup.child_branch_[setup.child_branch_.size()-1] : setup.origin_;
			last_otherbranch_frame = (setup.parent_branch_.size() > 0) ? setup.parent_branch_[setup.parent_branch_.size()-1] : setup.origin_;
		}

		TFBranchEndsToMarkers branch_ends_to_markers;
		branch_ends_to_markers.corresponding_uncertainty_idx_ = i;
		std::vector<TFInfo> &to_parent_markers = branch_ends_to_markers.otherbranch_to_parent_markers_;
		for ( std::string parent_marker : setup.uncertainties_list_[i].parent_markers_ )
		{
			cv::Mat trafo;
			TFInfo info;
			if ( transform_utilities::getTransform(transform_listener_, last_otherbranch_frame, parent_marker, trafo, transform_discard_timeout_, false) )
			{
				info.parent_ = last_otherbranch_frame;
				info.child_ = parent_marker;
				info.transform_ = trafo.clone();
			}

			// Even store info if it is empty (e.g. due to timeout), so to_parent_markers size has the same as setup.parent_branch_uncertainties_[i].parent_markers_
			to_parent_markers.push_back(info);
		}

		std::vector<TFInfo> &to_child_markers = branch_ends_to_markers.branch_to_child_markers_;
		for ( std::string child_marker : setup.uncertainties_list_[i].child_markers_ )
		{
			cv::Mat trafo;
			TFInfo info;
			if ( transform_utilities::getTransform(transform_listener_, last_branch_frame, child_marker, trafo, transform_discard_timeout_, false) )
			{
				info.parent_ = last_branch_frame;
				info.child_ = child_marker;
				info.transform_ = trafo.clone();
			}

			// Also store empty info so that to_child_markers and to_parent_markers have same size, this way we can filter out bad entries easily later on
			to_child_markers.push_back(info);
		}

		if ( to_parent_markers.size() != to_child_markers.size() )
		{
			ROS_ERROR("RobotCalibration::populateTFSnapshot - to_parent_markers vector has not same size as to_child_markers vector, skipping to snapshot current configuration!");
			return;
		}

		// check for bad entries now
		for ( int j=0; j<to_parent_markers.size(); ++j )  // to_parent_markers and to_child_markers have same size
		{
			if ( to_parent_markers[j].transform_.empty() || to_child_markers[j].transform_.empty() )  // remove bad entries from both lists, so that both vectors always have same size
			{
				to_parent_markers.erase(to_parent_markers.begin()+j);
				to_child_markers.erase(to_child_markers.begin()+j);
				--j;
			}
		}

		if ( to_parent_markers.size() > 0 )  // if to_parent_markers == 0 then to_child_markers == 0
		{
			snapshot.branch_ends_to_markers_.push_back(branch_ends_to_markers);
			std::cout << "Markers found for <" << setup.uncertainties_list_[i].parent_ << "> to <" << setup.uncertainties_list_[i].child_ << ">: " << to_parent_markers.size() << std::endl;
		}
		else
		{
			ROS_WARN("No relevant markers have been found during current configuration.");  // return here, don't check other uncertainties (even tho we might get detections), so we avoid empty entries in our snapshot vector
			return;
		}
	}

	snapshot.valid_ = true;
}

void RobotCalibration::displayAndSaveCalibrationResult()
{
	// display and save calibration parameters
	std::string output_file_name = calibration_interface_->getFileName("result", true);

	if ( output_file_name.empty() )
		output_file_name = "cailbration_result.txt";

	// check if file extension exists
	if( output_file_name.rfind(".") == std::string::npos )
		output_file_name += ".txt";

	std::stringstream output;

	output << "----- Replace the follwing parameters within the urdf file of your robot -----" << std::endl;
	output << "Created at: " << time_utilities::getCurrentTimeStamp() << std::endl;
	for ( int i=0; i<calibration_setups_.size(); ++i )
	{
		for ( int j=0; j<calibration_setups_[i].uncertainties_list_.size(); ++j )
		{
			cv::Vec3d ypr = transform_utilities::YPRFromRotationMatrix(calibration_setups_[i].uncertainties_list_[j].current_trafo_);

			output << "<!-- " << calibration_setups_[i].uncertainties_list_[j].child_ << " mount positions | camera_base_calibration | relative to " << calibration_setups_[i].uncertainties_list_[j].parent_ << "-->" << std::endl
				   << "  <property name=\"" << calibration_setups_[i].uncertainties_list_[j].child_ << "_x\" value=\"" << calibration_setups_[i].uncertainties_list_[j].current_trafo_.at<double>(0,3) << "\"/>" << std::endl
				   << "  <property name=\"" << calibration_setups_[i].uncertainties_list_[j].child_ << "_y\" value=\"" << calibration_setups_[i].uncertainties_list_[j].current_trafo_.at<double>(1,3) << "\"/>" << std::endl
				   << "  <property name=\"" << calibration_setups_[i].uncertainties_list_[j].child_ << "_z\" value=\"" << calibration_setups_[i].uncertainties_list_[j].current_trafo_.at<double>(2,3) << "\"/>" << std::endl
				   << "  <property name=\"" << calibration_setups_[i].uncertainties_list_[j].child_ << "_roll\" value=\"" << ypr.val[2] << "\"/>" << std::endl
				   << "  <property name=\"" << calibration_setups_[i].uncertainties_list_[j].child_ << "_pitch\" value=\"" << ypr.val[1] << "\"/>" << std::endl
				   << "  <property name=\"" << calibration_setups_[i].uncertainties_list_[j].child_ << "_yaw\" value=\"" << ypr.val[0] << "\"/>" << std::endl
				   << std::endl << std::endl;
		}
	}

	std::cout << std::endl << std::endl << output.str();

	if ( ros::ok() )  // if program has been killed, do not save results
		file_utilities::saveCalibrationResult((calibration_storage_path_+"/"+output_file_name), output.str());  // save results to disk
	else
		ROS_WARN("RobotCalibration::displayAndSaveCalibrationResult - Not saving calibration results.");
}

bool RobotCalibration::extrinsicCalibration(const int current_setup_idx, const int current_uncertainty_idx)
{
	CalibrationSetup &setup = calibration_setups_[current_setup_idx];
	CalibrationInfo &current_uncertainty = setup.uncertainties_list_[current_uncertainty_idx];
	bool on_parent_branch = current_uncertainty.parent_branch_uncertainty_;

	std::vector<cv::Point3d> points_3d_uncertainty_parent;  // parent marker points in uncertainty parent frame
	std::vector<cv::Point3d> points_3d_uncertainty_child;  // child marker points in uncertainty child frame

	for ( int i=0; i<tf_snapshots_.size(); ++i )  // go through all snapshots for this setup
	{
		std::vector<TFInfo>* branch_tmp;  			// either parent or child_branch
		std::vector<TFInfo>* other_branch_tmp;		// if branch = child_branch then this is parent_branch and vice versa
		TFSnapshot &snapshot = tf_snapshots_[i][current_setup_idx];

		if ( &snapshot == 0 )  // should never happen that a snapshot is empty, but just in case
			continue;

		if ( on_parent_branch )
		{
			branch_tmp = &snapshot.parent_branch_;
			other_branch_tmp = &snapshot.child_branch_;
		}
		else
		{
			branch_tmp = &snapshot.child_branch_;
			other_branch_tmp = &snapshot.parent_branch_;
		}

		// now with references
		std::vector<TFInfo> &branch = *branch_tmp;
		std::vector<TFInfo> &other_branch = *other_branch_tmp;

		bool parent_markers = true;
		std::vector<TFInfo>* otherbranch_last_to_parent_markers_tmp = getBranchEndToMarkers(current_uncertainty_idx, parent_markers, snapshot);
		std::vector<TFInfo>* branch_last_to_child_markers_tmp = getBranchEndToMarkers(current_uncertainty_idx, !parent_markers, snapshot);

		if ( branch_last_to_child_markers_tmp == 0 || otherbranch_last_to_parent_markers_tmp == 0 )
		{
			ROS_WARN("RobotCalibration::extrinsicCalibration - Child or parent markers vector cannot be found in current snapshot %d, skipping", i);
			continue;
		}

		std::vector<TFInfo> &branch_last_to_child_markers = *branch_last_to_child_markers_tmp;  // vector that contains all trafos from last branch frame to child markers
		std::vector<TFInfo> &otherbranch_last_to_parent_markers = *otherbranch_last_to_parent_markers_tmp; // vector that contains all trafos from last otherbranch frame to parent markers

		if ( branch_last_to_child_markers.size() == 0 || otherbranch_last_to_parent_markers.size() == 0 )
		{
			ROS_WARN("RobotCalibration::extrinsicCalibration - Child or parent markers vector is empty in snapshot %d, skipping.", i);
			continue;
		}

		if ( branch_last_to_child_markers.size() != otherbranch_last_to_parent_markers.size() )
		{
			ROS_WARN("RobotCalibration::extrinsicCalibration - branch_to_child_markers vector and otherbranch_to_parent_markers vector do not have the same size in snapshot %d, skipping.", i);
			continue;
		}

		bool success = true;
		cv::Mat uc_to_last_branch_frame;  // uc = uncertainty child, uncertainty child to last frame on branch
		cv::Mat up_to_origin;  // up = uncertainty parent, uncertainty parent to origin
		cv::Mat origin_to_last_otherbranch_frame;  // origin to last frame on other-branch

		// build transform chains
		if ( branch.size() > 0 )
		{
			TFInfo last_trafo = branch[branch.size()-1];
			success &= buildTransformChain(current_uncertainty.child_, last_trafo.child_, branch, uc_to_last_branch_frame);
		}
		else
			uc_to_last_branch_frame = cv::Mat::eye(4,4,CV_64FC1);  // identity -> has no effect on transformation chain

		success &= buildTransformChain(current_uncertainty.parent_, setup.origin_, branch, up_to_origin);

		if ( other_branch.size() > 0 )
		{
			TFInfo last_trafo = other_branch[other_branch.size()-1];
			success &= buildTransformChain(setup.origin_, last_trafo.child_, other_branch, origin_to_last_otherbranch_frame);
		}
		else
			origin_to_last_otherbranch_frame = cv::Mat::eye(4,4,CV_64FC1);

		if ( !success )
		{
			ROS_ERROR("RobotCalibration::extrinsicCalibration - Failed to build one or more necessary transforms, skipping snapshot %d.", i);
			continue;
		}

		// build marker points for extrinsic calibration of uncertainty parent
		cv::Mat up_to_last_otherbranch_frame = up_to_origin * origin_to_last_otherbranch_frame;
		for ( int j=0; j<otherbranch_last_to_parent_markers.size(); ++j )
		{
			std::string parent_marker_frame = otherbranch_last_to_parent_markers[j].child_;
			std::vector<cv::Point3f> pattern_points_3d;
			calibration_interface_->getPatternPoints3D(parent_marker_frame, pattern_points_3d);  // get pattern points of current marker
			cv::Mat uncertainty_parent_to_marker = up_to_last_otherbranch_frame * otherbranch_last_to_parent_markers[j].transform_;

			for ( int k=0; k<pattern_points_3d.size(); ++k )
			{
				cv::Mat marker_point = cv::Mat(cv::Vec4d(pattern_points_3d[k].x, pattern_points_3d[k].y, pattern_points_3d[k].z, 1.0));
				cv::Mat point_parent = uncertainty_parent_to_marker * marker_point;
				points_3d_uncertainty_parent.push_back( cv::Point3d(point_parent.at<double>(0), point_parent.at<double>(1), point_parent.at<double>(2)) );
			}
		}

		// build marker points for extrinsic calibration of uncertainty child
		for ( int j=0; j<branch_last_to_child_markers.size(); ++j )
		{
			std::string child_marker_frame = branch_last_to_child_markers[j].child_;
			std::vector<cv::Point3f> pattern_points_3d;
			calibration_interface_->getPatternPoints3D(child_marker_frame, pattern_points_3d);  // get pattern points of current marker
			cv::Mat uncertainty_child_to_marker = uc_to_last_branch_frame * branch_last_to_child_markers[j].transform_;

			for ( int k=0; k<pattern_points_3d.size(); ++k )
			{
				cv::Mat marker_point = cv::Mat(cv::Vec4d(pattern_points_3d[k].x, pattern_points_3d[k].y, pattern_points_3d[k].z, 1.0));
				cv::Mat point_child = uncertainty_child_to_marker * marker_point;
				points_3d_uncertainty_child.push_back( cv::Point3d(point_child.at<double>(0), point_child.at<double>(1), point_child.at<double>(2)) );
			}
		}
	}

	if ( points_3d_uncertainty_parent.size() == 0 || points_3d_uncertainty_child.size() == 0 )
	{
		ROS_ERROR("RobotCalibration::extrinsicCalibration - One uncertainty points vector is empty, transform from %s to %s not calibrated, skipping uncertainty!", current_uncertainty.parent_.c_str(), current_uncertainty.child_.c_str());
		return false;
	}

	// compute extrinsic transform
	if ( points_3d_uncertainty_parent.size() == points_3d_uncertainty_child.size() )
	{
		current_uncertainty.current_trafo_ = transform_utilities::computeExtrinsicTransform(points_3d_uncertainty_parent, points_3d_uncertainty_child);
		current_uncertainty.calibrated_ = true;  // uncertainty has been calibrated, this flag leads to that snapshotted TF values won't be used anymore
		return true;
	}
	else
	{
		ROS_ERROR("RobotCalibration::extrinsicCalibration - Uncertainty points vectors do not have same size, transform from %s to %s not calibrated, skipping uncertainty!", current_uncertainty.parent_.c_str(), current_uncertainty.child_.c_str());
	}

	return false;
}

std::vector<TFInfo>* RobotCalibration::getBranchEndToMarkers(const int uncertainty_index, const bool parent_markers, TFSnapshot &snapshot)
{
	for ( int i=0; i<snapshot.branch_ends_to_markers_.size(); ++i )
	{
		if ( snapshot.branch_ends_to_markers_[i].corresponding_uncertainty_idx_ == uncertainty_index )  // snapshot entry found for our uncertainty
		{
			if ( parent_markers )
				return &snapshot.branch_ends_to_markers_[i].otherbranch_to_parent_markers_;
			else
				return &snapshot.branch_ends_to_markers_[i].branch_to_child_markers_;
		}
	}

	return 0;
}

bool RobotCalibration::buildTransformChain(const std::string start, const std::string end, const std::vector<TFInfo> &branch, cv::Mat &trafo)  // get transform chain
{
	// start and end are not necessarily next to one another
	int start_idx = -1;
	int end_idx = -1;

	std::vector<std::string> branch_chain;  // consecutive chain of transforms instead of parent and child
	branch_chain.reserve(branch.size()+1);
	for ( int i=0; i<branch.size(); ++i )
		branch_chain.push_back(branch[i].parent_);

	if ( branch.size() > 0 )  // add last child
		branch_chain.push_back(branch[branch.size()-1].child_);

	for ( int i=0; i<branch_chain.size(); ++i )  // find indexes of start and end point
	{
		if ( start_idx == -1 && start.compare(branch_chain[i]) == 0 )
			start_idx = i;

		if ( end_idx == -1 && end.compare(branch_chain[i]) == 0 )
			end_idx = i;

		if ( start_idx != -1 && end_idx != -1 )  // indexes found, exit loop
			break;
	}

	if ( start_idx < 0 || end_idx < 0 )  // invalid index
	{
		ROS_ERROR("RobotCalibration::buildTransformChain - Could not build transform chain from %s to %s in current snapshot", start.c_str(), end.c_str());
		return false;
	}
	else if ( start_idx < end_idx )  // normal forward transform
	{
		for ( int i=start_idx; i<end_idx; ++i )  // retrieve trafo from start_idx to end_idx one by one
		{
			cv::Mat temp;

			if ( retrieveTransform(branch_chain[i], branch_chain[i+1], branch, temp) )
			{
				if ( trafo.empty() )
					trafo = temp;
				else
					trafo *= temp;
			}
			else
				return false;
		}
	}
	else if ( start_idx > end_idx )  // return inverse
	{
		for ( int i=start_idx; i>end_idx; --i )  // retrieve trafo from end_idx to start_idx one by one
		{
			cv::Mat temp;

			if ( retrieveTransform(branch_chain[i], branch_chain[i-1], branch, temp) )
			{
				if ( trafo.empty() )
					trafo = temp;
				else
					trafo *= temp;
			}
			else
				return false;
		}
	}
	else  // start_idx == end_idx -> return identity matrix
	{
		trafo = cv::Mat::eye(4,4,CV_64FC1);
	}

	return !trafo.empty();
}

bool RobotCalibration::retrieveTransform(const std::string parent, const std::string child, const std::vector<TFInfo> &branch, cv::Mat &trafo)  // get the next trafo
{
	// search whether it is a uncertainty and if it is calibrated. in that case return it instead of what's in the snapshot
	for ( int i=0; i<calibration_setups_.size(); ++i )
	{
		for ( int j=0; j<calibration_setups_[i].uncertainties_list_.size(); ++j )
		{
			CalibrationInfo &uncertainty = calibration_setups_[i].uncertainties_list_[j];

			if ( uncertainty.calibrated_ )
			{
				if ( uncertainty.child_.compare(child) == 0 && uncertainty.parent_.compare(parent) == 0 )
				{
					trafo = calibration_setups_[i].uncertainties_list_[j].current_trafo_.clone();
					return true;
				}
				else if ( uncertainty.child_.compare(parent) == 0 && uncertainty.parent_.compare(child) == 0 )  // return inverse
				{
					trafo = calibration_setups_[i].uncertainties_list_[j].current_trafo_.inv();
					return true;
				}
			}
		}
	}

	// in a second step search through the snapshotted branch and return the corresponding transform
	for ( int i=0; i<branch.size(); ++i )
	{
		if ( branch[i].child_.compare(child) == 0 && branch[i].parent_.compare(parent) == 0 )  // in right order
		{
			trafo = branch[i].transform_.clone();
			return true;
		}
		else if ( branch[i].child_.compare(parent) == 0 && branch[i].parent_.compare(child) == 0 )  // order swapped -> inverse
		{
			trafo = branch[i].transform_.inv();
			return true;
		}
	}

	ROS_ERROR("RobotCalibration::retrieveTransform - Could not retrieve transform from %s to %s in current snapshot", parent.c_str(), child.c_str());
	return false;
}


/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
 * the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with ORB-SLAM3.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <time.h>

#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"

// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl_conversions/pcl_conversions.h>

// #include "MapPoint.h"
// #include <opencv2/highgui/highgui_c.h>
// #include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>

#include "../../../include/System.h"

using namespace std;

bool pub_all_pts = true;
int pub_count = 0;

class ImageGrabber
{
public:
	ImageGrabber(ORB_SLAM3::System *pSLAM) : mpSLAM(pSLAM) {}

	void GrabImage(const sensor_msgs::ImageConstPtr &msg);

	ORB_SLAM3::System *mpSLAM;
};

void publish(ORB_SLAM3::System &SLAM, ros::Publisher &pub_pts_and_pose,
			 ros::Publisher &pub_all_kf_and_pts, int frame_id);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Mono_publisher");
	ros::start();

	if (argc != 3)
	{
		cerr << endl
			 << "Usage: rosrun ORB_SLAM3 Mono path_to_vocabulary path_to_settings" << endl;
		ros::shutdown();
		return 1;
	}

	// Create SLAM system. It initializes all system threads and gets ready to process frames.
	ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true);

	ImageGrabber igb(&SLAM);

	ros::NodeHandle nodeHandler;
	ros::Publisher pub_pts_and_pose = nodeHandler.advertise<geometry_msgs::PoseArray>("pts_and_pose", 1000);
	ros::Publisher pub_all_kf_and_pts = nodeHandler.advertise<geometry_msgs::PoseArray>("all_kf_and_pts", 1000);
	ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage, &igb);

	ros::spin();
	
    // Stop all threads
    SLAM.Shutdown();
    cout<< "shutting down all thread\n"<< endl;

	// Save 3D points as obj file

	// Save camera trajectory
	// SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

	// // Save 3D points and timestamps of all keyframes they are visible in
	// SLAM.SaveWithTimestamps("cam_map_pts_and_keyframes.txt");

	ros::shutdown();

	return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr &msg)
{
	// Copy the ros image message to cv::Mat.
	cv_bridge::CvImageConstPtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvShare(msg);
	}
	catch (cv_bridge::Exception &e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	mpSLAM->TrackMonocular(cv_ptr->image, cv_ptr->header.stamp.toSec());
	publish(SLAM, pub_pts_and_pose, pub_all_kf_and_pts, frame_id);
}

void publish(ORB_SLAM3::System &SLAM, ros::Publisher &pub_pts_and_pose,
			 ros::Publisher &pub_all_kf_and_pts, int frame_id)
{
	// TODO: Loop Closing
	/**
	if (pub_all_pts || SLAM.GetLoopCloser().GetmbLoopDetected){
		// pub_all_pts = SLAM.getTracker()->loop_detected = SLAM.getLoopClosing()->loop_detected = false;

	}
	else if (SLAM.GetTracker()->mCurrentFrame.is_keyframe){

	}
	**/
	// vector<ORB_SLAM3::KeyFrame *> vpKFs = SLAM.GetAtlas().GetAllMaps()->GetAllKeyFrames();
	// sort(vpKFs.begin(), vpKFs.end(), ORB_SLAM3::KeyFrame::lId);

	vector<ORB_SLAM3::Map *> vpMaps = SLAM.GetAtlas()->GetAllMaps();

	std::cout << "There are " << std::to_string(vpMaps.size()) << " maps in the atlas" << std::endl;
	for (ORB_SLAM3::Map *pMap : vpMaps)
	{
		std::cout << "  Map " << std::to_string(pMap->GetId()) << " has " << std::to_string(pMap->GetAllKeyFrames().size()) << " KFs" << std::endl;
		std::cout << "  Map " << std::to_string(pMap->GetId()) << " has " << std::to_string(pMap->GetAllMapPoints().size()) << " Map points" << std::endl;

		// All map points in this atlas
		vector<ORB_SLAM3::MapPoint *> all_map_points = SLAM.GetAtlas()->GetAllMapPoints();

		// for (auto each = all_map_points.begin(); each != all_map_points.end(); ++mp){
		for (auto mp : all_map_points)
		{
			// pMap->_WriteMapPoint(fout, mp, "");
			// std::map<KeyFrame*, size_t> keyframes = mp->GetObservations_int();
			map<ORB_SLAM3::KeyFrame *, std::tuple<int, int>> keyframes = mp->GetObservations();
			// for (std::map<KeyFrame*, size_t>::iterator it = keyframes.begin(); it != keyframes.end(); it++) {
			for (std::map<ORB_SLAM3::KeyFrame *, std::tuple<int, int>>::iterator it = keyframes.begin(); it != keyframes.end(); it++)
			{
				// fout << setprecision(6) << " " << it->first->mTimeStamp;
			}
		}

		// }
	}
}
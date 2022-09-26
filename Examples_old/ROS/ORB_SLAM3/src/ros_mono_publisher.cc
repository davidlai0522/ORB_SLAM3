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
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/PointCloud.h"

// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl_conversions/pcl_conversions.h>

// #include "MapPoint.h"
// #include <opencv2/highgui/highgui_c.h>
// #include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include "../../../include/System.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ros::NodeHandle *nh, ORB_SLAM3::System *pSLAM) : mpSLAM(pSLAM)
    {
        pub_all_mp_pt = nh->advertise<sensor_msgs::PointCloud>("all_mp_pt", 1000);
        pub_all_kf_pt = nh->advertise<geometry_msgs::PoseArray>("all_kf_pt", 1000);
        // camera_sub = nh->subscribe("/camera/rgb/image_raw", 1, &ImageGrabber::GrabImage, this);
        camera_sub = nh->subscribe("/camera_right_corrected/color/image_raw", 1, &ImageGrabber::GrabImage, this);
        // camera_sub = nh->subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage, this);
        
    }
    void GrabImage(const sensor_msgs::ImageConstPtr &msg)
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
        PublishKFPts(pub_all_mp_pt, pub_all_kf_pt, frame_id);
    }
    void PublishKFPts(ros::Publisher &pub_all_mp_pt,
                      ros::Publisher &pub_all_kf_pt, int frame_id)
    {
        first = false;
        if (first){
            tf::TransformListener listener;
            
            while (true){
                try{
                    ros::Time now = ros::Time(0);
                    listener.waitForTransform("/odom", "/camera_rgb_frame",now, ros::Duration(3.0));
                    listener.lookupTransform("/odom", "/camera_rgb_frame",now, transform);
                    cout<< "tranform" << transform.getOrigin().x()<< endl;
                    cout<< "Rotate"<< transform.getRotation().x()<< endl;
                    first=false;
                    break;
                }
                catch (tf::TransformException &ex) {
                    ROS_ERROR("%s",ex.what());
                    ros::Duration(1.0).sleep();
                    continue;
                }
            }
        }

        geometry_msgs::PoseArray kf_pt_array;
        // geometry_msgs::PointStamped mp_pt_msg;
        sensor_msgs::PointCloud mp_pt_array;
        kf_pt_array.poses.push_back(geometry_msgs::Pose());

        vector<ORB_SLAM3::Map *> vpMaps = mpSLAM->GetAtlas()->GetAllMaps();
        std::cout << "There are " << std::to_string(vpMaps.size()) << " maps in the atlas" << std::endl;

        for (ORB_SLAM3::Map *pMap : vpMaps)
        {
            std::cout << "  Map " << std::to_string(pMap->GetId()) << " has " << std::to_string(pMap->GetAllKeyFrames().size()) << " KFs" << std::endl;
            std::cout << "  Map " << std::to_string(pMap->GetId()) << " has " << std::to_string(pMap->GetAllMapPoints().size()) << " Map points" << std::endl;
        }

        for (auto mp : mpSLAM->GetAtlas()->GetAllMapPoints())
        {
            map<ORB_SLAM3::KeyFrame *, std::tuple<int, int>> keyframes = mp->GetObservations();

            for (std::map<ORB_SLAM3::KeyFrame *, std::tuple<int, int>>::iterator it = keyframes.begin(); it != keyframes.end(); it++)
            {
                cv::Mat R;
                cv::eigen2cv(it->first->GetRotation(), R);
                R = R.t();
                vector<float> q = ORB_SLAM3::Converter::toQuaternion(R);
                cv::Mat twc;
                cv::eigen2cv(it->first->GetCameraCenter(), twc);
                geometry_msgs::Pose kf_pose;

                // apply rotation (to change frame to ROS from ORB SLAM 3)
                float point_kf[3] = {twc.at<float>(0),twc.at<float>(1),twc.at<float>(2)};
                float rotation_kf[3][3] = {{0,0,1},{-1,0,0},{0,-1,0}};
                float results_kf[3] = {0, 0, 0};

                for (int i = 0; i < 3; i++){
                    for (int j = 0; j < 3; j++) {
                        results_kf[i] += rotation_kf[i][j] * point_kf[j];
                    }
                }

                tf::Quaternion q_new(transform.getRotation().x()*q[0],transform.getRotation().y()*q[1],transform.getRotation().z()*q[2],transform.getRotation().w()*q[3]);
                q_new.normalize();

                // kf_pose.position.x = results_kf[0] + transform.getOrigin().x();
                // kf_pose.position.y = results_kf[1] + transform.getOrigin().y();
                // kf_pose.position.z = results_kf[2] + transform.getOrigin().z();
                // kf_pose.orientation.x = q_new.x();
                // kf_pose.orientation.y = q_new.y();
                // kf_pose.orientation.z = q_new.z();
                // kf_pose.orientation.w = q_new.w();
                kf_pose.position.x = results_kf[0];
                kf_pose.position.y = results_kf[1];
                kf_pose.position.z = results_kf[2];
                kf_pose.orientation.x = q[0];
                kf_pose.orientation.y = q[1];
                kf_pose.orientation.z = q[2];
                kf_pose.orientation.w = q[3];

                kf_pt_array.poses.push_back(kf_pose);

                std::set<ORB_SLAM3::MapPoint *> map_points = it->first->GetMapPoints();

                for (auto map_pt : map_points)
                {
                    if (!map_pt || map_pt->isBad())
                    {
                        // printf("Point %d is bad\n", pt_id);
                        continue;
                    }
                    cv::Mat pt_pose;
                    cv::eigen2cv(map_pt->GetWorldPos(), pt_pose);
                    if (pt_pose.empty())
                    {
                        // printf("World position for point %d is empty\n", pt_id);
                        continue;
                    }

                    geometry_msgs::Point32 curr_pt;
                    // printf("wp size: %d, %d\n", wp.rows, wp.cols);
                    // pcl_cloud->push_back(pcl::PointXYZ(wp.at<float>(0), wp.at<float>(1), wp.at<float>(2)));
                    float point_mp[3] = {pt_pose.at<float>(0),pt_pose.at<float>(1),pt_pose.at<float>(2)};

                    // apply rotation (to change frame to ROS from ORB SLAM 3)
                    float rotation_mp[3][3] = {{0,0,1},{-1,0,0},{0,-1,0}};
                    float results_mp[3] = {0, 0, 0};

                    for (int i = 0; i < 3; i++){
                        for (int j = 0; j < 3; j++) {
                            results_mp[i] += rotation_mp[i][j] * point_mp[j];
                        }
                    }

                    curr_pt.x = results_mp[0];
                    curr_pt.y = results_mp[1];
                    curr_pt.z = results_mp[2];
                    mp_pt_array.points.push_back(curr_pt);
                }

            }
        }

        geometry_msgs::Pose n_kf_msg;
        // n_kf_msg.position.x = n_kf_msg.position.y = n_kf_msg.position.z = n_kf;
        kf_pt_array.poses[0] = n_kf_msg;
        // TODO: check if it should be camera link. OR should put a global frame at the initial starting point
        kf_pt_array.header.frame_id = "camera_right_link";
        kf_pt_array.header.seq = frame_id + 1;
        kf_pt_array.header.stamp = ros::Time::now();
        printf("Publishing keyframe\n");
        pub_all_kf_pt.publish(kf_pt_array);

        mp_pt_array.header.frame_id = "camera_right_link";
        mp_pt_array.header.seq = frame_id + 1;
        mp_pt_array.header.stamp = ros::Time::now();
        printf("Publishing point\n");
        pub_all_mp_pt.publish(mp_pt_array);
    }
    

public:
    bool first=true;
    ORB_SLAM3::System *mpSLAM;
    tf::StampedTransform transform;
    
private:
    int frame_id;
    ros::Publisher pub_all_mp_pt;
    ros::Publisher pub_all_kf_pt;
    ros::Subscriber camera_sub;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono_publisher");
    // ros::start();

    if (argc != 3)
    {
        cerr << endl
             << "Usage: rosrun ORB_SLAM3 Mono path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true);
    ros::NodeHandle nh;
    ImageGrabber igb = ImageGrabber(&nh, &SLAM);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();
    cout << "shutting down all thread\n"
         << endl;

    // Save 3D points as obj file

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    // // Save 3D points and timestamps of all keyframes they are visible in
    // SLAM.SaveWithTimestamps("cam_map_pts_and_keyframes.txt");

    ros::shutdown();

    return 0;
}

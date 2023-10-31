/**
 * BSD 3-Clause License
 *
 * Copyright (c) 2020, Zhi Yan
 * All rights reserved.

 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:

 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.

 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.

 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **/

#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/String.h>
#include "point_cloud_features/point_cloud_features.h"
#include "autoware_tracker/DetectedObjectArray.h"

int main(int argc, char **argv) {
        int minimum_points; // The minimum points that a cluster should contain, e.g. 3 for PCA.
        bool number_of_points, min_distance, covariance_mat3D, normalized_MOIT, slice_feature, intensity_distribution;
        std::vector<float> covariance, moit, slice, intensity, features_dig;
        autoware_tracker::DetectedObjectArray::ConstPtr objects_msg;

        ros::init(argc, argv, "point_cloud_features");
        ros::NodeHandle private_nh("~");

        ros::Publisher features_pub = private_nh.advertise<autoware_tracker::DetectedObjectArray>("/point_cloud_features/features", 100, false); // c.f. https://github.com/amirsaffari/online-random-forests#data-format

        private_nh.param<int>("minimum_points", minimum_points, 5);
        private_nh.param<bool>("number_of_points", number_of_points, true);
        private_nh.param<bool>("min_distance", min_distance, true);
        private_nh.param<bool>("covariance_mat3D", covariance_mat3D, true);
        private_nh.param<bool>("normalized_MOIT", normalized_MOIT, true);
        private_nh.param<bool>("slice_feature", slice_feature, true);
        private_nh.param<bool>("intensity_distribution", intensity_distribution, true);

        int number_of_samples_count = 0;

        while (ros::ok()) {
                objects_msg = ros::topic::waitForMessage<autoware_tracker::DetectedObjectArray>("/autoware_tracker/cluster/objects"); // process blocked waiting

                int number_of_samples = 0;

                autoware_tracker::DetectedObjectArray detected_objects;
                detected_objects.header = objects_msg->header;

                for(int i = 0; i < objects_msg->objects.size(); i++) {
                        if(objects_msg->objects[i].pointcloud.data.size()/32 >= minimum_points) {   /* to check */
                                pcl::PointCloud<pcl::PointXYZI>::Ptr pc(new pcl::PointCloud<pcl::PointXYZI>);
                                pcl::fromROSMsg(objects_msg->objects[i].pointcloud, *pc);

                                covariance.clear();
                                moit.clear();
                                slice.clear();
                                intensity.clear();
                                features_dig.clear();

                                if(number_of_points) {
                                        features_dig.push_back(numberOfPoints(pc));
                                }
                                if(min_distance) {
                                        features_dig.push_back(minDistance(pc));
                                }
                                if(covariance_mat3D) {
                                        covarianceMat3D(pc, covariance);
                                        features_dig.insert(features_dig.end(), covariance.begin(), covariance.end());
                                }
                                if(normalized_MOIT) {
                                        normalizedMOIT(pc, moit);
                                        features_dig.insert(features_dig.end(), moit.begin(), moit.end());
                                }
                                if(slice_feature) {
                                        sliceFeature(pc, 10, slice);
                                        features_dig.insert(features_dig.end(), slice.begin(), slice.end());
                                }
                                if(intensity_distribution) {
                                        intensityDistribution(pc, 25, intensity);
                                        features_dig.insert(features_dig.end(), intensity.begin(), intensity.end());
                                }

                                std_msgs::String features_msg;
                                features_msg.data += "0"; // 0:car, 1:pedestrian, 2:cyclist
                                for(int j = 0; j < features_dig.size(); j++) {
                                        features_msg.data += " " + std::to_string(j+1) + ":" + std::to_string(features_dig[j]);
                                }
                                features_msg.data += "\n";
                                //features_msg.data.insert(0, std::to_string(1) + " " + std::to_string(features_dig.size()) + " 3 1\n"); // Samples + Features + Classes + FeatureMinIndex

                                autoware_tracker::DetectedObject detected_object;
                                detected_object.header = objects_msg->objects[i].header;
                                detected_object.label = "unknown";
                                detected_object.score = 1.;
                                detected_object.space_frame = objects_msg->objects[i].space_frame;
                                detected_object.pose = objects_msg->objects[i].pose;
                                detected_object.dimensions = objects_msg->objects[i].dimensions;
                                detected_object.pointcloud = objects_msg->objects[i].pointcloud;
                                detected_object.convex_hull = objects_msg->objects[i].convex_hull;
                                detected_object.user_defined_info = features_msg;
                                detected_object.valid = true;
                                detected_object.x = objects_msg->objects[i].x;
                                detected_object.y = objects_msg->objects[i].y;
                                detected_object.z = objects_msg->objects[i].z;
                                detected_object.length = objects_msg->objects[i].length;
                                detected_object.width = objects_msg->objects[i].width;
                                detected_object.height = objects_msg->objects[i].height;
                                detected_object.xmin =  objects_msg->objects[i].xmin;
                                detected_object.xmax = objects_msg->objects[i].xmax;
                                detected_object.ymin = objects_msg->objects[i].ymin;
                                detected_object.ymax = objects_msg->objects[i].ymax;

                                detected_objects.objects.push_back(detected_object);

                                // if(objects_msg->objects[i].label.compare("unknown") == 0) {
                                //         features_msg.data += "0";
                                // } else {
                                //         features_msg.data += objects_msg->objects[i].label; // 1:car, 2:pedestrian, 3:cyclist
                                // }

                                //features_msg.data += atoi((objects_msg->objects[i].label).c_str()); // 0:car, 1:pedestrian, 2:cyclist
                                //features_msg.data += objects_msg->objects[i].label; // 0:car, 1:pedestrian, 2:cyclist

                                //std_msgs::String model_file_name = "/home/epan/Rui/Feature_Evaluation/catkin_ws/src/feature_evaluation/train-data.model";
                                // FILE *fp=fopen("/home/epan/Rui/Feature_Evaluation/catkin_ws/src/feature_evaluation/test-data.model","at");
                                //
                                // fprintf(fp,"%d",atoi((objects_msg->objects[i].label).c_str()));
                                // for(int j = 0; j < features_dig.size(); j++) {
                                //         features_msg.data += " " + std::to_string(j+1) + ":" + std::to_string(features_dig[j]);
                                //         fprintf(fp," %d",j+1);
                                //         fprintf(fp,"%s",":");
                                //         fprintf(fp,"%f",features_dig[j]);
                                // }
                                // number_of_samples_count++;
                                // fprintf(fp,"\n");
                                // fclose(fp);

                                number_of_samples++;
                        }
                }

                if(number_of_samples > 0) {
                        features_pub.publish(detected_objects);
                }

                std::cerr<<"Features have been published !"<<std::endl;
                //std::cerr<<"number_of_samples_count : "<<number_of_samples_count<<std::endl;

                ros::spinOnce();
        }

        return EXIT_SUCCESS;
}

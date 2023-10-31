// (c) 2020 Zhi Yan, Rui Yang
// This code is licensed under MIT license (see LICENSE.txt for details)
#define GMM_USES_BLAS

// ROS
#include <ros/ros.h>
#include <std_msgs/String.h>
// Online Random Forests
#include "online_forests/onlinetree.h"
#include "online_forests/onlinerf.h"

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include "autoware_tracker/DetectedObjectArray.h"
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <jsk_recognition_msgs/BoundingBox.h>

int main(int argc, char **argv) {
  std::ofstream icra_log;
  std::string log_name = "orf_time_log_"+std::to_string(ros::WallTime::now().toSec());

  std::string conf_file_name;
  std::string model_file_name;
  int mode; // 1 - train, 2 - test, 3 - train and test.
  int minimum_samples;
  int total_samples = 0;

  ros::init(argc, argv, "online_forests_ros");
  ros::NodeHandle nh, private_nh("~");

  if(private_nh.getParam("conf_file_name", conf_file_name)) {
    ROS_INFO("Got param 'conf_file_name': %s", conf_file_name.c_str());
  } else {
    ROS_ERROR("Failed to get param 'conf_file_name'");
    exit(EXIT_SUCCESS);
  }

  if(private_nh.getParam("model_file_name", model_file_name)) {
    ROS_INFO("Got param 'model_file_name': %s", model_file_name.c_str());
  } else {
    ROS_ERROR("Failed to get param 'model_file_name'");
    exit(EXIT_SUCCESS);
  }

  if(private_nh.getParam("mode", mode)) {
    ROS_INFO("Got param 'mode': %d", mode);
  } else {
    ROS_ERROR("Failed to get param 'mode'");
    exit(EXIT_SUCCESS);
  }

  private_nh.param<int>("minimum_samples", minimum_samples, 1);

  ros::Publisher _pub_final_detected_objects_box_vis = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("autoware_tracker/cluster/final_detected_objects_box_vis", 100);

  Hyperparameters hp(conf_file_name);
  //std_msgs::String::ConstPtr features;
  autoware_tracker::DetectedObjectArray::ConstPtr features_msg;
  int result_file_path_ = 5001;

  /* */
  while (ros::ok()) {
    features_msg = ros::topic::waitForMessage<autoware_tracker::DetectedObjectArray>("/point_cloud_features/features"); // process blocked waiting
    std_msgs::String features_sum;
    int number_of_samples = 0;
    for (size_t i = 0; i < features_msg->objects.size(); i++){
      features_sum.data += features_msg->objects[i].user_defined_info.data;
      number_of_samples++;
    }
    features_sum.data.insert(0, std::to_string(number_of_samples) + " " + std::to_string(61) + " 3 1\n"); // Samples + Features + Classes + FeatureMinIndex

    // Creating the train data
    DataSet dataset_tr;
    dataset_tr.loadLIBSVM2(features_sum.data);

    // Creating the test data
    DataSet dataset_ts;

    vector<Result> results;

    if(atoi(features_sum.data.substr(0, features_sum.data.find(" ")).c_str()) >= minimum_samples) {
      OnlineRF model(hp, dataset_tr.m_numClasses, dataset_tr.m_numFeatures, dataset_tr.m_minFeatRange, dataset_tr.m_maxFeatRange); // TOTEST: OnlineTree

      //icra_log.open(log_name, std::ofstream::out | std::ofstream::app);
      time_t start_time = ros::WallTime::now().toSec();

      switch(mode) {
      case 1: // train only
        if(access( model_file_name.c_str(), F_OK ) != -1){
        model.loadForest(model_file_name);
        }
        model.train(dataset_tr);
        model.writeForest(model_file_name);
        break;
      case 2: // test only
        model.loadForest(model_file_name);
        results = model.test(dataset_tr);
        break;
      case 3: // train and test
        model.trainAndTest(dataset_tr, dataset_ts);
        break;
      default:
        ROS_ERROR("Unknown 'mode'");
      }

      std::cout << "[online_forests_ros] Training time: " << ros::WallTime::now().toSec() - start_time << " s" << std::endl;
      //icra_log << (total_samples+=dataset_tr.m_numSamples) << " " << ros::WallTime::now().toSec()-start_time << "\n";
      //icra_log.close();
    }

    jsk_recognition_msgs::BoundingBoxArray bounding_box_array;

    std::ofstream outputfile(("/home/epan/Rui/results/00" + std::to_string(result_file_path_) + ".txt"), std::ofstream::out | std::ofstream::app);
    for (size_t i = 0; i < features_msg->objects.size(); i++){

      if(features_msg->objects[i].z<0) continue;

      double score_;
      score_ = 100 * ( (float)(*max_element(results[i].confidence.begin(), results[i].confidence.end())) );

      std::string prediction_;
      switch(results[i].prediction) {
      case 0:
        prediction_ = "Car";
        if(features_msg->objects[i].height < 0.5 || features_msg->objects[i].height > 3.0) continue;
        if(features_msg->objects[i].length < 1.0 || features_msg->objects[i].length > 5.0) continue;
        if(features_msg->objects[i].width < 1.0 || features_msg->objects[i].width > 5.0) continue;
        if((features_msg->objects[i].length * features_msg->objects[i].width) > 8.0) continue;
        if(score_ < 50) continue;
        break;
      case 1:
        prediction_ = "Pedestrian";
        if(features_msg->objects[i].height < 0.5 || features_msg->objects[i].height > 2.0) continue;
        if(features_msg->objects[i].length < 0.1 || features_msg->objects[i].length > 2.0) continue;
        if(features_msg->objects[i].width < 0.1 || features_msg->objects[i].width > 2.0) continue;
        if((features_msg->objects[i].length * features_msg->objects[i].width) > 1.0) continue;
        if(score_ < 50) continue;
        break;
      case 2:
        prediction_ = "Cyclist";
        if(features_msg->objects[i].height < 0.5 || features_msg->objects[i].height > 2.0) continue;
        if(features_msg->objects[i].length < 0.2 || features_msg->objects[i].length > 2.0) continue;
        if(features_msg->objects[i].width < 0.2 || features_msg->objects[i].width > 2.0) continue;
        if((features_msg->objects[i].length * features_msg->objects[i].width) > 2.0) continue;
        if(score_ < 50) continue;
        break;
      default:
        ROS_ERROR("Unknown");
      }

      tf::Quaternion quat;
      tf::quaternionMsgToTF(features_msg->objects[i].pose.orientation, quat);
      double roll, pitch, yaw;
      tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

      jsk_recognition_msgs::BoundingBox detected_object;
      detected_object.pose = features_msg->objects[i].pose;
      detected_object.dimensions.x = features_msg->objects[i].length;
      detected_object.dimensions.y = features_msg->objects[i].width;
      detected_object.dimensions.z = features_msg->objects[i].height;
      detected_object.header.frame_id = "velodyne";
      detected_object.label = 0;
      bounding_box_array.boxes.push_back(detected_object);
      // std::cerr<<"------------------------------------"<<std::endl;
      // std::cerr<<"m_numClasses:  "<<dataset_tr.m_numClasses<<std::endl;
      // std::cerr<<"m_numFeatures:  "<<dataset_tr.m_numFeatures<<std::endl;
      // std::cerr<<"m_minFeatRange:  "<<dataset_tr.m_minFeatRange<<std::endl;
      // std::cerr<<"m_maxFeatRange:  "<<dataset_tr.m_maxFeatRange<<std::endl;

      // outputfile <<prediction_ <<" "
      //            <<"-1" <<" "
      //            <<"-1" <<" "
      //            <<"-1" <<" "
      //            <<"-1 -1 -1 -1" <<" "
      //            <<std::to_string((float)(features_msg->objects[i].dimensions.z)) << " "
      //            <<std::to_string((float)(features_msg->objects[i].dimensions.x)) << " "
      //            <<std::to_string((float)(features_msg->objects[i].dimensions.y)) << " "
      //            <<std::to_string((float)(features_msg->objects[i].pose.position.x)) << " "
      //            <<std::to_string((float)(features_msg->objects[i].pose.position.y)) << " "
      //            <<std::to_string((float)(features_msg->objects[i].pose.position.z)) << " "
      //            <<std::to_string((float)(tf::getYaw(features_msg->objects[i].pose.orientation))) << " "
      //            <<score_ << "\n";
      outputfile <<prediction_ <<" "
                 <<"-1" <<" "
                 <<"-1" <<" "
                 <<"-10" <<" "
                 <<std::to_string((float)(features_msg->objects[i].xmin)) << " "
                 <<std::to_string((float)(features_msg->objects[i].ymin)) << " "
                 <<std::to_string((float)(features_msg->objects[i].xmax)) << " "
                 <<std::to_string((float)(features_msg->objects[i].ymax)) << " "
                 <<std::to_string((float)(features_msg->objects[i].height)) << " "
                 <<std::to_string((float)(features_msg->objects[i].length)) << " "
                 <<std::to_string((float)(features_msg->objects[i].width)) << " "
                 <<std::to_string(features_msg->objects[i].x) << " "
                 <<std::to_string(features_msg->objects[i].y) << " "
                 <<std::to_string(features_msg->objects[i].z) << " "
                 <<std::to_string((float)(0-yaw)) << " "
                 <<score_ << "\n";

    }

    bounding_box_array.header.frame_id = "velodyne";
    _pub_final_detected_objects_box_vis.publish(bounding_box_array);
    outputfile.close();
    std::cerr<<"Result frame "<<std::to_string(result_file_path_)<<std::endl;
    result_file_path_++;
    ros::spinOnce();
  }

  // while (ros::ok()) {
  //   features_msg = ros::topic::waitForMessage<autoware_tracker::DetectedObjectArray>("/point_cloud_features/features"); // process blocked waiting
  //
  //   OnlineRF model(hp, 3, 31, dataset_tr.m_minFeatRange, dataset_tr.m_maxFeatRange);
  //
  //   std::ofstream outputfile(("/home/epan/Rui/results/00" + std::to_string(result_file_path_) + ".txt"), std::ofstream::out | std::ofstream::app);
  //   for (size_t i = 0; i < features_msg->objects.size(); i++){
  //     vector<Result> results;
  //
  //     // Creating the train data
  //     DataSet dataset_tr;
  //     dataset_tr.loadLIBSVM2(features_msg->objects[i].user_defined_info.data);
  //
  //     // Creating the test data
  //     DataSet dataset_ts;
  //
  //     if(atoi(features_msg->objects[i].user_defined_info.data.substr(0, features_msg->objects[i].user_defined_info.data.find(" ")).c_str()) >= minimum_samples) {
  //        // TOTEST: OnlineTree
  //
  //       //icra_log.open(log_name, std::ofstream::out | std::ofstream::app);
  //       time_t start_time = ros::WallTime::now().toSec();
  //
  //       switch(mode) {
  //       case 1: // train only
  //         if(access( model_file_name.c_str(), F_OK ) != -1){
  //         model.loadForest(model_file_name);
  //         }
  //         model.train(dataset_tr);
  //         model.writeForest(model_file_name);
  //         break;
  //       case 2: // test only
  //         model.loadForest(model_file_name);
  //         results = model.test(dataset_tr);
  //         outputfile <<results[0].prediction <<" "
  //                    <<"-1" <<" "
  //                    <<"-1" <<" "
  //                    <<"-1" <<" "
  //                    <<"-1 -1 -1 -1" <<" "
  //                    <<std::to_string(features_msg->objects[i].dimensions.z) << " "
  //                    <<std::to_string(features_msg->objects[i].dimensions.x) << " "
  //                    <<std::to_string(features_msg->objects[i].dimensions.y) << " "
  //                    <<std::to_string(features_msg->objects[i].pose.position.x) << " "
  //                    <<std::to_string(features_msg->objects[i].pose.position.y) << " "
  //                    <<std::to_string(features_msg->objects[i].pose.position.z) << " "
  //                    <<std::to_string(tf::getYaw(features_msg->objects[i].pose.orientation)) << " "
  //                    <<*max_element(results[0].confidence.begin(), results[0].confidence.end()) << "\n";
  //         break;
  //       case 3: // train and test
  //         model.trainAndTest(dataset_tr, dataset_ts);
  //         break;
  //       default:
  //         ROS_ERROR("Unknown 'mode'");
  //       }
  //
  //       std::cout << "[online_forests_ros] Training time: " << ros::WallTime::now().toSec() - start_time << " s" << std::endl;
  //       //icra_log << (total_samples+=dataset_tr.m_numSamples) << " " << ros::WallTime::now().toSec()-start_time << "\n";
  //       //icra_log.close();
  //     }
  //   }
  //   outputfile.close();
  //   result_file_path_++;
  //   ros::spinOnce();
  // }

  return EXIT_SUCCESS;
}

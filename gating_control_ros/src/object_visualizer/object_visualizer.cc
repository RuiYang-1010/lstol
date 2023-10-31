#include "object_visualizer/object_visualizer.h"

// .msg from autoware_tracker
#include "autoware_tracker/DetectedObjectArray.h"
#include "autoware_tracker/DetectedObject.h"

namespace kitti_visualizer {

ObjectVisualizer::ObjectVisualizer(ros::NodeHandle nh, ros::NodeHandle pnh)
        : nh_(nh), pnh_(pnh) {
        pnh_.param<std::string>("data_path", data_path_, "");
        pnh_.param<std::string>("dataset", dataset_, "");
        pnh_.param<int>("frame_size", frame_size_, 0);
        pnh_.param<int>("current_frame", current_frame_, 0);

        // Judge whether the files number are valid
        AssertFilesNumber();

        // Subscriber
        sub_command_button_ =
                nh_.subscribe("/kitti_visualizer/command_button", 2,
                              &ObjectVisualizer::CommandButtonCallback, this);

        // Publisher
        pub_point_cloud_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZI> >(
                "kitti_visualizer/object/point_cloud", 2);
        pub_image_ =
                nh_.advertise<sensor_msgs::Image>("kitti_visualizer/object/image", 2);
        pub_bounding_boxes_ = nh_.advertise<jsk_recognition_msgs::BoundingBoxArray>(
                "kitti_visualizer/object/bounding_boxes", 2);

        // Publisher: objects & objects_vis
        pub_objects_ = nh_.advertise<autoware_tracker::DetectedObjectArray>(
                "kitti_visualizer/object/objects", 1000);
        pub_objects_vis_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZI> >(
                "kitti_visualizer/object/objects_vis", 2);
        // Statistics
        count_car_ = 0;
        count_pedestrian_ = 0;
        count_cyclist_ = 0;
}


void ObjectVisualizer::Visualizer() {
        if (dataset_ == "training") {
                for (int frame = 0; frame < frame_size_; frame++) {
                        // Get current file name
                        std::ostringstream file_prefix;
                        file_prefix << std::setfill('0') << std::setw(6) << current_frame_;
                        ROS_INFO("Visualizing frame %s ...", file_prefix.str().c_str());

                        // Visualize point cloud
                        PointCloudVisualizer(file_prefix.str(), pub_point_cloud_);

                        // Visualize image
                        //ImageVisualizer(file_prefix.str(), pub_image_);

                        // Visualize 3D bounding boxes
                        //BoundingBoxesVisualizer(file_prefix.str(), pub_bounding_boxes_);

                        //current_frame_ = (frame_size_ + current_frame_ + 1) % frame_size_;
                        current_frame_++;
                }
                std::cerr << "The Number of Car:  " << count_car_ << std::endl;
                std::cerr << "The Number of Pedestrian:  " << count_pedestrian_ << std::endl;
                std::cerr << "The Number of Cyclist:  " << count_cyclist_ << std::endl;
        } else if (dataset_ == "testing") {

                // Get current file name
                std::ostringstream file_prefix;
                file_prefix << std::setfill('0') << std::setw(6) << current_frame_;
                ROS_INFO("Visualizing frame %s ...", file_prefix.str().c_str());

                // Visualize image
                ImageVisualizer(file_prefix.str(), pub_image_);

                // Visualize 3D bounding boxes
                BoundingBoxesVisualizer(file_prefix.str(), pub_bounding_boxes_);

                // Visualize point cloud
                PointCloudVisualizer(file_prefix.str(), pub_point_cloud_);

                // for (int frame = 0; frame < 200; frame++) {
                //         // Get current file name
                //         std::ostringstream file_prefix;
                //         file_prefix << std::setfill('0') << std::setw(6) << current_frame_;
                //         ROS_INFO("Visualizing frame %s ...", file_prefix.str().c_str());
                //
                //         // Visualize image
                //         ImageVisualizer(file_prefix.str(), pub_image_);
                //
                //         // Visualize 3D bounding boxes
                //         BoundingBoxesVisualizer(file_prefix.str(), pub_bounding_boxes_);
                //
                //         // Visualize point cloud
                //         PointCloudVisualizer(file_prefix.str(), pub_point_cloud_);
                //
                //         current_frame_++;
                // }
        }
}

void ObjectVisualizer::PointCloudVisualizer(const std::string& file_prefix,
                                            const ros::Publisher publisher) {
        // Read point cloud
        std::string cloud_file_name =
                data_path_ + dataset_ + "/velodyne/" + file_prefix + ".bin";
        pcl::PointCloud<pcl::PointXYZI>::Ptr raw_cloud(
                new pcl::PointCloud<pcl::PointXYZI>);
        ReadPointCloud(cloud_file_name, raw_cloud);

        // Publish point cloud
        //raw_cloud->header.frame_id = "base_link";
        raw_cloud->header.frame_id = "velodyne";
        publisher.publish(raw_cloud);

        // Publish extracted objects
        ROS_INFO("Extracting frame: %s ...", file_prefix.c_str());
        ROS_INFO("Visualizing frame %s ... Velodyne", file_prefix.c_str());

        std::vector<std::vector<float> > detections = ParseDetections(file_prefix);
        jsk_recognition_msgs::BoundingBoxArray bounding_box_array =
                TransformBoundingBoxes(detections, file_prefix);

        autoware_tracker::DetectedObjectArray detected_objects;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_count {new pcl::PointCloud<pcl::PointXYZI>};

        for (const auto bounding_box : bounding_box_array.boxes) {
                /* Rotate the points first and then build the box */
                Eigen::Vector3f min_raw(bounding_box.pose.position.x - (bounding_box.dimensions.x / 2.0),
                                        bounding_box.pose.position.y - (bounding_box.dimensions.y / 2.0),
                                        bounding_box.pose.position.z - (bounding_box.dimensions.z / 2.0));
                Eigen::Vector3f max_raw(bounding_box.pose.position.x + (bounding_box.dimensions.x / 2.0),
                                        bounding_box.pose.position.y + (bounding_box.dimensions.y / 2.0),
                                        bounding_box.pose.position.z + (bounding_box.dimensions.z / 2.0));
                tf::Quaternion quat;
                tf::quaternionMsgToTF(bounding_box.pose.orientation, quat);
                double roll, pitch, yaw;
                tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
                float min_x = (min_raw(0) - bounding_box.pose.position.x) * cos(yaw) - (min_raw(1) - bounding_box.pose.position.y) * sin(yaw) + bounding_box.pose.position.x;
                float min_y = (min_raw(1) - bounding_box.pose.position.y) * cos(yaw) + (min_raw(0) - bounding_box.pose.position.x) * sin(yaw) + bounding_box.pose.position.y;
                float max_x = (max_raw(0) - bounding_box.pose.position.x) * cos(yaw) - (max_raw(1) - bounding_box.pose.position.y) * sin(yaw) + bounding_box.pose.position.x;
                float max_y = (max_raw(1) - bounding_box.pose.position.y) * cos(yaw) + (max_raw(0) - bounding_box.pose.position.x) * sin(yaw) + bounding_box.pose.position.y;
                crop_box.setMin(
                        Eigen::Vector4f((min_x<max_x ? min_x : max_x), (min_y<max_y ? min_y : max_y), min_raw(2), 1.0));
                crop_box.setMax(
                        Eigen::Vector4f((min_x>max_x ? min_x : max_x), (min_y>max_y ? min_y : max_y), max_raw(2), 1.0));

                /* Build the box first, then rotate the box
                   Eigen::Vector3f min_raw(bounding_box.pose.position.x - (bounding_box.dimensions.x / 2.0),
                                        bounding_box.pose.position.y - (bounding_box.dimensions.y / 2.0),
                                        bounding_box.pose.position.z - (bounding_box.dimensions.z / 2.0));
                   Eigen::Vector3f max_raw(bounding_box.pose.position.x + (bounding_box.dimensions.x / 2.0),
                                        bounding_box.pose.position.y + (bounding_box.dimensions.y / 2.0),
                                        bounding_box.pose.position.z + (bounding_box.dimensions.z / 2.0));
                   tf::Quaternion quat;
                   tf::quaternionMsgToTF(bounding_box.pose.orientation, quat);
                   double roll, pitch, yaw;
                   tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
                   Eigen::Vector3d eigen_euler;
                   eigen_euler << roll, pitch, yaw;
                   Eigen::Matrix3d rotation;
                   // rotation = Eigen::AngleAxisd(eigen_euler[2], Eigen::Vector3d::UnitZ()) *
                   //            Eigen::AngleAxisd(eigen_euler[1], Eigen::Vector3d::UnitY()) *
                   //            Eigen::AngleAxisd(eigen_euler[0], Eigen::Vector3d::UnitX());
                   // Eigen::Vector3d eigen_ret_euler = rotation.eulerAngles(0,1,2);
                   // Eigen::Vector3f eigen_ret_euler_f = eigen_ret_euler.cast <float> ();
                   Eigen::Vector3f eigen_ret_euler_f = eigen_euler.cast <float> ();

                   crop_box.setRotation(eigen_ret_euler_f);
                   crop_box.setMin(
                     Eigen::Vector4f(min_raw(0), min_raw(1), min_raw(2), 1.0));
                   crop_box.setMax(
                     Eigen::Vector4f(max_raw(0), max_raw(1), max_raw(2), 1.0));
                 */

                // std::cerr << "The Min_Raw:  " << min_raw(0) << " " << min_raw(1) << " " << min_raw(2) << std::endl;
                // std::cerr << "The Max_Raw:  " << max_raw(0) << " " << max_raw(1) << " " << max_raw(2) << std::endl;
                // std::cerr << "The Min_New:  " << min_x << " " << min_y << " " << min_raw(2) << std::endl;
                // std::cerr << "The Max_New:  " << max_x << " " << max_y << " " << max_raw(2) << std::endl;
                // std::cerr << "The Min_Corr:  " << (min_x_new<max_x_new ? min_x_new : max_x_new) << " " << (min_y_new<max_y_new ? min_y_new : max_y_new) << " " << min_raw(2) << std::endl;
                // std::cerr << "The Min_Corr:  " << (min_x_new>max_x_new ? min_x_new : max_x_new) << " " << (min_y_new>max_y_new ? min_y_new : max_y_new) << " " << max_raw(2) << std::endl;

                crop_box.setInputCloud(raw_cloud);
                crop_box.filter(*cloud_final);

                // Publish point cloud count
                *cloud_count = *cloud_count + *cloud_final;
                //cloud_final->header.frame_id = "base_link";
                //cloud_count->header.frame_id = "base_link";
                cloud_final->header.frame_id = "velodyne";
                cloud_count->header.frame_id = "velodyne";
                //cloud_count->header.stamp = ros::Time::now();

                pcl::toROSMsg(*cloud_final,cloud_out);
                //cloud_out.header.frame_id = "base_link";
                cloud_out.header.frame_id = "velodyne";

                autoware_tracker::DetectedObject detected_object;
                detected_object.label = std::to_string(bounding_box.label);
                detected_object.score = 1;
                detected_object.space_frame = cloud_out.header.frame_id;
                detected_object.pose = bounding_box.pose;
                detected_object.dimensions = bounding_box.dimensions;
                detected_object.pointcloud = cloud_out;
                detected_object.valid = true;

                detected_objects.objects.push_back(detected_object);
        }
        if(cloud_count->points.size()!= 0) {
                pub_objects_vis_.publish(cloud_count);
        }
        if(detected_objects.objects.size() != 0) {
                pub_objects_.publish(detected_objects);
        }
        if (dataset_ == "training") {
                usleep(80000); // Set up when training
        } else if (dataset_ == "testing") {
                usleep(15000000); // Set up when testing
        }
}

void ObjectVisualizer::ImageVisualizer(const std::string& file_prefix,
                                       const ros::Publisher publisher) {
        ROS_INFO("Visualizing frame %s ... Image", file_prefix.c_str());
        // Read image
        std::string image_file_name =
                data_path_ + dataset_ + "/image_2/" + file_prefix + ".png";
        cv::Mat raw_image = cv::imread(image_file_name.c_str());

        // Draw 2D bounding boxes in image
        Draw2DBoundingBoxes(file_prefix, raw_image);

        // Publish image
        sensor_msgs::ImagePtr raw_image_msg =
                cv_bridge::CvImage(std_msgs::Header(), "bgr8", raw_image).toImageMsg();
        //raw_image_msg->header.frame_id = "base_link";
        raw_image_msg->header.frame_id = "velodyne";
        publisher.publish(raw_image_msg);
}

void ObjectVisualizer::Draw2DBoundingBoxes(const std::string& file_prefix,
                                           cv::Mat& raw_image) {
        // Read bounding boxes data
        std::vector<std::vector<float> > detections = ParseDetections(file_prefix);

        // Draw bounding boxes in image
        for (const auto detection : detections) {
                cv::rectangle(raw_image, cv::Point(detection[3], detection[4]),
                              cv::Point(detection[5], detection[6]), cv::Scalar(0, 255, 0),
                              2, 8, 0);
        }
}

void ObjectVisualizer::BoundingBoxesVisualizer(const std::string& file_prefix,
                                               const ros::Publisher publisher) {
        // Read bounding boxes data
        std::vector<std::vector<float> > detections = ParseDetections(file_prefix);

        // Transform bounding boxes to jsk_recognition_msgs
        jsk_recognition_msgs::BoundingBoxArray bounding_box_array =
                TransformBoundingBoxes(detections, file_prefix);

        // Publish bounding boxes
        //bounding_box_array.header.frame_id = "base_link";
        bounding_box_array.header.frame_id = "velodyne";
        publisher.publish(bounding_box_array);
}

jsk_recognition_msgs::BoundingBoxArray ObjectVisualizer::TransformBoundingBoxes(
        const std::vector<std::vector<float> > detections,
        const std::string& file_prefix) {
        // Read transform matrixs from calib file
        std::string calib_file_name =
                data_path_ + dataset_ + "/calib/" + file_prefix + ".txt";
        Eigen::MatrixXd trans_velo_to_cam = Eigen::MatrixXd::Identity(4, 4);
        ReadCalibMatrix(calib_file_name, "Tr_velo_to_cam:", trans_velo_to_cam);
        Eigen::MatrixXd trans_cam_to_rect = Eigen::MatrixXd::Identity(4, 4);
        ReadCalibMatrix(calib_file_name, "R0_rect:", trans_cam_to_rect);

        // Set bounding boxes to jsk_recognition_msgs::BoundingBoxArray
        jsk_recognition_msgs::BoundingBoxArray bounding_box_array;
        srand(time(0)); // Use the time function to get a "seed” value for srand
        for (const auto detection : detections) {
                jsk_recognition_msgs::BoundingBox bounding_box;
                // Bounding box position
                Eigen::Vector4d rect_position(detection[10], detection[11], detection[12],
                                              1.0);
                Eigen::MatrixXd velo_position = trans_velo_to_cam.inverse() *
                                                trans_cam_to_rect.inverse() * rect_position;
                bounding_box.pose.position.x = velo_position(0);
                bounding_box.pose.position.y = velo_position(1);
                bounding_box.pose.position.z = velo_position(2) + detection[7] / 2.0;

                // Rui: only learn objects whose distance from the center is less than 50
                //std::cerr << "Distance: " << sqrt(pow(bounding_box.pose.position.x, 2) + pow(bounding_box.pose.position.y, 2)) << std::endl;
                //if (sqrt(pow(bounding_box.pose.position.x, 2) + pow(bounding_box.pose.position.y, 2)) > 50) continue;

                // Bounding box orientation
                tf::Quaternion bounding_box_quat =
                        tf::createQuaternionFromRPY(0.0, 0.0, 0.0 - detection[13]);
                tf::quaternionTFToMsg(bounding_box_quat, bounding_box.pose.orientation);
                // Bounding box dimensions
                bounding_box.dimensions.x = detection[8];
                bounding_box.dimensions.y = detection[9];
                bounding_box.dimensions.z = detection[7];

                // Bounding box header
                bounding_box.header.stamp = ros::Time::now();
                //bounding_box.header.frame_id = "base_link";
                bounding_box.header.frame_id = "velodyne";

                // Bounding box label
                bounding_box.label = detection[14];
                if(bounding_box.label == 0) {
                        /* normal_sampling */
                        count_car_++;
                        bounding_box_array.boxes.push_back(bounding_box);

                        /* down_sampling
                           int i = int(rand()%15);
                           if(i==1) {
                                count_car_++;
                                bounding_box_array.boxes.push_back(bounding_box);
                           } */
                } else if(bounding_box.label == 1) {
                        /* normal_sampling */
                        count_pedestrian_++;
                        bounding_box_array.boxes.push_back(bounding_box);

                        /* over_sampling
                           count_pedestrian_ = count_pedestrian_ + 5;
                           for(int i=0; i<5; i++){
                           bounding_box_array.boxes.push_back(bounding_box);
                           } */
                        /* down_sampling
                           int i= int(rand()%3);
                           if(i==1) {
                             count_pedestrian_++;
                             bounding_box_array.boxes.push_back(bounding_box);
                           } */
                } else if(bounding_box.label == 2) {
                        count_cyclist_++;
                        bounding_box_array.boxes.push_back(bounding_box);
                        /* over_sampling
                           count_cyclist_ = count_cyclist_ + 15;
                           for(int i=0; i<15; i++) {
                                bounding_box_array.boxes.push_back(bounding_box);
                           } */

                }
                //bounding_box_array.boxes.push_back(bounding_box);
        }

        return bounding_box_array;
}

std::vector<std::vector<float> > ObjectVisualizer::ParseDetections(
        const std::string& file_prefix) {
        // Open bounding boxes file
        std::string detections_file_name;
        if (dataset_ == "training") {
                detections_file_name =
                        data_path_ + dataset_ + "/label_2/" + file_prefix + ".txt";
        } else if (dataset_ == "testing") {
                // detections_file_name =
                //         data_path_ + dataset_ + "/results/" + file_prefix + ".txt";
                detections_file_name =
                        data_path_ + dataset_ + "/label_2/" + file_prefix + ".txt";
        }
        std::ifstream detections_file(detections_file_name);
        if (!detections_file) {
                ROS_ERROR("File %s does not exist", detections_file_name.c_str());
                ros::shutdown();
        }

        // Parse objects data
        std::vector<std::vector<float> > detections;
        std::string line_str;
        while (getline(detections_file, line_str)) {
                // Store std::string into std::stringstream
                std::stringstream line_ss(line_str);
                // Parse object type
                std::string object_type;
                getline(line_ss, object_type, ' ');
                if (object_type == "DontCare" || object_type == "Misc" || object_type == "Tram") continue;
                // Parse object data
                std::vector<float> detection;
                std::string str;
                while (getline(line_ss, str, ' ')) {
                        detection.push_back(boost::lexical_cast<float>(str));
                }
                /* ---Rui--- */
                if(object_type == "Car"|| object_type == "Van" || object_type == "Truck") {
                        detection.push_back(0);
                } else if(object_type == "Pedestrian" || object_type == "Person_sitting") {
                        detection.push_back(1);
                } else if(object_type == "Cyclist") {
                        detection.push_back(2);
                }

                detections.push_back(detection);
        }
        return detections;
}

void ObjectVisualizer::CommandButtonCallback(
        const std_msgs::String::ConstPtr& in_command) {
        // Parse frame number form command
        if (in_command->data == "Next") {
                //current_frame_ = (frame_size_ + current_frame_ + 1) % frame_size_;
                current_frame_++;
        } else if (in_command->data == "Prev") {
                //current_frame_ = (frame_size_ + current_frame_ - 1) % frame_size_;
                current_frame_++;
        } else {
                int frame = std::stoi(in_command->data);
                if (frame >= 0 && frame < frame_size_)
                        current_frame_ = frame;
                else
                        ROS_ERROR("No frame %s", in_command->data.c_str());
        }

        // Visualize object data
        Visualizer();
}

void ObjectVisualizer::AssertFilesNumber() {
        // Assert velodyne files numbers
        ROS_ASSERT(FolderFilesNumber(data_path_ + dataset_ + "/velodyne") ==
                   frame_size_);
        // Assert image_2 files numbers
        ROS_ASSERT(FolderFilesNumber(data_path_ + dataset_ + "/image_2") ==
                   frame_size_);
        // Assert calib files numbers
        ROS_ASSERT(FolderFilesNumber(data_path_ + dataset_ + "/calib") ==
                   frame_size_);
        if (dataset_ == "training") {
                // Assert label_2 files numbers
                ROS_ASSERT(FolderFilesNumber(data_path_ + dataset_ + "/label_2") ==
                           frame_size_);
        } else if (dataset_ == "testing") {
                // Assert results files numbers
                // ROS_ASSERT(FolderFilesNumber(data_path_ + dataset_ + "/results") ==
                //            frame_size_);
                ROS_ASSERT(FolderFilesNumber(data_path_ + dataset_ + "/label_2") ==
                           frame_size_);
        } else {
                ROS_ERROR("Dataset input error: %s", dataset_.c_str());
                ros::shutdown();
        }
}
}

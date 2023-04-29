#include <ros/ros.h>
#include <librealsense2/rs.hpp>
#include <opencv2/core/types.hpp>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <geometry_msgs/PoseArray.h>
#include <aruco_msgs/MarkerArray.h>
#include <aruco_msgs/Marker.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <string>

class CamInfo
{
	public:
		int width;
		int height;
		float ppx;
		float ppy;
		float fx;
		float fy;
		rs2_distortion model;
		float coeffs[5];
};

class Node
{
private:
	ros::NodeHandle nh;
	image_transport::ImageTransport it;
	ros::Subscriber bbox_sub_1;
	ros::Subscriber pose_sub;
	ros::Subscriber caminfo_sub_1;
	image_transport::Subscriber image_sub_1;
	ros::Subscriber caminfo_sub_2;
	image_transport::Subscriber image_sub_2;
	ros::Publisher pose_pub;
	ros::Publisher cake_pub;
	ros::Publisher cherry_near_pub;
	ros::Publisher cherry_far_pub;
	
	bool cam1_info_received = false;
	bool cam2_info_received = false;
	bool image1_received = false;
	bool image2_received = false;
	CamInfo rs_intrin1;
	CamInfo rs_intrin2;
	cv::Mat depth_image1;
	cv::Mat depth_image2;
	tf::TransformListener _tfListener;
    std::string camera1_frame;
	std::string camera2_frame;
	int image_height;
	int image_width;
	bool cherry0 = false;
	bool cherry1 = false;
	bool cherry3 = false;
	
public:
	Node():
		nh("~"), it(nh)
	{
		bbox_sub_1 = nh.subscribe("/darknet_ros/bounding_boxes", 1, &Node::Bbox_callback, this);
		pose_sub = nh.subscribe("/openvino/boundingboxes", 1, &Node::pose_callback, this);
		image_sub_1 = it.subscribe("/camera1/aligned_depth_to_color/image_raw", 1, &Node::Image_callback1, this);
		caminfo_sub_1 = nh.subscribe("/camera1/aligned_depth_to_color/camera_info", 1, &Node::Info_callback1, this);
		image_sub_2 = it.subscribe("/camera2/aligned_depth_to_color/image_raw", 1, &Node::Image_callback2, this);
		caminfo_sub_2 = nh.subscribe("/camera2/aligned_depth_to_color/camera_info", 1, &Node::Info_callback2, this);
		pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose",10);
		cake_pub = nh.advertise<aruco_msgs::MarkerArray>("cakes",10);
		cherry_near_pub = nh.advertise<std_msgs::Int32>("cherry_near",10);
		cherry_far_pub = nh.advertise<std_msgs::Int32MultiArray>("cherry_far",10);
		nh.param<std::string>("camera1_frame", camera1_frame, "camera_color_frame_1");
		nh.param<std::string>("camera2_frame", camera2_frame, "camera_color_frame_2");
		nh.param<int>("image_height", image_height, 720);
		nh.param<int>("image_width", image_width, 1280);
	}
	
	void pose_callback(const geometry_msgs::PoseArray &msg)
	{
		ros::Time curr_stamped = ros::Time::now();
		auto points = msg.poses;
		aruco_msgs::MarkerArray cakes;
		for (std::size_t i = 0; i < points.size(); i++)
		{
			if (points[i].position.z == 2 || points[i].position.z == 3)
			{
				if (points[i].position.z == 2)
					cherry0 = true;
				else
					cherry0 = false;

				std_msgs::Int32 cherry_near_state;
				cherry_near_state.data = cherry0 ? 1 : 0;
				cherry_near_pub.publish(cherry_near_state);
			}

			if (points[i].position.z == 0 || points[i].position.z == 4 || points[i].position.z == 5 || points[i].position.z == 1)
			{
				float pixel_to_find[2];
				float point_3d[3];
				pixel_to_find[0] = points[i].position.x;
				pixel_to_find[1] = points[i].position.y;
				if ( tooclose(pixel_to_find[0], pixel_to_find[1]) )
				{
					continue;
				}
				if (cam2_info_received == true && image2_received == true)
				{
					float depth = depth_image2.at<float>(pixel_to_find[1], pixel_to_find[0]);
					rs2_deproject_pixel_to_point(point_3d, rs_intrin2, pixel_to_find, depth);
					tf::Transform transform;
					transform.setOrigin(tf::Vector3(point_3d[0]/1000, point_3d[1]/1000, point_3d[2]/1000));
					// std::cout << point_3d[0]/1000 << std::endl;
					// std::cout << point_3d[1]/1000 << std::endl;
					// std::cout << point_3d[2]/1000 << std::endl;
					// std::cout << "================" << std::endl;
					tf::Quaternion q;
					q.setRPY(0,0,2.36);
					transform.setRotation(q);
					tf::StampedTransform transformstamped;
					if (getTransform("world_frame", camera2_frame, transformstamped))
					{
						// tf::Transform transform_get = static_cast<tf::Transform>(transformstamped);
						// transform = transform_get * transform;
						transform = static_cast<tf::Transform>(transformstamped) * transform;
						// std::cout << transform_get.getOrigin().x() << std::endl;
						// std::cout << transform_get.getOrigin().y() << std::endl;
						// std::cout << transform_get.getOrigin().z() << std::endl;
						// std::cout << "---" << std::endl;
						// std::cout << transform.getOrigin().x() << std::endl;
						// std::cout << transform.getOrigin().y() << std::endl;
						// std::cout << transform.getOrigin().z() << std::endl;
						// std::cout << transform.getRotation().x() << std::endl;
            			// std::cout << transform.getRotation().y() << std::endl;
            			// std::cout << transform.getRotation().z() << std::endl;
            			// std::cout << transform.getRotation().w() << std::endl;
						// std::cout << "-----------------" << std::endl;
					}					
					aruco_msgs::Marker marker;
					// change id to correct: cake1 pink2 yellow3 brown4
					if (points[i].position.z == 0)
						marker.id = 4;
					else if (points[i].position.z == 1)
						marker.id = 1;
					else if (points[i].position.z == 4)
						marker.id = 2;
					else if (points[i].position.z == 5)
						marker.id = 3;
					else
						marker.id = points[i].position.z;
					geometry_msgs::PoseStamped poseMsg;
					tf::poseTFToMsg(transform, poseMsg.pose);
					poseMsg.header.frame_id = "world_frame";
					poseMsg.header.stamp = curr_stamped;
					pose_pub.publish(poseMsg);
					//cake_pub
					marker.pose.pose = poseMsg.pose;
					cakes.markers.push_back(marker);
				}
			}
			else
				continue;
		}
		if (cam2_info_received == true && image2_received == true)
		{
			cake_pub.publish(cakes);
		}
		image2_received = false;
	}

	void Bbox_callback(const darknet_ros_msgs::BoundingBoxes &msg)
	{
		ros::Time curr_stamped = ros::Time::now();
		auto bboxes = msg.bounding_boxes;
		aruco_msgs::MarkerArray cakes;
		for (std::size_t i = 0; i < bboxes.size(); i++)
		{
			if (bboxes[i].id == 2 || bboxes[i].id == 3)
			{
				if (bboxes[i].xmin < 640)
				{
					if (bboxes[i].id == 2)
					cherry1 = true;
					else
					cherry1 = false;
				}
				else
				{
					if (bboxes[i].id == 2)
					cherry3 = true;
					else
					cherry3 = false;
				}
				std_msgs::Int32MultiArray cherry_far_state; 
				cherry_far_state.data.clear();
				cherry_far_state.data.push_back(cherry1 ? 1 : 0);
				cherry_far_state.data.push_back(cherry3 ? 1 : 0);
				cherry_far_pub.publish(cherry_far_state);
			}

			else if (bboxes[i].id == 0 || bboxes[i].id == 4 || bboxes[i].id == 5 || bboxes[i].id == 1)
			{
				float pixel_to_find[2];
				float point_3d[3];
				pixel_to_find[0] = (bboxes[i].xmin + bboxes[i].xmax)/2;
				pixel_to_find[1] = (bboxes[i].ymin + bboxes[i].ymax)/2;
				if ( tooclose(pixel_to_find[0], pixel_to_find[1]) )
				{
					continue;
				}
				if (cam1_info_received == true && image1_received == true)
				{
					float depth = depth_image1.at<float>(pixel_to_find[1], pixel_to_find[0]);
					rs2_deproject_pixel_to_point(point_3d, rs_intrin2, pixel_to_find, depth);
					tf::Transform transform;
					transform.setOrigin(tf::Vector3(point_3d[0]/1000, point_3d[1]/1000, point_3d[2]/1000));
					tf::Quaternion q;
					q.setRPY(0,0,2.36);
					transform.setRotation(q);
					tf::StampedTransform transformstamped;
					if (getTransform("world_frame", camera1_frame, transformstamped))
					{
						transform = static_cast<tf::Transform>(transformstamped) * transform;
					}					
					aruco_msgs::Marker marker;
					// change id to correct: cake1 pink2 yellow3 brown4
					if (bboxes[i].id == 0)
						marker.id = 4;
					else if (bboxes[i].id == 1)
						marker.id = 1;
					else if (bboxes[i].id == 4)
						marker.id = 2;
					else if (bboxes[i].id == 5)
						marker.id = 3;
					else
						marker.id = bboxes[i].id;
					geometry_msgs::PoseStamped poseMsg;
					tf::poseTFToMsg(transform, poseMsg.pose);
					poseMsg.header.frame_id = "world_frame";
					poseMsg.header.stamp = curr_stamped;
					pose_pub.publish(poseMsg);
					//cake_pub
					marker.pose.pose = poseMsg.pose;
					cakes.markers.push_back(marker);
				}
			}
			else
				continue;
		}
		if (cam1_info_received == true && image1_received == true)
		{
			cake_pub.publish(cakes);
		}
		image1_received = false;
	}

	void Image_callback1(const sensor_msgs::ImageConstPtr &msg)
	{
		cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
		depth_image1 = cv_ptr->image;
		image1_received = true;
	}

	void Image_callback2(const sensor_msgs::ImageConstPtr &msg)
	{
		cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
		depth_image2 = cv_ptr->image;
		image2_received = true;
	}
	
	void Info_callback1(const sensor_msgs::CameraInfo &msg)
	{
		rs_intrin1.fx = msg.K[0]; //fx
		rs_intrin1.fy = msg.K[4]; //fy
		rs_intrin1.ppx = msg.K[2]; //cx
		rs_intrin1.ppy = msg.K[5]; //cy
		rs_intrin1.width = msg.width;
		rs_intrin1.height = msg.height;
		rs_intrin1.coeffs[0] = msg.D[0];
		rs_intrin1.coeffs[1] = msg.D[1];
		rs_intrin1.coeffs[2] = msg.D[2];
		rs_intrin1.coeffs[3] = msg.D[3];
		rs_intrin1.coeffs[4] = msg.D[4];
		rs_intrin1.model = RS2_DISTORTION_BROWN_CONRADY;
		cam1_info_received = true;
		caminfo_sub_1.shutdown();
	}

	void Info_callback2(const sensor_msgs::CameraInfo &msg)
	{
		rs_intrin2.fx = msg.K[0]; //fx
		rs_intrin2.fy = msg.K[4]; //fy
		rs_intrin2.ppx = msg.K[2]; //cx
		rs_intrin2.ppy = msg.K[5]; //cy
		rs_intrin2.width = msg.width;
		rs_intrin2.height = msg.height;
		rs_intrin2.coeffs[0] = msg.D[0];
		rs_intrin2.coeffs[1] = msg.D[1];
		rs_intrin2.coeffs[2] = msg.D[2];
		rs_intrin2.coeffs[3] = msg.D[3];
		rs_intrin2.coeffs[4] = msg.D[4];
		rs_intrin2.model = RS2_DISTORTION_BROWN_CONRADY;
		cam2_info_received = true;
		caminfo_sub_2.shutdown();
	}
	
	static void rs2_deproject_pixel_to_point(float point[3], CamInfo intrin, const float pixel[2], float depth)
	{
		assert(intrin.model != RS2_DISTORTION_MODIFIED_BROWN_CONRADY); // Cannot deproject from a forward-distorted image
		assert(intrin.model != RS2_DISTORTION_FTHETA); // Cannot deproject to an ftheta image
		//assert(intrin->model != RS2_DISTORTION_BROWN_CONRADY); // Cannot deproject to an brown conrady model

		float x = (pixel[0] - intrin.ppx) / intrin.fx;
		float y = (pixel[1] - intrin.ppy) / intrin.fy;
		if(intrin.model == RS2_DISTORTION_INVERSE_BROWN_CONRADY)
		{
		    float r2  = x*x + y*y;
		    float f = 1 + intrin.coeffs[0]*r2 + intrin.coeffs[1]*r2*r2 + intrin.coeffs[4]*r2*r2*r2;
		    float ux = x*f + 2*intrin.coeffs[2]*x*y + intrin.coeffs[3]*(r2 + 2*x*x);
		    float uy = y*f + 2*intrin.coeffs[3]*x*y + intrin.coeffs[2]*(r2 + 2*y*y);
		    x = ux;
		    y = uy;
		}
		point[0] = depth * x;
		point[1] = depth * y;
		point[2] = depth;
	}
	
	bool getTransform(const std::string& refFrame, const std::string& childFrame, tf::StampedTransform& transform)
	{
		std::string errMsg;
		if (!_tfListener.waitForTransform(refFrame, childFrame, ros::Time(0), ros::Duration(0.5), ros::Duration(0.01), &errMsg))
		{
			ROS_ERROR_STREAM("Unable to get pose from TF: " << errMsg);
			return false;
		}
		else
		{
			try
			{
				_tfListener.lookupTransform(refFrame, childFrame, ros::Time(0), transform);
			}
			catch (const tf::TransformException& e)
			{
				ROS_ERROR_STREAM("Error in lookupTransform of " << childFrame << " in " << refFrame);
				return false;
			}
		}
		return true;
	}

	bool tooclose(double x, double y)
	{
		int thresh = 20;
		if ( (abs(image_width-x) < thresh) || (x < thresh) || (abs(image_height-y) < thresh) || (y < thresh ) )
			return true;
		else
			return false;
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mklc_node");
	Node node;
	ros::spin();
}

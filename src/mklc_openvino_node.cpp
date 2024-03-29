#include <ros/ros.h>
#include <librealsense2/rs.hpp>
#include <opencv2/core/types.hpp>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/PoseArray.h>
#include <aruco_msgs/MarkerArray.h>
#include <aruco_msgs/Marker.h>

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
	ros::Subscriber pose_sub;
	ros::Subscriber caminfo_sub;
	image_transport::Subscriber image_sub;
	ros::Publisher pose_pub;
	ros::Publisher cake_pub;
	
	bool cam_info_received = false;
	bool image_received = false;
	CamInfo rs_intrin;
	cv::Mat depth_image;
	float point_3d[3];
	tf::TransformListener _tfListener;
	std::string camera_frame;
	
public:
	Node():
		nh("~"), it(nh)
	{
		pose_sub = nh.subscribe("/openvino/boundingboxes", 1, &Node::Bbox_callback, this);
		image_sub = it.subscribe("/camera/aligned_depth_to_color/image_raw", 1, &Node::Image_callback, this);
		caminfo_sub = nh.subscribe("/camera/aligned_depth_to_color/camera_info", 1, &Node::Info_callback, this);
		pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose",100);
		cake_pub = nh.advertise<aruco_msgs::MarkerArray>("cakes",100);
		nh.param<std::string>("camera_frame", camera_frame, "camera_color_frame");
    }
	
	void Bbox_callback(const geometry_msgs::PoseArray &msg)
	{
		ros::Time curr_stamped = ros::Time::now();
		auto points = msg.poses;
		int brown_count = 0;
		int pink_count = 0;
		int yellow_count = 0;
		int cake_count = 0;
		aruco_msgs::MarkerArray cakes;
		for (std::size_t i = 0; i < points.size(); i++)
		{
			if (points[i].position.z == 0 || points[i].position.z == 4 || points[i].position.z == 5 || points[i].position.z == 1)
			{
				float pixel_to_find[2];
				pixel_to_find[0] = points[i].position.x;
				pixel_to_find[1] = points[i].position.y;
				float depth = depth_image.at<float>(pixel_to_find[1], pixel_to_find[0]);
				//std::cout << depth << std::endl;
				std::string name;
				aruco_msgs::Marker marker;
				marker.id = points[i].position.z;
				if (points[i].position.z == 0)
				{
					brown_count++;
					name = "brown";
					name.append(std::to_string(brown_count));
				}
				else if (points[i].position.z == 4)
				{
					pink_count++;
					name = "pink";
					name.append(std::to_string(pink_count));
				}
				else if (points[i].position.z == 5)
				{
					yellow_count++;
					name = "yellow";
					name.append(std::to_string(yellow_count));
				}
				else if (points[i].position.z == 1)
				{
					cake_count++;
					name = "cake";
					name.append(std::to_string(cake_count));
				}
				else
					std::cout << "invalid id" << std::endl;
				
				if (cam_info_received == true && image_received == true)
				{
					rs2_deproject_pixel_to_point(point_3d, rs_intrin, pixel_to_find, depth, name, camera_frame);
					tf::StampedTransform transformstamped;
					if (getTransform("world_frame", name, transformstamped))
					{
						geometry_msgs::PoseStamped poseMsg;
						tf::poseTFToMsg(static_cast<tf::Transform>(transformstamped), poseMsg.pose);
						poseMsg.header.frame_id = "world_frame";
						poseMsg.header.stamp = curr_stamped;
						pose_pub.publish(poseMsg);
						//cake_pub
						marker.pose.pose = poseMsg.pose;
						cakes.markers.push_back(marker);				
					}
				}
			}
			else
				continue;
		}
		if (cam_info_received == true && image_received == true)
		{
			cake_pub.publish(cakes);
		}
		image_received = false;
	}

	void Image_callback(const sensor_msgs::ImageConstPtr &msg)
	{
		cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
		depth_image = cv_ptr->image;
		image_received = true;
	}
	
	void Info_callback(const sensor_msgs::CameraInfo &msg)
	{
		rs_intrin.fx = msg.K[0]; //fx
		rs_intrin.fy = msg.K[4]; //fy
		rs_intrin.ppx = msg.K[2]; //cx
		rs_intrin.ppy = msg.K[5]; //cy
		rs_intrin.width = msg.width;
		rs_intrin.height = msg.height;
		rs_intrin.coeffs[0] = msg.D[0];
		rs_intrin.coeffs[1] = msg.D[1];
		rs_intrin.coeffs[2] = msg.D[2];
		rs_intrin.coeffs[3] = msg.D[3];
		rs_intrin.coeffs[4] = msg.D[4];
		rs_intrin.model = RS2_DISTORTION_BROWN_CONRADY;
		cam_info_received = true;
	}
	
	static void rs2_deproject_pixel_to_point(float point[3], CamInfo intrin, const float pixel[2], float depth, std::string name, std::string camera_frame)
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
		//std::cout << name <<std::endl;
		//std::cout << "x: " << point[0]/1000 << std::endl;
		//std::cout << "y: " << point[1]/1000 << std::endl;
		//std::cout << "z: " << point[2]/1000 << std::endl;
		//std::cout << "---------------" << std::endl;
		static tf::TransformBroadcaster br;
		tf::Transform transform;
		transform.setOrigin(tf::Vector3(point[0]/1000, point[1]/1000, point[2]/1000));
		tf::Quaternion q;
		q.setRPY(2.36,0,0);
		transform.setRotation(q);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), camera_frame, name));
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
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mklc_node");
	Node node;
	ros::spin();
}

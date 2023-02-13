#include <ros/ros.h>
#include <librealsense2/rs.hpp>
#include <opencv2/core/types.hpp>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <string>

class Marker
{
	public:
		std::string frame;
		int id;
		cv::Point pt;
};

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
	ros::Subscriber bbox_sub;
	ros::Subscriber caminfo_sub;
	image_transport::Subscriber image_sub;
	ros::Publisher pose_pub;
	
	bool cam_info_received = false;
	CamInfo rs_intrin;
	std::vector<Marker> markers_;
	cv::Mat depth_image;
	float point_3d[3];
	tf::TransformListener _tfListener;
	
public:
	Node():
		nh("~"), it(nh)
	{
		bbox_sub = nh.subscribe("/darknet_ros/bounding_boxes", 1, &Node::Bbox_callback, this);
		image_sub = it.subscribe("/camera/aligned_depth_to_color/image_raw", 1, &Node::Image_callback, this);
		caminfo_sub = nh.subscribe("/camera/aligned_depth_to_color/camera_info", 1, &Node::Info_callback, this);
		pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose",100);
	}
	
	void Bbox_callback(const darknet_ros_msgs::BoundingBoxes &msg)
	{
		ros::Time curr_stamped = ros::Time::now();
		auto bboxes = msg.bounding_boxes;
		int brown_count = 0;
		int pink_count = 0;
		int yellow_count = 0;
		for (std::size_t i = 0; i < bboxes.size(); i++)
		{
			/*
			Marker marker;
			marker.pt = cv::Point((bboxes[i].xmin + bboxes[i].xmax) / 2, (bboxes[i].ymin+ bboxes[i].ymax) / 2);
			marker.id = bboxes[i].id;
			marker.frame = "marker";
			markers_.push_back(marker);
			*/
			float pixel_to_find[2];
			pixel_to_find[0] = (bboxes[i].xmin + bboxes[i].xmax)/2;
			pixel_to_find[1] = (bboxes[i].ymin + bboxes[i].ymax)/2;
			float depth = depth_image.at<float>(pixel_to_find[1], pixel_to_find[0]);
			//std::cout << depth << std::endl;
			std::string name;
			if (bboxes[i].id == 0)
			{
				brown_count++;
				name = "brown";
				name.append(std::to_string(brown_count));
			}
			else if (bboxes[i].id == 1)
			{
				pink_count++;
				name = "pink";
				name.append(std::to_string(pink_count));
			}
			else if (bboxes[i].id == 2)
			{
				yellow_count++;
				name = "yellow";
				name.append(std::to_string(yellow_count));
			}
			else
				std::cout << "invalid id" << std::endl;
			
			if (cam_info_received == true)
			{
				rs2_deproject_pixel_to_point(point_3d, rs_intrin, pixel_to_find, depth, name);
				tf::StampedTransform transformstamped;
				if (getTransform("world_frame", name, transformstamped))
				{
					geometry_msgs::PoseStamped poseMsg;
					tf::poseTFToMsg(static_cast<tf::Transform>(transformstamped), poseMsg.pose);
					poseMsg.header.frame_id = "world_frame";
					poseMsg.header.stamp = curr_stamped;
					pose_pub.publish(poseMsg);
				}
			}
			
		}
	}

	void Image_callback(const sensor_msgs::ImageConstPtr &msg)
	{
		cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
		depth_image = cv_ptr->image;
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
	
	static void rs2_deproject_pixel_to_point(float point[3], CamInfo intrin, const float pixel[2], float depth, std::string name)
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
		q.setRPY(0,0,2.36);
		transform.setRotation(q);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_color_frame", name));
		
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

#include <ros/ros.h>
#include <librealsense2/rs.hpp>
#include <opencv2/core/types.hpp>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <aruco_msgs/MarkerArray.h>
#include <aruco_msgs/Marker.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <string>

#include <opencv2/highgui/highgui.hpp>

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
	ros::Subscriber caminfo_sub;
	image_transport::Subscriber image_sub;
	image_transport::Subscriber imagecolor_sub;
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
		image_sub = it.subscribe("/depth_image_topic", 2, &Node::Image_callback, this);
		imagecolor_sub = it.subscribe("/color_image_topic", 2, &Node::Imagecolor_callback, this);
		caminfo_sub = nh.subscribe("/camera_info_topic", 2, &Node::Info_callback, this);
		nh.param<std::string>("camera_frame", camera_frame, "");
		cv::namedWindow("test");
		cv::setMouseCallback("test", &Node::mouseCall, this);
	}
	
	cv::Mat color_image;

	void Image_callback(const sensor_msgs::ImageConstPtr &msg)
	{
		cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
		depth_image = cv_ptr->image;
		image_received = true;
	}
	
	void Imagecolor_callback(const sensor_msgs::ImageConstPtr &msg)
	{
		cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		color_image = cv_ptr->image;
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
		caminfo_sub.shutdown();
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
		return;
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

	static void mouseCall(int event, int x, int y, int flags, void* param)
	{
		if (event == cv::EVENT_LBUTTONDBLCLK)
		{
			float pixel_to_find[2];
			pixel_to_find[0] = x;
			pixel_to_find[1] = y;
			float depth = ((Node*)param)->depth_image.at<float>(pixel_to_find[1], pixel_to_find[0]);
			if (((Node*)param)->cam_info_received == true && ((Node*)param)->image_received == true)
			{
				float point_3d_[3];
				rs2_deproject_pixel_to_point(point_3d_, ((Node*)param)->rs_intrin, pixel_to_find, depth);
				tf::Transform transform;
				transform.setOrigin(tf::Vector3(point_3d_[0]/1000, point_3d_[1]/1000, point_3d_[2]/1000));
				tf::Quaternion q;
				q.setRPY(0,0,0);
				transform.setRotation(q);
				tf::StampedTransform transformstamped;
				if ( ((Node*)param)->getTransform("world_frame", ((Node*)param)->camera_frame, transformstamped) )
				{
					transform = static_cast<tf::Transform>(transformstamped) * transform;
					ROS_INFO("reference to world");
					ROS_INFO("x: %f, y: %f, z: %f",transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z());
				}
				else
				{
					ROS_INFO("reference to camera");
					ROS_INFO("x: %f, y: %f, z: %f",point_3d_[0]/1000,point_3d_[1]/1000,point_3d_[2]/1000);
				}	
			}
		}
		return;
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "testdepth");
	Node node;
	while(ros::ok())
	{
		ros::spinOnce();
		if( node.color_image.size[0] != 0)
		{
			cv::imshow("test",node.color_image);
			char k = cv::waitKey(200);
    		if( k == 'ESC')
        		ros::shutdown();
		}
	}
	// ros::spin();
}

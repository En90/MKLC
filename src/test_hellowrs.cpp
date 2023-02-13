#include <librealsense2/rs.hpp>
#include <ros/ros.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_hollowrs_node");
	rs2::pipeline pipe;
	pipe.start();
	while(ros::ok())
	{
		rs2::frameset frames = pipe.wait_for_frames();
		rs2::depth_frame depth = frames.get_depth_frame();
		float width = depth.get_width(); //848
		float height = depth.get_height(); //480
		float dist_to_center = depth.get_distance(width / 2 , height / 2);
		std::cout << "facing on object " << dist_to_center << " meters away \r";
	}
	pipe.stop();
}

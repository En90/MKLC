
#include <ros/ros.h>
#include <vector>
#include <cmath>
//msgs_type
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <aruco_msgs/MarkerArray.h>
#include <aruco_msgs/Marker.h>
#include <geometry_msgs/PoseArray.h>
class Cake
{
	public:
		int id;
		float tx;
		float ty;
		float tz;
		float rx;
		float ry;
		float rz;
		float rw;
};

class Interface
{
	private:
		ros::NodeHandle nh;
		ros::Timer timer;
		std::vector<Cake> CakeCandidate;
		
		ros::Publisher cakes_pub;
		ros::Subscriber m_sub1;
		ros::Subscriber m_sub2;
		ros::Subscriber m_sub3;
		ros::Subscriber m_subyolo; //yolo
		std::string sub1;
		std::string sub2;
		std::string sub3;
		std::string subyolo;
		float min_dist; //m
			
	public:
		Interface():
			nh("~")
		{
			timer = nh.createTimer(ros::Duration(3), &Interface::pub_callback, this);
			cakes_pub = nh.advertise<geometry_msgs::PoseArray>("obstacle_position_array", 100);
			nh.param<float>("min_dist", min_dist, 0.08);
			if(nh.getParam("sub1_topic", sub1))
			{
				m_sub1 = nh.subscribe(sub1, 1, &Interface::sub_callback, this);
				std::cout << "init sub1" << std::endl;
			}
			if(nh.getParam("sub2_topic", sub2))
			{
				m_sub2 = nh.subscribe(sub2, 1, &Interface::sub_callback, this);
				std::cout << "init sub2" << std::endl;
			}
			if(nh.getParam("sub3_topic", sub3))
			{
				m_sub3 = nh.subscribe(sub3, 1, &Interface::sub_callback, this);
				std::cout << "init sub3" << std::endl;
			}
			if(nh.getParam("subyolo_topic", subyolo))
			{
				m_subyolo = nh.subscribe(subyolo, 1, &Interface::sub_callback, this);
				std::cout << "init subyolo" << std::endl;
			}
		}

		void sub_callback(const aruco_msgs::MarkerArray &msg)
		{
			for (std::size_t i=0; i < msg.markers.size(); i++)
			{
				Cake cake;
				cake.id = msg.markers[i].id;
				cake.tx = msg.markers[i].pose.pose.position.x;
				cake.ty = msg.markers[i].pose.pose.position.y;
				cake.tz = msg.markers[i].pose.pose.position.z;
				cake.rx = msg.markers[i].pose.pose.orientation.x;
				cake.ry = msg.markers[i].pose.pose.orientation.y;
				cake.rz = msg.markers[i].pose.pose.orientation.z;
				cake.rw = msg.markers[i].pose.pose.orientation.w;
				CakeCandidate.push_back(cake);
			}
		}

		void pub_callback(const ros::TimerEvent& event)
		{
			if(cakes_pub.getNumSubscribers()!=0)
			{
				filtercandidate(CakeCandidate, min_dist);
				std::size_t total = CakeCandidate.size();
				std::cout << "total: " << total << std::endl;
				geometry_msgs::PoseArray posearray_msg;
				for (std::size_t i=0; i < total; i++)
				{
					std::cout << CakeCandidate.at(i).tx <<std::endl;
					std::cout << CakeCandidate.at(i).ty <<std::endl;
					std::cout << CakeCandidate.at(i).tz <<std::endl;
					std::cout << "---------------------" <<std::endl;
					geometry_msgs::Pose p;
					p.position.x = CakeCandidate.at(i).tx;
					p.position.y = CakeCandidate.at(i).ty;
					p.position.z = CakeCandidate.at(i).tz;
					p.orientation.x = CakeCandidate.at(i).rx;
					p.orientation.y = CakeCandidate.at(i).ry;
					p.orientation.z = CakeCandidate.at(i).rz;
					p.orientation.w = CakeCandidate.at(i).rw;
					posearray_msg.poses.push_back(p);
				}
				posearray_msg.header.frame_id = "sample_camera";
				cakes_pub.publish(posearray_msg);
				CakeCandidate.clear();
			}
			else
			{
				CakeCandidate.clear();
				return;
			}
		}
		
		void filtercandidate(std::vector<Cake> &candidate, float &min_dist)
		{
			std::cout << "candidate number:" << candidate.size() << std::endl;
			for (std::size_t i=0; i < candidate.size(); i++)
			{
				for (std::size_t j = i+1; j < candidate.size(); j++)
				{
					float distance = pow(candidate[i].tx-candidate[j].tx,2) + pow(candidate[i].ty-candidate[j].ty,2) + pow(candidate[i].ty-candidate[j].ty,2);
					if (distance < pow(min_dist,2))
					{
						candidate[j].tx = (candidate[i].tx + candidate[j].tx)/2;
						candidate[j].ty = (candidate[i].ty + candidate[j].ty)/2;
						candidate[j].tz = (candidate[i].tz + candidate[j].tz)/2;
						candidate.erase(candidate.begin()+i);
						i--;
						continue;
					}
				}
			}
		}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cake_node");
	Interface node;
	ros::spin();
}

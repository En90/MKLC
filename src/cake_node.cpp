#include <ros/ros.h>
#include <vector>
#include <cmath>
//msgs_type
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <aruco_msgs/MarkerArray.h>
#include <aruco_msgs/Marker.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Bool.h>

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

// en add
class Point
{
	private:
		int y_new_flag = 0;
		int x_new_flag = 0;
		int y_old_flag = 0;
		int x_old_flag = 0;
		float k_x = 0.4;
		float k_y = 0.4;
		int x_change;
		int y_change;
		double threshold_lowpass = 0.1; // m
		double threshold_send = 0.05; // m
		float pub_x;
		float pub_y;
		bool big_change = false;
		ros::Time start_time;
		bool start = false;
    
	public:		
		float x;
        float y;
		int id = -1;
		float old_x = 0;
		float old_y = 0;
		float new_x = -1;
		float new_y = -1;
		int color_id;
		int disappeared_count = 0;

		Point(float in_x, float in_y, int in_id, int in_color_id)
		{
			x = in_x;
			y = in_y;
			id = in_id;
			color_id = in_color_id;
			old_x = in_x;
			old_y = in_y;
			pub_x = in_x;
			pub_y = in_y;
			start = false;
		}

		//  constructor with time
		Point(float in_x, float in_y, int in_id, int in_color_id, ros::Time begin)
		{
			x = in_x;
			y = in_y;
			id = in_id;
			color_id = in_color_id;
			old_x = in_x;
			old_y = in_y;
			pub_x = in_x;
			pub_y = in_y;
			start_time = begin;
			start = true;
			threshold_send = 0.1;	
		}

		// en add
		void low_pass_filter(double x, double y)
		{
			big_change = false;
			//x
			if(x - old_x > 0)
				x_new_flag = 1;
			else
				x_new_flag = 0;
			
			if(x_new_flag == x_old_flag)
			{
				if(abs(x - old_x) > threshold_lowpass)
				{
					x_change += 1;
					big_change = true;
				}
				
				if(x_change >= 2)
					k_x += 0.3;
			}
			else
			{
				x_change = 0;
				k_x = 0.5;
				x_old_flag = x_new_flag;
			}

			if(k_x > 0.95)
				k_x = 0.95;
			new_x = (1 - k_x) * (old_x) + (k_x) * x;
			old_x = new_x;
			//y
			if(y - old_y > 0)
				y_new_flag = 1;
			else
				y_new_flag = 0;
			
			if(y_new_flag == y_old_flag)
			{
				if(abs(y - old_y) > threshold_lowpass)
				{
					y_change += 1;
					big_change = true;
				}
				
				if(y_change >= 2)
					k_y += 0.3;
			}
			else
			{
				y_change = 0;
				k_y = 0.5;
				y_old_flag = y_new_flag;
			}

			if(k_y > 0.95)
				k_y = 0.95;
			new_y = (1 - k_y) * (old_y) + (k_y) * y;
			old_y = new_y;	
		}

		bool send()
		{
			adapt_threshold();
			double distance = sqrt( pow(pub_x - new_x, 2) + pow(pub_y - new_y, 2) );
			if(distance > threshold_send || big_change == true)
			{
				pub_x = new_x;
				pub_y = new_y;
				big_change = false;
				return true;
			}
			else
				return false;
		}

		void adapt_threshold()
		{
			if(start==false)
				return;
			else
			{
				ros::Time now = ros::Time::now();
				//threshold_send from 0.1(0 sec) to 0.05(50sec) 
				threshold_send =  0.1 - (now.toSec()-start_time.toSec())/100*0.05;
				if(threshold_send < 0.05)
					threshold_send = 0.05;
				
				if(now.toSec()-start_time.toSec() >= 100)
				{
					ROS_INFO("competition end");
					start=false;
				}
			}
		}
};

class Interface
{
	private:
		ros::NodeHandle nh;
		ros::Timer timer;
		ros::Timer cherry_timer;
		std::vector<Cake> CakeCandidate;
		// en add
		std::vector<Point> Idealpoint;
		std::vector<std::pair<double, std::pair<int, int>>> Memories;
		//dis Ideal_index Input_index

		ros::Publisher cakes_pub;
		ros::Publisher cherry_pub;
		ros::Publisher taste_pub;
		ros::Subscriber competition_start;
		ros::Subscriber m_sub1;
		ros::Subscriber m_sub2;
		ros::Subscriber m_sub3;
		ros::Subscriber m_subyolo; //yolo
		ros::Subscriber m_subyolo2; //yolo
		ros::Subscriber cherry_near_sub;
		ros::Subscriber cherry_far_sub;
		ros::Subscriber cherry_side_sub;
		std::string sub1;
		std::string sub2;
		std::string sub3;
		std::string subyolo;
		std::string subyolo2;
		float min_dist; //m

		int cherry0 = 1;
		int cherry1 = 1;
		int cherry2 = 1;
		int cherry3 = 1;
		bool cherry_change = false;

		// en add
		void init_idealpoint()
		{
			Idealpoint.clear();
			//1
			Point point1(1.125, 0.725, 0, 4);
			Idealpoint.push_back(point1);
			//2
			Point point2(1.125, 1.275, 1, 4);
			Idealpoint.push_back(point2);
			//3
			Point point3(1.875, 0.725, 2, 4);
			Idealpoint.push_back(point3);
			//4
			Point point4(1.875, 1.275, 3, 4);
			Idealpoint.push_back(point4);
			//5
			Point point5(0.775, 0.225, 4, 3);
			Idealpoint.push_back(point5);
			//6
			Point point6(0.775, 1.775, 5, 3);
			Idealpoint.push_back(point6);
			//7
			Point point7(2.225, 0.225, 6, 3);
			Idealpoint.push_back(point7);
			//8
			Point point8(2.225, 1.775, 7, 3);
			Idealpoint.push_back(point8);
			//9
			Point point9(0.575, 0.225, 8, 2);
			Idealpoint.push_back(point9);
			//10
			Point point10(0.575, 1.775, 9, 2);
			Idealpoint.push_back(point10);
			//11
			Point point11(2.425, 0.225, 10, 2);
			Idealpoint.push_back(point11);
			//12
			Point point12(2.425, 1.775, 11, 2);
			Idealpoint.push_back(point12);
		}

		// en add: with time
		void init_idealpoint(const ros::Time& begin)
		{
			Idealpoint.clear();
			//1
			Point point1(1.125, 0.725, 0, 4, begin);
			Idealpoint.push_back(point1);
			//2
			Point point2(1.125, 1.275, 1, 4, begin);
			Idealpoint.push_back(point2);
			//3
			Point point3(1.875, 0.725, 2, 4, begin);
			Idealpoint.push_back(point3);
			//4
			Point point4(1.875, 1.275, 3, 4, begin);
			Idealpoint.push_back(point4);
			//5
			Point point5(0.775, 0.225, 4, 3, begin);
			Idealpoint.push_back(point5);
			//6
			Point point6(0.775, 1.775, 5, 3, begin);
			Idealpoint.push_back(point6);
			//7
			Point point7(2.225, 0.225, 6, 3, begin);
			Idealpoint.push_back(point7);
			//8
			Point point8(2.225, 1.775, 7, 3, begin);
			Idealpoint.push_back(point8);
			//9
			Point point9(0.575, 0.225, 8, 2, begin);
			Idealpoint.push_back(point9);
			//10
			Point point10(0.575, 1.775, 9, 2, begin);
			Idealpoint.push_back(point10);
			//11
			Point point11(2.425, 0.225, 10, 2, begin);
			Idealpoint.push_back(point11);
			//12
			Point point12(2.425, 1.775, 11, 2, begin);
			Idealpoint.push_back(point12);
		}

		// en add
		void align()
		{
			if (CakeCandidate.size()==0)
			{
				for(auto &point : Idealpoint)
				{
					point.disappeared_count++;
					if(point.disappeared_count > 5)
					{
						point.new_x = -1;
						point.new_y = -1;
						point.disappeared_count = 0;
					}
				}
			}
			else
			{
				Memories.clear();
				std::set<int> used_Idealpoint;
				std::set<int> used_Cake;
				for(std::size_t i = 0; i < Idealpoint.size(); i++)
				{
					for(std::size_t j = 0; j < CakeCandidate.size(); j++)
					{
						if (CakeCandidate[j].id == 1 || Idealpoint[i].color_id != CakeCandidate[j].id)
						{
							continue;
						}
						double distance = sqrt( pow(CakeCandidate[j].tx - Idealpoint[i].x, 2) + pow(CakeCandidate[j].ty - Idealpoint[i].y, 2) );
						std::pair<int, int> counter(i, j);
						std::pair<double, std::pair<int, int>> Mem(distance, counter);
						Memories.push_back(Mem);
					}
				}
				if(Memories.size() != 0)
				{
					for(int i = 0; i < Memories.size()-1; i++)
					{
						for(int j = i + 1; j < Memories.size(); j++)
						{
							if (Memories[j].first < Memories[i].first)
							{
								auto temp = Memories[j];
								Memories[j] = Memories[i];
								Memories[i] = temp;
							}
						}
					}
				}
				for(auto &mem : Memories)
				{
					// Has not been assigned and is no more than 30 cm away from the ideal point
					if(!used_Cake.count(mem.second.second) && !used_Idealpoint.count(mem.second.first)) //&& (mem.first < 0.3)
					{
						Idealpoint[mem.second.first].low_pass_filter(CakeCandidate[mem.second.second].tx, CakeCandidate[mem.second.second].ty);
						Idealpoint[mem.second.first].disappeared_count = 0;
						used_Idealpoint.insert(mem.second.first);
						used_Cake.insert(mem.second.second);
					}
					else
						continue;
				}
				for(std::size_t i = 0; i < Idealpoint.size(); i++)
				{
					if(!used_Idealpoint.count(i))
					{
						Idealpoint[i].disappeared_count++;
						if(Idealpoint[i].disappeared_count > 2)
						{
							Idealpoint[i].new_x = -1;
							Idealpoint[i].new_y = -1;
							Idealpoint[i].disappeared_count = 0;
						}	
					}
				}
			}
			
			//CENTRAL TRACKING
			// else
			// {
			// 	std::vector<std::vector<double>> Distances;
			// 	for(auto &point : Idealpoint)
			// 	{
			// 		std::vector<double> temp_distance;
			// 		for(auto &cake : CakeCandidate)
			// 		{
			// 			double distance = sqrt( pow(cake.tx - point.old_x, 2) + pow(cake.ty - point.old_y, 2) );
			// 			temp_distance.push_back(distance);
			// 		}
			// 		Distances.push_back(temp_distance);
			// 		temp_distance.clear();
			// 	}
			// 	std::vector<int> Idealpoint_indexes;
			// 	std::vector<int> InputCake_indexes;
			// 	for(auto &v : Distances)
			// 	{
			// 		if(v.size() <= 1)
			// 		{
			// 			Idealpoint_indexes.push_back(v.size());
			// 			break;
			// 		}
			// 		int min = 1;
			// 		for(std::size_t i = 0; i < v.size(); i++)
			// 		{
			// 			if(v[i] < v[min]) min = i;
			// 		}
			// 		Idealpoint_indexes.push_back(min);
			// 	}
			// 	std::vector<std::vector<double>> Distances_sorted;
			// 	for(auto &v : Distances)
			// 	{
			// 		std::sort(v.begin(), v.end());
			// 		Distances_sorted.push_back(v);
			// 	}
			// }

			// 
			// if (CakeCandidate.size() == 0)
			// 	return;
			// for(std::size_t i = 0; i < Idealpoint.size(); i++)
			// {
			// 	int s;
			// 	double dis = 9999;
			// 	for(std::size_t j = 0; j < CakeCandidate.size(); j++)
			// 	{
			// 		std::cout << CakeCandidate[j].id << std::endl;
			// 		if (Idealpoint[i].color_id != CakeCandidate[j].id)
			// 			continue;
			// 		else
			// 		{
			// 			double distance = sqrt( pow(CakeCandidate[j].tx - Idealpoint[i].x, 2) + pow(CakeCandidate[j].ty - Idealpoint[i].y, 2) );
			// 			if (distance < dis)
			// 			{
			// 				s = j;
			// 				dis = distance;
			// 			}
			// 		}
			// 	}
			// 	if(dis != 9999)
			// 	{
			// 		Idealpoint[i].low_pass_filter(CakeCandidate[s].tx, CakeCandidate[s].ty);
			// 		double distance = sqrt( pow(Idealpoint[i].old_x - Idealpoint[i].new_x, 2) + pow(Idealpoint[i].old_y - Idealpoint[i].new_y, 2) );
			// 		if (distance > 0.08)
			// 		{
			// 			//publish
			// 		}	
			// 	}
			// }
		}
			
	public:
		Interface():
			nh("~")
		{
			timer = nh.createTimer(ros::Duration(2), &Interface::pub_callback, this);
			cherry_timer = nh.createTimer(ros::Duration(1), &Interface::cherry_pub_callback, this);
			cakes_pub = nh.advertise<geometry_msgs::PoseArray>("/cake_position_array", 10);
			cherry_pub = nh.advertise<std_msgs::Int32MultiArray>("/cherryExistence", 10);
			taste_pub = nh.advertise<geometry_msgs::Point>("/adjustCake", 15);
			competition_start = nh.subscribe("/startornot", 1, &Interface::start_callback, this);
			cherry_near_sub = nh.subscribe("/mklc_double/cherry_near", 1, &Interface::near_callback, this);
			cherry_far_sub = nh.subscribe("/mklc_double/cherry_far", 1, &Interface::far_callback, this);
			cherry_side_sub = nh.subscribe("/mklc_side/cherry_side", 1, &Interface::side_callback, this);
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
				m_subyolo = nh.subscribe(subyolo, 5, &Interface::sub_callback2, this);
				std::cout << "init subyolo" << std::endl;
			}
			if(nh.getParam("subyolo2_topic", subyolo2))
			{
				m_subyolo2 = nh.subscribe(subyolo2, 5, &Interface::sub_callback2, this);
				std::cout << "init subyolo2" << std::endl;
			}
			// en add: init Idealpoint
			init_idealpoint();
		}

		void sub_callback(const aruco_msgs::MarkerArray &msg)
		{
			for (std::size_t i=0; i < msg.markers.size(); i++)
			{
				if((msg.markers[i].id == 1)||(msg.markers[i].id == 2)||(msg.markers[i].id == 3))
				{
					Cake cake;
					cake.id = msg.markers[i].id+1;
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
		}

		void sub_callback2(const aruco_msgs::MarkerArray &msg)
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
			if(cakes_pub.getNumSubscribers()!=0 || taste_pub.getNumSubscribers()!=0)
			{
				filtercandidate(CakeCandidate, min_dist);
				align();
				for (int i = 0; i < Idealpoint.size() ; i++)
				{
					std::cout << Idealpoint[i].id << std::endl;
					std::cout << Idealpoint[i].new_x << ";" << Idealpoint[i].new_y << std::endl;
					std::cout << "================" << std::endl;
					if(Idealpoint[i].send())
					{
						ROS_INFO("send");
						geometry_msgs::Point point;
						point.x = Idealpoint[i].new_x;
						point.y = Idealpoint[i].new_y;
						point.z = Idealpoint[i].id;
						taste_pub.publish(point);
					}
				}

				std::size_t total = CakeCandidate.size();
				std::cout << "total: " << total << std::endl;
				geometry_msgs::PoseArray posearray_msg;
				for (std::size_t i=0; i < total; i++)
				{
					// std::cout << CakeCandidate.at(i).tx <<std::endl;
					// std::cout << CakeCandidate.at(i).ty <<std::endl;
					// std::cout << CakeCandidate.at(i).tz <<std::endl;
					// std::cout << "---------------------" <<std::endl;
					geometry_msgs::Pose p;
					p.position.x = CakeCandidate.at(i).tx;
					p.position.y = CakeCandidate.at(i).ty;
					p.position.z = CakeCandidate.at(i).tz;
					// if legendary recipe, z is 1;
					if( CakeCandidate.at(i).id == 0 )
						p.position.z = 1;
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
					float distance = pow(candidate[i].tx-candidate[j].tx,2) + pow(candidate[i].ty-candidate[j].ty,2);
					// cake1 pink2 yellow3 brown4
					// en add: both brown or yellow
					if ( distance < pow(min_dist,2) )
					{
						candidate[j].tx = (candidate[i].tx + candidate[j].tx)/2;
						candidate[j].ty = (candidate[i].ty + candidate[j].ty)/2;
						candidate[j].tz = (candidate[i].tz + candidate[j].tz)/2;
						// en add: aruco see pink but yolo see cake, any detection is a cake, it must be a cake
						if( (candidate[j].id == 1) || (candidate[i].id == 1) )
							candidate[j].id = 1;
						candidate.erase(candidate.begin()+i);
						i--;
						continue;
					}
				}
			}
		}

		void near_callback(const std_msgs::Int32 &msg)
		{
			if (msg.data != cherry0)
				cherry_change = true;
			cherry0 = msg.data;
		}

		void far_callback(const std_msgs::Int32MultiArray &msg)
		{
			if (msg.data.at(0) != cherry1 ||  msg.data.at(1) != cherry3)
				cherry_change = true;
			cherry1 = msg.data.at(0);
			cherry3 = msg.data.at(1); 
		}

		void side_callback(const std_msgs::Int32 &msg)
		{
			if (msg.data != cherry2)
				cherry_change = true;
			cherry2 = msg.data;
		}

		void cherry_pub_callback(const ros::TimerEvent& event)
		{
			if (cherry_change == true)
			{
				std_msgs::Int32MultiArray cherry_msg;
				cherry_msg.data.clear();
				// cherry_msg.data.push_back(1);
				cherry_msg.data.push_back(cherry0);
				cherry_msg.data.push_back(cherry1);
				cherry_msg.data.push_back(cherry2);
				cherry_msg.data.push_back(cherry3);
				cherry_pub.publish(cherry_msg);
				cherry_change = false;
			}
		}

		void start_callback(const std_msgs::Bool &msg)
		{
			if(msg.data == true)
			{
				ROS_INFO("start competision");
				init_idealpoint(ros::Time::now());
			}
		}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cake_node");
	Interface node;
	ros::spin();
}

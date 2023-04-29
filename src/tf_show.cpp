#include <geometry_msgs/PoseArray.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

class Show
{
	private:
		ros::NodeHandle nh;
        ros::Subscriber sub;
		tf::TransformBroadcaster br;

    public:
        Show():
			nh("~")
		{
			sub = nh.subscribe("/cake_node/obstacle_position_array", 1, &Show::callback, this);
        }

    void callback(const geometry_msgs::PoseArray &msg)
    {
        for(int i = 0; i < msg.poses.size(); i++)
		{
			tf::Transform t = tf::Transform(tf::Quaternion(msg.poses[i].orientation.x,
                            msg.poses[i].orientation.y,
                            msg.poses[i].orientation.z,
                            msg.poses[i].orientation.w),
                tf::Vector3(msg.poses[i].position.x, msg.poses[i].position.y, msg.poses[i].position.z));
			std::string name = "object";
			br.sendTransform(tf::StampedTransform(t, ros::Time::now(), "world_frame", name.append(std::to_string(i))));
		}
    }
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "tf_show");
	Show node;
	ros::spin();
}
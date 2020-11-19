#include <ros/ros.h>
#include <string>
#include "grid3d.hpp"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "grid3d_generator_node");  
	
	// Particle filter instance
	std::string node_name = "grid3d_generator_node";
	if(argc != 2){
		ROS_ERROR("You should give the .bt path as the first argument");
		exit(1);
	}
	std::string map_path = std::string(argv[1]);
	Grid3d pf(node_name, map_path);
  
	return 0;
}





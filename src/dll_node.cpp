#include <ros/ros.h>
#include <string>
#include "dllnode.hpp"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "dll_node");  
	
	// Particle filter instance
	std::string node_name = "dll_node";
	DLLNode node(node_name);
	
	// Process data at given rate
	ros::spin();

	return 0;
} 





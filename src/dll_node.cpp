#include <rclcpp/rclcpp.hpp>
#include <string>
#include "dllnode.hpp"

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv, "dll_node");  
	
	// Particle filter instance
	std::string node_name = "dll_node";
	DLLNode node(node_name);
	
	// Process data at given rate
	rclcpp::spin(node);

	rclcpp::shutdown();

	return 0;
} 





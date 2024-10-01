#include <rclcpp/rclcpp.hpp>
#include <string>
#include "dllnode.hpp"

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);  
	
	// Process data at given rate
	rclcpp::spin(std::make_shared<DLLNode>("dll_node"));

	rclcpp::shutdown();

	return 0;
} 





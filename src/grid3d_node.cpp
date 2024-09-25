#include <rclcpp/rclcpp.hpp>
#include <string>
#include <memory>
#include <chrono>
#include "grid3d.hpp"
 

class Grid3dNode : public rclcpp::Node 
{
	public:
	Grid3dNode(const std::string &node_name, const std::string map_path) : rclcpp::Node(node_name)
	{
		using namespace std::chrono_literals;

		m_grid=std::unique_ptr<Grid3d>(new Grid3d(map_path));
		//Build the msg with a slice of the grid if needed
		if(m_grid->m_gridSlice >= 0 && m_grid->m_gridSlice <=m_grid->m_maxZ)
		{
			m_grid->buildGridSliceMsg(m_grid->m_gridSlice);
			m_gridSlicePub = this->create_publisher<nav_msgs::msg::OccupancyGrid>(node_name+"/grid_slice", rclcpp::QoS(1).transient_local());
			this->create_wall_timer(1s/m_grid->m_publishGridSliceRate,std::bind(&Grid3dNode::publishGridSlice,this));	
		}
			// Setup point-cloud publisher
		if(m_grid->m_publishPc)
		{	
			m_pcPub = this->create_publisher<sensor_msgs::msg::PointCloud2>(node_name+"/map_point_cloud", rclcpp::QoS(1).transient_local());
			this->create_wall_timer(1s/m_grid->m_publishPointCloudRate,std::bind(&Grid3dNode::publishMapPointCloud,this));
		}
	}
	private:

	rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr m_gridSlicePub; 
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pcPub;
	std::unique_ptr <Grid3d> m_grid;
	void publishMapPointCloud(void)
	{
		m_grid->m_pcMsg.header.stamp = rclcpp::Node::now();
		m_pcPub->publish(m_grid->m_pcMsg);
	}
	
	void publishGridSlice(void)
	{
		m_grid->m_gridSliceMsg.header.stamp = rclcpp::Node::now();
		m_gridSlicePub->publish(m_grid->m_gridSliceMsg);
	}
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);  
	
	// Particle filter instance
	auto node = std::make_shared<rclcpp::Node>("grid3d_generator_node");
	if(argc != 2){
		RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"You should give the .bt path as the first argument");
		return(1);
	}
	std::string map_path = std::string(argv[1]);
	Grid3dNode grid3d(node->get_name(), map_path);
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}





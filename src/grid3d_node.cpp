#include <rclcpp/rclcpp.hpp>
#include <string>
#include <memory>
#include <chrono>
#include "grid3d.hpp"
 

class Grid3dNode : public rclcpp::Node 
{
	public:
	Grid3dNode(const std::string &node_name, const std::string &map_path) : rclcpp::Node(node_name)
	{

		this->declare_parameter("publish_grid_slice_rate",0.2);
		this->get_parameter("publish_grid_slice_rate",m_publishGridSliceRate);
		this->declare_parameter("publish_point_cloud_rate",0.2);
		this->get_parameter("publish_point_cloud_rate",m_publishPointCloudRate);
		m_grid=std::unique_ptr<Grid3d>(new Grid3d(map_path));

		RCLCPP_INFO(get_logger(), "Created Grid3D object.");
		//Build the msg with a slice of the grid if needed
		if(m_grid->m_gridSlice >= 0 && m_grid->m_gridSlice <=m_grid->m_maxZ)
		{
			if(m_publishGridSliceRate > 0)
			{
				m_grid->buildGridSliceMsg(m_grid->m_gridSlice);
				double periodGS=1/m_publishGridSliceRate;
				RCLCPP_INFO(this->get_logger(), "Period grid slice: %f", periodGS);
				m_gridSlicePub = this->create_publisher<nav_msgs::msg::OccupancyGrid>(node_name+"/grid_slice",10);
				if(periodGS<0){
					RCLCPP_ERROR(this->get_logger(), "Negative Period value GS: %f",periodGS);
				}else{
					this->create_wall_timer(std::chrono::duration<double>(periodGS),std::bind(&Grid3dNode::publishGridSlice,this));		
				}
			}else {
            	RCLCPP_ERROR(this->get_logger(), "Invalid m_gridSliceRate value: %f", m_publishGridSliceRate);
        	}
		}
			// Setup point-cloud publisher
		
		if(m_publishPointCloudRate > 0)
		{
			m_pcPub = this->create_publisher<sensor_msgs::msg::PointCloud2>(node_name+"/map_point_cloud",10);
			double periodPC=1/m_publishPointCloudRate;
			RCLCPP_INFO(this->get_logger(), "Period grid slice: %f", periodPC);
			if(periodPC<0){
				RCLCPP_ERROR(this->get_logger(), "Negative Period value PC: %f",periodPC);	
			}else
			{
				this->create_wall_timer(std::chrono::duration<double>(periodPC),std::bind(&Grid3dNode::publishMapPointCloud,this));
			}
		}else {
			RCLCPP_ERROR(this->get_logger(), "Invalid m_publishPointCloudRate value: %f",m_publishPointCloudRate);
		}
	}
	private:
	double m_publishPointCloudRate=0.2;
	double m_publishGridSliceRate=0.2;

	rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr m_gridSlicePub; 
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pcPub;
	std::unique_ptr <Grid3d> m_grid;
	void publishMapPointCloud()
	{
		m_grid->m_pcMsg.header.stamp = rclcpp::Node::now();
		m_pcPub->publish(m_grid->m_pcMsg);
	}
	
	void publishGridSlice()
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





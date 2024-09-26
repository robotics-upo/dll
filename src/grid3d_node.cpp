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
		using namespace std::chrono_literals;
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
				std::chrono::duration<double> periodGS(1/m_publishGridSliceRate);
				m_gridSlicePub = this->create_publisher<nav_msgs::msg::OccupancyGrid>(node_name+"/grid_slice",10);
				periodGS=1s;
				timergs_=this->create_wall_timer(periodGS,std::bind(&Grid3dNode::publishGridSlice,this));		
			}else {
            	RCLCPP_ERROR(this->get_logger(), "Invalid m_gridSliceRate value: %f", m_publishGridSliceRate);
        	}
		}		
		if(m_publishPointCloudRate > 0)
		{
			m_pcPub = this->create_publisher<sensor_msgs::msg::PointCloud2>(node_name+"/map_point_cloud",10);
			std::chrono::duration<double>periodPC(1/m_publishPointCloudRate);
			periodPC=1s;
			timerpc_=this->create_wall_timer(periodPC,std::bind(&Grid3dNode::publishMapPointCloud,this));
		}else {
			RCLCPP_ERROR(this->get_logger(), "Invalid m_publishPointCloudRate value: %f",m_publishPointCloudRate);
		}
	}
	private:
	double m_publishPointCloudRate=0.2;
	double m_publishGridSliceRate=0.2;
	rclcpp::TimerBase::SharedPtr timergs_;
	rclcpp::TimerBase::SharedPtr timerpc_;
	rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr m_gridSlicePub; 
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pcPub;
	std::unique_ptr <Grid3d> m_grid;
	
	void publishMapPointCloud()
	{
		RCLCPP_INFO(this->get_logger(),"Publishing Map PointCloud");
		m_grid->m_pcMsg.header.stamp = rclcpp::Node::now();
		m_pcPub->publish(m_grid->m_pcMsg);
	}
	
	void publishGridSlice()
	{
		RCLCPP_INFO(this->get_logger(),"Publishing Grid Slice");
		m_grid->m_gridSliceMsg.header.stamp = rclcpp::Node::now();
		m_gridSlicePub->publish(m_grid->m_gridSliceMsg);
	}
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);  
	
	// Particle filter instance
	if(argc != 2){
		RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"You should give the .bt path as the first argument");
		return(1);
	}
	std::string map_path = std::string(argv[1]);
	rclcpp::spin(std::make_shared<Grid3dNode>("grid3d_Node",map_path));
	rclcpp::shutdown();
	return 0;
}





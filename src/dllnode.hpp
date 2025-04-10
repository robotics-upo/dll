#ifndef __DLLNODE_HPP__
#define __DLLNODE_HPP__

#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <pcl_ros/transforms.hpp>
#include <pcl/point_types.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <vector>
#include "grid3d.hpp"
#include "dllsolver.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/transform_datatypes.h>
#include <time.h>
#include <chrono>
#include <thread>
using std::isnan;

// Forward declare functions needed in this header
namespace tf2 {
	void fromMsg(const geometry_msgs::msg::TransformStamped & in, tf2::Transform & out);
}
//Class definition
class DLLNode : public rclcpp::Node
{
public:
	//!Default contructor 
	DLLNode(const std::string &node_name) : rclcpp::Node(node_name)
	{	
		m_tfBr = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
		m_tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
		m_tfListener = std::make_unique<tf2_ros::TransformListener>(*m_tfBuffer);
		// Read node parameters
		this->declare_parameter("in_cloud","/pointcloud");
		this->declare_parameter("base_frame_id","base_link");
		this->declare_parameter("odom_frame_id","odom");
		this->declare_parameter("global_frame_id","map");
		this->declare_parameter("use_imu",false);
		this->declare_parameter("use_yaw_increments",false);
		this->declare_parameter("initial_x",0.0);
		this->declare_parameter("initial_y",0.0);
		this->declare_parameter("initial_z",0.0);
		this->declare_parameter("initial_a",0.0);
		this->declare_parameter("update_rate",10.0);	
		this->declare_parameter("update_min_d",0.1);
		this->declare_parameter("update_min_a",0.1);
		this->declare_parameter("update_min_time",1.0);
		this->declare_parameter("initial_z_offset",0.0);
		this->declare_parameter("align_method",1);
		this->declare_parameter("solver_max_iter",75);
		this->declare_parameter("solver_max_threads",8);
		this->declare_parameter("map_path","map.ot");
		this->declare_parameter("publish_point_cloud_rate",0.2);
		
		this->get_parameter("publish_point_cloud_rate",m_publishPointCloudRate);
		this->get_parameter("map_path",m_mapPath);
		this->get_parameter("in_cloud",m_inCloudTopic);
		this->get_parameter("base_frame_id",m_baseFrameId);
		this->get_parameter("odom_frame_id",m_odomFrameId);
		this->get_parameter("global_frame_id",m_globalFrameId);
		this->get_parameter("use_imu",m_use_imu);
		this->get_parameter("use_yaw_increments",m_useYawIncrements);

		m_roll_imu = m_pitch_imu = m_yaw_imu = 0.0;
		
		this->get_parameter("update_rate",m_updateRate);
		this->get_parameter("initial_x",m_initX);
		this->get_parameter("initial_y",m_initY);
		this->get_parameter("initial_z",m_initZ);
		this->get_parameter("initial_a",m_initA);
		this->get_parameter("update_min_d",m_dTh);
		this->get_parameter("update_min_a",m_aTh);
		this->get_parameter("update_min_time",m_tTh);
		this->get_parameter("initial_z_offset",m_initZOffset);
		this->get_parameter("align_method",m_alignMethod);
		this->get_parameter("solver_max_iter",m_solverMaxIter);
		this->get_parameter("solver_max_threads",m_solverMaxThreads);
		m_grid3d=std::make_unique<Grid3d>(m_mapPath);
		m_solver=std::make_unique<DLLSolver>(*m_grid3d);
		// Init internal variables
		m_init = false;
		m_doUpdate = false;
		m_tfCache = false;
		
		// Compute trilinear interpolation map 
		//m_grid3d.computeTrilinearInterpolation(); /* Now we compute the approximation online */
		RCLCPP_INFO(this->get_logger(), "Initializing dll: \n Odom_frame_id: %s base_frame_id: %s global_frame_id: %s",m_odomFrameId.c_str(),m_baseFrameId.c_str(),m_globalFrameId.c_str());
		// Setup solver parameters
		m_solver->setMaxNumIterations(m_solverMaxIter);
		m_solver->setMaxNumThreads(m_solverMaxThreads);

		// Launch subscribers
		m_pcSub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            m_inCloudTopic, rclcpp::QoS(1).reliability(rclcpp::ReliabilityPolicy::BestEffort), std::bind(&DLLNode::pointcloudCallback, this, std::placeholders::_1));
        m_initialPoseSub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "initial_pose", rclcpp::QoS(2).reliability(rclcpp::ReliabilityPolicy::BestEffort), std::bind(&DLLNode::initialPoseReceived, this, std::placeholders::_1));
        if (m_use_imu) {
            m_imuSub = this->create_subscription<sensor_msgs::msg::Imu>(
                "imu",rclcpp::QoS(1).reliability(rclcpp::ReliabilityPolicy::BestEffort), std::bind(&DLLNode::imuCallback, this, std::placeholders::_1));
        }

		// Time stamp for periodic update
		m_lastPeriodicUpdate = this->now();

		// Launch updater timer
		updateTimer = this->create_wall_timer(
            std::chrono::duration<double>(1.0/m_updateRate),
            std::bind(&DLLNode::checkUpdateThresholdsTimer, this));

		// Initialize TF from odom to map as identity
		m_lastGlobalTf.setIdentity();
				
		if(m_initX != 0 || m_initY != 0 || m_initZ != 0 || m_initA != 0)
		{
			tf2::Transform pose;
			tf2::Vector3 origin(m_initX, m_initY, m_initZ);
			tf2::Quaternion q;
			q.setRPY(0,0,m_initA);
			RCLCPP_INFO(this->get_logger(), "Initial Pose: X: %f, Y: %f, Z %f, YAW: %f", m_initX, m_initY, m_initZ, m_initA);
			pose.setOrigin(origin);
			pose.setRotation(q);
			
			while(!setInitialPose(pose)){
				RCLCPP_INFO(this->get_logger(), "Trying to set Initial Pose: ");
				sleep(1);
			}
			m_init = true;	
		}
		if(m_publishPointCloudRate > 0)
		{
			m_pcPub = this->create_publisher<sensor_msgs::msg::PointCloud2>(node_name+"/map_point_cloud",10);
			std::chrono::duration<double>periodPC(1/m_publishPointCloudRate);
			publishMapPointCloud();
			timerpc_=this->create_wall_timer(periodPC,std::bind(&DLLNode::publishMapPointCloud,this));
		}else {
			RCLCPP_ERROR(this->get_logger(), "Invalid m_publishPointCloudRate value: %f",m_publishPointCloudRate);
		}
	}

	//!Default destructor
	~DLLNode()
	{
	}
		
	//! Check motion and time thresholds for AMCL update
	bool checkUpdateThresholds()
	{
		// RCLCPP_INFO(this->get_logger(),"checkUpdateThresholds");
		// If the filter is not initialized then exit
		if(!m_init)
			return false;
					
		// Publish current TF from odom to map
		transformStamped.header.stamp=last_cloud_time;
		// transformStamped.header.stamp=this->get_clock()->now();
		transformStamped.header.frame_id=m_globalFrameId;
		transformStamped.child_frame_id=m_odomFrameId;
		transformStamped.transform.translation.x=m_lastGlobalTf.getOrigin().x();
		transformStamped.transform.translation.y=m_lastGlobalTf.getOrigin().y();
		transformStamped.transform.translation.z=m_lastGlobalTf.getOrigin().z();
		transformStamped.transform.rotation.x=m_lastGlobalTf.getRotation().x();
		transformStamped.transform.rotation.y=m_lastGlobalTf.getRotation().y();
		transformStamped.transform.rotation.z=m_lastGlobalTf.getRotation().z();
		transformStamped.transform.rotation.w=m_lastGlobalTf.getRotation().w();
		m_tfBr->sendTransform(transformStamped);
		
		// Compute odometric translation and rotation since last update 
		auto t = this->now();
		tf2::Transform odomTf;
		try
		{
			tf2::fromMsg(m_tfBuffer->lookupTransform(m_odomFrameId, m_baseFrameId, last_cloud_time),odomTf);


			tf2::Transform T = m_lastOdomTf.inverse() *odomTf;
		
			// Check translation threshold
			if(T.getOrigin().length() > m_dTh)
			{
            	//ROS_INFO("Translation update");
            	m_doUpdate = true;
				m_lastPeriodicUpdate = t;
				return true;
			}
		
			// Check yaw threshold
			double yaw, pitch, roll;
			T.getBasis().getRPY(roll, pitch, yaw);
			if(fabs(yaw) > m_aTh)
			{
            	//ROS_INFO("Rotation update");
				m_doUpdate = true;
				m_lastPeriodicUpdate = t;
				return true;
			}

		}
		catch (tf2::TransformException& ex)
		{
			//ROS_ERROR("DLL error: %s",ex.what());
			// return false;
		}
		
		// Check time threshold
		RCLCPP_INFO(this->get_logger(), "Time to update: %f", (t - m_lastPeriodicUpdate).seconds());
		if((t-m_lastPeriodicUpdate).seconds() > m_tTh)
		{
			RCLCPP_INFO(this->get_logger(), "Periodic update");
			m_doUpdate = true;
			m_lastPeriodicUpdate = t;
			return true;
		}
		
		return false;
	}

	void callSolver(std::vector<pcl::PointXYZ> &p, double tx, double ty, double tz, double roll, double pitch, double yaw, tf2::Stamped<tf2::Transform> odomTf) {
		m_thread = true;
		if(m_alignMethod == 1) { // DLL solver 
			m_solver->solve(p, tx, ty, tz, yaw);
		}
		else if(m_alignMethod == 2) // NDT solver
			m_grid3d->alignNDT(p, tx, ty, tz, yaw);
		else if(m_alignMethod == 3) // ICP solver
			m_grid3d->alignICP(p, tx, ty, tz, yaw);
		m_thread = false;
		RCLCPP_INFO(this->get_logger(),"Solved!");

		// Update global TF
		tf2::Quaternion q;
		q.setRPY(roll, pitch, yaw);
		m_lastGlobalTf = tf2::Transform(q, tf2::Vector3(tx, ty, tz))*odomTf.inverse();

		// Update time and transform information
		m_lastOdomTf = odomTf;
		m_doUpdate = false;
		RCLCPP_INFO(this->get_logger(),"TF actualizado");
	}
		                                   
private:
	void publishMapPointCloud()
	{
		RCLCPP_INFO(this->get_logger(),"Publishing Map PointCloud");
		m_grid3d->m_pcMsg.header.stamp = rclcpp::Node::now();
		m_grid3d->m_pcMsg.header.frame_id = m_globalFrameId;
		m_pcPub->publish(m_grid3d->m_pcMsg);
	}

	void checkUpdateThresholdsTimer()
	{
		
			checkUpdateThresholds();
	}

	void initialPoseReceived(const std::shared_ptr<const geometry_msgs::msg::PoseWithCovarianceStamped>& msg)
	{
		// We only accept initial pose estimates in the global frame
		if(msg->header.frame_id != m_globalFrameId)
		{
			RCLCPP_WARN(this->get_logger(),
			"Ignoring initial pose in frame \"%s\"; initial poses must be in the global frame, \"%s\"",
			msg->header.frame_id.c_str(),
			m_globalFrameId.c_str());
			return;	
		}
		
		// Transform into the global frame
		tf2::Transform pose;
		tf2::fromMsg(msg->pose.pose, pose);
		//ROS_INFO("Setting pose (%.6f): %.3f %.3f %.3f %.3f", ros::Time::now().toSec(), pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z(), getYawFromTf(pose));
		
		// Initialize the filter
		setInitialPose(pose);
	}
	
	//! IMU callback
	void imuCallback(const std::shared_ptr<const sensor_msgs::msg::Imu>& msg) 
	{
		double r = m_roll_imu;
		double p = m_pitch_imu;
		double y = m_yaw_imu;
		auto o = msg->orientation;
		tf2::Quaternion q;
		tf2::fromMsg(o, q);
		tf2::Matrix3x3 M(q);
		M.getRPY(m_roll_imu, m_pitch_imu, m_yaw_imu);
		if (isnan(m_roll_imu) || isnan(m_pitch_imu) || isnan(m_yaw_imu)) 
		{
			m_roll_imu = r;
			m_pitch_imu = p;
			m_yaw_imu = y;
		}
	}

	//! 3D point-cloud callback
	void pointcloudCallback(const std::shared_ptr<const sensor_msgs::msg::PointCloud2>& cloud)
	{	
		static double lastYaw_imu = -1000.0;
		double deltaYaw_imu = 0;
		last_cloud_time = cloud->header.stamp;
		last_cloud_time += rclcpp::Duration(std::chrono::microseconds(100000));
		// If the filter is not initialized then exit
		if(!m_init)
			return;
			
		// Check if an update must be performed or not
		if(!m_doUpdate)
			return;

		if (m_thread) {
			return;
		}

		// Compute odometric translation and rotation since last update 
		tf2::Stamped<tf2::Transform> odomTf;
		try
		{

			m_tfBuffer->canTransform(m_odomFrameId, m_baseFrameId, last_cloud_time);//waitForTransform(m_odomFrameId, m_baseFrameId, rclcpp::Time(0), std::chrono::seconds(1));
			tf2::fromMsg(m_tfBuffer->lookupTransform(m_odomFrameId, m_baseFrameId, last_cloud_time),odomTf);
		}
		catch (tf2::TransformException& ex)
		{
			RCLCPP_ERROR(this->get_logger(),"%s",ex.what());
			return;
		}
		tf2::Transform mapTf;
		mapTf = m_lastGlobalTf * odomTf;

		// Pre-cache transform for point-cloud to base frame and transform the pc
		if(!m_tfCache)
		{	
			try
			{
                m_tfBuffer->canTransform(m_baseFrameId,cloud->header.frame_id, last_cloud_time);//waitForTransform(m_baseFrameId, cloud->header.frame_id, rclcpp::Time(0), std::chrono::Duration(2.0))                tf2::fromMsg(m_tfBuffer->lookupTransform(m_baseFrameId, cloud->header.frame_id), m_pclTf);
				tf2::fromMsg(m_tfBuffer->lookupTransform(m_baseFrameId, cloud->header.frame_id,last_cloud_time),m_pclTf);
				m_tfCache = true;
			}
			catch (tf2::TransformException& ex)
			{
				RCLCPP_ERROR(this->get_logger(),"%s",ex.what());
				return;
			}
		}
		sensor_msgs::msg::PointCloud2 baseCloud;
		pcl_ros::transformPointCloud(m_baseFrameId, m_pclTf, *cloud, baseCloud);
		
		// PointCloud2 to PointXYZ conevrsion, with range limits [0,1000]
		std::vector<pcl::PointXYZ> downCloud;
		PointCloud2_to_PointXYZ(baseCloud, downCloud);
			
		// Get estimated position into the map
		double tx, ty, tz;
		tx = mapTf.getOrigin().getX();
		ty = mapTf.getOrigin().getY();
		tz = mapTf.getOrigin().getZ();
		
		// Get estimated orientation into the map
		double roll, pitch, yaw;
		if(m_use_imu)
		{
			// Get roll and pitch from IMU, yaw from TF
			double r, p;
		    mapTf.getBasis().getRPY(r, p, yaw);   
			roll = m_roll_imu;
			pitch = m_pitch_imu;

			// Get yaw increment from IMU, if so required
			if(m_useYawIncrements)
			{
				if(lastYaw_imu < -100.0)
					lastYaw_imu = m_yaw_imu;
				deltaYaw_imu = m_yaw_imu-lastYaw_imu;
				lastYaw_imu = m_yaw_imu;
			}
		}
		else
			mapTf.getBasis().getRPY(roll, pitch, yaw);
		
		// Tilt-compensate point-cloud according to roll and pitch
		static std::vector<pcl::PointXYZ> points;
		float cr, sr, cp, sp;
		float r00, r01, r02, r10, r11, r12, r20, r21, r22;
		sr = sin(roll);
		cr = cos(roll);
		sp = sin(pitch);
		cp = cos(pitch);
		r00 = cp; 	r01 = sp*sr; 	r02 = cr*sp;
		r10 =  0; 	r11 = cr;		r12 = -sr;
		r20 = -sp;	r21 = cp*sr;	r22 = cp*cr;
		points.resize(downCloud.size());
		for(size_t i=0; i<downCloud.size(); i++) 
		{
			float x = downCloud[i].x, y = downCloud[i].y, z = downCloud[i].z;
			points[i].x = x*r00 + y*r01 + z*r02;
			points[i].y = x*r10 + y*r11 + z*r12;
			points[i].z = x*r20 + y*r21 + z*r22;			
		}

		// Launch DLL solver
		double a = yaw;
		if(m_use_imu && m_useYawIncrements)
			a = yaw + deltaYaw_imu;

		if (!m_thread) {
			RCLCPP_INFO(this->get_logger(),"Solving");


			
			std::thread t(std::bind(&DLLNode::callSolver,this,points,tx,ty,tz, roll, pitch, a, odomTf));
			t.detach();
			
			
		}
		
	}
	
	//! Set the initial pose of the particle filter
	bool setInitialPose(tf2::Transform initPose)
	{
		// Extract TFs for future updates
		try
		{
			m_tfBuffer->canTransform(m_odomFrameId,m_baseFrameId,rclcpp::Time(0));//.waitForTransform(m_odomFrameId, m_baseFrameId, rclcpp::Time(0), rclcpp::chrono::Duration(1.0));
			tf2::fromMsg(m_tfBuffer->lookupTransform(m_odomFrameId, m_baseFrameId, rclcpp::Time(0)),m_lastOdomTf);
		}
		catch (tf2::TransformException& ex)
		{
			RCLCPP_ERROR(this->get_logger(),"%s",ex.what());
			return false;
		}

		// Get estimated orientation from IMU if available
		double roll, pitch, yaw;
		if(m_use_imu)
		{
			// Get roll and pitch from IMU, yaw from TF
			double r, p;
		    m_lastOdomTf.getBasis().getRPY(r, p, yaw);   
			roll = m_roll_imu;
			pitch = m_pitch_imu;
		}
		else
			m_lastOdomTf.getBasis().getRPY(roll, pitch, yaw);
		
		// Get position information from pose
		tf2::Vector3 t = initPose.getOrigin();
		yaw = getYawFromTf(initPose);
		
		// Update global TF
		tf2::Quaternion q;
		q.setRPY(roll, pitch, yaw);
		m_lastGlobalTf = tf2::Transform(q, tf2::Vector3(t.x(), t.y(), t.z()+m_initZOffset))*m_lastOdomTf.inverse();

		// Prepare next iterations		
		m_doUpdate = false;
		m_init = true;
		return true;
	}
	
	//! Return yaw from a given TF
	float getYawFromTf(tf2::Transform& pose)
	{
		double yaw, pitch, roll;
		
		pose.getBasis().getRPY(roll, pitch, yaw);
		
		return (float)yaw;
	}

	bool PointCloud2_to_PointXYZ(sensor_msgs::msg::PointCloud2 &in, std::vector<pcl::PointXYZ> &out)
	{		
		sensor_msgs::PointCloud2Iterator<float> iterX(in, "x");
		sensor_msgs::PointCloud2Iterator<float> iterY(in, "y");
		sensor_msgs::PointCloud2Iterator<float> iterZ(in, "z");
		out.clear();
		for(unsigned int i=0; i<in.width*in.height; i++, ++iterX, ++iterY, ++iterZ) 
		{
			pcl::PointXYZ p(*iterX, *iterY, *iterZ);
			float d2 = p.x*p.x + p.y*p.y + p.z*p.z;
			if(d2 > 1 && d2 < 10000)
				out.push_back(p);			
		}

		return true;
	}

	//! Indicates if the filter was initialized
	bool m_init;

	//! Use IMU flag
	bool m_use_imu, m_useYawIncrements;
	
	//! Indicates that the local transfrom for the pint-cloud is cached
	bool m_tfCache;
	tf2::Transform m_pclTf;
	
	//! Particles roll and pich (given by IMU)
	double m_roll_imu, m_pitch_imu, m_yaw_imu;
	
	//! Filter initialization
    double m_initX, m_initY, m_initZ, m_initA, m_initZOffset;
		
	//! Thresholds and params for filter updating
	double m_dTh, m_aTh, m_tTh;
	tf2::Transform m_lastOdomTf;
	tf2::Transform m_lastGlobalTf;
	bool m_doUpdate;
	double m_updateRate;
	int m_alignMethod, m_solverMaxIter, m_solverMaxThreads;
	rclcpp::Time m_lastPeriodicUpdate;
	rclcpp::Time last_cloud_time;	
	//! Node parameters
	std::string m_inCloudTopic;
	std::string m_baseFrameId;
	std::string m_odomFrameId;
	std::string m_globalFrameId;
	std::string m_mapPath;
	//! ROS msgs and data
	std::unique_ptr<tf2_ros::TransformBroadcaster> m_tfBr;
	std::unique_ptr<tf2_ros::Buffer> m_tfBuffer;
	std::unique_ptr<tf2_ros::TransformListener> m_tfListener;
	geometry_msgs::msg::TransformStamped transformStamped;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_pcSub;
	rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr m_initialPoseSub;
	rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imuSub;
	rclcpp::TimerBase::SharedPtr updateTimer;
	double m_publishPointCloudRate;
	rclcpp::TimerBase::SharedPtr timerpc_;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pcPub;
	//! 3D distance drid
   	//	Grid3d m_grid3d;
	std::unique_ptr <Grid3d> m_grid3d;

	//! Non-linear optimization solver
	std::unique_ptr <DLLSolver> m_solver;

	bool m_thread = false;
	

};
namespace tf2{

void fromMsg(const geometry_msgs::msg::TransformStamped & in, Transform & out)  {
	out.setOrigin(Vector3(in.transform.translation.x,
	              in.transform.translation.y,
				  in.transform.translation.z));
	Quaternion rot;
	fromMsg(in.transform.rotation, rot);
	out.setRotation(rot);
}
}

#endif



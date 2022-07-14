#ifndef __DLLNODE_HPP__
#define __DLLNODE_HPP__

#include <vector>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <vector>
#include "grid3d.hpp"
#include "dllsolver.hpp"
#include <time.h>

using std::isnan;

//Class definition
class DLLNode
{
public:

	//!Default contructor 
	DLLNode(std::string &node_name) : 
	m_grid3d(node_name), m_solver(m_grid3d)
	{		
		// Read node parameters
		ros::NodeHandle lnh("~");
		if(!lnh.getParam("in_cloud", m_inCloudTopic))
			m_inCloudTopic = "/pointcloud";	
		if(!lnh.getParam("base_frame_id", m_baseFrameId))
			m_baseFrameId = "base_link";	
		if(!lnh.getParam("odom_frame_id", m_odomFrameId))
			m_odomFrameId = "odom";	
		if(!lnh.getParam("global_frame_id", m_globalFrameId))
			m_globalFrameId = "map";	
		if (!lnh.getParam("use_imu", m_use_imu)) 
			m_use_imu = false;
		if (!lnh.getParam("use_yaw_increments", m_useYawIncrements)) 
			m_useYawIncrements = false;
		m_roll_imu = m_pitch_imu = m_yaw_imu = 0.0;
		
		// Read DLL parameters
		if(!lnh.getParam("update_rate", m_updateRate))
			m_updateRate = 10.0;
		if(!lnh.getParam("initial_x", m_initX))
			m_initX = 0.0;
		if(!lnh.getParam("initial_y", m_initY))
			m_initY = 0.0;
		if(!lnh.getParam("initial_z", m_initZ))
			m_initZ = 0.0;
		if(!lnh.getParam("initial_a", m_initA))
			m_initA = 0.0;	
		if(!lnh.getParam("update_min_d", m_dTh))
			m_dTh = 0.1;
		if(!lnh.getParam("update_min_a", m_aTh))
			m_aTh = 0.1;
		if (!lnh.getParam("update_min_time", m_tTh))
			m_tTh = 1.0;
	    if(!lnh.getParam("initial_z_offset", m_initZOffset))
            m_initZOffset = 0.0;  
		if(!lnh.getParam("align_method", m_alignMethod))
            m_alignMethod = 1;
		if(!lnh.getParam("solver_max_iter", m_solverMaxIter))
			m_solverMaxIter = 75;
		if(!lnh.getParam("solver_max_threads", m_solverMaxThreads))
			m_solverMaxThreads = 8;
		lnh.param("n_downsample", m_n_downsample, 1);
		ROS_INFO("Taking only one of each %d PC samples", m_n_downsample);
		
		// Init internal variables
		m_init = false;
		m_doUpdate = false;
		m_tfCache = false;
		
		// Compute trilinear interpolation map 
		//m_grid3d.computeTrilinearInterpolation(); /* Now we compute the approximation online */

		// Setup solver parameters
		m_solver.setMaxNumIterations(m_solverMaxIter);
		m_solver.setMaxNumThreads(m_solverMaxThreads);

		// Launch subscribers
		m_pcSub = m_nh.subscribe(m_inCloudTopic, 1, &DLLNode::pointcloudCallback, this);
		m_initialPoseSub = lnh.subscribe("initial_pose", 2, &DLLNode::initialPoseReceived, this);
		if(m_use_imu)
			m_imuSub = m_nh.subscribe("imu", 1, &DLLNode::imuCallback, this);

		// Time stamp for periodic update
		m_lastPeriodicUpdate = ros::Time::now();

		// Launch updater timer
		updateTimer = m_nh.createTimer(ros::Duration(1.0/m_updateRate), &DLLNode::checkUpdateThresholdsTimer, this);
		
		// Initialize TF from odom to map as identity
		m_lastGlobalTf.setIdentity();
				
		if(m_initX != 0 || m_initY != 0 || m_initZ != 0 || m_initA != 0)
		{
			tf::Pose pose;
			tf::Vector3 origin(m_initX, m_initY, m_initZ);
			tf::Quaternion q;
			q.setRPY(0,0,m_initA);

			pose.setOrigin(origin);
			pose.setRotation(q);
			
			setInitialPose(pose);
			m_init = true;
		}
	}

	//!Default destructor
	~DLLNode()
	{
	}
		
	//! Check motion and time thresholds for AMCL update
	bool checkUpdateThresholds()
	{
		// If the filter is not initialized then exit
		if(!m_init)
			return false;
					
		// Publish current TF from odom to map
		m_tfBr.sendTransform(tf::StampedTransform(m_lastGlobalTf, ros::Time::now(), m_globalFrameId, m_odomFrameId));
		
		// Compute odometric translation and rotation since last update 
		ros::Time t = ros::Time::now();
		tf::StampedTransform odomTf;
		try
		{
			m_tfListener.waitForTransform(m_odomFrameId, m_baseFrameId, ros::Time(0), ros::Duration(.0));
			m_tfListener.lookupTransform(m_odomFrameId, m_baseFrameId, ros::Time(0), odomTf);
		}
		catch (tf::TransformException ex)
		{
			//ROS_ERROR("DLL error: %s",ex.what());
			return false;
		}
		tf::Transform T = m_lastOdomTf.inverse()*odomTf;
		
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

		// Check time threshold
		if((t-m_lastPeriodicUpdate).toSec() > m_tTh)
		{
			//ROS_INFO("Periodic update");
			m_doUpdate = true;
			m_lastPeriodicUpdate = t;
			return true;
		}
		
		return false;
	}
		                                   
private:

	void checkUpdateThresholdsTimer(const ros::TimerEvent& event)
	{
		checkUpdateThresholds();
	}

	void initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
	{
		// We only accept initial pose estimates in the global frame
		if(msg->header.frame_id != m_globalFrameId)
		{
			ROS_WARN("Ignoring initial pose in frame \"%s\"; initial poses must be in the global frame, \"%s\"",
			msg->header.frame_id.c_str(),
			m_globalFrameId.c_str());
			return;	
		}
		
		// Transform into the global frame
		tf::Pose pose;
		tf::poseMsgToTF(msg->pose.pose, pose);
		//ROS_INFO("Setting pose (%.6f): %.3f %.3f %.3f %.3f", ros::Time::now().toSec(), pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z(), getYawFromTf(pose));
		
		// Initialize the filter
		setInitialPose(pose);
	}
	
	//! IMU callback
	void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) 
	{
		double r = m_roll_imu;
		double p = m_pitch_imu;
		double y = m_yaw_imu;
		auto o = msg->orientation;
		tf::Quaternion q;
		tf::quaternionMsgToTF(o, q);
		tf::Matrix3x3 M(q);
		M.getRPY(m_roll_imu, m_pitch_imu, m_yaw_imu);
		if (isnan(m_roll_imu) || isnan(m_pitch_imu) || isnan(m_yaw_imu)) 
		{
			m_roll_imu = r;
			m_pitch_imu = p;
			m_yaw_imu = y;
		}
	}

	//! 3D point-cloud callback
	void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud)
	{	
		static double lastYaw_imu = -1000.0;
		double deltaYaw_imu = 0;
	
		// If the filter is not initialized then exit
		if(!m_init)
			return;
			
		// Check if an update must be performed or not
		if(!m_doUpdate)
			return;

		// Compute odometric translation and rotation since last update 
		tf::StampedTransform odomTf;
		try
		{
			m_tfListener.waitForTransform(m_odomFrameId, m_baseFrameId, ros::Time(0), ros::Duration(1.0));
			m_tfListener.lookupTransform(m_odomFrameId, m_baseFrameId, ros::Time(0), odomTf);
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s",ex.what());
			return;
		}
		tf::Transform mapTf;
		mapTf = m_lastGlobalTf * odomTf;

		// Pre-cache transform for point-cloud to base frame and transform the pc
		if(!m_tfCache)
		{	
			try
			{
                m_tfListener.waitForTransform(m_baseFrameId, cloud->header.frame_id, ros::Time(0), ros::Duration(2.0));
                m_tfListener.lookupTransform(m_baseFrameId, cloud->header.frame_id, ros::Time(0), m_pclTf);
				m_tfCache = true;
			}
			catch (tf::TransformException ex)
			{
				ROS_ERROR("%s",ex.what());
				return;
			}
		}
		sensor_msgs::PointCloud2 baseCloud;
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
		std::vector<pcl::PointXYZ> points;
		float cr, sr, cp, sp, cy, sy, rx, ry;
		float r00, r01, r02, r10, r11, r12, r20, r21, r22;
		sr = sin(roll);
		cr = cos(roll);
		sp = sin(pitch);
		cp = cos(pitch);
		r00 = cp; 	r01 = sp*sr; 	r02 = cr*sp;
		r10 =  0; 	r11 = cr;		r12 = -sr;
		r20 = -sp;	r21 = cp*sr;	r22 = cp*cr;
		points.resize(downCloud.size());
		for(int i=0; i<downCloud.size(); i++) 
		{
			float x = downCloud[i].x, y = downCloud[i].y, z = downCloud[i].z;
			points[i].x = x*r00 + y*r01 + z*r02;
			points[i].y = x*r10 + y*r11 + z*r12;
			points[i].z = x*r20 + y*r21 + z*r22;			
		}

		// Launch DLL solver
		double a = yaw;
		if(m_use_imu && m_useYawIncrements)
			a = yaw+deltaYaw_imu;
		if(m_alignMethod == 1) // DLL solver
			m_solver.solve(points, tx, ty, tz, a);
		else if(m_alignMethod == 2) // NDT solver
			m_grid3d.alignNDT(points, tx, ty, tz, a);
		else if(m_alignMethod == 3) // ICP solver
			m_grid3d.alignICP(points, tx, ty, tz, a);
		yaw = a;
		
		// Update global TF
		tf::Quaternion q;
		q.setRPY(roll, pitch, yaw);
		m_lastGlobalTf = tf::Transform(q, tf::Vector3(tx, ty, tz))*odomTf.inverse();

		// Update time and transform information
		m_lastOdomTf = odomTf;
		m_doUpdate = false;
	}
	
	//! Set the initial pose of the particle filter
	void setInitialPose(tf::Pose initPose)
	{
		// Extract TFs for future updates
		try
		{
			m_tfListener.waitForTransform(m_odomFrameId, m_baseFrameId, ros::Time(0), ros::Duration(1.0));
			m_tfListener.lookupTransform(m_odomFrameId, m_baseFrameId, ros::Time(0), m_lastOdomTf);
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s",ex.what());
			return;
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
		tf::Vector3 t = initPose.getOrigin();
		yaw = getYawFromTf(initPose);
		
		// Update global TF
		tf::Quaternion q;
		q.setRPY(roll, pitch, yaw);
		m_lastGlobalTf = tf::Transform(q, tf::Vector3(t.x(), t.y(), t.z()+m_initZOffset))*m_lastOdomTf.inverse();

		// Prepare next iterations		
		m_doUpdate = false;
		m_init = true;
	}
	
	//! Return yaw from a given TF
	float getYawFromTf(tf::Pose& pose)
	{
		double yaw, pitch, roll;
		
		pose.getBasis().getRPY(roll, pitch, yaw);
		
		return (float)yaw;
	}

	bool PointCloud2_to_PointXYZ(sensor_msgs::PointCloud2 &in, std::vector<pcl::PointXYZ> &out)
	{		
		sensor_msgs::PointCloud2Iterator<float> iterX(in, "x");
		sensor_msgs::PointCloud2Iterator<float> iterY(in, "y");
		sensor_msgs::PointCloud2Iterator<float> iterZ(in, "z");
		out.clear();
		for(int i=0; i<in.width*in.height; i++, ++iterX, ++iterY, ++iterZ) 
		{
			pcl::PointXYZ p(*iterX, *iterY, *iterZ);
			float d2 = p.x*p.x + p.y*p.y + p.z*p.z;
			if(d2 > 1 && d2 < 10000 && i%m_n_downsample == 0)
				out.push_back(p);			
		}

		return true;
	}

	//! Indicates if the filter was initialized
	bool m_init;

	//! Use IMU flag
	bool m_use_imu, m_useYawIncrements;

	//! To take one of each n samples (defaults to 1 --> keep all samples)
	int m_n_downsample;
	
	//! Indicates that the local transfrom for the pint-cloud is cached
	bool m_tfCache;
	tf::StampedTransform m_pclTf;
	
	//! Particles roll and pich (given by IMU)
	double m_roll_imu, m_pitch_imu, m_yaw_imu;
	
	//! Filter initialization
    double m_initX, m_initY, m_initZ, m_initA, m_initZOffset;
		
	//! Thresholds and params for filter updating
	double m_dTh, m_aTh, m_tTh;
	tf::StampedTransform m_lastOdomTf;
	tf::Transform m_lastGlobalTf;
	bool m_doUpdate;
	double m_updateRate;
	int m_alignMethod, m_solverMaxIter, m_solverMaxThreads;
	ros::Time m_lastPeriodicUpdate;
		
	//! Node parameters
	std::string m_inCloudTopic;
	std::string m_baseFrameId;
	std::string m_odomFrameId;
	std::string m_globalFrameId;
	
	//! ROS msgs and data
	ros::NodeHandle m_nh;
	tf::TransformBroadcaster m_tfBr;
	tf::TransformListener m_tfListener;
    ros::Subscriber m_pcSub, m_initialPoseSub, m_imuSub;
	ros::Timer updateTimer;
	
	//! 3D distance drid
    Grid3d m_grid3d;
		
	//! Non-linear optimization solver
	DLLSolver m_solver;
};

#endif



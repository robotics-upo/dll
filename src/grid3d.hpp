#ifndef __GRID3D_HPP__
#define __GRID3D_HPP__

/**
 * @file prob_map.cpp
 * @brief This file includes the ROS node implementation.
 * @author Francisco J. Perez Grau and Fernando Caballero
 */

#include <sys/time.h>
#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <nav_msgs/OccupancyGrid.h>
#include <stdio.h> 

// PCL
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>

#define MAX(a,b) (((a)>(b))?(a):(b))
#define MIN(a,b) (((a)<(b))?(a):(b))


struct TrilinearParams
{
	float a0, a1, a2, a3, a4, a5, a6, a7;

	TrilinearParams(void)
	{
		a0 = a1 = a2 = a3 = a4 = a5 = a6 = a7 = 0.0;
	}

	double interpolate(double x, double y, double z)
	{
		return a0 + a1*x + a2*y + a3*z + a4*x*y + a5*x*z + a6*y*z + a7*x*y*z;
	}
};

class Grid3d
{
private:
	
	// Ros parameters
	ros::NodeHandle m_nh;
	bool m_saveGrid, m_publishPc;
	std::string m_mapPath, m_nodeName;
	std::string m_globalFrameId;
	float m_gridSlice;
	double m_publishPointCloudRate, m_publishGridSliceRate;
	
	// Octomap parameters
	float m_maxX, m_maxY, m_maxZ;
	float m_resolution, m_oneDivRes;
	octomap::OcTree *m_octomap;
	float m_offsetX, m_offsetY, m_offsetZ;
	
	// 3D probabilistic grid cell
	uint16_t *m_grid;
	uint64_t m_gridSize, m_gridSizeX, m_gridSizeY, m_gridSizeZ;
	uint64_t m_gridStepY, m_gridStepZ;
	
	// 3D point clound representation of the map
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud;
	pcl::KdTreeFLANN<pcl::PointXYZ> m_kdtree;
	
	// Visualization of the map as pointcloud
	sensor_msgs::PointCloud2 m_pcMsg;
	ros::Publisher m_pcPub;
	ros::Timer mapTimer;
			
	// Visualization of a grid slice as 2D grid map msg
	nav_msgs::OccupancyGrid m_gridSliceMsg;
	ros::Publisher m_gridSlicePub;
	ros::Timer gridTimer;

	// Trilinear approximation parameters (for each grid cell)
	TrilinearParams *m_triGrid;

	// ICP 
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> m_icp;

	// NDT 
	pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> m_ndt;
	
public:
	Grid3d(std::string &node_name) : m_cloud(new pcl::PointCloud<pcl::PointXYZ>), m_triGrid(NULL)
	{
	  
		// Load paraeters
		double value;
		ros::NodeHandle lnh("~");
		m_nodeName = node_name;
		if(!lnh.getParam("global_frame_id", m_globalFrameId))
			m_globalFrameId = "map";	
		if(!lnh.getParam("map_path", m_mapPath))
			m_mapPath = "map.ot";
		if(!lnh.getParam("publish_point_cloud", m_publishPc))
			m_publishPc = false;
		if(!lnh.getParam("publish_point_cloud_rate", m_publishPointCloudRate))
			m_publishPointCloudRate = 0.2;	
		if(!lnh.getParam("publish_grid_slice", value))
			value = -1.0;
		if(!lnh.getParam("publish_grid_slice_rate", m_publishGridSliceRate))
			m_publishGridSliceRate = 0.2;
		m_gridSlice = (float)value;
		
		// Load octomap 
		m_octomap = NULL;
		m_grid = NULL;
		if(loadOctomap(m_mapPath))
		{
			// Compute the point-cloud associated to the ocotmap
			computePointCloud();
			
			// Try to load tha associated grid-map from file
			std::string path;
			if(m_mapPath.compare(m_mapPath.length()-3, 3, ".bt") == 0)
				path = m_mapPath.substr(0,m_mapPath.find(".bt"))+".grid";
			if(m_mapPath.compare(m_mapPath.length()-3, 3, ".ot") == 0)
				path = m_mapPath.substr(0,m_mapPath.find(".ot"))+".grid";
			if(!loadGrid(path))
			{						
				// Compute the gridMap using kdtree search over the point-cloud
				std::cout << "Computing 3D occupancy grid. This will take some time..." << std::endl;
				computeGrid();
				std::cout << "\tdone!" << std::endl;
				
				// Save grid on file
				if(saveGrid(path))
					std::cout << "Grid map successfully saved on " << path << std::endl;
			}
			
			// Build the msg with a slice of the grid if needed
			if(m_gridSlice >= 0 && m_gridSlice <= m_maxZ)
			{
				buildGridSliceMsg(m_gridSlice);
				m_gridSlicePub = m_nh.advertise<nav_msgs::OccupancyGrid>(node_name+"/grid_slice", 1, true);
				gridTimer = m_nh.createTimer(ros::Duration(1.0/m_publishGridSliceRate), &Grid3d::publishGridSliceTimer, this);	
			}
			
			// Setup point-cloud publisher
			if(m_publishPc)
			{
				m_pcPub = m_nh.advertise<sensor_msgs::PointCloud2>(node_name+"/map_point_cloud", 1, true);
				mapTimer = m_nh.createTimer(ros::Duration(1.0/m_publishPointCloudRate), &Grid3d::publishMapPointCloudTimer, this);
			}
		}

		// Setup ICP
		m_icp.setMaximumIterations (50);
  		m_icp.setMaxCorrespondenceDistance (0.1);
  		m_icp.setRANSACOutlierRejectionThreshold (1.0);

		// Setup NDT
		m_ndt.setTransformationEpsilon (0.01);  // Setting minimum transformation difference for termination condition.
  		m_ndt.setStepSize (0.1);   // Setting maximum step size for More-Thuente line search.
  		m_ndt.setResolution (1.0);   //Setting Resolution of NDT grid structure (VoxelGridCovariance).
		m_ndt.setMaximumIterations (50);   // Setting max number of registration iterations.
	}

	Grid3d(std::string &node_name, std::string &map_path) : m_cloud(new pcl::PointCloud<pcl::PointXYZ>), m_triGrid(NULL)
	{
	  
		// Load paraeters
		double value;
		ros::NodeHandle lnh("~");
		m_nodeName = node_name;
		m_mapPath = map_path;
		// Load octomap 
		m_octomap = NULL;
		m_grid = NULL;
		
		if(loadOctomap(m_mapPath))
		{
			// Compute the point-cloud associated to the ocotmap
			computePointCloud();
			
			// Try to load tha associated grid-map from file
			std::string path;
			if(m_mapPath.compare(m_mapPath.length()-3, 3, ".bt") == 0)
				path = m_mapPath.substr(0,m_mapPath.find(".bt"))+".grid";
			if(m_mapPath.compare(m_mapPath.length()-3, 3, ".ot") == 0)
				path = m_mapPath.substr(0,m_mapPath.find(".ot"))+".grid";
			if(!loadGrid(path))
			{						
				// Compute the gridMap using kdtree search over the point-cloud
				std::cout << "Computing 3D occupancy grid. This will take some time..." << std::endl;
				computeGrid();
				std::cout << "\tdone!" << std::endl;
				
				// Save grid on file
				if(saveGrid(path))
					std::cout << "Grid map successfully saved on " << path << std::endl;
			}			
		}

		// Setup ICP
		m_icp.setMaximumIterations (50);
  		m_icp.setMaxCorrespondenceDistance (0.1);
  		m_icp.setRANSACOutlierRejectionThreshold (1.0);

		// Setup NDT
		m_ndt.setTransformationEpsilon (0.01);  // Setting minimum transformation difference for termination condition.
  		m_ndt.setStepSize (0.1);   // Setting maximum step size for More-Thuente line search.
  		m_ndt.setResolution (1.0);   //Setting Resolution of NDT grid structure (VoxelGridCovariance).
		m_ndt.setMaximumIterations (50);   // Setting max number of registration iterations.
	}

	~Grid3d(void)
	{
		if(m_octomap != NULL)
			delete m_octomap;
		if(m_grid != NULL)
			delete []m_grid;
		if(m_triGrid != NULL)
			delete []m_triGrid;
	}
  
	void publishMapPointCloud(void)
	{
		m_pcMsg.header.stamp = ros::Time::now();
		m_pcPub.publish(m_pcMsg);
	}
	
	void publishGridSlice(void)
	{
		m_gridSliceMsg.header.stamp = ros::Time::now();
		m_gridSlicePub.publish(m_gridSliceMsg);
	}
	
	bool isIntoMap(double x, double y, double z)
	{
		return (x >= 0.0 && y >= 0.0 && z >= 0.0 && x < m_maxX && y < m_maxY && z < m_maxZ);
	}

	double getPointDist(double x, double y, double z)
	{
		return m_grid[point2grid(x, y, z)]*0.01;
	}

	TrilinearParams getPointDistInterpolation(double x, double y, double z)
	{
		TrilinearParams r;
		if(x >= 0.0 && y >= 0.0 && z >= 0.0 && x < m_maxX && y < m_maxY && z < m_maxZ)
			r = m_triGrid[point2grid(x, y, z)];
		return r;
	}

	TrilinearParams computeDistInterpolation(const double x, const double y, const double z)
	{
		TrilinearParams r;

		if(isIntoMap(x, y, z) && isIntoMap(x+m_resolution, y+m_resolution, z+m_resolution))
		{
			// Get 3D point index
			uint64_t i = point2grid(x, y, z); 

			// Get neightbour values to compute trilinear interpolation
			float c000, c001, c010, c011, c100, c101, c110, c111;
			c000 = m_grid[i]*0.01; 
			c001 = m_grid[i+m_gridStepZ]*0.01; 
			c010 = m_grid[i+m_gridStepY]*0.01; 
			c011 = m_grid[i+m_gridStepY+m_gridStepZ]*0.01; 
			c100 = m_grid[i+1]*0.01; 
			c101 = m_grid[i+1+m_gridStepZ]*0.01; 
			c110 = m_grid[i+1+m_gridStepY]*0.01; 
			c111 = m_grid[i+1+m_gridStepY+m_gridStepZ]*0.01; 

			// Compute trilinear parameters
			const float div = -m_oneDivRes*m_oneDivRes*m_oneDivRes;
			float x0, y0, z0, x1, y1, z1;
			x0 = ((int)(x*m_oneDivRes))*m_resolution;
			x1 = x0+m_resolution;
			y0 = ((int)(y*m_oneDivRes))*m_resolution;
			y1 = y0+m_resolution;
			z0 = ((int)(z*m_oneDivRes))*m_resolution;
			z1 = z0+m_resolution;
			r.a0 = (-c000*x1*y1*z1 + c001*x1*y1*z0 + c010*x1*y0*z1 - c011*x1*y0*z0 
			+ c100*x0*y1*z1 - c101*x0*y1*z0 - c110*x0*y0*z1 + c111*x0*y0*z0)*div;
			r.a1 = (c000*y1*z1 - c001*y1*z0 - c010*y0*z1 + c011*y0*z0
			- c100*y1*z1 + c101*y1*z0 + c110*y0*z1 - c111*y0*z0)*div;
			r.a2 = (c000*x1*z1 - c001*x1*z0 - c010*x1*z1 + c011*x1*z0 
			- c100*x0*z1 + c101*x0*z0 + c110*x0*z1 - c111*x0*z0)*div;
			r.a3 = (c000*x1*y1 - c001*x1*y1 - c010*x1*y0 + c011*x1*y0 
			- c100*x0*y1 + c101*x0*y1 + c110*x0*y0 - c111*x0*y0)*div;
			r.a4 = (-c000*z1 + c001*z0 + c010*z1 - c011*z0 + c100*z1 
			- c101*z0 - c110*z1 + c111*z0)*div;
			r.a5 = (-c000*y1 + c001*y1 + c010*y0 - c011*y0 + c100*y1 
			- c101*y1 - c110*y0 + c111*y0)*div;
			r.a6 = (-c000*x1 + c001*x1 + c010*x1 - c011*x1 + c100*x0 
			- c101*x0 - c110*x0 + c111*x0)*div;
			r.a7 = (c000 - c001 - c010 + c011 - c100
			+ c101 + c110 - c111)*div;
		}

		return r;
	}

	bool computeTrilinearInterpolation(void)
	{
		// Delete existing parameters if the exists
		if(m_triGrid != NULL)
			delete []m_triGrid;
		
		// Reserve memory for the parameters
		m_triGrid = new TrilinearParams[m_gridSize];

		// Compute the distance to the closest point of the grid
		int ix, iy, iz;
		double count = 0.0;
		double size = m_gridSizeX*m_gridSizeY*m_gridSizeZ;
		double x0, y0, z0, x1, y1, z1;
		double div = -1.0/(m_resolution*m_resolution*m_resolution);
		for(iz=0, z0=0.0, z1=m_resolution; iz<m_gridSizeZ-1; iz++, z0+=m_resolution, z1+=m_resolution)
		{
			printf("Computing trilinear interpolation map: : %3.2lf%%        \r", count/size * 100.0);
			for(iy=0, y0=0.0, y1=m_resolution; iy<m_gridSizeY-1; iy++, y0+=m_resolution, y1+=m_resolution)
			{
				for(ix=0, x0=0.0, x1=m_resolution; ix<m_gridSizeX-1; ix++, x0+=m_resolution, x1+=m_resolution)
				{
					double c000, c001, c010, c011, c100, c101, c110, c111;
					TrilinearParams p;
					count++;

					c000 = m_grid[(ix+0) + (iy+0)*m_gridStepY + (iz+0)*m_gridStepZ]*0.01;
					c001 = m_grid[(ix+0) + (iy+0)*m_gridStepY + (iz+1)*m_gridStepZ]*0.01;
					c010 = m_grid[(ix+0) + (iy+1)*m_gridStepY + (iz+0)*m_gridStepZ]*0.01;
					c011 = m_grid[(ix+0) + (iy+1)*m_gridStepY + (iz+1)*m_gridStepZ]*0.01;
					c100 = m_grid[(ix+1) + (iy+0)*m_gridStepY + (iz+0)*m_gridStepZ]*0.01;
					c101 = m_grid[(ix+1) + (iy+0)*m_gridStepY + (iz+1)*m_gridStepZ]*0.01;
					c110 = m_grid[(ix+1) + (iy+1)*m_gridStepY + (iz+0)*m_gridStepZ]*0.01;
					c111 = m_grid[(ix+1) + (iy+1)*m_gridStepY + (iz+1)*m_gridStepZ]*0.01;
					
					p.a0 = (-c000*x1*y1*z1 + c001*x1*y1*z0 + c010*x1*y0*z1 - c011*x1*y0*z0 
					+ c100*x0*y1*z1 - c101*x0*y1*z0 - c110*x0*y0*z1 + c111*x0*y0*z0)*div;
					p.a1 = (c000*y1*z1 - c001*y1*z0 - c010*y0*z1 + c011*y0*z0
					- c100*y1*z1 + c101*y1*z0 + c110*y0*z1 - c111*y0*z0)*div;
					p.a2 = (c000*x1*z1 - c001*x1*z0 - c010*x1*z1 + c011*x1*z0 
					- c100*x0*z1 + c101*x0*z0 + c110*x0*z1 - c111*x0*z0)*div;
					p.a3 = (c000*x1*y1 - c001*x1*y1 - c010*x1*y0 + c011*x1*y0 
					- c100*x0*y1 + c101*x0*y1 + c110*x0*y0 - c111*x0*y0)*div;
					p.a4 = (-c000*z1 + c001*z0 + c010*z1 - c011*z0 + c100*z1 
					- c101*z0 - c110*z1 + c111*z0)*div;
					p.a5 = (-c000*y1 + c001*y1 + c010*y0 - c011*y0 + c100*y1 
					- c101*y1 - c110*y0 + c111*y0)*div;
					p.a6 = (-c000*x1 + c001*x1 + c010*x1 - c011*x1 + c100*x0 
					- c101*x0 - c110*x0 + c111*x0)*div;
					p.a7 = (c000 - c001 - c010 + c011 - c100
					+ c101 + c110 - c111)*div;

					m_triGrid[ix + iy*m_gridStepY + iz*m_gridStepZ] = p;
				}
			}
		}
		printf("Computing trilinear interpolation map: 100%%                               \n");

		return true;
	}

	bool alignICP(std::vector<pcl::PointXYZ> &p, double &tx, double &ty, double &tz, double &a)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr c (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ> Final;

		// Copy cloud into PCL struct
		c->width = p.size();
		c->height = 1;
		c->points.resize(c->width * c->height);
		for(unsigned int i=0; i<p.size(); i++)
		{
			c->points[i].x = p[i].x;
			c->points[i].y = p[i].y;
			c->points[i].z = p[i].z;
		}

		// Setup initial solution (poinc-cloud is tilt compensated)
  		Eigen::AngleAxisf initRotation (a, Eigen::Vector3f::UnitZ ());
  		Eigen::Translation3f initTranslation (tx, ty, tz);
  		Eigen::Matrix4f initGuess = (initTranslation * initRotation).matrix ();

		// Setup icp and perform alignement
		m_icp.setInputSource(c);
		m_icp.align(Final, initGuess);

		// Get solution
		Eigen::Matrix4f T = m_icp.getFinalTransformation();
		tx = T(0,3);
		ty = T(1,3);
		tz = T(2,3);
		a = atan2(T(1,0),T(0,0));

		return true;
	}

	bool alignNDT(std::vector<pcl::PointXYZ> &p, double &tx, double &ty, double &tz, double &a)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr c (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ> Final;

		// Copy cloud into PCL struct
		c->width = p.size();
		c->height = 1;
		c->points.resize(c->width * c->height);
		for(unsigned int i=0; i<p.size(); i++)
		{
			c->points[i].x = p[i].x;
			c->points[i].y = p[i].y;
			c->points[i].z = p[i].z;
		}

		// Setup initial solution (poinc-cloud is tilt compensated)
  		Eigen::AngleAxisf initRotation (a, Eigen::Vector3f::UnitZ ());
  		Eigen::Translation3f initTranslation (tx, ty, tz);
  		Eigen::Matrix4f initGuess = (initTranslation * initRotation).matrix ();

		// Downsample input cloud
		pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
		approximate_voxel_filter.setLeafSize (2.0, 2.0, 2.0);
		approximate_voxel_filter.setInputCloud (c);
		approximate_voxel_filter.filter (*filtered_cloud);

		// Setup icp and perform alignement
		m_ndt.setInputSource(filtered_cloud);
		m_ndt.align(Final, initGuess);

		// Get solution
		Eigen::Matrix4f T = m_ndt.getFinalTransformation();
		tx = T(0,3);
		ty = T(1,3);
		tz = T(2,3);
		a = atan2(T(1,0),T(0,0));

		return true;
	}

protected:

	void publishMapPointCloudTimer(const ros::TimerEvent& event)
	{
		publishMapPointCloud();
	}
	
	void publishGridSliceTimer(const ros::TimerEvent& event)
	{
		publishGridSlice();
	}

	bool loadOctomap(std::string &path)
	{
		// release previously loaded data
		if(m_octomap != NULL)
			delete m_octomap;
		if(m_grid != NULL)
			delete []m_grid;
		
		// Load octomap
		octomap::AbstractOcTree *tree;
		if(path.length() > 3 && (path.compare(path.length()-3, 3, ".bt") == 0))
		{
			octomap::OcTree* binaryTree = new octomap::OcTree(0.1);
			if (binaryTree->readBinary(path) && binaryTree->size() > 1)
				tree = binaryTree;
			else 
				return false;
		} 
		else if(path.length() > 3 && (path.compare(path.length()-3, 3, ".ot") == 0))
		{
			tree = octomap::AbstractOcTree::read(path);
			if(!tree)
				return false;
		}	
		else
			return false;
		
		/*
		// Load octomap
		octomap::AbstractOcTree *tree = octomap::AbstractOcTree::read(path);
		if(!tree)
			return false;*/
		m_octomap = dynamic_cast<octomap::OcTree*>(tree);
		std::cout << "Octomap loaded" << std::endl;

		// Check loading and alloc momery for the grid
		if(m_octomap == NULL)
		{
			std::cout << "Error: NULL octomap!!" << std::endl;
			return false;
		}
		
		// Get map parameters
		double minX, minY, minZ, maxX, maxY, maxZ, res;
		m_octomap->getMetricMin(minX, minY, minZ);
		m_octomap->getMetricMax(maxX, maxY, maxZ);
		res = m_octomap->getResolution();
		m_maxX = (float)(maxX-minX);
		m_maxY = (float)(maxY-minY);
		m_maxZ = (float)(maxZ-minZ);
		m_offsetX = minX;
		m_offsetY = minY;
		m_offsetZ = minZ;
		m_resolution = (float)res;
		m_oneDivRes = 1.0/m_resolution;
		std::cout << "Map size:\n\tx: " << minX << " to " << maxX << std::endl;
		std::cout << "\ty: " << minY << " to " << maxY << std::endl;
		std::cout << "\tz: " << minZ << " to " << maxZ << std::endl;
		std::cout << "\tRes: " << m_resolution << std::endl;
		
		return true;
	}
	
	bool saveGrid(std::string &fileName)
	{
		FILE *pf;
		
		// Open file
		pf = fopen(fileName.c_str(), "wb");
		if(pf == NULL)
		{
			std::cout << "Error opening file " << fileName << " for writing" << std::endl;
			return false;
		}
		
		// Write grid general info 
		int version = 3;
		fwrite(&version, sizeof(int), 1, pf);
		fwrite(&m_gridSize, sizeof(uint64_t), 1, pf);
		fwrite(&m_gridSizeX, sizeof(uint64_t), 1, pf);
		fwrite(&m_gridSizeY, sizeof(uint64_t), 1, pf);
		fwrite(&m_gridSizeZ, sizeof(uint64_t), 1, pf);
		fwrite(&m_offsetX, sizeof(float), 1, pf);
		fwrite(&m_offsetY, sizeof(float), 1, pf);
		fwrite(&m_offsetZ, sizeof(float), 1, pf);
		
		// Write grid cells
		fwrite(m_grid, sizeof(uint16_t), m_gridSize, pf);
		
		// Close file
		fclose(pf);
		
		return true;
	}
	
	bool loadGrid(std::string &fileName)
	{
		FILE *pf;
		
		// Open file
		pf = fopen(fileName.c_str(), "rb");
		if(pf == NULL)
		{
			std::cout << fileName << " not found!" << std::endl;
			return false;
		}
		
		// Write grid general info 
		int version;
		fread(&version, sizeof(int), 1, pf);
		if(version != 3)
		{
			std::cout << "Incorrect grid file encoding version. " << fileName << " has version " <<  version << ", version 3 required." << std::endl;
			return false;
		}
		fread(&m_gridSize, sizeof(uint64_t), 1, pf);
		fread(&m_gridSizeX, sizeof(uint64_t), 1, pf);
		fread(&m_gridSizeY, sizeof(uint64_t), 1, pf);
		fread(&m_gridSizeZ, sizeof(uint64_t), 1, pf);
		fread(&m_offsetX, sizeof(float), 1, pf);
		fread(&m_offsetY, sizeof(float), 1, pf);
		fread(&m_offsetZ, sizeof(float), 1, pf);
		m_gridStepY = m_gridSizeX;
		m_gridStepZ = m_gridSizeX*m_gridSizeY;
		
		// Write grid cells
		if(m_grid != NULL)
			delete []m_grid;
		m_grid = new uint16_t[m_gridSize];
		fread(m_grid, sizeof(uint16_t), m_gridSize, pf);
		
		// Close file
		fclose(pf);
		
		return true;
	}
	
	void computePointCloud(void)
	{
		// Get map parameters
		double minX, minY, minZ;
		m_octomap->getMetricMin(minX, minY, minZ);
		
		// Load the octomap in PCL for easy nearest neighborhood computation
		// The point-cloud is shifted to have (0,0,0) as min values
		int i = 0;
		m_cloud->width = m_octomap->size();
		m_cloud->height = 1;
		m_cloud->points.resize(m_cloud->width * m_cloud->height);
		for(octomap::OcTree::leaf_iterator it = m_octomap->begin_leafs(), end = m_octomap->end_leafs(); it != end; ++it)
		{
			if(it != NULL && m_octomap->isNodeOccupied(*it))
			{
				m_cloud->points[i].x = it.getX()-minX;
				m_cloud->points[i].y = it.getY()-minY;
				m_cloud->points[i].z = it.getZ()-minZ;
				i++;
			}
		}
		m_cloud->width = i;
		m_cloud->points.resize(i);
		
		// Create the point cloud msg for publication
		pcl::toROSMsg(*m_cloud, m_pcMsg);
		m_pcMsg.header.frame_id = m_globalFrameId;

		// Create the ICP object for future registrations aginst the map
		m_icp.setInputTarget(m_cloud);
	}
	
	void computeGrid(void)
	{
		// Alloc the 3D grid
		m_gridSizeX = (uint64_t)(m_maxX*m_oneDivRes);
		m_gridSizeY = (uint64_t)(m_maxY*m_oneDivRes); 
		m_gridSizeZ = (uint64_t)(m_maxZ*m_oneDivRes);
		m_gridSize = m_gridSizeX*m_gridSizeY*m_gridSizeZ;
		m_gridStepY = m_gridSizeX;
		m_gridStepZ = m_gridSizeX*m_gridSizeY;
		m_grid = new uint16_t[m_gridSize];

		// Setup kdtree
		m_kdtree.setInputCloud(m_cloud);

		// Compute the distance to the closest point of the grid
		#pragma omp parallel for num_threads(16) shared(m_grid) 
		for(int iz=0; iz<m_gridSizeZ; iz++)
		{
			uint64_t index;
			pcl::PointXYZ searchPoint;
			std::vector<int> pointIdxNKNSearch(1);
			std::vector<float> pointNKNSquaredDistance(1);
			printf("Processing z=%i out of %lu\n", iz, m_gridSizeZ);
			for(int iy=0; iy<m_gridSizeY; iy++)
			{
				for(int ix=0; ix<m_gridSizeX; ix++)
				{
					searchPoint.x = ix*m_resolution;
					searchPoint.y = iy*m_resolution;
					searchPoint.z = iz*m_resolution;
					index = ix + iy*m_gridStepY + iz*m_gridStepZ;
					if(m_kdtree.nearestKSearch(searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
						m_grid[index] = (uint16_t)(sqrt(pointNKNSquaredDistance[0])*100.0);
					else
						m_grid[index] = 0.0;
				}
			}
		}
		#pragma omp barrier
	}
	
	void buildGridSliceMsg(float z)
	{
		static int seq = 0;
		
		// Setup grid msg
		m_gridSliceMsg.header.frame_id = m_globalFrameId;
		m_gridSliceMsg.header.stamp = ros::Time::now();
		m_gridSliceMsg.header.seq = seq++;
		m_gridSliceMsg.info.map_load_time = ros::Time::now();
		m_gridSliceMsg.info.resolution = m_resolution;
		m_gridSliceMsg.info.width = m_gridSizeX;
		m_gridSliceMsg.info.height = m_gridSizeY;
		m_gridSliceMsg.info.origin.position.x = 0.0;
		m_gridSliceMsg.info.origin.position.y = 0.0;
		m_gridSliceMsg.info.origin.position.z = z;
		m_gridSliceMsg.info.origin.orientation.x = 0.0;
		m_gridSliceMsg.info.origin.orientation.y = 0.0;
		m_gridSliceMsg.info.origin.orientation.z = 0.0;
		m_gridSliceMsg.info.origin.orientation.w = 1.0;
		m_gridSliceMsg.data.resize(m_gridSizeX*m_gridSizeY);

		// Extract max distance
		int offset = (int)(z*m_oneDivRes)*m_gridSizeX*m_gridSizeY;
		int end = offset + m_gridSizeX*m_gridSizeY;
		float maxProb = -1.0;
		for(int i=offset; i<end; i++)
			if(m_grid[i] > maxProb)
				maxProb = m_grid[i];

		// Copy data into grid msg and scale the probability to [0,100]
		if(maxProb < 0.000001)
			maxProb = 0.000001;
		maxProb = 100.0/maxProb;
		for(int i=0; i<m_gridSizeX*m_gridSizeY; i++)
			m_gridSliceMsg.data[i] = (int8_t)(m_grid[i+offset]*maxProb);
	}
	
	inline uint64_t point2grid(const float &x, const float &y, const float &z)
	{
		return (uint64_t)(x*m_oneDivRes) + ((uint64_t)(y*m_oneDivRes))*m_gridStepY + ((uint64_t)(z*m_oneDivRes))*m_gridStepZ;
	}
};


#endif

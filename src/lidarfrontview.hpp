#ifndef __LIDARFRONTVIEW_HPP__
#define __LIDARFRONTVIEW_HPP__

#include <vector>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

struct pixelData
{
  bool occupied;
  float range;
  pcl::PointXYZ p;
};

class LidarFrontView
{
  public:

    // Image with bicubic params for each pixel
    pixelData **img;

    // 3D LIDAR parameters
    int _width; 
    int _height;
    double _fov; 
    double _fov_down;
    LidarFrontView(void) 
    {
      img = NULL;
      _width = 0;
      _height = 0; 
      _fov = 0; 
      _fov_down = 0;
    }

    LidarFrontView(int width, int height, double fov, double fov_down) 
    {
      init(width, height, fov, fov_down);
    }

    void init(int width, int height, double fov, double fov_down) 
    {
      _width = width;
      _height = height; 
      _fov = fov; 
      _fov_down = fov_down;
      img = new pixelData*[_height];
      img[0] = new pixelData[_width*_height];
      for(int i=1; i<_height; i++)
        img[i] = img[i-1]+_width;
    }

    ~LidarFrontView(void)
    {
      delete[] img[0];
      delete[] img;
    } 

    bool setPointCloud(std::vector<pcl::PointXYZ> &p)
    {
        // Initialize front view to zero 
        for(int i=0; i<_height; i++)
          for(int j=0; j<_width; j++)
            img[i][j].occupied = false;

        // Project every 3D point into the front view
        int outFov = 0, invalidRange = 0;
        float _1_fov = 1/_fov, _1_PI = 1/M_PI;
        for(unsigned int i=0; i<p.size(); i++)
        {
          // Get Spherical parameterization
          float r, pitch, yaw;
          r = sqrt(p[i].x*p[i].x + p[i].y*p[i].y + p[i].z*p[i].z);
          if(r < 1.0 || r > 100.0)
          {
            invalidRange++;
            continue;
          }
          pitch = asin(p[i].z/r);
          yaw = atan2(p[i].y, p[i].x);

          // Get projection pixel into image
          int u, v;
          u = round(_height*(1-(pitch+_fov_down)*_1_fov));
          v = round(_width*(0.5*(1+yaw*_1_PI)));
          if(u<0 || v<0 || u>=_height || v>=_width)
          {
            outFov++;
            continue;
          }

          // Stores data into the frontview pixel
          img[u][v].occupied = true;
          img[u][v].range = r;
          img[u][v].p = p[i];
        }

        return true;
    }

    bool getCloud(std::vector<pcl::PointXYZ> &v)
    {
      v.clear();
      pixelData *pData = img[0];
      for(int i=0; i<_width*_height; i++)
      {
        if(pData[i].occupied)
          v.push_back(pData[i].p); 
      }
    }
};


#endif


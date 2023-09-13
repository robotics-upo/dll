#ifndef __DLLSOLVER_HPP__
#define __DLLSOLVER_HPP__

#include <vector>
#include "ceres/ceres.h"
#include "glog/logging.h"
#include "grid3d.hpp"
#include <pcl/point_cloud.h>

using ceres::CostFunction;
using ceres::SizedCostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;
using ceres::HuberLoss;

class DLLCostFunction
  : public SizedCostFunction<1 /* number of residuals */,
                             4 /* size of first parameter */> 
{
 public:
    DLLCostFunction(double px, double py, double pz, Grid3d &grid, double weight = 1.0)
      : _px(px), _py(py), _pz(pz), _grid(grid), _weight(weight)
    {

    }

    virtual ~DLLCostFunction(void) 
    {

    }

    virtual bool Evaluate(double const* const* parameters,
                          double* residuals,
                          double** jacobians) const 
    {
        double tx = parameters[0][0];
        double ty = parameters[0][1];
        double tz = parameters[0][2];
        double a  = parameters[0][3];

        // Compute the residual
        TrilinearParams p;
        double sa, ca, nx, ny, nz, dxa, dya;
        double c0, c1, c2, c3, c4, c5, c6, c7;
        sa = sin(a);
        ca = cos(a);
        nx = ca*_px - sa*_py + tx;
        ny = sa*_px + ca*_py + ty;
        nz = _pz + tz;

        if(_grid.isIntoMap(nx, ny, nz))
        {
            p = _grid.computeDistInterpolation(nx, ny, nz);
            c0 = p.a0; c1 = p.a1; c2 = p.a2; c3 = p.a3; c4 = p.a4; c5 = p.a5; c6 = p.a6; c7 = p.a7;

            residuals[0] =  _weight*(c0 + c1*nx + c2*ny + c3*nz + c4*nx*ny + c5*nx*nz + c6*ny*nz + c7*nx*ny*nz);

            if (jacobians != NULL && jacobians[0] != NULL) 
            {
                double dxa, dya;
                dxa = _py*ca + _px*sa;
                dya = _px*ca - _py*sa;
                jacobians[0][0] = _weight*(c1 + c5*nz + c4*ny + c7*nz*ny);
                jacobians[0][1] = _weight*(c2 + c6*nz + c4*nx + c7*nz*nx);
                jacobians[0][2] = _weight*(c3 + c5*nx + c6*ny + c7*nx*ny);
                jacobians[0][3] = _weight*(c2*dya - c1*dxa + c4*dya*nx - c4*dxa*ny - c5*nz*dxa + c6*nz*dya + c7*nz*dya*nx - c7*nz*dxa*ny);
            }
        }
        else
        {
            residuals[0] =  0;
            if (jacobians != NULL && jacobians[0] != NULL) 
            {
                jacobians[0][0] = 0;
                jacobians[0][1] = 0;
                jacobians[0][2] = 0;
                jacobians[0][3] = 0;
            }
        }

        return true;
    }

  private:

    // Point to be evaluated
    double _px; 
    double _py; 
    double _pz;

    // Distance grid
    Grid3d &_grid;

    // Constraint weight factor
    double _weight;
};

class DLLSolver
{
  private:

    // Distance grid
    Grid3d &_grid;

    // Optimizer parameters
    int _max_num_iterations;
    int _max_num_threads;
    double _robusKernelScale;

  public:

    DLLSolver(Grid3d &grid) : _grid(grid)
    {
        google::InitGoogleLogging("DLLSolver");
        _max_num_iterations = 100;
        _max_num_threads = 8;
        _robusKernelScale = 5;
    }

    ~DLLSolver(void)
    {

    } 

    bool setMaxNumIterations(int n)
    {
        if(n>0)
        {
            _max_num_iterations = n;
            return true;
        } 
        else
            return false;
    }

    bool setMaxNumThreads(int n)
    {
        if(n>0)
        {
            _max_num_threads = n;
            return true;
        } 
        else
            return false;
    }

    bool setRobustKernelScale(double s)
    {
        if(s > 0.9)
        {
            _robusKernelScale = s;
            return true;
        } 
        else
            return false;
    }


    bool solve(std::vector<pcl::PointXYZ> &p, double &tx, double &ty, double &tz, double &yaw)
    {
        // Initial solution
        double x[4];
        x[0] = tx; x[1] = ty; x[2] = tz; x[3] = yaw; 

        // Build the problem.
        Problem problem;

        // Set up a cost funtion per point into the cloud
        double sa, ca, nx, ny, nz;
        sa = sin(yaw);
        ca = cos(yaw);
        for(unsigned int i=0; i<p.size(); i++)
        {
            // Compute position of the point into the grid according to initial transform
            nx = ca*p[i].x - sa*p[i].y + tx;
            ny = sa*p[i].x + ca*p[i].y + ty;
            nz = p[i].z + tz;

            // Remove points out of the grid-map
            if(_grid.isIntoMap(nx, ny, nz))
            {           
                CostFunction* cost_function = new DLLCostFunction(p[i].x, p[i].y, p[i].z, _grid);
                problem.AddResidualBlock(cost_function, new ceres::CauchyLoss(0.1), x); 
            }
        }

        // Run the solver!
        Solver::Options options;
        options.minimizer_progress_to_stdout = false;
        options.linear_solver_type = ceres::DENSE_QR;
        options.max_num_iterations = _max_num_iterations;
        options.num_threads = _max_num_threads; 
        Solver::Summary summary;
        Solve(options, &problem, &summary);

        // Get the solution
        tx = x[0]; ty = x[1]; tz = x[2]; yaw = x[3];

        return true; 
    }
};




#endif

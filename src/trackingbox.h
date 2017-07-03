#ifndef TRACKINGBOX_H
#define TRACKINGBOX_H

#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include <pcl/point_types.h>
#include <pcl/tracking/tracking.h>
#include <pcl/features/normal_3d_omp.h>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/eigen.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "csfeature.h"
#include "csgpu.h"
#include "common.h"

using namespace pcl;
using namespace Eigen;
using namespace std;
using namespace tracking;
using namespace pcl;

class trackingbox
{
public:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr model, m_scene, m_transformedModel;
    tracking::ParticleXYZRPY m_poseForVis;
    trackingbox();
    ~trackingbox();
    void run(pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene);
    void setModel();
    void initSamples(tracking::ParticleXYZRPY initpose);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getTransformedModel();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getRoi();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getParticleCloud();
    Affine3f getPose();
    void resample(float varDist, float varAng, bool firstRun);
    void weight();
    void refinement();
    float weightParticle(tracking::ParticleXYZRPY hyppose);
    float weightParticle(Affine3f hyppose);
    void getXYZQ(float &tx, float &ty, float &tz, float &qx, float &qy, float &qz, float &qw);;
    
    std::vector<float> weightGPU();
    std::vector<float> weightCPU();
    CsFeature  m_modelF;
    CsFeatureEstimation csest;
    
private:
   boost::mt19937 m_gen;                         // for random number
   std::string modelfile = "/home/dhri-dz/rosbuild_ws/realtrack/realtrack/kamille.pcd";
   Affine3f initpose;
   tracking::ParticleXYZRPY  m_finalParticle, m_lastFinalParticle,
                             m_lastlastFinalParticle,m_lastlastlastFinalParticle,
			     m_vel,m_lastvel,m_acc,m_velerr,m_velSum;
   
   tracking::ParticleXYZRPY sampleWithVar(tracking::ParticleXYZRPY part, float varDist, 
					  float varAng, tracking::ParticleXYZRPY motion, bool motionmode);
   std::vector<tracking::ParticleXYZRPY> m_particles, m_lastspeed, m_bestspeed;
   int m_cnt;
   int m_particlenum;           //particle number
   float m_modelsize;
   float m_effective;
   float minDist, maxDist;
   MatrixXf m_noise;
   VectorXf m_velerrV;
   MatrixXf m_normTransform;
   
   CSGPU m_gpuest;
   
   Quaternion<float> m_lastq, m_lastlastq, m_speedq;
   CloudPtr m_lastWholeScene;
     
};

#endif
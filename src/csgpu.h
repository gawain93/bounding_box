#ifndef CSGPU_H
#define CSGPU_H

#include <vector>
#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_runtime_api.h>
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>



//typedef pcl::PointXYZRGB PointT;
//typedef pcl::PointCloud<PointT> Cloud;
//typedef Cloud::Ptr CloudPtr;
//typedef Cloud::ConstPtr CloudConstPtr;

//typedef pcl::PointXYZRGBNormal PointNT;
//typedef pcl::PointCloud<PointNT> CloudN;
//typedef CloudN::Ptr CloudNPtr;

class CSGPU
{
public:
    CSGPU() {}
    ~CSGPU() {}
    void loadData(float gridsize_, int xnr_, int ynr_, int znr_, int partnr_);
    int aa;
    int histsize;
    int modelsize;
    size_t partNr, cloudSize, refcloudSize;
    size_t xNr, yNr, zNr;
    float3 minPt, maxPt;
    float gridsize;
    float *refhist;                        // color spatial feature of the  model, as reference
//    thrust::device_vector<float3> cloudpos;
    float3 *cloudpos;
    float3 *cloudhsv;
    float3 *refcloud;
    float3 *partpos;
    float3 *partrot;


    float3 *d_cloudpos, *d_cloudhsv, *d_partpos, *d_partrot;
    float *d_weights, *weights;

    //gpu varibale
    float *d_refhist;         // pointer in cuda for reference feature of the model

    void uploadCurrentCloud(int cloudSize_);
    void uploadRefHist();
    std::vector<float> compute();

    void reloadData(float gridsize_, int xnr_, int ynr_, int znr_, int partnr_);
    void reuploadRefHist();
};

#endif // CSGPU_H

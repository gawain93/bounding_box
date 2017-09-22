#include "csfeature.h"


CsFeature::CsFeature()
{
}

CsFeature::~CsFeature()
{
}

CsFeatureEstimation::CsFeatureEstimation()
{
    ros::param::get("/gpu/gridsize", m_gridsize);
}

CsFeatureEstimation::~CsFeatureEstimation()
{
}


void CsFeatureEstimation::initBox(CloudPtr &cloud)
{
    pcl::getMinMax3D(*cloud, m_boxMin, m_boxMax);
    float extra = 0.0f;
    m_boxMin[0]-=extra; m_boxMin[1]-=extra; m_boxMin[2]-=extra;
    m_boxMax[0]+=extra; m_boxMax[1]+=extra; m_boxMax[2]+=extra;
    m_xNr = static_cast<int>( ceil((m_boxMax[0] - m_boxMin[0])/m_gridsize) );
    m_yNr = static_cast<int>( ceil((m_boxMax[1] - m_boxMin[1])/m_gridsize) );
    m_zNr = static_cast<int>( ceil((m_boxMax[2] - m_boxMin[2])/m_gridsize) );
    m_gridNr = m_xNr * m_yNr * m_zNr;
}

CsFeature CsFeatureEstimation::compute(CloudPtr &cloud)
{
    outF.hist.clear();
    double lastT = pcl::getTime ();
    outF.hist.resize((m_xNr+1) * (m_yNr+1) * (m_zNr+1) * 8);

    std::fill(outF.hist.begin(), outF.hist.end(), 0);
    for (size_t i=0; i<cloud->points.size(); ++i){                       // color feature for every point in the model
        PointT pt = cloud->points[i];
        int xInd, yInd, zInd, theInd;
        xInd = (int)floor( (pt.x - m_boxMin[0])/m_gridsize );
        if((xInd >= m_xNr) || (xInd<0)) {continue;}
        yInd = (int)floor( (pt.y - m_boxMin[1])/m_gridsize );
        if((yInd >= m_yNr) || (yInd<0)) {continue;}
        zInd = (int)floor( (pt.z - m_boxMin[2])/m_gridsize );
        if((zInd >= m_zNr) || (zInd<0)) {continue;}
        theInd = zInd*m_xNr*m_yNr + xInd*m_yNr + yInd;
        pt.x = pt.x - m_gridsize*(float)xInd - m_boxMin[0];
        pt.y = pt.y - m_gridsize*(float)yInd - m_boxMin[1];
        pt.z = pt.z - m_gridsize*(float)zInd - m_boxMin[2];

        float h,s,v;
        rgbTohsv(pt.r, pt.g, pt.b, h, s, v);
        int grayindex;
        float weightgray, weightcolor;
        int hindex1,hindex2;
        float hweight1, hweight2;
        h = h*360.0f;
        if(h>=0   && h <=60)  {hindex1=0; hindex2=1; hweight2= h/60.0f;          hweight1=1.0f -hweight2;}
        if(h>60  && h <=120)  {hindex1=1; hindex2=2; hweight2= (h-60.0f)/60.0f;  hweight1=1.0f -hweight2;}
        if(h>120 && h <=180)  {hindex1=2; hindex2=3; hweight2= (h-120.0f)/60.0f; hweight1=1.0f -hweight2;}
        if(h>180 && h <=240)  {hindex1=3; hindex2=4; hweight2= (h-180.0f)/60.0f; hweight1=1.0f -hweight2;}
        if(h>240 && h <=300)  {hindex1=4; hindex2=5; hweight2= (h-240.0f)/60.0f; hweight1=1.0f -hweight2;}
        if(h>300 && h <=360)  {hindex1=5; hindex2=0; hweight2= (h-300.0f)/60.0f; hweight1=1.0f -hweight2;}
        if(v<0.5) {grayindex = 6;}
        if(v>=0.5){grayindex = 7;}
        weightcolor = std::pow(s,(0.14f * std::pow(1.0f/v,0.9f)));
        weightgray  = 1.0f - weightcolor;
        if( v<0.2f || s<0.1f){weightcolor=0.0f; weightgray=1.0f;}

//        weightcolor=0.f; weightgray=1.f; grayindex=7;


        hweight1 = weightcolor * hweight1;
        hweight2 = weightcolor * hweight2;

        float x=pt.x, x_=m_gridsize-pt.x, y=pt.y, y_=m_gridsize-pt.y, z= pt.z, z_=m_gridsize-pt.z;
        float dist0 = sqrt(x*x + y*y +z*z);
        int xPInd = zInd*m_xNr*m_yNr + (xInd+1)*m_yNr + yInd;  float distxP = sqrt(x_*x_ + y*y +z*z) ;
        int xyPInd = zInd*m_xNr*m_yNr + (xInd+1)*m_yNr + yInd+1; float distxyP = sqrt(x_*x_ + y_*y_ +z*z);
        int xzPInd = (zInd+1)*m_xNr*m_yNr + (xInd+1)*m_yNr + yInd; float distxzP = sqrt(x_*x_ + y*y +z_*z_);
        int xyzPInd = (zInd+1)*m_xNr*m_yNr + (xInd+1)*m_yNr + yInd+1; float distxyzP = sqrt(x_*x_ + y_*y_ +z_*z_);
        int yPInd = zInd*m_xNr*m_yNr + xInd*m_yNr + (yInd+1); float distyP = sqrt(x*x + y_*y_ +z*z);
        int yzPInd = (zInd+1)*m_xNr*m_yNr + xInd*m_yNr + (yInd+1); float distyzP = sqrt(x*x + y_*y_ +z_*z_);
        int zPInd = (zInd+1)*m_xNr*m_yNr + xInd*m_yNr + yInd;  float distzP = sqrt(x*x + y*y +z_*z_);

        float incrValue =1.f; float weight; int offset; float bandwidth=m_gridsize;
        weight = max(0.f, (bandwidth-dist0))/m_gridsize;  offset = 8*theInd;
        outF.hist[hindex1 + offset]     += hweight1*weight*incrValue;
        outF.hist[hindex2 + offset]     += hweight2*weight*incrValue;
        outF.hist[grayindex + offset]   += weightgray*weight*incrValue;

        weight = max(0.f, (bandwidth-distxP))/m_gridsize;  offset = 8*xPInd;
        outF.hist[hindex1 + offset]     += hweight1*weight*incrValue;
        outF.hist[hindex2 + offset]     += hweight2*weight*incrValue;
        outF.hist[grayindex + offset]   += weightgray*weight*incrValue;

        weight = max(0.f, (bandwidth-distxyP))/m_gridsize;  offset = 8*xyPInd;
        outF.hist[hindex1 + offset]     += hweight1*weight*incrValue;
        outF.hist[hindex2 + offset]     += hweight2*weight*incrValue;
        outF.hist[grayindex + offset]   += weightgray*weight*incrValue;

        weight = max(0.f, (bandwidth-distxzP))/m_gridsize;  offset = 8*xzPInd;
        outF.hist[hindex1 + offset]     += hweight1*weight*incrValue;
        outF.hist[hindex2 + offset]     += hweight2*weight*incrValue;
        outF.hist[grayindex + offset]   += weightgray*weight*incrValue;

        weight = max(0.f, (bandwidth-distxyzP))/m_gridsize;  offset = 8*xyzPInd;
        outF.hist[hindex1 + offset]     += hweight1*weight*incrValue;
        outF.hist[hindex2 + offset]     += hweight2*weight*incrValue;
        outF.hist[grayindex + offset]   += weightgray*weight*incrValue;

        weight = max(0.f, (bandwidth-distyP))/m_gridsize;  offset = 8*yPInd;
        outF.hist[hindex1 + offset]     += hweight1*weight*incrValue;
        outF.hist[hindex2 + offset]     += hweight2*weight*incrValue;
        outF.hist[grayindex + offset]   += weightgray*weight*incrValue;

        weight = max(0.f, (bandwidth-distyzP))/m_gridsize;  offset = 8*yzPInd;
        outF.hist[hindex1 + offset]     += hweight1*weight*incrValue;
        outF.hist[hindex2 + offset]     += hweight2*weight*incrValue;
        outF.hist[grayindex + offset]   += weightgray*weight*incrValue;

        weight = max(0.f, (bandwidth-distzP))/m_gridsize;  offset = 8*zPInd;
        outF.hist[hindex1 + offset]     += hweight1*weight*incrValue;
        outF.hist[hindex2 + offset]     += hweight2*weight*incrValue;
        outF.hist[grayindex + offset]   += weightgray*weight*incrValue;
    }
    return outF;
}

float CsFeatureEstimation::evalPoints(CloudPtr &cloud)
{
    float score = 0.f;
    for (size_t i=0; i<cloud->points.size(); i++){
        score += evalOnePoint(cloud->points[i]);
    }
    score /= float(cloud->points.size());
//    score /= outF.modelPointSize;
    return score;
}


float CsFeatureEstimation::evalOnePoint(PointT pt)
{
    float h,s,v;
    rgbTohsv(pt.r, pt.g, pt.b, h, s, v);
    int grayindex;
    float weightgray, weightcolor;
    int hindex1,hindex2;
    float hweight1, hweight2;

    h = h*360.0f;

    hindex1 = (int)floor(h/60.f);
    if (hindex1==5){
        hindex2=0;
    }else{
        hindex2=hindex1+1;
    }

    hweight2 = (h- 60.f*(float)hindex1)/60.f; hweight1 = 1.f - hweight2;

    if(v<0.5) {grayindex = 6;}
    if(v>=0.5){grayindex = 7;}
    weightcolor = pow(s,(0.14f * pow(1.0f/v,0.9f)));
    weightgray  = 1.0f - weightcolor;
    if( v<0.2f || s<0.1f){weightcolor=0.0f; weightgray=1.0f;}

    hweight1 = weightcolor * hweight1;
    hweight2 = weightcolor * hweight2;

    float x=pt.x, y=pt.y, z=pt.z;
    int xInd, yInd, zInd, theInd;
    xInd = (int)floor( (x - m_boxMin[0])/m_gridsize );
    if((xInd >= m_xNr) || (xInd<0)) {return 0.f;}
    yInd = (int)floor( (y - m_boxMin[1])/m_gridsize );
    if((yInd >= m_yNr) || (yInd<0)) {return 0.f;}
    zInd = (int)floor( (z - m_boxMin[2])/m_gridsize );
    if((zInd >= m_zNr) || (zInd<0)) {return 0.f;}
    theInd = zInd*m_xNr*m_yNr + xInd*m_yNr + yInd;

    x = x - m_gridsize*(float)xInd - m_boxMin[0];
    y = y - m_gridsize*(float)yInd - m_boxMin[1];
    z = z - m_gridsize*(float)zInd - m_boxMin[2];

    float x_=m_gridsize-x, y_=m_gridsize-y, z_=m_gridsize-z;
    int xPInd = zInd*m_xNr*m_yNr + (xInd+1)*m_yNr + yInd;
    int xyPInd = zInd*m_xNr*m_yNr + (xInd+1)*m_yNr + yInd+1;
    int xzPInd = (zInd+1)*m_xNr*m_yNr + (xInd+1)*m_yNr + yInd;
    int xyzPInd = (zInd+1)*m_xNr*m_yNr + (xInd+1)*m_yNr + yInd+1;
    int yPInd = zInd*m_xNr*m_yNr + xInd*m_yNr + (yInd+1);
    int yzPInd = (zInd+1)*m_xNr*m_yNr + xInd*m_yNr + (yInd+1);
    int zPInd = (zInd+1)*m_xNr*m_yNr + xInd*m_yNr + yInd;

    float weight; int offset;
    float gridsize3= 0.000001f;//gridsize*gridsize*gridsize;
    weight = max(0.f, ((x_*y_*z_)/gridsize3));
    offset = 8*theInd;
    float thisCorr = 0.f;
    thisCorr += (outF.hist[hindex1 + offset] * hweight1*weight);
    thisCorr += (outF.hist[hindex2 + offset] * hweight2*weight);
    thisCorr += (outF.hist[grayindex + offset] * weightgray*weight);

//        weight = max(0.f, (bandwidth-distxP))/gridsize;
    weight = max(0.f, ((x*y_*z_)/gridsize3));
    offset = 8*xPInd;
    thisCorr += (outF.hist[hindex1 + offset] * hweight1*weight);
    thisCorr += (outF.hist[hindex2 + offset] * hweight2*weight);
    thisCorr += (outF.hist[grayindex + offset] * weightgray*weight);

//        weight = max(0.f, (bandwidth-distxyP))/gridsize;
    weight = max(0.f, ((x*y*z_)/gridsize3));
    offset = 8*xyPInd;
    thisCorr += (outF.hist[hindex1 + offset] * hweight1*weight);
    thisCorr += (outF.hist[hindex2 + offset] * hweight2*weight);
    thisCorr += (outF.hist[grayindex + offset] * weightgray*weight);

//        weight = max(0.f, (bandwidth-distxzP))/gridsize;
    weight = max(0.f, ((x*y_*z)/gridsize3));
    offset = 8*xzPInd;
    thisCorr += (outF.hist[hindex1 + offset] * hweight1*weight);
    thisCorr += (outF.hist[hindex2 + offset] * hweight2*weight);
    thisCorr += (outF.hist[grayindex + offset] * weightgray*weight);

//        weight = max(0.f, (bandwidth-distxyzP))/gridsize;
    weight = max(0.f, ((x*y*z)/gridsize3));
    offset = 8*xyzPInd;
    thisCorr += (outF.hist[hindex1 + offset] * hweight1*weight);
    thisCorr += (outF.hist[hindex2 + offset] * hweight2*weight);
    thisCorr += (outF.hist[grayindex + offset] * weightgray*weight);

//        weight = max(0.f, (bandwidth-distyP))/gridsize;
    weight = max(0.f, ((x_*y*z_)/gridsize3));
    offset = 8*yPInd;
    thisCorr += (outF.hist[hindex1 + offset] * hweight1*weight);
    thisCorr += (outF.hist[hindex2 + offset] * hweight2*weight);
    thisCorr += (outF.hist[grayindex + offset] * weightgray*weight);

//        weight = max(0.f, (bandwidth-distyzP))/gridsize;
    weight = max(0.f, ((x_*y*z)/gridsize3));
    offset = 8*yzPInd;
    thisCorr += (outF.hist[hindex1 + offset] * hweight1*weight);
    thisCorr += (outF.hist[hindex2 + offset] * hweight2*weight);
    thisCorr += (outF.hist[grayindex + offset] * weightgray*weight);

//        weight = max(0.f, (bandwidth-distzP))/gridsize;
    weight = max(0.f, ((x_*y_*z)/gridsize3));
    offset = 8*zPInd;
    thisCorr += (outF.hist[hindex1 + offset] * hweight1*weight);
    thisCorr += (outF.hist[hindex2 + offset] * hweight2*weight);
    thisCorr += (outF.hist[grayindex + offset] * weightgray*weight);

    return thisCorr;
}


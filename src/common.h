#ifndef COMMON_H
#define COMMON_H

// pcl
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl_ros/transforms.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;

typedef pcl::PointXYZRGBNormal PointNT;
typedef pcl::PointCloud<PointNT> CloudN;
typedef CloudN::Ptr CloudNPtr;


inline float angDiff(float ang1, float ang2)
{
    float diff = ang1-ang2;
    while(diff > M_PI) {diff -= 2.f*M_PI;}
    while(diff < -M_PI) {diff += 2.f*M_PI;}
    return diff;
}


inline void rgbTohsv(int r, int g, int b, float &hr, float &sr, float &vr)
{
  float rd, gd, bd, h, s, v, max, min, del, rc, gc, bc;

  /* convert RGB to HSV */
  rd = (float)r / 255.f;            /* rd,gd,bd range 0-1 instead of 0-255 */
  gd = (float)g / 255.f;
  bd = (float)b / 255.f;

  /* compute maximum of rd,gd,bd */
  if (rd>=gd) { if (rd>=bd) max = rd;  else max = bd; }
         else { if (gd>=bd) max = gd;  else max = bd; }

  /* compute minimum of rd,gd,bd */
  if (rd<=gd) { if (rd<=bd) min = rd;  else min = bd; }
         else { if (gd<=bd) min = gd;  else min = bd; }

  del = max - min;
  v = max;
  if (max != 0.f) s = (del) / max;
             else s = 0.f;

  if (s != 0.f) {
    rc = (max - rd) / del;
    gc = (max - gd) / del;
    bc = (max - bd) / del;

    if      (rd==max) h = bc - gc;
    else if (gd==max) h = 2.f + rc - bc;
    else if (bd==max) h = 4.f + gc - rc;

    h = h * 60.f;
    if (h<0.f) h += 360.f;
  }
  h /=360.f;

  hr = h;  sr = s;  vr = v;
}

inline float colordist(PointNT a, PointNT b)
{
    float h1,s1,v1,h2,s2,v2;
    rgbTohsv(a.r,a.g,a.b,h1,s1,v1);
    rgbTohsv(b.r,b.g,b.b,h2,s2,v2);

//    if(s1>0.3 && v1>0.3){ v1=0; }
//    else{h1 = -2.f;}
//    if(s2>0.3 && v2>0.3){ v2=0; }
//    else{h2 = -2.f;}
//    float hdist = fabs(h1-h2);
//    while (hdist>0.5f) {hdist -= 0.5f;}
//    float vdist = fabs(v1-v2);
//    return (hdist*2 + vdist)/2.f;
    float hdist = fabs(h1-h2);
    while (hdist>0.5f) {hdist -= 0.5f;}
    return(hdist/0.5f);

}

inline float rand_01() //random number in range 0 to 1
{
    return static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
}

inline float rand_11()   //random number in range -1 to 1
{
    return ((static_cast <float> (rand()) / static_cast <float> (RAND_MAX)) -0.5f)*2.f;
}



#endif // COMMON_H

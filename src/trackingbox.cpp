#include "trackingbox.h"

trackingbox::trackingbox()
{
  initpose= Affine3f::Identity();
  initpose(2,3)=0.9f;
  initpose(1,3)=0.1f;
  
  m_cnt = 0;
  m_poseForVis.zero();
  m_finalParticle.zero();
  m_lastFinalParticle.zero();           // set x,y,z,r,p,y all to zero
  m_lastlastFinalParticle.zero();
  m_lastlastlastFinalParticle.zero();
  
  m_vel.zero();
  m_lastvel.zero();
  m_acc.zero();
  m_velerr.zero();
  m_velSum.zero();
  
  m_noise = Matrix<float,6,6>::Identity() * 0.001f;
  
}

trackingbox::~trackingbox()
{}


void trackingbox::setModel()
{
  model.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::io::loadPCDFile<pcl::PointXYZRGB> (modelfile.c_str(), *model);
  tracking::ParticleXYZRPY temp;
  pcl::getTranslationAndEulerAngles(initpose, temp.x,temp.y,temp.z,temp.roll,temp.pitch,temp.yaw);
  ros::param::get("/gpu/particlenumber", m_particlenum);                   // remenber to modify the parameter files
  initSamples(temp); m_finalParticle=temp; m_poseForVis = temp;
  std::cout << "There are " << model-> size() << "points in the model." << std::endl;
  
  
  for(size_t i=0; i<model->points.size();i++)
  {               
      model->points[i].y +=0.04f;
  }
  
   csest.initBox(model);
   m_modelF = csest.compute(model);                 // get the histogram of the box model
   
    m_modelsize = -1.f;
    for(size_t i=0; i<model->points.size();i++){
        float dist = model->points[i].getVector3fMap().norm();   // norm of the point
        if (dist>m_modelsize) m_modelsize=dist;
    }

    //initial setup for GPU
    m_gpuest.loadData(csest.m_gridsize, csest.m_xNr, csest.m_yNr,  csest.m_zNr, m_particlenum);
    m_gpuest.modelsize = model->points.size();
    m_gpuest.minPt.x=csest.m_boxMin[0];
    m_gpuest.minPt.y=csest.m_boxMin[1];
    m_gpuest.minPt.z=csest.m_boxMin[2];
    m_gpuest.maxPt.x=csest.m_boxMax[0];
    m_gpuest.maxPt.y=csest.m_boxMax[1];
    m_gpuest.maxPt.z=csest.m_boxMax[2];
    for (size_t i=0; i<m_modelF.hist.size(); i++)
    {
        m_gpuest.refhist[i] = m_modelF.hist[i];
    }
    m_gpuest.uploadRefHist();

    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setInputCloud(model);
    vg.setLeafSize(0.004f, 0.004f, 0.004f); // down sampling
    vg.filter(*model);
  
}


void trackingbox::initSamples(ParticleXYZRPY initpose)
{
    tracking::ParticleXYZRPY temp; tracking::ParticleXYZRPY temp2; temp2.zero();
    for (size_t i=0; i<m_particlenum; i++)
    {
        temp=sampleWithVar(initpose, 0.01f, 1.8f,temp2,false); // dist in m, ang in degrees, for variance of translation and rotation
        m_particles.push_back(temp);

    }
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr trackingbox::getTransformedModel()
{
  //check for NaNs!
    if (!(m_finalParticle.x==m_finalParticle.x && m_finalParticle.y==m_finalParticle.y && m_finalParticle.z == m_finalParticle.z &&
          m_finalParticle.roll == m_finalParticle.roll && m_finalParticle.pitch == m_finalParticle.pitch && m_finalParticle.yaw == m_finalParticle.yaw))
    {m_finalParticle = m_lastFinalParticle;} 
    
    Quaternion<float> q1, q2,q;
    Eigen::AngleAxis<float> aaX(m_poseForVis.roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxis<float> aaY(m_poseForVis.pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxis<float> aaZ(m_poseForVis.yaw, Eigen::Vector3f::UnitZ());
    q1 = aaZ * aaY * aaX;
    Eigen::AngleAxis<float> bbX(m_finalParticle.roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxis<float> bbY(m_finalParticle.pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxis<float> bbZ(m_finalParticle.yaw, Eigen::Vector3f::UnitZ());
    q2 = bbZ * bbY * bbX;

    float w1=0.0f, w2=1.0f;
    m_poseForVis.x = w1 * m_poseForVis.x + w2 * m_finalParticle.x;
    m_poseForVis.y = w1 * m_poseForVis.y + w2 * m_finalParticle.y;
    m_poseForVis.z = w1 * m_poseForVis.z + w2 * m_finalParticle.z;

    q = Quaternion<float>(w1*q1.w() + w2*q2.w(), w1*q1.x() + w2*q2.x(),
                          w1*q1.y() + w2*q2.y(), w1*q1.z() + w2*q2.z());             // is the order right?????????????
    q.normalize();
    Affine3f rot(q);
    pcl::getEulerAngles(rot, m_poseForVis.roll, m_poseForVis.pitch, m_poseForVis.yaw);


    Eigen::Affine3f finaltrans = m_finalParticle.toEigenMatrix();
    std::cout<<"current object 6DOF pose is : " <<m_finalParticle<<endl;
    m_transformedModel.reset(new Cloud);
    transformPointCloud(*model, *m_transformedModel, finaltrans);
    return m_transformedModel;
}


pcl::PointCloud< PointXYZRGB >::Ptr trackingbox::getRoi()
{
  return m_scene;
}


PointCloud< PointXYZRGB >::Ptr trackingbox::getParticleCloud()
{
    PointCloud< PointXYZRGB >::Ptr out(new Cloud);
    for (size_t i =0; i<m_particlenum; i++)
    {
        pcl::PointXYZRGB pt;
        pt.x=m_particles[i].x;  pt.y=m_particles[i].y;  pt.z=m_particles[i].z;
        pt.r=255; pt.g=255-255*(m_particles[i].weight*(1.f)*float(m_particlenum)); pt.b=pt.g;
        out->points.push_back(pt);
    }
    return out;
}


void trackingbox::run(PointCloud< PointXYZRGB >::Ptr scene)
{
   m_cnt ++;
   cout<<"current rame index is "<<m_cnt<<endl;
   float thresvel = M_PI;
   while(m_finalParticle.roll > thresvel) m_finalParticle.roll -= 2.f*M_PI;
   while(m_finalParticle.roll < -thresvel) m_finalParticle.roll += 2.f*M_PI;
   while(m_finalParticle.pitch > thresvel) m_finalParticle.pitch -= 2.f*M_PI;
   while(m_finalParticle.pitch < -thresvel) m_finalParticle.pitch += 2.f*M_PI;
   while(m_finalParticle.yaw > thresvel) m_finalParticle.yaw -= 2.f*M_PI;
   while(m_finalParticle.yaw < -thresvel) m_finalParticle.yaw += 2.f*M_PI;
   m_lastlastlastFinalParticle = m_lastlastFinalParticle;
   m_lastlastFinalParticle = m_lastFinalParticle;
   m_lastFinalParticle = m_finalParticle;     
   
   tracking::ParticleXYZRPY veltemp = m_lastFinalParticle - m_lastlastFinalParticle;     // - is to substract every element?
   while(veltemp.roll > M_PI) veltemp.roll -= 2.f*M_PI;
   while(veltemp.roll < -M_PI) veltemp.roll += 2.f*M_PI;
   while(veltemp.pitch > M_PI) veltemp.pitch -= 2.f*M_PI;
   while(veltemp.pitch < -M_PI) veltemp.pitch += 2.f*M_PI;
   while(veltemp.yaw > M_PI) veltemp.yaw -= 2.f*M_PI;
   while(veltemp.yaw < -M_PI) veltemp.yaw += 2.f*M_PI;

   m_velSum.x += fabs(veltemp.x);
   m_velSum.y+=fabs(veltemp.y);
   m_velSum.z+=fabs(veltemp.z);
   m_velSum.roll +=fabs(veltemp.roll)*180.f/M_PI;
   m_velSum.pitch +=fabs(veltemp.pitch)*180.f/M_PI;
   m_velSum.yaw +=fabs(veltemp.yaw)*180.f/M_PI;
   m_velSum = m_velSum + veltemp;               // m_velSum += veltemp

   cout<<"the average speed is: "<<m_velSum*(1.f/float(m_cnt))<<endl;
   tracking::ParticleXYZRPY average=m_velSum*(1.f/float(m_cnt));
   cout<<"average transl vel: "<<sqrt(pow(average.x,2)+pow(average.y,2)+pow(average.z,2))<<endl
       <<"average rotat vel:"<<sqrt(pow(average.roll,2)+pow(average.pitch,2)+pow(average.yaw,2))<<endl;
       
   if(m_cnt >5)
   {
      m_velerr = veltemp - (m_vel+m_acc*0.f);          
//       m_velerrV(0) = m_velerr.x ; m_velerrV(1) = m_velerr.y ; m_velerrV(2) = m_velerr.z ;
//       m_velerrV(3) = m_velerr.roll ; m_velerrV(4) = m_velerr.pitch ; m_velerrV(5) = m_velerr.yaw ;
      m_lastvel = m_vel;
      m_vel = m_vel*0.2f + veltemp*0.8f;
      while(m_vel.roll > thresvel) {m_vel.roll -= 2.f*M_PI;}
      while(m_vel.roll < -thresvel) {m_vel.roll += 2.f*M_PI;}
      while(m_vel.pitch > thresvel){ m_vel.pitch -= 2.f*M_PI;}
      while(m_vel.pitch < -thresvel){ m_vel.pitch += 2.f*M_PI;}
      while(m_vel.yaw > thresvel){ m_vel.yaw -= 2.f*M_PI;}
      while(m_vel.yaw < -thresvel) {m_vel.yaw += 2.f*M_PI;}

      m_acc = m_acc*0.4f + (m_vel - m_lastvel)*0.4f;       // update the velocity
      while(m_acc.roll > M_PI) {m_acc.roll -= 2.f*M_PI;}
      while(m_acc.roll < -M_PI) {m_acc.roll += 2.f*M_PI;}
      while(m_acc.pitch > M_PI) {m_acc.pitch -= 2.f*M_PI;}
      while(m_acc.pitch < -M_PI) {m_acc.pitch += 2.f*M_PI;}
      while(m_acc.yaw > M_PI) {m_acc.yaw -= 2.f*M_PI;}
      while(m_acc.yaw < -M_PI) {m_acc.yaw += 2.f*M_PI;}
      
   }

  cout<<"velocity is: "<<m_vel<<endl;
  cout<<"velocity prediction error is: "<<m_velerr<<endl<<endl;
  cout<<"accelaration is: "<<m_acc<<endl<<endl;
  
  m_scene.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  tracking::ParticleXYZRPY predict = m_finalParticle + m_vel + m_acc*0.f;

  int xCor = (int)round(570.f/predict.z * predict.x + 319.5);      // transform to the camera coordinate
  int yCor = (int)round(570.f/predict.z * predict.y + 239.5);
  cout << "error flag1" << endl;
  int xB,yB,xE,yE;
  int modelPixelSize = m_modelsize * 570.f/predict.z;              // m_modelsize: norm of the points of model
  xB = max(xCor-modelPixelSize, 0); xE = min(639, xCor+modelPixelSize);        //???????????????????
  yB = max(yCor-modelPixelSize, 0); yE = min(479, yCor+modelPixelSize);
  cout << "error flag2" << endl;
  int ct=0;
  for(size_t i=0; i<modelPixelSize*modelPixelSize; i++)
  {
    if (ct>100) break;
    int x_ = min(max(rand() % (2*modelPixelSize) + xB, 0),639);
    int y_ = min(max(rand() % (2*modelPixelSize) + yB, 0),479);
    if ( (scene->points[y_*640+x_].getVector3fMap()
          -Vector3f(m_finalParticle.x,m_finalParticle.y,m_finalParticle.z)).norm() <=1.f*m_modelsize )
    {
       m_scene->points.push_back(scene->points[y_*640+x_]);
       ct++;
    }
  }
  cout<<"sampled "<<m_scene->points.size()<<"observation points"<<endl;
  cout << "error flag3" << endl;
  vector<int> indices;
  pcl::removeNaNFromPointCloud(*m_scene, *m_scene, indices);
  cout << "error flag4" << endl;
 //do the actual particle filtering
 float vardist=0.012f, varang=2.405f;
 resample(vardist, varang,true);
 cout << "error flag5" << endl;
 weight();
 
}


void trackingbox::resample(float varDist, float varAng, bool firstRun)
{
    double probabilities[m_particlenum];
    for (size_t i=0; i<m_particlenum; i++)
    {
        probabilities[i] = m_particles[i].weight; 
	std::cout << probabilities[i] <<"  ";
    }
    std::vector<double> cumulative;
    std::partial_sum(&probabilities[0], &probabilities[0] + m_particlenum,
                     std::back_inserter(cumulative));
        cout << "error flag6" << cumulative.back() << endl;
    boost::uniform_real<> dist(0, cumulative.back());
    boost::variate_generator<boost::mt19937&, boost::uniform_real<> > die(m_gen, dist);
    std::lower_bound(cumulative.begin(), cumulative.end(), die()) - cumulative.begin();

    std::vector<tracking::ParticleXYZRPY> templist;
    templist = m_particles;
    float motionratio; ros::param::get("/gpu/motionratio", motionratio);

    if (!firstRun){motionratio = 0.f;}

    int motionnum = (int)floor(m_particlenum * motionratio);
    tracking::ParticleXYZRPY motion = m_vel+m_acc*0.2f;
    for (size_t i=0; i<motionnum; i++){
        int index = std::lower_bound(cumulative.begin(), cumulative.end(), die()) - cumulative.begin();
        templist[i] = sampleWithVar(m_particles[index], varDist, varAng,motion, true); // dist in m, ang in degrees
        }
    motion.zero();
    for (size_t i=motionnum; i<m_particlenum; i++){
        int index = std::lower_bound(cumulative.begin(), cumulative.end(), die()) - cumulative.begin();
//        templist[i] = sampleWithVar(m_finalParticle, varDist, varAng, motion, false); // dist in m, ang in degrees
        templist[i] = sampleWithVar(m_particles[index], varDist, varAng, motion, false); // dist in m, ang in degrees
    }
    m_particles = templist;
}


ParticleXYZRPY trackingbox::sampleWithVar(ParticleXYZRPY part, float varDist, float varAng, ParticleXYZRPY motion, bool motionmode)
{
   tracking::ParticleXYZRPY temp;
    // adaptive variance
    if (motionmode)
    {
        Affine3f partMat= (part+motion).toEigenMatrix();
        float fach=3.6f;
        temp.x = tracking::sampleNormal(0.f, pow(fabs(m_velerr.x)*fach+varDist, 2) );
        temp.y = tracking::sampleNormal(0.f, pow(fabs(m_velerr.y)*fach+varDist, 2) );
        temp.z = tracking::sampleNormal(0.f, pow(fabs(m_velerr.z)*fach+varDist, 2) );
        temp.roll  = tracking::sampleNormal(0.f, pow(fabs(m_velerr.roll)*fach+(varAng/180.f)*M_PI, 2) );
        temp.pitch = tracking::sampleNormal(0.f, pow(fabs(m_velerr.pitch)*fach+(varAng/180.f)*M_PI, 2) );
        temp.yaw   = tracking::sampleNormal(0.f,  pow(fabs(m_velerr.yaw)*fach+(varAng/180.f)*M_PI, 2) );
        Affine3f transMat= partMat*temp.toEigenMatrix() ;
        pcl::getTranslationAndEulerAngles(transMat,
                                          temp.x,temp.y,temp.z,temp.roll,temp.pitch,temp.yaw);
    } 
    else
    {
        varDist *=0.5f; varAng*=0.5f;
        temp.x = tracking::sampleNormal(0, pow(varDist, 2) );
        temp.y = tracking::sampleNormal(0, pow(varDist, 2) );
        temp.z = tracking::sampleNormal(0, pow(varDist, 2) );
        temp.roll  = tracking::sampleNormal(0,  pow((varAng/180.f)*M_PI, 2) );
        temp.pitch = tracking::sampleNormal(0, pow((varAng/180.f)*M_PI, 2) );
        temp.yaw   = tracking::sampleNormal(0,  pow((varAng/180.f)*M_PI, 2) );
        Affine3f transMat=  part.toEigenMatrix()*temp.toEigenMatrix();
        pcl::getTranslationAndEulerAngles(transMat,                                              //Extract x,y,z and the Euler angles (XYZ-convention) from the given transformation. 
                                          temp.x,temp.y,temp.z,temp.roll,temp.pitch,temp.yaw);
    }
    while (temp.roll>M_PI)  {temp.roll -= 2*M_PI;}
    while (temp.roll<-M_PI) {temp.roll += 2*M_PI;}
    while (temp.pitch>M_PI)  {temp.pitch -= 2*M_PI;}
    while (temp.pitch<-M_PI) {temp.pitch += 2*M_PI;}
    while (temp.yaw>M_PI)  {temp.yaw -= 2*M_PI;}
    while (temp.yaw<-M_PI) {temp.yaw += 2*M_PI;}
    temp.weight = 1.f;
    return temp;
}


void trackingbox::weight()
{
  bool usegpu;
  ros::param::get("/gpu/usegpu", usegpu);

  std::vector<float> partweights;
  if (usegpu)
  {
      partweights = weightGPU ();
  }
  else
  {
      partweights = weightCPU ();
  }

  float minValue = 1e22, maxValue=-1e22;
  for(size_t i=0; i<m_particlenum; i++)
  {
      m_particles[i].weight = partweights[i];
      if (m_particles[i].weight > maxValue)
          maxValue = m_particles[i].weight;
      if (m_particles[i].weight < minValue)
          minValue = m_particles[i].weight;
    }

    ///////////////////////////////normarlize////////////////////////////////////
   float weightSum = 0.f;
//    minValue =0.f;
    for (size_t i=0; i<m_particlenum; i++)
    {
        float value;
        value= 1.0001f-(m_particles[i].weight - minValue)/(maxValue-minValue);
        m_particles[i].weight *= 10000.f*min(exp(-15.f * value),1.1f);

        if (minValue == 1e22){m_particles[i].weight=1.f;}
        weightSum += m_particles[i].weight;
    }
    float effectiveSum = 0.f;
    for (size_t i=0; i<m_particlenum; i++)
    {
        m_particles[i].weight = m_particles[i].weight/weightSum;
        effectiveSum += pow(m_particles[i].weight,2.f);
    }
    m_effective = 1.f/effectiveSum;
    m_finalParticle.zero();

    Quaternion<float> finalq; finalq=  Quaternion<float>::Identity();
    for (size_t i=0; i<m_particlenum; i++)
    {
        Quaternion<float> q;
        Eigen::AngleAxis<float> aaX(m_particles[i].roll, Eigen::Vector3f::UnitX());
        Eigen::AngleAxis<float> aaY(m_particles[i].pitch, Eigen::Vector3f::UnitY());
        Eigen::AngleAxis<float> aaZ(m_particles[i].yaw, Eigen::Vector3f::UnitZ());

        q = aaZ * aaY * aaX;
        if ((finalq.w()*q.w() + finalq.x()*q.x() + finalq.y()*q.y() + finalq.z()*q.z())<0)
	{
            q =  Quaternion<float>(-q.w(), -q.x(), -q.y(), -q.z());
        }
        finalq = Quaternion<float>(finalq.w()+10000.f*q.w()*m_particles[i].weight, finalq.x()+10000.f*q.x()*m_particles[i].weight,
                                   finalq.y()+10000.f*q.y()*m_particles[i].weight, finalq.z()+10000.f*q.z()*m_particles[i].weight);
        m_finalParticle = m_finalParticle + m_particles[i] * m_particles[i].weight;
    }
    finalq = Quaternion<float>(finalq.w()/10000.f, finalq.x()/10000.f,
                               finalq.y()/10000.f, finalq.z()/10000.f);
    finalq.normalize();
    Affine3f rot(finalq);
    pcl::getEulerAngles(rot, m_finalParticle.roll, m_finalParticle.pitch, m_finalParticle.yaw);
    if (!(m_finalParticle.x==m_finalParticle.x && m_finalParticle.y==m_finalParticle.y && m_finalParticle.z == m_finalParticle.z &&
          m_finalParticle.roll == m_finalParticle.roll && m_finalParticle.pitch == m_finalParticle.pitch && m_finalParticle.yaw == m_finalParticle.yaw))
        m_finalParticle = m_lastFinalParticle;

    bool refine;
    ros::param::get("/userefinement", refine);
    if (refine)
    refinement();
}


vector< float > trackingbox::weightGPU()
{
    m_gpuest.uploadCurrentCloud(m_scene->points.size());
    float h,s,v;
    for (size_t i=0; i<m_scene->points.size(); i++)
    {
        PointT pt = m_scene->points[i];
        m_gpuest.cloudpos[i].x = pt.x;
        m_gpuest.cloudpos[i].y = pt.y;
        m_gpuest.cloudpos[i].z = pt.z;
        rgbTohsv(pt.r, pt.g, pt.b, h, s, v);
        m_gpuest.cloudhsv[i].x = h;
        m_gpuest.cloudhsv[i].y = s;
        m_gpuest.cloudhsv[i].z = v;
    }

    float x_,y_,z_,roll_,pitch_,yaw_;
    for (size_t i=0; i<m_particlenum; i++)
    {
        Affine3f temp = m_particles[i].toEigenMatrix().inverse();
        pcl::getTranslationAndEulerAngles (temp,x_,y_,z_,roll_,pitch_,yaw_);
        m_gpuest.partpos[i].x = x_;
        m_gpuest.partpos[i].y = y_;
        m_gpuest.partpos[i].z = z_;
        m_gpuest.partrot[i].x = roll_;
        m_gpuest.partrot[i].y = pitch_;
        m_gpuest.partrot[i].z = yaw_;
    }
    return (m_gpuest.compute());
}


vector< float > trackingbox::weightCPU()
{
    std::vector<float> weights; weights.resize(m_particlenum);
    CloudPtr hypoRevCloud;
    Affine3f hypoRevPose;
    for (size_t i=0; i<m_particlenum; i++)
    {
      hypoRevPose = m_particles[i].toEigenMatrix().inverse();
      hypoRevCloud.reset(new Cloud);
      pcl::transformPointCloud(*m_scene, *hypoRevCloud, hypoRevPose);
      weights[i]=csest.evalPoints(hypoRevCloud);
    }
    return weights;
}


void trackingbox::refinement()
{
    float translStep = 1e-2f , rotStep = 1e-2;
    float learningStep = 1e-6f;
    ParticleXYZRPY jacobian,tempPartP,tempPartN;
    ParticleXYZRPY oldfinal= m_finalParticle;

    float lastObj = weightParticle (m_finalParticle);

    //iterations
    for (size_t i=0; i< 20; i++)
    {
        tempPartP = m_finalParticle;  tempPartN = m_finalParticle;
        tempPartP.x += translStep; tempPartN.x -= translStep;
        jacobian.x = (weightParticle(tempPartP) - weightParticle(tempPartN))/translStep;

        tempPartP = m_finalParticle;  tempPartN = m_finalParticle;
        tempPartP.y += translStep; tempPartN.y -= translStep;
        jacobian.y = (weightParticle(tempPartP) - weightParticle(tempPartN))/translStep;

        tempPartP = m_finalParticle;  tempPartN = m_finalParticle;
        tempPartP.z += translStep; tempPartN.z -= translStep;
        jacobian.z = (weightParticle(tempPartP) - weightParticle(tempPartN))/translStep;

        tempPartP.zero();  tempPartN.zero();
        tempPartP.roll += rotStep; tempPartN.roll -= rotStep;
        jacobian.roll = (weightParticle(tempPartP.toEigenMatrix()* m_finalParticle.toEigenMatrix())
                         - weightParticle(tempPartN.toEigenMatrix()*m_finalParticle.toEigenMatrix()))/rotStep/2.f;

        tempPartP.zero();  tempPartN.zero();
        tempPartP.pitch += rotStep; tempPartN.pitch -= rotStep;
        jacobian.pitch = (weightParticle(tempPartP.toEigenMatrix()* m_finalParticle.toEigenMatrix())
                          - weightParticle(tempPartN.toEigenMatrix()*m_finalParticle.toEigenMatrix()))/rotStep/2.f;


        tempPartP.zero();  tempPartN.zero();
        tempPartP.yaw += rotStep; tempPartN.yaw -= rotStep;
        jacobian.yaw = (weightParticle(tempPartP.toEigenMatrix()* m_finalParticle.toEigenMatrix())
                        - weightParticle(tempPartN.toEigenMatrix()*m_finalParticle.toEigenMatrix()))/rotStep/2.f;

        Affine3f increPose = (jacobian*learningStep).toEigenMatrix() *m_finalParticle.toEigenMatrix();
        pcl::getTranslationAndEulerAngles(increPose,
        m_finalParticle.x, m_finalParticle.y, m_finalParticle.z,
                                          m_finalParticle.roll, m_finalParticle.pitch, m_finalParticle.yaw);

        float thisObj = weightParticle(m_finalParticle);
        lastObj=thisObj;
        cout<<jacobian<<endl;
    }
}


float trackingbox::weightParticle(Affine3f hyppose)
{
   CloudPtr hypoRevCloud;
   Affine3f hypoRevPose;
   hypoRevPose = hyppose.inverse();
   hypoRevCloud.reset(new Cloud);
   pcl::transformPointCloud(*m_scene, *hypoRevCloud, hypoRevPose);
   return csest.evalPoints(hypoRevCloud);
}


float trackingbox::weightParticle(ParticleXYZRPY hyppose)
{
   CloudPtr hypoRevCloud;
   Affine3f hypoRevPose;
   hypoRevPose = hyppose.toEigenMatrix().inverse();
   hypoRevCloud.reset(new Cloud);
   pcl::transformPointCloud(*m_scene, *hypoRevCloud, hypoRevPose);
   return csest.evalPoints(hypoRevCloud);
}


Affine3f trackingbox::getPose()
{
   return m_finalParticle.toEigenMatrix();
}


void trackingbox::getXYZQ(float& tx, float& ty, float& tz, 
			  float& qx, float& qy, float& qz, float& qw)
{
    tracking::ParticleXYZRPY temp = m_finalParticle;
    Affine3f pose = temp.toEigenMatrix();
    tx=pose(0,3); ty=pose(1,3); tz=pose(2,3);
    Quaternion<float> q(pose.rotation());
    q.normalize();
    qx=q.x(); qy=q.y(); qz=q.z(); qw=q.w();
}



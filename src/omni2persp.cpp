/* 2020/07/13 - G. CARON
   ROS node for central omnidirectional to perspective mapping
   listens an omni image on topic /...
   publishes the tranformed perspective image on topic /...
*/

// ROS includes
#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>

// VISP includes
#include <visp/vpImage.h>

// Bridges
#include <visp_bridge/image.h>

#include <per/prOmni.h>

#include <fstream>

//#define VERBOSE
//#define MEASUREMAPPINGTIME

// global variables
image_transport::Publisher pub_perspective_image;
sensor_msgs::Image PerspImage_msg;

/*unsigned */int haut, larg;
double px, py, u0, v0, xi;
prCameraModel *ocam, *pcam;

vpImage<unsigned char> I, Ip;
unsigned int nbPixels;
int *coordMapping[4] = {NULL, NULL, NULL, NULL}; // perspective and omni image pixel coordinates

double opt_focalFact;

// functions prototypes
void init();
void imageCallback(const sensor_msgs::ImageConstPtr& Image);

int main (int argc, char **argv)
{
  // ROS init
  ros::init(argc, argv, "omni2persp");
  ros::NodeHandle nH;

#ifdef VERBOSE
  ROS_INFO("omni2persp::main");
#endif

  // ROS read launch file parameters
  // coordinates table filename

  ros::NodeHandle nHp("~"); //to get the parameters for that node
  std::string inputImagesTopic, outputImagesTopic;
  nHp.param("inputImagesTopic", inputImagesTopic, std::string(""));
  nHp.param("outputImagesTopic", outputImagesTopic, std::string(""));

  nHp.param("imWidth", larg, 0);
  nHp.param("imHeight", haut, 0);

  nHp.param("opt_focalFact", opt_focalFact, 0.33);

  nHp.getParam("inputImagesTopic", inputImagesTopic);
  nHp.getParam("outputImagesTopic", outputImagesTopic);

  nHp.getParam("imWidth", larg);
  nHp.getParam("imHeight", haut);
  nHp.getParam("opt_focalFact", opt_focalFact);

  nbPixels = larg*haut;

  //params sequence 2020_02_18 left 848x800 | RealSense SN 804
  double fact = 1.0;
  px = (830.40*fact);
  py = (829.69*fact);
  u0 = (413.31*fact);
  v0 = (404.85*fact);
  xi = 1.8862;

  init();

  // ROS Listner
  image_transport::ImageTransport i_t(nH);
  image_transport::Subscriber sub_image = i_t.subscribe(inputImagesTopic, 1, imageCallback);

  // ROS Publisher
  pub_perspective_image = i_t.advertise(outputImagesTopic, 1);

  ros::spin();

  //after killing ros node
  for(unsigned int c = 0 ; c < 4 ; c++)
    if(coordMapping[c] != NULL)
    {
      delete [] coordMapping[c];
      coordMapping[c] = NULL;
    }

  return 0;
}

void init()
{
  if(nbPixels == 0)
  {
    ROS_INFO("omni2persp::init: nbPixels == 0");
    return;
  }

  ocam = new prOmni(px, py, u0, v0, xi);
  pcam = new prOmni(px*opt_focalFact, px*opt_focalFact, larg*0.5, haut*0.5, 0);

  // Get pixel coordinates mapping
  for(unsigned int c = 0 ; c < 4 ; c++)
    coordMapping[c] = new int[nbPixels];

  // Compute the mapping
  double Xs, Ys, Zs;
  int *pt_up = coordMapping[0], *pt_vp = coordMapping[1], *pt_uo = coordMapping[2], *pt_vo = coordMapping[3];

  for(unsigned int v = 0; v < haut ; v++)
    for(unsigned int u = 0; u < larg ; u++, pt_up++, pt_vp++, pt_uo++, pt_vo++)
    {
      prPointFeature P;
      P.setPixUV(u,v);
      pcam->pixelMeterConversion(P);
      if(((prOmni *)pcam)->projectImageSphere(P, Xs, Ys, Zs) == 0)
      {
        P.set_X(Xs); P.set_Y(Ys); P.set_Z(Zs);

        ((prOmni *)ocam)->project3DImage(P); // projectSphereImage
        ocam->meterPixelConversion(P);
      }
      else
      {
        P.setPixUV(0,0);
      }

       *pt_up = u;
       *pt_vp = v;
       *pt_uo = P.get_u();
       *pt_vo = P.get_v();
    }

  // Initialize the image to be published
  I.resize(haut, larg, false);
  Ip.resize(haut, larg, false);
}

void imageCallback(const sensor_msgs::ImageConstPtr& Image)
{
#ifdef VERBOSE
  ROS_INFO("omni2persp::imageCallback");
#endif

  //ATTENTION : visp_bridge not working with Realsense images (encoding as OpenCV)
  memcpy(I.bitmap, &(Image->data[0]), Image->height*Image->width);

#ifdef MEASUREMAPPINGTIME
  ros::WallTime start_, end_;
  start_ = ros::WallTime::now();
#endif

  int *pt_up = coordMapping[0], *pt_vp = coordMapping[1], *pt_uo = coordMapping[2], *pt_vo = coordMapping[3];

  //Optimized version with specific knowledge on the equirect coordinates (a few ms less)
  unsigned char *pt_persp = Ip.bitmap, *pt_omni = I.bitmap;

  unsigned int pix = 0;
  //for(unsigned int p = 0; p < nbPixels ; p++, pt_persp+=3, pt_ud++, pt_vd++)
  for(unsigned int p = 0; p < nbPixels ; p++, pt_persp++, pt_uo++, pt_vo++)
  {
//    memcpy(pt_persp, pt_DualImage->image.data + 3*((*pt_vd)*imWidth+(*pt_ud)), 3);
//    memcpy(pt_persp, I.bitmap + ((*pt_vo)*larg+(*pt_uo)), 1);
//    *pt_persp = I[*pt_vo][*pt_uo];
    *pt_persp = *(pt_omni + ((*pt_vo)*larg+(*pt_uo))); //fastest
  }

#ifdef MEASUREMAPPINGTIME
  end_ = ros::WallTime::now();

  double execution_time = (end_ - start_).toNSec() * 1e-6;
  ROS_INFO_STREAM("Exectution time (ms): " << execution_time);
#endif

  PerspImage_msg = visp_bridge::toSensorMsgsImage(Ip);
  pub_perspective_image.publish(PerspImage_msg);
}


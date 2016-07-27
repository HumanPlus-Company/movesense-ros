/*********************************************************************
*
*********************************************************************/

#include <ros/ros.h>
#include <ros/time.h>
#include <MoveSenseCamera.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sstream>
#include <std_srvs/Empty.h>
#include <sensor_msgs/fill_image.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/transform_broadcaster.h>

#ifdef HAVE_NEW_YAMLCPP
 //The >> operator disappeared in yaml-cpp 0.5, so this function is
 //added to provide support for code written under the yaml-cpp 0.3 API.
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
  i = node.as<T>();
}
#endif

// for pointcloud
struct _Point
{
    // point
    float x;
    float y;
    float z;
    // color
    unsigned char r;
    unsigned char g;
    unsigned char b;
};

class MoveSenceSensorNode
{
public:
  // private ROS node handle
  ros::NodeHandle node_;

  // shared image message
  sensor_msgs::Image img_l_, img_r_, img_d_, img_depth_;
  image_transport::CameraPublisher image_pub_l,image_pub_r,image_pub_d,image_pub_depth;

  ros::Publisher point_cloud_pub_;
  tf::TransformBroadcaster tf_broadcaster_;

  boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;

  // parameters
  std::string camera_mode;
  std::string camera_param;
  std::string camera_name_, camera_info_url_;

  // member
  movesense::CameraMode sel;
  movesense::MoveSenseCamera* cam_;

  int width, height, len;

  unsigned char* img_data;

  unsigned char* img_l;
  unsigned char* img_r;
  unsigned char* img_d;
  unsigned char* img_depth;

  // camear param
  float c_k1, c_k2, c_p1, c_p2;
  float c_f, c_cu, c_cv, c_b;

  typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

  MoveSenceSensorNode() :
      node_("~"),
      cam_(NULL),
      img_data(NULL),
      img_l(NULL),
      img_r(NULL),
      img_d(NULL),
      img_depth(NULL)
  {
    // advertise the main image topic
    image_transport::ImageTransport it(node_);
    image_pub_l = it.advertiseCamera("left/image_raw", 1);
    image_pub_r = it.advertiseCamera("right/image_raw", 1);
    image_pub_d = it.advertiseCamera("disp/image_raw", 1);
    image_pub_depth = it.advertiseCamera("depth/image_raw", 1);
    point_cloud_pub_ = node_.advertise<PointCloud>("depth/point_cloud", 1);


    // grab the parameters
    node_.param("camera_mode", camera_mode, std::string("CAM_STEREO_752X480_LRD_30FPS"));


    node_.param("camera_name", camera_name_, std::string("my_stereo_camera"));
    node_.param("camera_info_url", camera_info_url_, std::string(""));

    // remap camera mode
    if (ros::names::remap("mode") != "mode")
    {
        std::string _mode = ros::names::remap("mode");
        camera_mode = _mode.substr(1,_mode.length()-1); // remove '/'
    }

    if (ros::names::remap("param") != "param")
    {
        std::string _param = ros::names::remap("param");
        camera_param = _param.substr(1,_param.length()-1); // remove '/'
    }

    // need judge usb3.0

    // Load settings
    std::string fname = "./config/camera_param.yaml";

    std::ifstream fin(fname.c_str());
          if (fin.fail()) {
            ROS_ERROR("camear param config file could not open %s.", fname.c_str());
            node_.shutdown();
            return;
          }

#ifdef HAVE_NEW_YAMLCPP
          // The document loading process changed in yaml-cpp 0.5.
          YAML::Node doc = YAML::Load(fin);
#else
          YAML::Parser parser(fin);
          YAML::Node doc;
          parser.GetNextDocument(doc);
#endif

         try {
            doc["Camera.f"] >>c_f;
          } catch (YAML::InvalidScalar) {
            ROS_ERROR("The map does not contain a Camera.c_f tag or it is invalid.");
            node_.shutdown();
            return;
          }
          try {
            doc["Camera.cu"] >> c_cu;
          } catch (YAML::InvalidScalar) {
            ROS_ERROR("The map does not contain a Camera.cu tag or it is invalid.");
            node_.shutdown();
            return;
          }
          try {
            doc["Camera.cv"] >> c_cv;
          } catch (YAML::InvalidScalar) {
            ROS_ERROR("The map does not contain a Camera.cv tag or it is invalid.");
            node_.shutdown();
            return;
          }
          try {
            doc["Camera.b"] >> c_b;
          } catch (YAML::InvalidScalar) {
            ROS_ERROR("The map does not contain a Camera.b tag or it is invalid.");
            node_.shutdown();
            return;
          }
          try {
            doc["Camera.k1"] >> c_k1;
          } catch (YAML::InvalidScalar) {
            ROS_ERROR("The map does not contain a Camera.k1 tag or it is invalid.");
            node_.shutdown();
            return;
          }
          try {
            doc["Camera.k2"] >> c_k2;
          } catch (YAML::InvalidScalar) {
            ROS_ERROR("The map does not contain a Camera.k2 tag or it is invalid.");
            node_.shutdown();
            return;
          }
          try {
            doc["Camera.p1"] >> c_p1;
          } catch (YAML::InvalidScalar) {
            ROS_ERROR("The map does not contain a Camera.p1 tag or it is invalid.");
            node_.shutdown();
            return;
          }
          try {
            doc["Camera.p2"] >> c_p2;
          } catch (YAML::InvalidScalar) {
            ROS_ERROR("The map does not contain a Camera.p2 tag or it is invalid.");
            node_.shutdown();
            return;
          }

    // init   
    if (0 == camera_mode.compare(std::string("CAM_STEREO_752X480_LRD_30FPS"))) // only surport usb3.0 now
    {
      	width = 752;
      	height = 480;
      	len = width*height*3;
		sel = CAM_STEREO_752X480_LRD_30FPS;
    }
    else if (0 == camera_mode.compare(std::string("CAM_STEREO_376X240_LRD_30FPS"))) // only surport usb3.0 now
    {
      	width = 376;
      	height = 240;
      	len = width*height*2;
		sel = CAM_STEREO_376X240_LRD_30FPS;
    }
    else
    {
      	ROS_INFO("Warning! no matching mode! input: '%s'",camera_mode.c_str());
      	node_.shutdown();
        return;
    }

	ROS_INFO("The camera mode is : '%s' width = %d height = %d sel = %d.",camera_mode.c_str(), width, height, sel);

    // new camera
    cam_ = new movesense::MoveSenseCamera(sel);

    if (cam_ == NULL)
    {
		ROS_INFO("New Camera Failed!");
      	node_.shutdown();
        return;
    }

    // start the camera
    if (!(movesense::MS_SUCCESS==cam_->OpenCamera()))
    {
      	ROS_INFO("Open Camera Failed!");
      	node_.shutdown();
        return;
    }
    
    // new image data
    img_data  = new unsigned char[len];
    img_l  = new unsigned char[width*height];
    img_r  = new unsigned char[width*height];
    img_d  = new unsigned char[width*height];
    img_depth  = new unsigned char[width*height];

    // new camera_info_manager
    cinfo_.reset(new camera_info_manager::CameraInfoManager(node_, camera_name_, camera_info_url_));

     // set default camera info
     cinfo_->setCameraName("camera_name_");
     sensor_msgs::CameraInfo camera_info;
     //camera_info.header.frame_id = img_.header.frame_id;
     camera_info.width = width;
     camera_info.height = height;

     camera_info.R[0] = 1;
     camera_info.R[4] = 1;
     camera_info.R[8] = 1;

     camera_info.K[0] = c_f;
     camera_info.K[2] = c_cu;
     camera_info.K[4] = c_f;
     camera_info.K[5] = c_cv;
     camera_info.K[8] = 1;

     camera_info.P[0] = c_f;
     camera_info.P[2] = c_cu;
     camera_info.P[3] = -c_f * c_b;
     camera_info.P[5] = c_f;
     camera_info.P[6] = c_cv;
     camera_info.P[7] = 0;
     camera_info.P[10] = 1;

     cinfo_->setCameraInfo(camera_info);
}


  virtual ~MoveSenceSensorNode()
  {
	// close camera
	if (cam_ != NULL)
	{
		cam_->CloseCamera();
		delete cam_;
	}
    

	//delete image data
	if (img_data) delete img_data;
	if (img_l) delete img_l;
	if (img_r) delete img_r;
	if (img_d) delete img_d;
	if (img_depth) delete img_depth;
  }
  

  bool spin()
  {
    ros::Rate loop_rate(30);

    while (node_.ok())
    {
      cam_->GetImageData(img_data ,len);

      //ROS_INFO("get");

      if(len>0)
      {
          //ROS_INFO("publish");
          
          // grab the camera info
          sensor_msgs::CameraInfoPtr ci_l(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));
          sensor_msgs::CameraInfoPtr ci_r(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));
          sensor_msgs::CameraInfoPtr ci_d(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));

          ros::Time stamp = ros::Time::now();
          ci_l->header.stamp = stamp;
          ci_r->header.stamp = stamp;
          ci_d->header.stamp = stamp;


		  for(int i = 0 ; i < height; i++)
          {
          	memcpy(img_l+width*i,	img_data+(3*i)*width, width);
          	memcpy(img_r+width*i,	img_data+(3*i+1)*width,	width);
			memcpy(img_d+width*i,	img_data+(3*i+2)*width,	width);
		  }

          // _Point vector
         std::vector<_Point> ms_point;

          // change disp to depth
          for(int i = 0 ; i < height; i++)
          {
              for(int j = 0; j<width; j++)
              {
                  if (img_d[i*width + j] >128)  {
                      if (2*img_d[i*width + j] - 128 < 1e-6) img_depth[i*width + j] = 0;
                      else {
                          img_depth[i*width + j]  = 4.0 * c_f * c_b / (2*img_d[i*width + j] - 128);
                          _Point p;
                          p.x = (j - c_cu)*(img_depth[i*width + j]/c_f);
                          p.y = (i - c_cv)*(img_depth[i*width + j]/c_f);
                          p.z = img_depth[i*width + j];

                          p.r = p.g = p.b = img_l[i*width + j];
                          ms_point.push_back(p);
                      }
                  }
                  else {
                      if (img_d[i*width + j]  < 1e-6) img_depth[i*width + j] = 0;
                      else {
                          img_depth[i*width + j]  =4.0 *  c_f * c_b / img_d[i*width + j] ;
                          _Point p;
                          p.x = (j - c_cu)*(img_depth[i*width + j]/c_f);
                          p.y = (i - c_cv)*(img_depth[i*width + j]/c_f);
                          p.z = img_depth[i*width + j];

                          p.r = p.g = p.b = img_l[i*width + j];
                          ms_point.push_back(p);
                      }
                  }
              }
          }

          // compute pintcloud
          PointCloud::Ptr point_cloud(new PointCloud());
          point_cloud->header.frame_id = std::string("base_link");
          point_cloud->header.stamp = pcl_conversions::toPCL(ci_l->header).stamp;
          point_cloud->width = 1;
          point_cloud->height = ms_point.size();
          point_cloud->points.resize(ms_point.size());

          for (int i = 0; i < ms_point.size(); ++i)
          {
            point_cloud->points[i].x = ms_point[i].x;
            point_cloud->points[i].y = ms_point[i].y;
            point_cloud->points[i].z = ms_point[i].z;

            point_cloud->points[i].r = ms_point[i].r;
            point_cloud->points[i].g = ms_point[i].g;
            point_cloud->points[i].b = ms_point[i].b;
        }

		  fillImage(img_l_, "mono8", height, width, width, img_l);
		  fillImage(img_r_, "mono8", height, width, width, img_r);
		  fillImage(img_d_, "mono8", height, width, width, img_d);
          fillImage(img_depth_, "32FC1", height, width, sizeof(float)*width, img_depth);

          img_l_.header.stamp = stamp;
          img_r_.header.stamp = stamp;
          img_d_.header.stamp = stamp;
          img_depth_.header.stamp = stamp;
          // publish the image
          image_pub_l.publish(img_l_,*ci_l);
          image_pub_r.publish(img_r_,*ci_r);
          image_pub_d.publish(img_d_,*ci_d);
		  image_pub_d.publish(img_depth_,*ci_d);

          point_cloud_pub_.publish(point_cloud);
      }
      
      ros::spinOnce();
      loop_rate.sleep();

    }
    return true;
  }

};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "movesense_sensor");

  if (ros::names::remap("mode") != "mode" || ros::names::remap("param") != "param") {

      if (ros::names::remap("mode") != "mode" )
      {
          ROS_WARN("mode has been remapped, The used mode is '%s'.",
                   ros::names::remap("mopde").c_str());
      }
      if (ros::names::remap("param") != "param") {

        ROS_WARN("param  has been remappe, The used path is '%s'.",
                 ros::names::remap("mode").c_str());
      }
  }
  else
  {
    ROS_INFO("The camera default launch mode is : 'CAM_STEREO_752X480_LRD_30FPS'.\n"
             "default param path  is: ./config"
             "default topic is :\n"
             "\t/movesense_sensor/left/image_raw\n"
             "\t/movesense_sensor/left/camera_info\n"
             "\t/movesense_sensor/right/image_raw\n"
             "\t/movesense_sensor/right/camera_info\n"
             "\t/movesense_sensor/disp/image_raw\n"
             "\t/movesense_sensor/disp/camera_info\n"
             "\t/movesense_sensor/depth/image_raw\n"
             "\t/movesense_sensor/depth/camera_info\n"
             "\t/movesense_sensor/depth/point_cloud\n");
  }

  MoveSenceSensorNode a;
  a.spin();
  return EXIT_SUCCESS;
}

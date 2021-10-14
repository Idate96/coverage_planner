#include "coverage_planner/ImageToGridmap.hpp"

ImageToGridmap::ImageToGridmap(ros::NodeHandle& nodeHandle, std::string layer_name)
    : nodeHandle_(nodeHandle), map_(grid_map::GridMap({"elevation"})), mapInitialized_(false) {
  readParameters();
  map_.setBasicLayers({"elevation"});
  imageSubscriber_ = nodeHandle_.subscribe(imageTopic_, 1, &ImageToGridmap::imageCallback, this);
  gridMapPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
  layerName_ = layer_name;
}

ImageToGridmap::~ImageToGridmap() {}

bool ImageToGridmap::readParameters() {
  nodeHandle_.param("image_topic", imageTopic_, std::string("/image"));
  nodeHandle_.param("resolution", resolution_, 0.03);
  nodeHandle_.param("min_height", minHeight_, 0.0);
  nodeHandle_.param("max_height", maxHeight_, 10.0);
  return true;
}
/*!
 * \brief Saves the images and publishes the grid map.
 * \param msg The image message.
 */
void ImageToGridmap::imageCallback(const sensor_msgs::Image& msg) {
  // There should be a better way to do this. Maybe a service.
  if (!mapInitialized_) {
    grid_map::GridMapRosConverter::initializeFromImage(msg, resolution_, map_);
    ROS_INFO("Initialized map with size %f x %f m (%i x %i cells).", map_.getLength().x(), map_.getLength().y(),
             map_.getSize()(0), map_.getSize()(1));
    mapInitialized_ = true;
  }
  grid_map::GridMapRosConverter::addLayerFromImage(msg, "elevation", map_, minHeight_, maxHeight_);
  map_.add("occupancy");
  map_.get("occupancy") = map_.get("elevation") * 10;
  grid_map::GridMapRosConverter::addColorLayerFromImage(msg, "color", map_);
  // Publish as grid map.
  grid_map_msgs::GridMap mapMessage;
  grid_map::GridMapRosConverter::toMessage(map_, mapMessage);
  ROS_INFO("Publishing new map");
  gridMapPublisher_.publish(mapMessage);

  if (!mapSaved_) {
    std::string pathToBag = ros::package::getPath("coverage_planner") + "/data/occupancy_map.bag";
    std::string topicName = "grid_map";
    grid_map::GridMapRosConverter::saveToBag(map_, pathToBag, topicName);
    mapSaved_ = true;
  }
}

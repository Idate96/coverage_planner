#include "coverage_planner/ImageToGridmap.hpp"

ImageToGridmap::ImageToGridmap(ros::NodeHandle& nodeHandle, std::string layer_name)
    : nodeHandle_(nodeHandle), map_(grid_map::GridMap({"elevation"})), mapInitialized_(false) {
  this->readParameters();
  map_.setBasicLayers({"elevation"});
  map_.setGeometry(grid_map::Length(mapLengthX_, mapLengthY_), resolution_,
                   grid_map::Position(mapOffsetX_, mapOffsetY_));
  imageSubscriber_ = nodeHandle_.subscribe(imageTopic_, 1, &ImageToGridmap::imageCallback, this);
  gridMapPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
  layerName_ = layer_name;
}

ImageToGridmap::~ImageToGridmap() {}

bool ImageToGridmap::readParameters() {
  nodeHandle_.param("image_topic", imageTopic_, std::string("/image"));
  ROS_INFO_STREAM("Image topic: " << imageTopic_);
  nodeHandle_.param("resolution", resolution_, .1);
  nodeHandle_.param("map_offset_x", mapOffsetX_, 0.0);
  nodeHandle_.param("map_offset_y", mapOffsetY_, 0.0);
  nodeHandle_.param("map_length_x", mapLengthX_, 10.0);
  nodeHandle_.param("map_length_y", mapLengthY_, 10.0);
  nodeHandle_.param("min_height", minHeight_, 0.0);
  nodeHandle_.param("max_height", maxHeight_, 1.0);
  return true;
}
/*!
 * \brief Saves the images and publishes the grid map.
 * \param msg The image message.
 */
void ImageToGridmap::imageCallback(const sensor_msgs::Image& msg) {
  // There should be a better way to do this. Maybe a service.
  if (!mapInitialized_) {
    // the image size is overwritten by the number of pixels in the image
    ROS_INFO_STREAM("Map resolution : " << resolution_);
    grid_map::GridMapRosConverter::initializeFromImage(msg, resolution_, map_);
    ROS_INFO("Initialized map with size %f x %f m (%i x %i cells).", map_.getLength().x(),
             map_.getLength().y(), map_.getSize()(0), map_.getSize()(1));
    mapInitialized_ = true;
  }
  grid_map::GridMapRosConverter::addLayerFromImage(msg, "occupancy", map_, minHeight_, maxHeight_);
  // iterate over all cells and set the value to 1 if the cell is occupied
  grid_map::GridMapIterator iterator(map_);
  while (!iterator.isPastEnd()) {
    // ROS_INFO_STREAM("Cell: " << map_.at("occupancy", *iterator));
    if (map_.at("occupancy", *iterator) > 0.5) {
      map_.at("occupancy", *iterator) = 0;
    } else {
      map_.at("occupancy", *iterator) = 1;
    }
    ++iterator;
  }
  map_.add("elevation", 0.0);
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

/*
 * Adapted from ImageToGridmap.hpp
 *
 *  Created on: May 4, 2015
 *      Author: Martin Wermelinger
 *	 Institute: ETH Zurich, ANYbotics
 *
 */

#pragma once

// ROS
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include <string>

class ImageToGridmap {
 public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  ImageToGridmap(ros::NodeHandle& nodeHandle, std::string layerName);

  /*!
   * Destructor.
   */
  virtual ~ImageToGridmap();

  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool readParameters();

  void imageCallback(const sensor_msgs::Image& msg);

  grid_map::GridMap& getMap() { return map_; };

 private:
  //! ROS nodehandle.
  ros::NodeHandle& nodeHandle_;

  //! Grid map publisher.
  ros::Publisher gridMapPublisher_;

  //! Grid map data.
  grid_map::GridMap map_;

  //! Image subscriber
  ros::Subscriber imageSubscriber_;

  //! Name of the grid map topic.
  std::string imageTopic_;

  // Name of the grid map layer;
  std::string layerName_ = "elevation";

  //! Length of the grid map in x direction.
  double mapLengthX_;
  double mapLengthY_;

  // Offset
  double mapOffsetY_ = 0.0;
  double mapOffsetX_ = 0.0;

  //! Resolution of the grid map.
  double resolution_;

  //! Range of the height values.
  double minHeight_;
  double maxHeight_;

  //! Frame id of the grid map.
  std::string mapFrameId_;

  bool mapInitialized_;
  bool mapSaved_ = false;
};

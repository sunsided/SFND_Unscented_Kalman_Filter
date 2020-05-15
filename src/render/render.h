/* \author Aaron Brown */
// Functions and structs used to render the enviroment
// such as cars and the highway

#ifndef RENDER_H
#define RENDER_H

#include <pcl/visualization/pcl_visualizer.h>
#include "box.h"
#include <iostream>
#include <vector>
#include <string>
#include "vect3.h"
#include "color.h"


enum CameraAngle {
    XY, TopDown, Side, FPS
};

void renderHighway(double distancePos, pcl::visualization::PCLVisualizer::Ptr &viewer);

void renderRays(pcl::visualization::PCLVisualizer::Ptr &viewer, const Vect3 &origin, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

void clearRays(pcl::visualization::PCLVisualizer::Ptr &viewer);

void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr &viewer, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                      std::string name, Color color = Color(1, 1, 1));

void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr &viewer, const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                      std::string name, Color color = Color(-1, -1, -1));

void renderBox(pcl::visualization::PCLVisualizer::Ptr &viewer, Box box, int id, Color color = Color(1, 0, 0),
               float opacity = 1);

void renderBox(pcl::visualization::PCLVisualizer::Ptr &viewer, BoxQ box, int id, Color color = Color(1, 0, 0),
               float opacity = 1);

#endif

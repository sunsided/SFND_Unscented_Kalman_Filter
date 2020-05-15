/* \author Aaron Brown */
// Create simple 3d highway environment using PCL
// for exploring self-driving car sensors

#include <fstream>
#include <iostream>
#include "highway.h"

int main(int argc, char **argv) {

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0.133, 0.133, 0.133);

    // set camera position and angle
    viewer->initCameraParameters();
    float x_pos = 0;
    viewer->setCameraPosition(x_pos - 26, 0, 15.0, x_pos + 25, 0, 0, 0, 0, 1);

    Highway highway(viewer);

    int frame_per_sec = 30;
    int sec_interval = 10;
    int frame_count = 0;
    int time_us = 0;

    double egoVelocity = 25;
    constexpr auto microseconds_per_second = 1000000;

    while (frame_count < (frame_per_sec * sec_interval)) {
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        highway.stepHighway(egoVelocity, time_us, frame_per_sec, viewer);
        viewer->spinOnce(1000 / frame_per_sec);
        frame_count++;
        time_us = microseconds_per_second * frame_count / frame_per_sec;
    }

    // Let's violate Liskov's substitution principle a little bit
    // by grabbing directly into some nested properties of some other properties.
    // What could possibly go wrong?
    std::ofstream nisScoresLidar;
    std::ofstream nisScoresRadar;
    nisScoresLidar.open("nis-lidar.csv", std::ios::trunc);
    nisScoresRadar.open("nis-radar.csv", std::ios::trunc);

    nisScoresLidar << "car,time [us],time [s],NIS" << std::endl;
    nisScoresRadar << "car,time [us],time [s],NIS" << std::endl;

    for (auto c = 0; c < highway.traffic.size(); ++c) {
        const auto lidarScores = highway.traffic[c].ukf.nisScoresLidar;
        for (const auto& pair : lidarScores) {
            nisScoresLidar << c << ", " << pair.first << ", " << static_cast<double>(pair.first) / microseconds_per_second << ", " << pair.second << std::endl;
        }

        const auto radarScores = highway.traffic[c].ukf.nisScoresRadar;
        for (const auto& pair : radarScores) {
            nisScoresRadar << c << ", " << pair.first << ", " << static_cast<double>(pair.first) / microseconds_per_second << ", " << pair.second << std::endl;
        }

        nisScoresLidar.flush();
        nisScoresRadar.flush();
    }

    nisScoresLidar.close();
    nisScoresRadar.close();
}

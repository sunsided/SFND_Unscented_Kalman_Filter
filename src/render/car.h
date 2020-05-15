//
// Created by markus on 15.05.20.
//

#ifndef PLAYBACK_CAR_H
#define PLAYBACK_CAR_H

#include <pcl/visualization/pcl_visualizer.h>
#include "box.h"
#include <iostream>
#include <vector>
#include <string>
#include "vect3.h"
#include "color.h"
#include "accuation.h"
#include "../ukf.h"

struct Car {

    // units in meters
    Vect3 position, dimensions;
    Eigen::Quaternionf orientation;
    std::string name;
    Color color;
    float velocity;
    float angle;
    float acceleration;
    float steering;
    // distance between front of vehicle and center of gravity
    float Lf;

    UKF ukf;

    //accuation instructions
    std::vector<accuation> instructions;
    int accuateIndex;

    double sinNegTheta;
    double cosNegTheta;

    Car();

    Car(Vect3 setPosition, Vect3 setDimensions, Color setColor, float setVelocity, float setAngle, float setLf,
        std::string setName);

    Car(Car &&car) = default;

    Car(const Car &car) = default;

    Car &operator=(const Car &) = default;

    Car &operator=(Car &&) = default;

    // angle around z axis
    Eigen::Quaternionf getQuaternion(float theta) const;

    void render(pcl::visualization::PCLVisualizer::Ptr &viewer) const;

    void setAcceleration(float setAcc) {
        acceleration = setAcc;
    }

    void setSteering(float setSteer) {
        steering = setSteer;
    }

    void setInstructions(const std::vector<accuation> setIn) {
        for (accuation a : setIn)
            instructions.push_back(a);
    }

    void setUKF(UKF& tracker) {
        ukf = tracker;
    }

    void move(float dt, int time_us);

    // collision helper function
    constexpr bool inbetween(double point, double center, double range) const {
        return (center - range <= point) && (center + range >= point);
    }

    bool checkCollision(const Vect3& point) const;
};

#endif //PLAYBACK_CAR_H

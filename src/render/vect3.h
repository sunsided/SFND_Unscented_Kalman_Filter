//
// Created by markus on 15.05.20.
//

#ifndef PLAYBACK_VECT3_H
#define PLAYBACK_VECT3_H

struct Vect3 {

    double x, y, z;

    Vect3(double setX, double setY, double setZ)
            : x(setX), y(setY), z(setZ) {}

    Vect3 operator+(const Vect3 &vec) {
        Vect3 result(x + vec.x, y + vec.y, z + vec.z);
        return result;
    }
};

#endif //PLAYBACK_VECT3_H

//
// Created by markus on 15.05.20.
//

#ifndef PLAYBACK_COLOR_H
#define PLAYBACK_COLOR_H

struct Color {

    float r, g, b;

    constexpr Color(float setR, float setG, float setB)
            : r(setR), g(setG), b(setB) {}
};

#endif //PLAYBACK_COLOR_H

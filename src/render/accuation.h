//
// Created by markus on 15.05.20.
//

#ifndef PLAYBACK_ACCUATION_H
#define PLAYBACK_ACCUATION_H

struct accuation {
    long long time_us;
    float acceleration;
    float steering;

    accuation(long long t, float acc, float s)
            : time_us(t), acceleration(acc), steering(s) {}
};

#endif //PLAYBACK_ACCUATION_H

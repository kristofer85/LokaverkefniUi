#ifndef RECTIFY_H
#define RECTIFY_H
#include "utils.h"
#include "histogram.h"

class Rectify
{
public:
    Rectify(matPair pair);
    matPair getCroppedPair();
private:
    matPair LocalMaxima(matPair images);
    matPair GradiantFrame(matPair images);
    void contourCropDiff(matPair images, int &left, int &top, int &right, int &bottom);
    void contourCropGray(matPair, int &left, int &top, int &right, int &bottom);
    matPair RectPair;


};

#endif // RECTIFY_H

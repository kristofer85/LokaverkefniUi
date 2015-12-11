#ifndef RECTIFY_H
#define RECTIFY_H
#include "utils.h"
#include "histogram.h"

class Rectify
{
public:
    //takes an image pair and rectifies and crops its by running it though
    //1 LocalMaxima
    //2 BorderRemoveal from the utils helper class
    //3 GradiantFrame
    //4 contourCropDiff
    //5 contourCropGray
    Rectify(matPair pair);
    //returns the image pair fully croped
    matPair getCroppedPair();
private:

    //This "process" tries to find the best shift for images
    //by finding the highest local maxima of the getCorrectPixelsRatioROI
    //function.
    //After a good shift has been found it crops the images and retuns them
    //to the next process.
    matPair LocalMaxima(matPair images);

    //This process uses a hillclimbing algorithm to try to find a local maximum of
    //the function getCorrectPixelsRatioROI.
    //It starts with the shift (dx,dy) = (0,0) and the looks at the 8 surrounding shifts
    //(-1,0), (1,1) etc and if any of them are better than the current shift
    //it moves there are repeats the process.
    //When none of the surrounding shifts are better than the current one we have found
    //a local maxima.
    //At the bottom commented out is a expandSeach. That is a function to look around when a
    //maximum is found to make sure its a maximum but since we are currently useing the function
    //with the asumption that the LocalMaxima process has already put us very close to the
    //correct shift its not a good idea to look around. But this can be put back in if a
    //hill climbing algorithm is needed for something else.
    //
    //The main point of doing this after localMaxima is that localMaxima is an insanely
    //time complex function O(n^2) where n is total amount of pixels or O(n^4) where n is the height
    //of the image but this function is linear to n if its the total amount of pixels and O(n^2)
    //where n is the height.
    //This allows us to use a higher resolution in this function to its really just to fix
    //the inaccurity of LocalMaxima.
    matPair GradiantFrame(matPair images);

    //crops the image pair based on a gausian threashold found with getThresholdedDiff
    void contourCropDiff(matPair images, int &left, int &top, int &right, int &bottom);

    //crops the image pair based on a threashold found with thresholdGrayToWhite
    //where a pixel is white iff the same pixel in image was gray
    //pixel is gray iff difference in its color channels is less than grayThreshold.
    //we also impose a requierd darkness of a pixel since we are searching for black to gray
    //object so the pixel intencity must be lower than darkness.
    void contourCropGray(matPair, int &left, int &top, int &right, int &bottom);
    matPair RectPair;


};

#endif // RECTIFY_H

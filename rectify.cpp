#include "Rectify.h"
using namespace std;
using namespace cv;
Rectify::Rectify(matPair inputPair)
{
    RectPair.left = inputPair.left;
    RectPair.right = inputPair.right;
    qDebug()<<"finding local maxima";
    RectPair = LocalMaxima(RectPair);
    imwrite("../Lokaverkefni2/croped4MaximaL.jpg",RectPair.left);
    imwrite("../Lokaverkefni2/croped4MaximaR.jpg",RectPair.right);
    qDebug()<<"removing borders";
    RectPair = BorderRemoveal(RectPair);
    imwrite("../Lokaverkefni2/croped4BorderL.jpg",RectPair.left);
    imwrite("../Lokaverkefni2/croped4BorderR.jpg",RectPair.right);
    qDebug()<<"gradiant frame";
    RectPair = GradiantFrame(RectPair);
    imwrite("../Lokaverkefni2/croped4GradiantL.jpg",RectPair.left);
    imwrite("../Lokaverkefni2/croped4GradiantR.jpg",RectPair.right);
    int top = 0;
    int bottom = 0;
    int left = 0;
    int right = 0;
    qDebug()<<"contour crop diff";
    contourCropDiff(RectPair,left,top,right,bottom);
    imwrite("../Lokaverkefni2/croped4contourDifL.jpg",RectPair.left);
    imwrite("../Lokaverkefni2/croped4contourDifR.jpg",RectPair.right);
    top = 0;
    bottom = 0;
    left = 0;
    right = 0;
    qDebug()<<"contour crop gray";
    contourCropGray(RectPair,left,top,right,bottom);
    imwrite("../Lokaverkefni2/croped4contourGrayL.jpg",RectPair.left);
    imwrite("../Lokaverkefni2/croped4contourGrayR.jpg",RectPair.right);
}

matPair Rectify::getCroppedPair()
{
    return RectPair;
}

matPair Rectify::GradiantFrame(matPair images)
{
    //First we check if the process has been canceld by setting our
    //json array to a string.
    /*
    if(cropping_instructions->isString())
    {
            return images;
    }
    */
    matPair grayImages;
    cvtColor(images.left, grayImages.left, CV_BGR2GRAY);
    cvtColor(images.right, grayImages.right, CV_BGR2GRAY);
    int heightSize = 350;
    float heightScale = images.left.size().height/(float)heightSize;
    //cvtColor(images[0],grayImages[0],CV_RGB2GRAY);
    //cvtColor(images[1],grayImages[1],CV_RGB2GRAY);
    if(images.left.size().height>heightSize)
    {
          qDebug()<<"------------------------------------------\n\n\n\n\n"
                    <<"image height was"<<images.left.size().height<<"so we scaled it and"
                      <<"the scale factor is"<<heightScale;
          grayImages.left = ScaleImage(grayImages.left,heightSize, true);
          grayImages.right = ScaleImage(grayImages.right,heightSize, true);
    }
    else
    {
            qDebug()<<"--------------------------------------\n\n\n\n\ndidnt scale the image since it was too smal\n\n\n\n--------------------------------------";
            heightScale = 1;
    }
    //Figure out where we need to place our orgin to match the orginal images by iterating
    //over the instuctions in the json

    //bleh
    //origins shiftedOrigins = getShiftedOrigins(images,*cropping_instructions);


    shiftInfo info = gradiantShiftAndDistortion(grayImages);
    //Mat new_left= Mat::zeros(images[0].size().height, images[0].size().width, images[0].type());
    //Mat new_right = Mat::zeros(images[1].size().height, images[1].size().width, images[1].type());
    //elliptic_transformation(images[0],new_left,info.distortion/images[0].cols,shiftedOrigins.left.x,shiftedOrigins.left.y);
    //elliptic_transformation(images[1],new_right,info.distortion/images[1].cols,shiftedOrigins.right.x, shiftedOrigins.right.y);
    //Rect distortionROILeft = getBarrelDistortionROI(images[0], info.distortion/images[0].cols, shiftedOrigins.left.x, shiftedOrigins.left.y);
    //Rect distortionROIRight = getBarrelDistortionROI(images[1], info.distortion/images[1].cols, shiftedOrigins.right.x, shiftedOrigins.right.y);
    //Rect distortionROI = distortionROILeft & distortionROIRight;
//
    //int top = 0;
    //int right = images[0].cols-(distortionROI.width+distortionROI.x);
    //int bottom = 0;
    //int left = distortionROI.x;
//
    //new_left = cropImageBorders(new_left,top,right,bottom,left);
    //new_right = cropImageBorders(new_right,top,right,bottom,left);
//
    ////int finalDx = dx*heightScale;
    ////Im setting the finalDx to 0 since by the looks of it when we use this function
    ////after local maxima we improove the y coords but mess up the x coords.
    int finalDx = 0;
    int finalDy = info.dy*heightScale;
    //images[0] = cropImage(new_left,finalDx,finalDy);
    //images[1] = cropImage(new_right,-finalDx,-finalDy);
    images.left = cropImage(images.left,finalDx,finalDy);
    images.right = cropImage(images.right,finalDx,finalDy);


    return images;
}

matPair Rectify::LocalMaxima(matPair images)
{
    int histogramSize = 121; //Keep this an odd number or stuff might fail.
    double histogramScaleX;
    double histogramScaleY;


    Vec4i facesROI = findROI(images.left);
    qDebug()<<"\n\n\n\n\n\n\n----------------------------------------------------------------\n"
              <<"Image size is "<<images.left.size().width<<"x"<<images.left.size().height<<"\n"
              <<"Found faces in the box with corners ("<<facesROI[0]<<","<<facesROI[1]<<") and ("
                <<facesROI[2]<<","<<facesROI[3]<<")"
                <<"\n\n\n\n\n\n\n----------------------------------------------------------------\n";

    int numberOfSquares;//Note that this is really the squareroot of the number of squares.
                              //Also since we dont want to select the squares in the border
                              //numberOfSquares need to be greater than 2.
                              //we will gather our data in a histogram object to see where the maxima are.
    //These 2 are really the number of squares our big square is composed of.
    int maxX;
    int maxY;
    int squareWidth;
    int squareHeight;
    if (facesROI[2] == 0 || facesROI[3] == 0) //the faces roi has width or height 0 so its empty
    {
            numberOfSquares = 10;
            maxX = 1;
            maxY = 2;
            squareWidth = 7;
            squareHeight = 6;
    }
    else
    {
            numberOfSquares = 100;
            maxX = (facesROI[0]*numberOfSquares)/images.left.size().width;
            maxY = (facesROI[1]*numberOfSquares)/images.left.size().height;
            squareWidth = (facesROI[2]*numberOfSquares)/images.left.size().width;
            squareHeight = (facesROI[3]*numberOfSquares)/images.left.size().height;
    }
    histogram *hist = new histogram(histogramSize, histogramSize);

    // First we select a (image.size().width()/numberOfSquares)X(images[0].size().height/numberOfSquares)
    // that has a high standard deviation of color values.
    //
    // NOTE: We will need another way of selecting a good square since a blue sky (RGB of (0,0,255) with
    // white clouds (RGB of (255,255,255)) has a very high deviation but its likely that that square
    // will have multiple matches.
    // What we really want to do here is select a square that is probably uniqe.
    // = ScaleImage::getInstance(340, true)->process(images);
    // for now we will select the big square that has a border of 1/5,1/5,1/5,1/10 starting at top and going
    // clockwise, uncomment the loop below for some selection.
   // double maxStdev = 0;
   // for (int y = 1; y < numberOfSquares-squareHeight; y++)
   // {
   //      for (int x = 0; x < numberOfSquares-squareWidth; x++)
   //      {
   //           double tempStdev = getROIStandardDeviation(scaledImages[1],y*scaledImages[1].size().height/numberOfSquares,(x+squareWidth)*scaledImages[1].size().width/numberOfSquares,
   //                                  (y+squareHeight)*scaledImages[1].size().height/numberOfSquares, x*scaledImages[1].size().width/numberOfSquares);
   //           if(tempStdev > maxStdev)
   //       {
   //                maxStdev = tempStdev;
   //                maxX = x;
   //                maxY = y;
   //       }
   //      }
   // }
    qDebug()<<"We are useing (x,y): ("<<maxX<<","<<maxY<<")";
    for(int res = 50; res < 60; res++)
    {
     //Now we search for the local maxima of the image scaled to
         //a resolution of res.
         matPair scaledImages;
         scaledImages.left = ScaleImage(images.left,res, true);
         scaledImages.right = ScaleImage(images.right,res, true);
         vector<Vec2i> pointsVector = findLocalMaxima(scaledImages.left,scaledImages.right,-scaledImages.left.size().width/10,(scaledImages.left.size().width*2)/3,-scaledImages.left.size().height/5,scaledImages.left.size().height/5,
                                      0.75,0.125,0,(maxY*scaledImages.right.size().height)/numberOfSquares,scaledImages.left.size().width-1,((maxX+squareWidth)*scaledImages.right.size().width)/numberOfSquares,
                                      scaledImages.left.size().height-1,((maxY+squareHeight)*scaledImages.right.size().height)/numberOfSquares,0,(maxX*scaledImages.right.size().width)/numberOfSquares,15);
         histogramScaleX = histogramSize/(double)((2*scaledImages.left.size().width)-1);
         histogramScaleY = histogramSize/(double)((2*scaledImages.right.size().height)-1);
         while(pointsVector.size() != 0)
         {
               hist->add(pointsVector.back()[0]*histogramScaleX+(histogramSize/2),
                        pointsVector.back()[1]*histogramScaleY+(histogramSize/2));
               pointsVector.pop_back();
         }
    }
    if(hist->numberOfElements() == 0)
    {
          return images;
    }
    //Vec2i maximum = hist->max();
    //qDebug()<<"histogram max:"<<maximum[0]<<maximum[1];
    histogramScaleX = histogramSize/(double)((2*images.left.size().width)-1);
    histogramScaleY = histogramSize/(double)((2*images.left.size().height)-1);

    //qDebug()<<(maximum[0]-1-(histogramSize/2))<<histogramScaleX;
    //qDebug()<<(maximum[1]-1-(histogramSize/2))<<histogramScaleY;

    //int dx = (maximum[0]-(histogramSize/2))/histogramScaleX;
    //int dy = (maximum[1]-(histogramSize/2))/histogramScaleY;

    int dy = hist->getHighestColumn();
    int dx = hist->getMedianOfColumn(dy);
    dy = hist->getMedianOfRow(dx);
    qDebug()<<"Median value:(x,y): ("<<dx<<","<<dy<<")";
    dx = (dx-(histogramSize/2))/histogramScaleX;
    dy = (dy-(histogramSize/2))/histogramScaleY;

    qDebug()<<"Shift by"<<dx<<dy;
    qDebug()<<hist->toString();

    images.left = cropImage(images.left,dx,-dy);
    images.right = cropImage(images.right,-dx,dy);

    //delete hist;



    return images;
}

void Rectify::contourCropDiff(matPair images, int &left, int &top, int &right, int &bottom)
{
        top = 0;
        bottom = 0;
        left = 0;
        right = 0;
        Mat diff = getThresholdedDiff(images);
        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;
        findContours(diff, contours,hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0));
        for (int i = 0; i< contours.size();i++)
        {
                if(contourArea(contours[i]) > diff.cols*diff.rows*0.02)
                {
                        Mat mask = Mat::zeros(diff.size(), CV_8UC1);
                        drawContours(mask,contours,i,Scalar(255),CV_FILLED);
                        int contourLeft = 0;
                        int contourRight = 0;
                        if(isVerticalRectangle(contours[i], mask, &contourLeft, &contourRight))
                        {
                                left = max(left, contourLeft);
                                right = max(right, contourRight);
                        }
                }
        }
        return;
}


//Post: left, top, right and bottom contain the cropping info for the images.
void Rectify::contourCropGray(matPair images, int &left, int &top, int &right, int &bottom)
{
        top = 0;
        bottom = 0;
        left = 0;
        right = 0;

        vector<vector<Point> > contoursL;
        vector<vector<Point> > contoursR;
        vector<Vec4i> hierarchyL;
        vector<Vec4i> hierarchyR;
        Mat grayThresholdL = thresholdGrayToWhite(images.left);
        Mat grayThresholdR = thresholdGrayToWhite(images.right);
        findContours(grayThresholdL, contoursL,hierarchyL, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0));
        findContours(grayThresholdR, contoursR,hierarchyR, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0));

                for (int i = 0; i< contoursL.size();i++)
                {
                        int areaL = contourArea(contoursL[i]);
                        if (contourArea(contoursL[i]) > 0.02*images.left.cols*images.left.rows)
                        {
                                vector<Point> convex_hull;
                                convexHull(contoursL[i], convex_hull);
                                int convexHullArea = contourArea(convex_hull);
                                Rect contourRect = boundingRect(contoursL[i]);
                                bool isConvex = areaL/(double)convexHullArea > 0.75;
                                bool isAtTop = contourRect.y < 0.05*images.left.rows;
                                bool isAtBottom = contourRect.y+contourRect.height > 0.95*images.left.rows;
                                bool isShortAndWide = contourRect.width > 2*contourRect.height;
                                if(isConvex && (isAtTop || isAtBottom) && isShortAndWide)
                                {
                                        if(isAtTop)
                                        {
                                                top = max(top, contourRect.y+contourRect.height);
                                        }
                                        else
                                        {
                                                bottom = max(bottom, images.left.rows-contourRect.y);
                                        }
                                }
                        }

                }
                for (int i = 0; i< contoursR.size();i++)
                {
                        int areaR = contourArea(contoursR[i]);
                        if (contourArea(contoursR[i]) > 0.02*images.right.cols*images.right.rows)
                        {
                                vector<Point> convex_hull;
                                convexHull(contoursR[i], convex_hull);
                                int convexHullArea = contourArea(convex_hull);
                                Rect contourRect = boundingRect(contoursR[i]);
                                bool isConvex = areaR/(double)convexHullArea > 0.75;
                                bool isAtTop = contourRect.y < 0.05*images.right.rows;
                                bool isAtBottom = contourRect.y+contourRect.height > 0.95*images.right.rows;
                                bool isShortAndWide = contourRect.width > 2*contourRect.height;
                                if(isConvex && (isAtTop || isAtBottom) && isShortAndWide)
                                {
                                        if(isAtTop)
                                        {
                                                top = max(top, contourRect.y+contourRect.height);
                                        }
                                        else
                                        {
                                                bottom = max(bottom, images.right.rows-contourRect.y);
                                        }
                                }
                        }

                }
    RectPair.left = cropImageBorders(images.left,0,right,0,left);
    RectPair.right = cropImageBorders(images.right,0,right,0,left);

}

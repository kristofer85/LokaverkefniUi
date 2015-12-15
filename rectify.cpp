#include "Rectify.h"
using namespace std;
using namespace cv;
Rectify::Rectify(matPair inputPair)
{
    RectPair.left = inputPair.left;
    RectPair.right = inputPair.right;
    qDebug()<<"finding local maxima";
    RectPair = LocalMaxima(RectPair);
    qDebug()<<"removing borders";
    RectPair = BorderRemoveal(RectPair);
    qDebug()<<"gradiant frame";
    RectPair = GradiantFrame(RectPair);
    int top = 0;
    int bottom = 0;
    int left = 0;
    int right = 0;
    qDebug()<<"contour crop diff";
    contourCropDiff(RectPair,left,top,right,bottom);
    top = 0;
    bottom = 0;
    left = 0;
    right = 0;
    qDebug()<<"contour crop gray";
    contourCropGray(RectPair,left,top,right,bottom);
}

matPair Rectify::getCroppedPair()
{
    return RectPair;
}

matPair Rectify::GradiantFrame(matPair images)
{
    matPair grayImages;
    // convert image to grayscale
    cvtColor(images.left, grayImages.left, CV_BGR2GRAY);
    cvtColor(images.right, grayImages.right, CV_BGR2GRAY);
    int heightSize = 350;
    float heightScale = images.left.size().height/(float)heightSize;

    if(images.left.size().height>heightSize)
    {
          qDebug()<<"------------------------------------------\n\n\n\n\n"<<"image height was"<<images.left.size().height<<"so we scaled it and"<<"the scale factor is"<<heightScale;
          grayImages.left = ScaleImage(grayImages.left,heightSize, true);
          grayImages.right = ScaleImage(grayImages.right,heightSize, true);
    }
    else
    {
            qDebug()<<"--------------------------------------\n\n\n\n\ndidnt scale the image since it was too smal\n\n\n\n--------------------------------------";
            heightScale = 1;
    }
    shiftInfo info = gradiantShiftAndDistortion(grayImages);

    int finalDx = 0;
    int finalDy = info.dy*heightScale;
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
    qDebug()<<"\n\n\n\n\n\n\n----------------------------------------------------------------\n"<<"Image size is "<<images.left.size().width<<"x"<<images.left.size().height<<"\n"<<"Found faces in the box with corners ("<<facesROI[0]<<","<<facesROI[1]<<") and ("<<facesROI[2]<<","<<facesROI[3]<<")"<<"\n\n\n\n\n\n\n----------------------------------------------------------------\n";

    int numberOfSquares;//Note that this is really the squareroot of the number of squares.
                              //Also since we dont want to select the squares in the border
                              //numberOfSquares need to be greater than 2.
                              //we will gather our data in a histogram object to see where the maxima are.
    //These 2 are really the number of squares our big square is composed of.
    int maxX;
    int maxY;
    int squareWidth;
    int squareHeight;
    //the faces roi has width or height 0 so its empty
    if (facesROI[2] == 0 || facesROI[3] == 0)
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
    //creates a blank histogram that is 121X121 big
    histogram *hist = new histogram(histogramSize, histogramSize);

    qDebug()<<"We are useing (x,y): ("<<maxX<<","<<maxY<<")";
    for(int res = 50; res < 60; res++)
    {
         // Now we search for the local maxima of the image scaled to a resolution of res.
         matPair scaledImages;
         //scale left and right images
         scaledImages.left = ScaleImage(images.left,res, true);
         scaledImages.right = ScaleImage(images.right,res, true);

         //returns a vector with the local maxima (peaks) of the input images.
         //A local peak is a data sample that is either larger than
         //its two neighboring samples or is equal to Inf.
         //Non-Inf signal endpoints are excluded. If a peak is flat,
         //the function returns only the point with the lowest index.
         vector<Vec2i> pointsVector = findLocalMaxima(scaledImages.left,scaledImages.right,-scaledImages.left.size().width/10,(scaledImages.left.size().width*2)/3,-scaledImages.left.size().height/5,scaledImages.left.size().height/5,
                                      0.75,0.125,0,(maxY*scaledImages.right.size().height)/numberOfSquares,scaledImages.left.size().width-1,((maxX+squareWidth)*scaledImages.right.size().width)/numberOfSquares,
                                      scaledImages.left.size().height-1,((maxY+squareHeight)*scaledImages.right.size().height)/numberOfSquares,0,(maxX*scaledImages.right.size().width)/numberOfSquares,15);
         histogramScaleX = histogramSize/(double)((2*scaledImages.left.size().width)-1);
         histogramScaleY = histogramSize/(double)((2*scaledImages.right.size().height)-1);

         //adds points from pointsVector into the histogram
         while(pointsVector.size() != 0)
         {
               hist->add(pointsVector.back()[0]*histogramScaleX+(histogramSize/2), pointsVector.back()[1]*histogramScaleY+(histogramSize/2));
               pointsVector.pop_back();
         }
    }
    //if there are no elements in the histogram it returns the image as it is
    if(hist->numberOfElements() == 0)
    {
          return images;
    }
    histogramScaleX = histogramSize/(double)((2*images.left.size().width)-1);
    histogramScaleY = histogramSize/(double)((2*images.left.size().height)-1);

    //get the highest column
    int dy = hist->getHighestColumn();

    //get the middle value of the column dy
    int dx = hist->getMedianOfColumn(dy);
    //get the middle value of the row dx
    dy = hist->getMedianOfRow(dx);
    qDebug()<<"Median value:(x,y): ("<<dx<<","<<dy<<")";
    dx = (dx-(histogramSize/2))/histogramScaleX;
    dy = (dy-(histogramSize/2))/histogramScaleY;

    qDebug()<<"Shift by"<<dx<<dy;
    qDebug()<<hist->toString();

    //crops the images based on dx and dy
    images.left = cropImage(images.left,dx,-dy);
    images.right = cropImage(images.right,-dx,dy);

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
        //finds shapes in the image

        //CV_RETR_LIST retrieves all of the contours without establishing any hierarchical relationships.
        //CV_CHAIN_APPROX_SIMPLE compresses horizontal, vertical, and diagonal segments
        //and leaves only their end points. For example, an up-right rectangular contour is encoded with 4 points.
        //CV_RETR_TREE retrieves all of the contours and reconstructs a full hierarchy of nested contours.
        findContours(diff, contours,hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0));

        for (int i = 0; i< contours.size();i++)
        {
                //checks if the calculateed contour area is bigger than diff.cols*diff.rows*0.02
                if(contourArea(contours[i]) > diff.cols*diff.rows*0.02)
                {
                        Mat mask = Mat::zeros(diff.size(), CV_8UC1);
                        //Draws contours outlines or filled contours.
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


//left, top, right and bottom contain the cropping info for the images.
void Rectify::contourCropGray(matPair images, int &left, int &top, int &right, int &bottom)
{
        top = 0;
        bottom = 0;
        left = 0;
        right = 0;
        //shapes and seperate areas in left image
        vector<vector<Point> > contoursL;
        //shapes and seperate areas in right image
        vector<vector<Point> > contoursR;

        //vectors, containing information about the left amd right image topology.
        //It has as many elements as the number of contours.
        //For each i-th contour contours[i] , the elements hierarchy[i][0] ,
        //hiearchy[i][1] , hiearchy[i][2] , and hiearchy[i][3] are set to
        //0-based indices in contours of the next and previous contours
        //at the same hierarchical level, the first child contour and the parent
        //contour, respectively. If for the contour i there are no next,
        //previous, parent, or nested contours, the corresponding elements of
        //hierarchy[i] will be negative
        vector<Vec4i> hierarchyL;
        vector<Vec4i> hierarchyR;

        //replaces all pixels in the input image with luminance greater than set threshold with the value for white (white)
        Mat grayThresholdL = thresholdGrayToWhite(images.left);
        Mat grayThresholdR = thresholdGrayToWhite(images.right);
        //finds shapes in the image

        //CV_RETR_LIST retrieves all of the contours without establishing any hierarchical relationships.
        //CV_CHAIN_APPROX_SIMPLE compresses horizontal, vertical, and diagonal segments
        //and leaves only their end points. For example, an up-right rectangular contour is encoded with 4 points.
        //CV_RETR_TREE retrieves all of the contours and reconstructs a full hierarchy of nested contours.
        findContours(grayThresholdL, contoursL,hierarchyL, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0));
        //finds shapes in the image

        //CV_RETR_LIST retrieves all of the contours without establishing any hierarchical relationships.
        //CV_CHAIN_APPROX_SIMPLE compresses horizontal, vertical, and diagonal segments
        //and leaves only their end points. For example, an up-right rectangular contour is encoded with 4 points.
        //CV_RETR_TREE retrieves all of the contours and reconstructs a full hierarchy of nested contours.
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
                        //Calculates a contour area.
                        int areaR = contourArea(contoursR[i]);
                        if (contourArea(contoursR[i]) > 0.02*images.right.cols*images.right.rows)
                        {
                                vector<Point> convex_hull;
                                //The functions find the convex hull of a 2D point set
                                //using the Sklansky’s algorithm that has O(N logN)
                                //complexity in the current implementation.
                                convexHull(contoursR[i], convex_hull);
                                //Calculates a contour area.
                                int convexHullArea = contourArea(convex_hull);
                                //Calculates the up-right bounding rectangle of a point set.
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
    //crops image pair
    RectPair.left = cropImageBorders(images.left,0,right,0,left);
    RectPair.right = cropImageBorders(images.right,0,right,0,left);

}

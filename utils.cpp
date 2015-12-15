#include "utils.h"
using namespace cv;
using namespace std;


//a helper class containing functions from kula code that I modified to work with our project



matPair Proccess_image(string impath)
{

    //fixes lens destortion caused by zoom
    Mat zoomCorrected = undestortZoom(impath);
    //orginalImage.release();
    matPair lensCorrected;
    imwrite("zoomCorrect.jpg",zoomCorrected);

    //splits image into left and right image
    lensCorrected = splitImage(zoomCorrected);
    imwrite("splitL.jpg",lensCorrected.left);
    imwrite("splitR.jpg",lensCorrected.right);

    //releases memory used by zoomCorrected
    zoomCorrected.release();
    qDebug()<<"split complete";

    //fixes lens destortion caused by Kula Deeper
    lensCorrected = undestort(lensCorrected);
    imwrite("lensCorrectL.png",lensCorrected.left);
    imwrite("lensCorrectR.png",lensCorrected.right);
    return lensCorrected;
}

//Pre: image1 ROI is larger than image2 ROI ( not sure if thats needed )
int deltaCostROI(const Mat &image1, const Mat &image2,int dx, int dy,int top1, int top2, int right1, int right2,
                  int bottom1, int bottom2, int left1, int left2, int eps)
{
    int cost = 0;
    for(int y = top2; y <= bottom2; y++)
    {
        for(int x = left2; x <= right2; x++)
         {
            int x0 = x+dx;
            int y0 = y+dy;
            //cheak if the pixel shifted by (dx,dy), that is (x0,y0) is in image1 ROI.
            if((left1<=x0) && (x0<=right1) && (top1 <= y0) && (y0 <= bottom1))
            {
                if((max(image1.at<uchar>(x0,y0),image2.at<uchar>(x,y))
                    - min(image1.at<uchar>(x0,y0),image2.at<uchar>(x,y)))<eps)
                {
                    cost += 1;
                }
            }
        }
    }
    return cost;
}

int deltaCostColorROI(const Mat &image1, const Mat &image2,int dx, int dy,int top1, int top2, int right1, int right2,
                  int bottom1, int bottom2, int left1, int left2, int eps)
{
    int cost = 0;
    //qDebug()<<image1.size().width<<image1.size().height;
    //qDebug()<<image2.size().width<<image2.size().height;
    for(int y = top2; y <= bottom2; y++)
    {
        for(int x = left2; x <= right2; x++)
        {
            int x0 = x+dx;
            int y0 = y+dy;
            if((left1<=x0) && (x0<=right1) && (top1 <= y0) && (y0 <= bottom1))
            {
               Vec3b currentPixel1 = image1.at<Vec3b>(y0,x0);
               Vec3b currentPixel2 = image2.at<Vec3b>(y,x);
               //qDebug()<<"("<<x0<<","<<y0<<")  ("<<x<<","<<y<<")";
               //qDebug()<<"Blue:"<<currentPixel1[0]<<","<<currentPixel2[0];
               //qDebug()<<"Green:"<<currentPixel1[1]<<","<<currentPixel2[1];
               //qDebug()<<"Red:"<<currentPixel1[2]<<","<<currentPixel2[2]<<"\n";
               if(((max(currentPixel1[0],currentPixel2[0])-min(currentPixel1[0],currentPixel2[0]))<eps)
                    &&  ((max(currentPixel1[1],currentPixel2[1])-min(currentPixel1[1],currentPixel2[1]))<eps)
                    &&  ((max(currentPixel1[2],currentPixel2[2])-min(currentPixel1[2],currentPixel2[2]))<eps))
               {
                   //qDebug()<<"Added to Cost \n\n";
                   //All the color values are within eps from each other.
                   cost++;
               }

             }
         }
     //qDebug()<<"Line:"<<y;
     }
     return cost;
}

float getCorrectPixelsRatioROI(const Mat &image1, const Mat &image2, int dx, int dy, int top1, int top2, int right1, int right2,
                  int bottom1, int bottom2, int left1, int left2, int eps,float maxDistRatio)
{
    if((image1.size().width-abs(dx)<(image1.size().width*(1-maxDistRatio))) || ((image1.size().height-abs(dy))<(image1.size().height*(1-maxDistRatio))))
    {
          //the shift is to close to the edge so we return 0.
          return 0;
    }
    int ROIWidth = min(right1,right2+dx)-max(left1,left2+dx)+1;
    int ROIHeight = min(bottom1,bottom2+dy)-max(top1,top2+dy)+1;
    //qDebug()<<"ROIWidth:"<<ROIWidth<<"| ROIHeight:"<<ROIHeight;

    //Some attempt at scaling the hightgraph so it better represents the reality
    float scaleDistance = max(image1.size().width, image1.size().height);
    int expectedValueX = image1.size().width*0.2;
    int expectedValueY = 0;
    int distFromExpectedValue = sqrt((expectedValueX-dx)*(expectedValueX-dx)+(expectedValueY-dy)*(expectedValueY-dy));
    float scaleFactor = 1-(distFromExpectedValue/scaleDistance);
    //núllum bara factorinn í smástund
    scaleFactor = 1;
        //return deltaCostROI(image1,image2,dx,dy,top1,top2,right1,right2,bottom1,bottom2,left1,left2, eps);//(float)(ROIWidth*ROIHeight);
        return scaleFactor*deltaCostColorROI(image1,image2,dx,dy,top1,top2,right1,right2,bottom1,bottom2,left1,left2, eps)/(float)(ROIWidth*ROIHeight);

}

Mat cropImage(Mat img,int finalDx,int finalDy)
{
    int shiftXL = 0; //how much is croped from the left side
    int shiftXR = 0;//how much is croped from the right side
    int shiftYT = 0;//how much is croped from the top side
    int shiftYB = 0;//how much is croped from the bottom side
    if(finalDx < 0)
    {
        shiftXR = finalDx;
    }
    else
    {
        shiftXL = finalDx;
    }
    if(finalDy < 0)
    {
        shiftYB = finalDy;
    }
    else
    {
        shiftYT = finalDy;
    }
    cout << "croping L=" << shiftXL << " R= " << shiftXR << " T= " << shiftYT << " B= " << shiftYB << endl;
    Mat temp;
    temp = img(Range(0+shiftYT, img.rows+shiftYB),Range(0+shiftXL, img.cols+shiftXR)).clone();

    return temp;
}

Mat getThresholdedDiff(matPair images, int threshold)
{
        Mat gauss0, gauss1;

        //Smooths an image using the Gaussian filter. with the kernel size 3x3
        GaussianBlur(images.left, gauss0, Size(3,3),0,0);
        GaussianBlur(images.right, gauss1, Size(3,3),0,0);
        Mat diff(images.left.size(), CV_8UC1);
        for (int i = 0; i < diff.rows; i++)
        {
                for (int j = 0; j < diff.cols; j++)
                {
                        Vec3b pixel0 = gauss0.at<Vec3b>(i,j);
                        Vec3b pixel1 = gauss1.at<Vec3b>(i,j);
                        uchar value = (uchar)(0.0722*abs(pixel0[0]-pixel1[0])+0.7152*abs(pixel0[1]-pixel1[1])+0.2126*abs(pixel0[2]-pixel1[2]));
                        if(value > threshold)
                        {
                                diff.at<uchar>(i,j) = 255;
                        }
                        else
                        {
                                diff.at<uchar>(i,j) = 0;
                        }

                }
        }
        return diff;
}

Mat thresholdGrayToWhite(Mat image, int grayThreshold, int darkness)
{

        Mat out = Mat::zeros(image.size(), CV_8UC1);
        for(int j = 0;j != image.cols;j++)
        {
                for(int i = 0; i != image.rows; i++)
                {
                        Vec3b pixel = image.at<Vec3b>(i,j);
                        //gets the pixel with the highest color value
                        uchar maxPixelValue = max(pixel[0], max(pixel[1], pixel[2]));
                        //gets the pixel with the lowest color value
                        uchar minPixelValue = min(pixel[0], min(pixel[1], pixel[2]));
                        if(maxPixelValue - minPixelValue < grayThreshold && maxPixelValue < darkness)
                        {
                                out.at<uchar>(i,j) = 255;
                        }
                }
        }
        return out;

}

bool isVerticalRectangle(vector<Point> contour, Mat innerPoints, int * left, int * right)
{
        //Calculates the up-right bounding rectangle of a point set.
        Rect contourRect =  boundingRect(contour);
        if(contourRect.x > innerPoints.cols*0.15 && contourRect.x+contourRect.width < innerPoints.cols*0.85)
        {
                return false;
        }
        areaByHeightAndWidthParams params;
        params.contour = contour;
        params.innerPoints = innerPoints;
        params.height = -1;
        params.width = contourRect.width;
        int area = contourArea(contour);
        int rectangleHeight = binarySearch(0,contourRect.height,area*0.9, (void*)&params, areaByHeightAndWidth);
        params.height = rectangleHeight;
        params.width = -1;
        int rectangleWidth = binarySearch(0,contourRect.width,area*0.8, (void*)&params, areaByHeightAndWidth);
        if((rectangleHeight*rectangleWidth*0.87 < area ) && (rectangleHeight > 2.5*rectangleWidth))
        {
                if(left != 0 && right != 0)
                {
                        *left = 0;
                        *right = 0;
                        //Calculates all of the moments up to the third order of a polygon or rasterized shape.
                        Moments contourMoments = moments(contour);
                        int centerWidth = (int)(contourMoments.m10/contourMoments.m00);
                        if(centerWidth < innerPoints.cols/2)
                        {
                                *left = centerWidth + (rectangleWidth/2);
                        }
                        else
                        {
                                *right = innerPoints.cols-centerWidth+rectangleWidth/2;
                        }
                }
                return true;
        }
        else
        {
                return false;
        }
}

//implements a simple version of a binary search
//a binary search works in a way that you have some boundrys lets say 1 and 100
//and you want to find in the least amount of tries a random number between those
//lets say we pick a random number 75
//then you check if it is half way between those two or 50 if it is higher 50 then
//lower bound becomes 50 and higher bounds stays 100 and you ask next is the number
//small larger or equal to 75 because the mid value between 50 an 100 is 75 and so on
//until you find it or lower bound and upper bound are the same then that number
//is not between lower and upper bound
int binarySearch(int lowerBound, int upperBound, double x , void * params,double(*f)(int, void *))
{
        //while you are not in the middle
        while(upperBound>lowerBound)
        {
                int m = (upperBound + lowerBound)/2;
                double f_m = f(m, params);
                qDebug()<<"Searching for"<<x<<"in ["<<lowerBound<<";"<<upperBound<<"]";
                qDebug()<<"f(m) ="<<f_m;
                if(f_m>x)
                {
                       upperBound = m;
                }
                else
                {
                        lowerBound = m+1;
                }
        }
        return lowerBound;
}

shiftInfo gradiantShiftAndDistortion(matPair undistorted_images)
{

    int dx = 0;//find better starting values (maby with line detection or statistics of correct shifts).
    int dy = 0;
    int maxNeighborX = dx+1;
    int maxNeighborY = dy;
    int eps = 20;
    float maxRatio, tmpRatio;
    bool facesUsed = false;
    Mat gray_left = undistorted_images.left.clone();
    Mat gray_right = undistorted_images.right.clone();
    matPair grayImages;
    grayImages.left = gray_left;
    grayImages.right = gray_right;

    Vec4i facesROI = findROI(grayImages.left);
    qDebug()<<"FACE: Didnt find a face";
    int numberOfSquares;
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
            maxX = (facesROI[0]*numberOfSquares)/grayImages.left.size().width;
            maxY = (facesROI[1]*numberOfSquares)/grayImages.left.size().height;
            squareWidth = (facesROI[2]*numberOfSquares)/grayImages.left.size().width;
            squareHeight = (facesROI[3]*numberOfSquares)/grayImages.left.size().height;
            //In most cases there is a body below the face that we also want to focus on
            //so we increase the hight
            squareHeight = squareHeight*4;
    }

    int top0 = (maxY*grayImages.left.size().height)/numberOfSquares;
    int top1 = (maxY*grayImages.left.size().height)/numberOfSquares;
    int bottom0 = ((maxY+squareHeight)*grayImages.left.size().height)/numberOfSquares;
    int bottom1 = ((maxY+squareHeight)*grayImages.left.size().height)/numberOfSquares;
    int left0 = (maxX*grayImages.left.size().width)/numberOfSquares;
    int left1 = (maxX*grayImages.left.size().width)/numberOfSquares;
    int right0 = ((maxX+squareWidth)*grayImages.left.size().width)/numberOfSquares;
    int right1 = ((maxX+squareWidth)*grayImages.left.size().width)/numberOfSquares;

    //Since we increased the height of the search boxes we might be at the bottom of the image
    //so we check that edge case and move the shorten the box if needed
    if(bottom0 > grayImages.left.size().height*0.9)
    {
            bottom0 = (int)(grayImages.left.size().height*0.9);
    }
    if(bottom1 > grayImages.right.size().height*0.9)
    {
            bottom1 = (int)(grayImages.right.size().height*0.9);
    }

    //since we increased the height of the roi we might be out of bounds
    bottom0 = min(bottom0,grayImages.left.size().height);
    bottom1 = min(bottom1,grayImages.left.size().height);
    double distortion = 0;
    //while(1)
    //{
         while((maxNeighborX != dx) || (maxNeighborY != dy))
         {
             dx = maxNeighborX;
             dy = maxNeighborY;
             maxRatio = getCorrectPixelsRatioROI(grayImages.left,grayImages.right,dx,dy,top0,top1,
                          right0,right1,bottom0,bottom1,left0,left1,eps)+0.000001;
             if(dx < grayImages.left.size().width) //we are not at the right edge of the image.
             {
                 tmpRatio = getCorrectPixelsRatioROI(grayImages.left,grayImages.right,dx+1,dy,top0,top1,
                                   right0,right1,bottom0,bottom1,left0,left1,eps);
                 if(maxRatio<tmpRatio)
                 {
                     maxRatio = tmpRatio;
                     maxNeighborX = dx+1;
                     maxNeighborY = dy;
                 }
                 if (dy<grayImages.left.size().height)
                 {
                     tmpRatio = getCorrectPixelsRatioROI(grayImages.left,grayImages.right,dx+1,dy+1,top0,top1,
                                      right0,right1,bottom0,bottom1,left0,left1,eps);
                     if(maxRatio<tmpRatio)
                     {
                         maxRatio = tmpRatio;
                         maxNeighborX = dx+1;
                         maxNeighborY = dy+1;
                     }
                 }
                 if (-grayImages.left.size().height<dy)
                 {
                     tmpRatio = getCorrectPixelsRatioROI(grayImages.left,grayImages.right,dx+1,dy-1,top0,top1,
                                      right0,right1,bottom0,bottom1,left0,left1,eps);
                     if(maxRatio<tmpRatio)
                     {
                         maxRatio = tmpRatio;
                         maxNeighborX = dx+1;
                         maxNeighborY = dy-1;
                     }
                 }
             }
             if(-grayImages.left.size().width < dx) //we are not at the left edge of the image.
             {
                 tmpRatio = getCorrectPixelsRatioROI(grayImages.left,grayImages.right,dx-1,dy,top0,top1,
                                      right0,right1,bottom0,bottom1,left0,left1,eps);
                 if(maxRatio<tmpRatio)
                 {
                     maxRatio = tmpRatio;
                     maxNeighborX = dx-1;
                     maxNeighborY = dy;
                 }
                 if (dy<grayImages.left.size().height)
                 {
                     tmpRatio = getCorrectPixelsRatioROI(grayImages.left,grayImages.right,dx-1,dy+1,top0,top1,
                                      right0,right1,bottom0,bottom1,left0,left1,eps);
                     if(maxRatio<tmpRatio)
                     {
                         maxRatio = tmpRatio;
                         maxNeighborX = dx-1;
                         maxNeighborY = dy+1;
                     }
                 }
                 if (0<dy)
                 {
                     tmpRatio = getCorrectPixelsRatioROI(grayImages.left,grayImages.right,dx-1,dy-1,top0,top1,
                                      right0,right1,bottom0,bottom1,left0,left1,eps);
                     if(maxRatio<tmpRatio)
                     {
                         maxRatio = tmpRatio;
                         maxNeighborX = dx-1;
                         maxNeighborY = dy-1;
                     }
                 }
             }
             if (dy<grayImages.left.size().height)
             {
                 tmpRatio = getCorrectPixelsRatioROI(grayImages.left,grayImages.right,dx,dy+1,top0,top1,
                                      right0,right1,bottom0,bottom1,left0,left1,eps);
                 if(maxRatio<tmpRatio)
                 {
                     maxRatio = tmpRatio;
                     maxNeighborX = dx;
                     maxNeighborY = dy+1;
                 }
             }
             if (0<dy)
             {
                 tmpRatio = getCorrectPixelsRatioROI(grayImages.left,grayImages.right,dx,dy-1,top0,top1,
                                      right0,right1,bottom0,bottom1,left0,left1,eps);
                 if(maxRatio<tmpRatio)
                 {
                     maxRatio = tmpRatio;
                     maxNeighborX = dx;
                     maxNeighborY = dy-1;
                 }
             }
         }

     shiftInfo returnInfo;
     returnInfo.dx = dx;
     returnInfo.dy = dy;
     returnInfo.distortion = distortion;

     return returnInfo;

}

vector<Vec2i> findLocalMaxima(const Mat &image1, const Mat &image2, int dx1, int dx2, int dy1, int dy2, float threshold,float maximaSize,
                              int top1, int top2, int right1, int right2, int bottom1, int bottom2, int left1, int left2,int eps)
{
     //First we gather all the point into a vector of vectors with 3 ints, dx,dy and 10000 times the
     //correct pixels ratio at (dx,dy). We do this since its expensive to look at the correct pixels ratio
     //and we will need to do that O(n*log(n)) times during the sorting.
     vector<Vec3i> pointsVector;
     pointsVector.reserve((dx2-dx1)*(dy2-dy1));
     for(int dy = dy1; dy <= dy2; dy++)
     {
          for(int dx = dx1; dx <= dx2; dx++)
          {
                 Vec3i temp;
                 temp[0] = dx;
                 temp[1] = dy;
                 temp[2] = (int)(10000*getCorrectPixelsRatioROI(image1,image2,dx,dy,top1,top2,right1,right2,
                                                                bottom1,bottom2,left1,left2,eps));
                 pointsVector.push_back(temp);
                 //qDebug()<<temp[0]<<temp[1]<<temp[2];
          }
     }
     //Sort the vector according the the ordering defined below (just decending order for the 3rd element).
     sort(pointsVector.begin(),pointsVector.end(),localMaximaCompare);
     //Now we define a variable that is the minimum value a point can have so it can qualify as a maxima.
     int minValue = threshold*pointsVector.back()[2];
     //Now we define a new vector that contains all points that are a local maxima.
     vector<Vec2i> localMaxima;
     //qDebug()<<"next Value:"<<pointsVector.back()[0]<<pointsVector.back()[1]<<pointsVector.back()[2];
     while((pointsVector.back()[2] > minValue) && pointsVector.size() != 0)
     {
         //qDebug()<<"seinni loopa";
         //We define a local maxima so that a point is a local maxima if it has a higher value than all points
         //in a max(dx2-dx1,dy2-dy1)*maximaSize radius. Since the points are in accending order (decending since
         //we always take the last element) a point is a local maxima if and only if its distance from the already
         //found maxima is greater than max(dx2-dx1,dy2-dy1)*maximaSize.
     int n = 0;
         int maxProximity = max(dx2-dx1,dy2-dy1)*maximaSize;
         bool isMaxima = true;
         while(n < localMaxima.size())
         {
              //iterate over the maxima vector to see if the new point is to close to a previus maxima.
              if (sqrt((pointsVector.back()[0]-localMaxima[n][0])*(pointsVector.back()[0]-localMaxima[n][0])
                + (pointsVector.back()[1]-localMaxima[n][1])*(pointsVector.back()[1]-localMaxima[n][1]))< maxProximity)
              {
                   //qDebug()<<pointsVector.back()[0]<<","<<pointsVector.back()[1]<<" is to close to"<<localMaxima[n][0]<<localMaxima[n][1];
                   //we found a local maxima to close to the point so its not a local maxima
                   isMaxima = false;
                   break;
              }
              n++;
         }
         if(isMaxima)
         {
          Vec2i newMaxima;
              newMaxima[0] = pointsVector.back()[0];
              newMaxima[1] = pointsVector.back()[1];
              localMaxima.push_back(newMaxima);
         }
         pointsVector.pop_back();
     }
     return localMaxima;
}

bool localMaximaCompare(Vec3i x,Vec3i y)
{
     if(x[2] < y[2])
     {
           return true;
     }
     return false;
}
//Returns the sum of the standard deviation of the color values of the selected ROI of an image.
double getROIStandardDeviation(const Mat &image, int top, int right, int bottom, int left)
{
     int sumRed = 0;
     int sumGreen = 0;
     int sumBlue = 0;
     int sum2Red = 0;
     int sum2Green = 0;
     int sum2Blue = 0;
     double stdevR, stdevG, stdevB;
     Vec3b currentPoint;
     for(int n = top; n <= bottom; n++)
     {
           for(int m = left; m <= right; m++)
           {
                 currentPoint = image.at<Vec3b>(n,m);
                 sumBlue += currentPoint[0];
                 sum2Blue += currentPoint[0]*currentPoint[0];
                 sumGreen += currentPoint[1];
                 sum2Green += currentPoint[1]*currentPoint[1];
                 sumRed += currentPoint[2];
                 sum2Red += currentPoint[2]*currentPoint[2];
       }
     }
     int N = (bottom-top+1)*(right-left+1);
     stdevB = sqrt((sum2Blue/N)-((sumBlue*sumBlue)/(N*N)));
     stdevG = sqrt((sum2Green/N)-((sumGreen*sumGreen)/(N*N)));
     stdevR = sqrt((sum2Red/N)-((sumRed*sumRed)/(N*N)));
     return stdevB+stdevG+stdevR;
}



//a function to split an image
matPair splitImage(cv::Mat fullImage)
{
    float midper = 0.00;

    //resize(fullImage,fullImage,Size(),0.5,0.5);
    Size imSize = fullImage.size();
    //int midArea = imSize.width * midper;

    //Mat leftImg = fullImage(Range(0, imSize.height),Range(0, imSize.width/2 + midArea)).clone();

    //Mat rightImg = fullImage(Range(0, imSize.height),Range(imSize.width/2 - midArea, imSize.width)).clone();
    matPair temp;
    //temp.left = fullImage(Range(0, imSize.height),Range(0, imSize.width/2 - midArea)).clone();
    //temp.right = fullImage(Range(0, imSize.height),Range(imSize.width/2 + midArea, imSize.width)).clone();
    temp.left = fullImage(Range(0, imSize.height),Range(0, imSize.width/2)).clone();
    temp.right = fullImage(Range(0, imSize.height),Range(imSize.width/2, imSize.width)).clone();
    /*
    namedWindow("left",WINDOW_NORMAL| WINDOW_KEEPRATIO);
    namedWindow("right",WINDOW_NORMAL| WINDOW_KEEPRATIO);
    imshow("left",temp.left);
    imshow("right",temp.right);
    */
    qDebug()<<"resizeing ";
    //temp.resize(0.50);
    return temp;
}


//a functin that reads focalResolution from the metadata saved in the image
double getFocalResolution(string imagePath)
{
    Exiv2::Image::AutoPtr image = Exiv2::ImageFactory::open(imagePath);
    if(image.get() == 0)
    {
            return 0;
    }
    image->readMetadata();

    Exiv2::ExifData &exifData = image->exifData();
    if (exifData.empty())
    {
            return 0;
    }
    Exiv2::ExifData::const_iterator end = exifData.end();
    for (Exiv2::ExifData::const_iterator i = exifData.begin(); i != end; ++i) {
            if(i->groupName() == "Photo" && i->tagName() == "FocalPlaneXResolution")
            {
                    return i->value().toFloat(0);
            }
    }
    return 0;
}

void tangent_distortion_correction(Mat src_mat, Mat * dst_mat, float left, float right)
{
        Mat map_x, map_y;
        map_x.create( src_mat.size(), CV_32FC1 );
        map_y.create( src_mat.size(), CV_32FC1 );
        for( int i = 0; i < src_mat.cols; i++ )
        {
                for( int j = 0; j < src_mat.rows; j++ )
                {
                        map_x.at<float>(j,i) = i;
                        map_y.at<float>(j,i) = ((j-0.5*src_mat.rows)*(left+(((right-left)*i)/src_mat.cols)))+(0.5*src_mat.rows);
                }
        }
        remap( src_mat, *dst_mat, map_x, map_y, CV_INTER_LINEAR, BORDER_CONSTANT, Scalar(0,0, 0) );

}
/*
void shift_image(Mat src_mat, Mat * dst_mat, float up, float left)
{
        Mat map_x, map_y;
        map_x.create( src_mat.size(), CV_32FC1 );
        map_y.create( src_mat.size(), CV_32FC1 );
        for( int i = 0; i < src_mat.cols; i++ )
        {
                for( int j = 0; j < src_mat.rows; j++ )
                {
                        map_x.at<float>(j,i) = i-left;
                        map_y.at<float>(j,i) = j-up;
                }
        }
        remap( src_mat, *dst_mat, map_x, map_y, CV_INTER_LINEAR, BORDER_CONSTANT, Scalar(0,0, 0) );

}
*/


int findSideBox(const Mat &image, double maxStdev,int numberOfLines, float maxSizeRatio, int maxColorDiff, double grayScaleSize, char
 selection)
{
       //selections: 0: top, 1: right, 2: bottom, 3: left.
       int firstRow, lastRow, firstColumn, lastColumn,numberOfPixels,outerBegin,outerEnd,innerBegin,innerEnd,incrementor;
       double meanRed,meanGreen,meanBlue,stdevRed,stdevGreen,stdevBlue;
       long sampleSumRed = 0;
       long sampleSumGreen = 0;
       long sampleSumBlue = 0;
       long sampleSumRed2 = 0;
       long sampleSumGreen2 = 0;
       long sampleSumBlue2 = 0;

       switch(selection)
       {
              //Find out wich case we are in and select the rows and columns of the
              //sample lines.
              case 0:
                    firstRow = 0;
                    lastRow = numberOfLines;
                    firstColumn = 0;
                    lastColumn = image.size().width-1;
                    outerBegin = 0;
                    outerEnd = image.size().height-1;
                    innerBegin = 0;
                    innerEnd = image.size().width-1;
                    incrementor = 1;
                    break;
              case 1:
                    firstRow = 0;
                    lastRow = image.size().height-1;
                    firstColumn = image.size().width-numberOfLines-1;
                    lastColumn =image.size().width -1;
                    outerBegin = image.size().width-1;
                    outerEnd = 0;
                    innerBegin = 0;
                    innerEnd = image.size().height-1;
                    incrementor = -1;
                    break;
              case 2:
                    firstRow = image.size().height - numberOfLines -1;
                    lastRow = image.size().height - 1;
                    firstColumn = 0;
                    lastColumn = image.size().width-1;
                    outerBegin = image.size().height-1;
                    outerEnd = 0;
                    innerBegin = 0;
                    innerEnd = image.size().width-1;
                    incrementor = -1;
                    break;
              case 3:
                    firstRow = 0;
                    lastRow = image.size().height-1;
                    firstColumn = 0;
                    lastColumn = numberOfLines;
                    outerBegin = 0;
                    outerEnd = image.size().width-1;
                    innerBegin = 0;
                    innerEnd = image.size().height-1;
                    incrementor = 1;
                    break;

       }
       //qDebug()<<"first loop of finding box";
       for(int n = firstRow; n <= lastRow;n++)
       {
              //Collect the sum and squared sum of the elements in the rows and columns we
              //selected.
             // qDebug()<<"inner loop 1 and n ="<<n;
              for(int m = firstColumn; m <= lastColumn; m++)
              {
                    Vec3b pointVector = image.at<Vec3b>(n,m);
                    sampleSumBlue += pointVector[0];
                    sampleSumBlue2 += pointVector[0]*pointVector[0];
                    sampleSumGreen += pointVector[1];
                    sampleSumGreen2 += pointVector[1]*pointVector[1];
                    sampleSumRed += pointVector[2];
                    sampleSumRed2 += pointVector[2]*pointVector[2];
              }
        }
        if (selection % 2 == 0)
        {
              //we are in case 0 or 2 so EF24-70mm f/2.8L USMwe are going verticaly
              numberOfPixels = image.size().width*(numberOfLines+1);
        }
        else
        {
              numberOfPixels = image.size().height*(numberOfLines+1);
        }
    //qDebug()<<"Second loop";
        for(int n = outerBegin; abs(n-outerBegin)<=abs(outerBegin - outerEnd)*maxSizeRatio; n+=incrementor)
        {
              long sumRed = sampleSumRed;
              long sumGreen = sampleSumGreen;
              long sumBlue = sampleSumBlue;
              long sumRed2 = sampleSumRed2;
              long sumGreen2 = sampleSumGreen2;
              long sumBlue2 = sampleSumBlue2;
             // qDebug()<<"mode:"<<(int)selection<<"second inner loop and n ="<<n;
              for(int m = 0; abs(m-innerBegin)<=abs(innerBegin - innerEnd);m++)
              {
                   // if( selection == 3) qDebug()<<"mode"<<selection<<"inside second inner loop"<<n<<m;
                    Vec3b pointVector;
                    if(selection % 2 == 0)
                    {
                          pointVector = image.at<Vec3b>(n,m);
                    }
                    else
                    {
                          pointVector = image.at<Vec3b>(m,n);
                    }
                    sumBlue += pointVector[0];
                    sumBlue2 += pointVector[0]*pointVector[0];
                    sumGreen += pointVector[1];
                    sumGreen2 += pointVector[1]*pointVector[1];
                    sumRed += pointVector[2];
                    sumRed2 += pointVector[2]*pointVector[2];
              }
              meanRed = sumRed/(double)numberOfPixels;
              meanGreen = sumGreen/(double)numberOfPixels;
              meanBlue = sumBlue/(double)numberOfPixels;

              stdevRed = sqrt((sumRed2/(double)numberOfPixels)-(meanRed*meanRed));
              stdevGreen = sqrt((sumGreen2/(double)numberOfPixels)-(meanGreen*meanGreen));
              stdevBlue = sqrt((sumBlue2/(double)numberOfPixels)-(meanBlue*meanBlue));

              if((stdevRed>maxStdev) || (stdevGreen>maxStdev) || (stdevBlue>maxStdev)
                || (max(max(meanRed,meanGreen),meanBlue)-min(min(meanRed,meanGreen),meanBlue)) > maxColorDiff
                ||  (meanRed+meanGreen+meanBlue)/3 > grayScaleSize*255)
              {
                      if (selection==0 || selection == 3)
                      {
                            return n;
                      }
                      if (selection == 1)
                      {
                            return image.size().width - n;
                      }
                      else return image.size().height - n;
              }
        }
        return 0;
}

matPair BorderRemoveal(matPair pair)
{

    int topL,bottomL,topR,bottomR,leftL,leftR,rightL,rightR,left,right,top,bottom;

    topL =    findSideBox(pair.left,5,0,0.4,10,0.5,0);
    bottomL = findSideBox(pair.left,5,0,0.4,10,0.5,2);
    leftL =   findSideBox(pair.left,5,0,0.4,10,0.5,3);
    rightL =  findSideBox(pair.left,5,0,0.4,10,0.5,1);
    topR =    findSideBox(pair.right,5,0,0.4,10,0.5,0);
    bottomR = findSideBox(pair.right,5,0,0.4,10,0.5,2);
    leftR =   findSideBox(pair.right,5,0,0.4,10,0.5,3);
    rightR =  findSideBox(pair.right,5,0,0.4,10,0.5,1);


    bottom = max(bottomL,bottomR);
    top = max(topL,topR);
    left = max(leftL,leftR);
    right = max(rightL,rightR);

    pair.left = cropImageBorders(pair.left,top,right,bottom,left);
    pair.right = cropImageBorders(pair.right,top,right,bottom,left);
    return pair;

}

Mat cropImageBorders(Mat image, int top, int right, int bottom, int left)
{
        qDebug()<<"croping borders";
        Mat subImage;
        subImage =image(Range(top,image.size().height-bottom), Range(left, image.size().width-right)).clone();
        return subImage;
}

Mat getCameraMatrix(float zoom, int width, int height)
{
       Mat cameraMatrix = Mat::zeros(3,3, CV_64F);
       cameraMatrix.at<double>(2,2) = 1.0;
       double fx,fy,cx,cy;
       //double focalResMm = 155.21693;
       fx = zoom;
       cx = width/2.0;
       cy = height/2.0;
       fy = fx;


       cameraMatrix.at<double>(0,0) = fx;
       cameraMatrix.at<double>(0,2) = cx;
       cameraMatrix.at<double>(1,1) = fy;
       cameraMatrix.at<double>(1,2) = cy;

       return cameraMatrix;
}

bool getDistCoeffs(Mat &distCoeffs, float zoom, string filename)
{
       distCoeffs = Mat::zeros(5,1, CV_64F);

       //distCoeffs.at<double>(0,0) = 3.6562459162927829e-01;
       //distCoeffs.at<double>(1,0) = 1.9073063052260267e+01;
       //distCoeffs.at<double>(2,0) = 0.0;
       //distCoeffs.at<double>(3,0) = 0.0;
       //distCoeffs.at<double>(4,0) = 1.9073063052260267e+01;
       distCoeffs.at<double>(2,0) = 0.0;
       distCoeffs.at<double>(3,0) = 0.0;
       double k1, k2, k3;
       string lens = "Sigma 17-35mm f/2.8-4 EX DG";


       //if(!getDistortionParameters(getLensName(filename),zoom,k1,k2,k3))
       if(!getDistortionParameters(lens,zoom,k1,k2,k3))
       {
               qDebug()<<"getDistortionParameters returned false";
               return false;
       }
       //Found Coeffs k1: 0.00722227 , k2: -0.0200405 , k3: 0
       distCoeffs.at<double>(0,0) = k1;
       distCoeffs.at<double>(1,0) = k2;
       distCoeffs.at<double>(4,0) = k3;
       qDebug()<<"Found Coeffs k1:"<<k1<<", k2:"<<k2<<", k3:"<<k3;

       return true;
}

bool getDistortionParameters(string cameraName, double focal, double &k1, double &k2, double &k3)
{

        setlocale(LC_ALL, "");
        struct lfDatabase *ldb;
        ldb = lf_db_new();
        ldb->Load();
        cout << ldb->HomeDataDir << endl;

        const lfLens **lenses = ldb->FindLenses (NULL, NULL, cameraName.c_str());
        if(!lenses)
        {
                qDebug()<<"WARNING: did not find the lens used";
                return false;
        }
        lfLensCalibDistortion results;
        lenses[0]->InterpolateDistortion(focal, results);
        k1 = results.Terms[0];
        k2 = results.Terms[1];
        k3 = results.Terms[2];
        ldb->Destroy();
        qDebug()<<"Found distortion coeffs";
        return true;

}

Exiv2::Image::AutoPtr unicodeExiv2Open(QString srcPath, QString *linkPath)
{

        qDebug()<<"Exif: Opening image:"<<srcPath;
        qDebug()<<"linkPath:"<<*linkPath;
        return Exiv2::ImageFactory::open(srcPath.toStdString());
        /*
        if(stringIsAscii(srcPath))
        {
                *linkPath = "";
                return Exiv2::ImageFactory::open(srcPath.toStdString());
        }
        else
        {
                qDebug()<<"Exif: image path not ascii";
                QUuid imageUuid = QUuid::createUuid();
                *linkPath = QDir::temp().absoluteFilePath(imageUuid).append(".jpg");
                qDebug()<<"Linking image to:";
                qDebug()<<*linkPath;
                windowsSafeLink(QFileInfo(srcPath).absoluteFilePath(), *linkPath);
                Exiv2::Image::AutoPtr returnVal = Exiv2::ImageFactory::open(linkPath->toStdString());
                //QFile::remove(QDir::temp().absoluteFilePath(imageUuid)).append(".jpg");
                //QFile::remove(*linkPath);
                qDebug()<<"Exif: opened non ascii path";
                return returnVal;
        }
        */
}

float getZoomValue(string imagePath)
{
        qDebug()<<"getting zoom value";
        QString * linkPath = new QString("");
        Exiv2::Image::AutoPtr image = unicodeExiv2Open(imagePath.c_str(), linkPath);
        float zoom = -1.0;
        if(image.get() == 0)
        {
                qDebug()<<"Could not load image";
        }
        else
        {
                image->readMetadata();

                Exiv2::ExifData &exifData = image->exifData();
                if (exifData.empty()) {
                        qDebug()<<"No exif data found";
                }
                else
                {
                        Exiv2::ExifData::const_iterator end = exifData.end();
                        for (Exiv2::ExifData::const_iterator i = exifData.begin(); i != end; ++i) {
                                if(i->groupName() == "Photo" && i->tagName() == "FocalLength")
                                {

                                        zoom = i->value().toFloat();
                                        qDebug()<<"Success" ;
                                        cout <<"Success focal length = "<< i->value() << endl;

                                }
                        }
                }
        }
        /*
        if(*linkPath != "")
        {
                QFile::remove(*linkPath);
        }
*/
        image.release();
        return zoom;
}

matPair undestort(matPair pair)
{

    Mat left_tangent= Mat::zeros(pair.left.size().height, pair.left.size().width, pair.left.type());
    Mat right_tangent = Mat::zeros(pair.right.size().height, pair.right.size().width, pair.right.type());
    tangent_distortion_correction(pair.left, &left_tangent, 1.0, 1.0-DISTORTION); // magic number found by mesuring images taken by deeper

    pair.left = left_tangent;
    left_tangent.release();
    tangent_distortion_correction(pair.right, &right_tangent, 1.0-DISTORTION, 1.0);//really just an educated guess.

    // for debuging
    //namedWindow("orginal",WINDOW_NORMAL);
    //namedWindow("tangentdestort",WINDOW_NORMAL);
    //imshow("orginal",left);
    //imshow("tangentdestort",left_tangent);
    //

    pair.right = right_tangent;
    right_tangent.release();
    //img1 = pair.left;
    //img2 = pair.right;
    //waitKey(0);

    return pair;
}

Mat undestortZoom(string file_name)
{
    Mat image = imread(file_name,IMREAD_COLOR);
    resize(image,image,Size(),0.5,0.5);
    qDebug()<<"Undistort via zoom info";
    Mat distCoeffs;
    float zoom_value = getZoomValue(file_name);
    if(!getDistCoeffs(distCoeffs, zoom_value, file_name))
    {
            qDebug()<<"Failed to get distortion coeffs";
            return image;
    }
    double focalRes = getFocalResolution(file_name);

    if (focalRes*zoom_value < 1)
    {
            qDebug()<<"No focal resolution found using a standin value";
            focalRes = 4615.05;
    }
    Mat cameraMatrix = getCameraMatrix(zoom_value*focalRes, image.cols, image.rows);

    Mat undistorted, map1, map2;

    qDebug()<<"Image size is"<<image.size().width<<"x"<<image.size().height;
    qDebug()<<"cols x rows"<<image.cols<<image.rows;



    initUndistortRectifyMap(cameraMatrix,distCoeffs, Mat(),
                            cameraMatrix,
                            image.size(), CV_16SC2, map1, map2);
    cout << "remaping" << endl;
    remap(image, undistorted, map1, map2, INTER_LINEAR);
    cout << "zoom remap complete" << endl;
    //char buffer[256];
    //sprintf(buffer, "Zoom %4.2f", zoom_value);
    //putText(undistorted,buffer ,Point(100,200),FONT_HERSHEY_SIMPLEX,5.0, Scalar(255.0,0.0,0.0), 5);
    image = undistorted;
    qDebug()<<"done undestort by zoom";
    return image;
}

Mat ScaleImage(Mat image,int scale,bool scalePx)
{
    cv::Mat scaled;
    cv::Size size = image.size();
    if(scalePx) {
        float scaleFactor = (float)scale / size.height;
        size.height = scale;
        size.width = size.width * scaleFactor;
        qDebug() << size.height << " " << size.width;
    } else {
        size.width = size.width * scale/100;
        size.height = size.height * scale/100;
    }

    cv::resize(image, scaled, size, 0, 0, cv::INTER_LANCZOS4);
    //return undistortViaZoom(scaled,0.0);
    return scaled;
}

//This function attempts to find a suitable search region of an image
//This is done via facedetection and later via other methods aswell.
//The function returns a vector a where
//a[0] = x0
//a[1] = y0
//a[2] = width
//a[3] = height
//where (x0,y0) is the upper left corner with the width and height
Vec4i findROI(Mat image)
{
        //first andso far only thing we do is to try to find large persons
        //that are likely to be the subject of the image.
        CascadeClassifier face_cascade;
        Vec4i returnVector;
        vector<Rect> faces;
        Mat image_gray;
        Size min_size(image.size().width*0.1,image.size().height*0.1);



        if( !face_cascade.load("haarcascade_frontalface_alt.xml") )
        {
               returnVector[0] = 0;
               returnVector[1] = 0;
               returnVector[2] = 0;
               returnVector[3] = 0;
               return returnVector;
        }

        cvtColor( image, image_gray, CV_BGR2GRAY );
        equalizeHist( image_gray, image_gray );
        face_cascade.detectMultiScale( image_gray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, min_size );

        //faces now contains all the faces that are more than 10% of the image height and width.
        //Now lets find the minimum bounding rectangle that contains all the faces.

        //first return an empty rect if we found no faces
        qDebug()<<"Found"<<faces.size()<<"faces";
        //if(faces.size() == 0)
        //{
               returnVector[0] = 0;
               returnVector[1] = 0;
               returnVector[2] = 0;
               returnVector[3] = 0;
        //       return returnVector;
        //}
        if(faces.size() == 1)
        {
                returnVector[0] = faces[0].x;
                returnVector[1] = faces[0].y;
                returnVector[2] = faces[0].width;
                returnVector[3] = faces[0].height;
        }

        //Then make the inital roi from the first face
        /*
        returnVector[0] = faces[0].x;
        returnVector[1] = faces[0].y;
        returnVector[2] = faces[0].width;
        returnVector[3] = faces[0].height;
        //now loop over the rest of the faces and make the rect bigger as needed
        for(int i = 1; i<faces.size();i++)
        {
               if(faces[i].x < returnVector[0])
               {
                       returnVector[0] = faces[i].x;
               }
               if(faces[i].y < returnVector[1])
               {
                       returnVector[1] = faces[i].y;
               }
               if(returnVector[2] < faces[i].width)
               {
                       returnVector[2] = faces[i].width;
               }
               if(returnVector[3] < faces[i].height)
               {
                       returnVector[3] = faces[i].height;
               }
        }*/
        return returnVector;



}

double areaByHeightAndWidth(int other,void * params)
{
        areaByHeightAndWidthParams * paramsInStruct = (areaByHeightAndWidthParams *) params;
        vector<Point> contour = paramsInStruct->contour;
        Mat innerPoints = paramsInStruct->innerPoints;
        int height = paramsInStruct->height;
        int width = paramsInStruct->width;
        if(height < 0)
        {
                height = other;
        }
        if(width < 0)
        {
                width = other;
        }


        Moments contourMoments = moments(contour);
        int centerHeight = (int)(contourMoments.m01/contourMoments.m00);
        int centerWidth = (int)(contourMoments.m10/contourMoments.m00);
        Rect contourROI = boundingRect(contour);
        int top = centerHeight - (height/2);
        int bottom = centerHeight + (height/2);
        int left = centerWidth - (width/2);
        int right = centerWidth + (width/2);
        if (top < 0)
        {
                top = 0;
        }
        if(bottom >= innerPoints.rows )
        {
                bottom = innerPoints.rows-1;
        }
        if(left < 0)
        {
                left = 0;
        }
        if(right >= innerPoints.cols)
        {
                right = innerPoints.cols-1;
        }
        int area = 0;
        for(int j = left;j != right;j++)
        {
                for(int i = top; i != bottom; i++)
                {
                        if(innerPoints.at<uchar>(i,j) == 255)
                        {
                                area++;
                        }
                }
        }
        return (double) area;
}

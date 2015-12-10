#include "histogram.h"
using namespace cv;
using namespace std;
histogram::histogram(int n, int m)
{
    rows = n;
    columns = m;
    dataVector.resize(n*m,0); //núllstilla
    numberOfEls = 0;
}

void histogram::add(int n, int m)
{
      dataVector[n+rows*m]++;
      numberOfEls++;
}

cv::Vec2i histogram::max()
{
      int maxValue = 0;
      Vec2i returnVector;
      returnVector[0] = 0;
      returnVector[1] = 0;
      for( int n = 0; n < rows; n++)
      {
           for( int m = 0; m < columns; m++)
           {
                if(maxValue < dataVector[n+m*rows])
                {
                      maxValue = dataVector[n+m*rows];
                      returnVector[0] = n;
                      returnVector[1] = m;
                }
           }
      }
      return returnVector;
}

QString histogram::toString()
{
      QString output = "Histogram:\n  || ";
      QString temp;
      for(int i = 0; i < rows;i++)
      {
           temp.setNum(i);
           if(temp.length()<2)
           {
                 output +=temp+" | ";
           }
           else
           {
         output += temp+"| ";
       }
      }
      output +="\n";
      for(int n = 0; n<rows;n++)
      {
           temp.setNum(n);
           if(temp.length()<2)
           {
                output +="\n"+temp+" || ";
           }
           else
           {
                output +="\n"+temp+"|| ";
           }
           for(int m = 0;m<columns;m++)
           {
                  temp.setNum(dataVector[n+rows*m]);
                  if(temp.toInt()==0)
                  {
                         output += "  | ";
                  }
                  else
                  {
                         if(temp.length()<2)
                         {
                                output += temp+" | ";
                         }
                         else
                         {
                                output += temp+"| ";
                         }
                  }
           }
      }
      return output;
}
int histogram::getHighestRow()
{
     int maxValue = 0;
     int maxRow = 0;
     for(int n = 0; n <rows; n++)
     {
          int currentRowValue = 0;
          for(int m = 0; m < columns; m++)
      {
                currentRowValue += dataVector[n+rows*m];
          }
          if(maxValue < currentRowValue)
          {
                maxRow = n;
                maxValue = currentRowValue;
          }
     }
     return maxRow;
}
int histogram::getHighestColumn()
{
     int maxValue = 0;
     int maxColumn = 0;
     for(int m = 0; m < columns; m++)
     {
          int currentColumnValue = 0;
          for(int n = 0; n < rows; n++)
          {
               currentColumnValue += dataVector[n+rows*m];
          }
          if(maxValue < currentColumnValue)
          {
                maxColumn = m;
                maxValue = currentColumnValue;
          }
     }
     return maxColumn;
}
int histogram::getMedianOfColumn(int column)
{
     int halfColumnTotal = 0;
     int columnTotal = 0;
     int medianRow = 0;
     for(int n = 0; n < rows; n++)
     {
           halfColumnTotal += dataVector[n+rows*column];
     }
     halfColumnTotal = halfColumnTotal/2;
     while(columnTotal < halfColumnTotal)
     {
          columnTotal += dataVector[medianRow+rows*column];
          medianRow++;
     }
     return medianRow-1;
}
int histogram::getMedianOfRow(int row)
{
     int halfRowTotal = 0;
     int rowTotal = 0;
     int medianColumn = 0;
     for(int m = 0; m < columns; m++)
     {
          halfRowTotal += dataVector[row + rows*m];
     }
     halfRowTotal = (halfRowTotal+1)/2;
     while(rowTotal < halfRowTotal)
     {
          rowTotal += dataVector[row + rows*medianColumn];
          medianColumn++;
     }
     return medianColumn-1;
}
bool histogram::numberOfElements()
{
     return numberOfEls;
}

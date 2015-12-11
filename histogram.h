#ifndef HISTOGRAM_H
#define HISTOGRAM_H
#include "utils.h"
class histogram
{
//This class is a histogram and the value in row n and column m is in
//dataVector[n+m*rows]
public:
    histogram(int n, int m);
    ~histogram();
    void add(int n, int m);

    //  returns the higerst color value for both row and column
    cv::Vec2i max();
    QString toString();

    // returns the row with rhe highest color value
    int getHighestRow();

    // returns the column with rhe highest color value
    int getHighestColumn();

    // returns median of rows
    int getMedianOfRow(int row);

    // returns median of columns
    int getMedianOfColumn(int column);
    bool numberOfElements();
private:
    std::vector<int> dataVector;
    int rows;
    int columns;
    int numberOfEls;
};
#endif // HISTOGRAM_H

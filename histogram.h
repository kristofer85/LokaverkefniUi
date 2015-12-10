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
    cv::Vec2i max();
    QString toString();
    int getHighestRow();
    int getHighestColumn();
    int getMedianOfRow(int row);
    int getMedianOfColumn(int column);
    bool numberOfElements();

private:
    std::vector<int> dataVector;
    int rows;
    int columns;
    int numberOfEls;
};

#endif // HISTOGRAM_H

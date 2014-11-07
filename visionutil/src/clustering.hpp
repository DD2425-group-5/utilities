#ifndef CLUSTERING_HPP
#define CLUSTERING_HPP

#include "opencv2/core/core.hpp"
#include "cmath"


namespace clustering {

class Cluster;

void cluster_img(const std::vector<cv::Vec2i> points, double dist, std::vector<Cluster>& clusters);
Cluster get_biggest_cluster(std::vector<Cluster>& clusters);

class Cluster {
public:
    int row;
    int col;
    std::vector<cv::Vec2i> points;

    //init a Cluster with one point
    Cluster(int row, int col) : row(row), col(col), sum_row(row), sum_col(col) {
        points.push_back(cv::Vec2i(row,col));
    }

    void add_point(int row, int col) {
        sum_row += row;
        sum_col += col;
        this->row = sum_row/points.size();
        this->col = sum_col/points.size();
        points.push_back(cv::Vec2i(row,col));
    }

    int size() {
        return points.size();
    }

    bool within_dist(double row, double col, double dist) {
        return dist >= std::sqrt(std::pow(std::abs(row-this->row),2) + std::pow(std::abs(col-this->col),2));
    }

private:
    double sum_row;
    double sum_col;

};

}




#endif // CLUSTERING_H

#include "clustering.hpp"

namespace clustering {

void cluster_img(const std::vector<cv::Vec2i> points, double dist, std::vector<Cluster>& clusters) {
    //clusters.clear();
    for(int i = 0; i < points.size(); ++i) {
        bool point_done = false;
        const cv::Vec2i& p = points[i];
        //check if p is within distance of a cluster.
        for(int c = 0; c < clusters.size();++c) {
            if(clusters[c].within_dist(p[0],p[1],dist)) {
                clusters[c].add_point(p[0],p[1]);
                point_done = true;
                break;
            }
        }
        if(!point_done) {
            clusters.push_back(Cluster(p[0],p[1]));
        }
    }

}

void cluster_img_mat(const cv::Mat& img, float thresh, double dist, std::vector<Cluster>& clusters) {


    for(int row = 0; row < img.rows; ++row) {
    for(int col = 0; col < img.cols; ++col) {
        bool point_done = false;
        if(img.at<float>(row,col) >= thresh) {
            //check if p is within distance of a cluster.
            for(int c = 0; c < clusters.size();++c) {
                if(clusters[c].within_dist(row,col,dist)) {
                    clusters[c].add_point(row,col);
                    point_done = true;
                    //break;
                }
            }
            if(!point_done) {
                clusters.push_back(Cluster(row,col));
            }
        }
    }
    }

}

/* Note: assumes sums is already initialized to sums.size==img.rows */
void rows_sum(const cv::Mat& img, std::vector<float>& sums, float min_val) {

    for(int i = 0; i < img.rows; ++i) {
        cv::Mat row = img.row(i);
        for(int j = 0; j < row.cols; ++j) {
            float tmp = row.at<float>(0,j);
            if(tmp >= min_val)
                sums[i] += tmp;
        }

        //sums[i] = cv::sum(img.row(i))[0];
    }
}

/* Note: assumes sums is already initialized to sums.size==img.cols */
void cols_sum(const cv::Mat& img, std::vector<float>& sums, float min_val) {
    for(int i = 0; i < img.cols; ++i) {
        cv::Mat col = img.col(i);
        for(int j = 0; j < col.rows; ++j) {
            float tmp = col.at<float>(0,j);
            if(tmp >= min_val)
                sums[i] += tmp;
        }

        //sums[i] = cv::sum(img.row(i))[0];
    }

    /*for(int i = 0; i < img.cols; ++i) {
        sums[i] = cv::sum(img.col(i))[0];
    }*/
}


Cluster get_biggest_cluster(std::vector<Cluster>& clusters) {
    int max_size = -1;
    int max_index = -1;
    for(int c = 0; c < clusters.size(); ++c) {
        if(clusters[c].size() > max_size) {
            max_size = clusters[c].size();
            max_index = c;
        }
    }

    if(max_index == -1) {
        return Cluster(-1,-1);
    } else {
        return clusters[max_index];
    }

}


}

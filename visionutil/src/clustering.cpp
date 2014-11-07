#include "clustering.hpp"

namespace clustering {

void cluster_img(const std::vector<cv::Vec2i> points, double dist, std::vector<Cluster>& clusters) {
    clusters.clear();
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

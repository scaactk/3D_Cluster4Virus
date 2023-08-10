#include"iostream"
#include<cmath>
#include"algorithm"
#include"vector"

struct Point {
    double x;
    double y;
    double z;
};


const static inline double distance(double_t x1, double_t y1, double_t z1, double_t x2, double_t y2, double_t z2) {
    double dx = x2 - x1;
    double dy = y2 - y1;
    double dz = z2 - z1;
    return sqrt(dx * dx + dy * dy + dz * dz);
}

const inline int region_query(const std::vector<Point> &input, int query, std::vector<int> &local_cluster, double eps) {
    for (int i = 0; i < (int) input.size(); i++) {
        if (distance(input[i].x, input[i].y, input[i].z,
                     input[query].x, input[query].y, input[query].z) < eps) {
            local_cluster.push_back(i); // index
        }
    }
    return local_cluster.size();
}

bool
expand_cluster(const std::vector<Point> &input, int query, std::vector<int> &output, int clusterID, double eps,
               int min) {
    std::vector<int> local_cluster;

    int local_cluster_size = region_query(input, query, local_cluster, eps);
    if (local_cluster_size < min) {
        // find noise
        output[query] = -1;
        return false;
    } else {
        // set cluster id
        for (int i = 0; i < (int) local_cluster.size(); i++) {
            output[local_cluster[i]] = clusterID;
        }
        // delete current queried point
        local_cluster.erase(std::remove(local_cluster.begin(), local_cluster.end(), query), local_cluster.end());

        // check other points which have been added in this cluster
        while (local_cluster.size() > 0) {
            int ptr = local_cluster.front();
            std::vector<int> temp_cluster;

            // if new temp_cluster size of current point from local_cluster > min ,,,
            // is this needed? if the current point is on the border
            // answer: yes, this is the requirement of dbscan, the algorithm is defined as this,
            // each point's density is high enough
            if (region_query(input, ptr, temp_cluster, eps)> min){
                for (int i=0; i<temp_cluster.size(); i++){
                    //find new required point
                    // -1 means noise before, 0 means new point as default
                    if (local_cluster[temp_cluster[i]]==-1){
                        // do not need to check whether this point has neighbour again
                        //local_cluster.push_back(temp_cluster[i]);
                        output[temp_cluster[i]] = clusterID;
                    }
                    else if(local_cluster[temp_cluster[i]]==0){
                        // add to local_cluster check list
                        local_cluster.push_back(temp_cluster[i]);
                        output[temp_cluster[i]] = clusterID;
                    }
                    else{ // already in local_cluster check list
                        continue;
                    }
                }
            }
            // remove front ,next will be the new front
            local_cluster.erase(std::remove(local_cluster.begin(), local_cluster.end(), ptr), local_cluster.end());

        }


    }


    return true;
}


int dbscan(const std::vector<Point> &input, std::vector<int> &labels, double eps, int min) {
    int size = input.size();
    int clusterID = 1;

    std::vector<int> output(size); // if only given size, all element for int vector is 0
    for (int i = 0; i < size; i++) {
        if (output[i]==0) {
            if (expand_cluster(input, i, output, clusterID, eps, min)) {
                clusterID++;
            }
        }
    }
    labels = output;
    return clusterID - 1;

}

int main(){
    // test
    std::vector<Point> points(10);

    points[0].x = 20; points[0].y = 21;
    points[1].x = 20; points[1].y = 25;
    points[2].x = 28; points[2].y = 22;
    points[3].x = 30; points[3].y = 52;
    points[4].x = 26; points[4].y = 70;
    points[5].x = 30; points[5].y = 75;
    points[6].x = 0; points[6].y = 70;
    points[7].x = 70; points[7].y = 50;
    points[8].x = 67; points[8].y = 69;
    points[9].x = 80; points[9].y = 35;

    std::vector<int> labels;

    int num = dbscan(points, labels, 20.0, 3);

    std::cout<< "cluster size is "<< num << std::endl;

    for(int i = 0; i < (int)points.size(); i++){
        std::cout<<"Point("<<points[i].x<<", "<<points[i].y<<"): "<<labels[i]<<std::endl;
    }

    return 0;
}

#include <cmath>
#include <nn.h>
#define INF 0x3f3f3f3f

int find_min(int row, std::vector<std::vector<float>> arr, std::vector<int> ignore){
    std::vector<float> rowVec = arr[row];
    float min = INF;
    int min_idx;
    bool skip;
    for (int j=0; j < rowVec.size(); j++){
        skip = false;
        for (int i=0; i < ignore.size(); i++){
            if (j == ignore[i]){
                skip = true;
            }
        }
        if (!skip){
            if (rowVec[j] < min){
                min = rowVec[j];
                min_idx = j;
            }
        }
    }
    return min_idx;
}

std::vector<std::vector<float>> nn(std::vector<std::vector<float>> V, std::vector<float> s){
    V.insert(V.begin(), s); // add source to beginning of locations vector
    std::vector<std::vector<float>> arr; // distances array
    std::vector<std::vector<float>> path; // destination vector

    std::vector<float> row;
    float d;

    // create distance array
    for (int i=0; i < V.size(); i++) {
        row.clear();
        for (int j=0; j < V.size(); j++) {
            if (i == j){
                row.push_back(INF);
            } else {
                d = sqrt(pow(V[j][0] - V[i][0], 2) + pow(V[j][1] - V[i][1], 2)); // get distance between coords
                row.push_back(d);
            }
        }
        arr.push_back(row);
    }

    // find minimums
    std::vector<int> ignore;
    int next = 0;
    for (int i=0; i < arr.size(); i++){
        ignore.push_back(next);
        next = find_min(next, arr, ignore);
    }

    // generate destination path, starting from 1 node after the source
    for (int i=1; i < ignore.size(); i++){
        path.push_back(V[ignore[i]]);
    }
    path.push_back(s); // add final path
    return path;
}
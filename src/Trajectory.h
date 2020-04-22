#ifndef PATH_PLANNING_TRAJECTORY_H
#define PATH_PLANNING_TRAJECTORY_H

#include <vector>

using namespace std;

class Trajectory {
public:
    Trajectory() : is_initialized(false) {}

    void init(vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_s);

    vector<vector<double>> Calc(vector<double> intended_goal, double car_x, double car_y, double car_yaw,
                                vector<double> previous_path_x, vector<double> previous_path_y);

private:
    // initialized flag
    bool is_initialized;
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
};


#endif //PATH_PLANNING_TRAJECTORY_H

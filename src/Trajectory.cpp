#include "Trajectory.h"
#include "vector"
#include "helpers.h"
#include "spline.h"

using namespace std;

void
::Trajectory::init(vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_s) {
    this->map_waypoints_x = map_waypoints_x;
    this->map_waypoints_y = map_waypoints_y;
    this->map_waypoints_s = map_waypoints_s;

    is_initialized = true;
}

vector<std::vector<double>>
Trajectory::Calc(std::vector<double> intended_goal, double car_x, double car_y, double car_yaw,
                 std::vector<double> previous_path_x, std::vector<double> previous_path_y) {
    vector<double> points_x;
    vector<double> points_y;

    double ref_x = car_x;
    double ref_y = car_y;
    double ref_yaw = deg2rad(car_yaw);

    int prev_size = previous_path_x.size();

    if (prev_size < 2) {
        // 2 points make path tangent for car
        double prev_car_x = car_x - cos(car_yaw);
        double prev_car_y = car_y - sin(car_yaw);

        points_x.push_back(prev_car_x);
        points_x.push_back(car_x);

        points_y.push_back(prev_car_y);
        points_y.push_back(car_y);
    } else {
        ref_x = previous_path_x[prev_size - 1];
        ref_y = previous_path_y[prev_size - 1];

        double ref_x_prev = previous_path_x[prev_size - 2];
        double ref_y_prev = previous_path_y[prev_size - 2];
        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

        points_x.push_back(ref_x_prev);
        points_x.push_back(ref_x);

        points_y.push_back(ref_y_prev);
        points_y.push_back(ref_y);
    }

    // next trajectory point as goal
    vector<double> intended_goal_points = getXY(intended_goal[0],
                                                intended_goal[1],
                                                map_waypoints_s,
                                                map_waypoints_x,
                                                map_waypoints_y);
    points_x.push_back(intended_goal_points[0]);
    points_y.push_back(intended_goal_points[1]);

    // to local car coordinate
    for (size_t i = 0; i < points_x.size(); ++i) {
        double shift_x = points_x[i] - ref_x;
        double shift_y = points_y[i] - ref_y;
        points_x[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
        points_y[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
    }

    // spline to calc trajectory
    tk::spline spline;
    spline.set_points(points_x, points_y);

    // next points from trajectory
    vector<double> next_x_vals;
    vector<double> next_y_vals;

    // from all the remaining previous path points from last time
    for (size_t i = 0; i < prev_size; ++i) {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }

    double target_x = 30.0;
    double target_y = spline(target_x);
    double target_dist = sqrt(target_x * target_x + target_y * target_y);
    double N = target_dist / (0.02 * intended_goal[2] / 2.24); // convert to m/s

    double last_x = 0.0;

    for (size_t i = 0; i < 50 - prev_size; ++i) {
        double next_x = last_x + target_x / N;
        double next_y = spline(next_x);

        last_x = next_x;

        // to map coordinate
        double x_ref = next_x;
        double y_ref = next_y;
        next_x = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
        next_y = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

        next_x += ref_x;
        next_y += ref_y;

        next_x_vals.push_back(next_x);
        next_y_vals.push_back(next_y);
    }

    vector<vector<double>> trajectory;
    trajectory.push_back(next_x_vals);
    trajectory.push_back(next_y_vals);

    return trajectory;
}
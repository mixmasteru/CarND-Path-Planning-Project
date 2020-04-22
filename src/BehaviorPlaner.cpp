#include "BehaviorPlaner.h"
#include <vector>
#include <cmath>

using namespace std;

BehaviorPlaner::BehaviorPlaner() {
    ref_vel = 0.0;
    speed_limit = 49.0; // mph
    safe_dist = 30.0; // meters
    max_acc = 0.224; // m/s2
}

BehaviorPlaner::~BehaviorPlaner() = default;


vector<double> BehaviorPlaner::CalcBehavior(vector<vector<double>> sensor_fusion, double car_s, double car_d) {
    bool should_change_lane = false;
    double front_car_speed = 0;
    double front_car_s = INFINITY;
    vector<vector<double>> car_from_left;
    vector<vector<double>> car_from_right;

    int lane = DetectLaneFromCarPos(car_d);

    for (auto &vehicle : sensor_fusion) {
        // get vehicle states
        double x = vehicle[3];
        double y = vehicle[4];
        double s = vehicle[5];
        double d = vehicle[6];

        // calc s value for the detected and ego car after step
        double speed = sqrt(x * x + y * y);
        s += (double) speed * 0.02;

        // check lane
        int detected_car_lane = DetectLaneFromCarPos(d);
        bool in_same_lane = lane == detected_car_lane;
        bool from_left = lane - 1 == detected_car_lane;
        bool from_right = lane + 1 == detected_car_lane;

        // check if close from car
        bool is_close = (s > car_s) && (s - car_s < safe_dist);

        // try lane change if car is close in the same lane
        if (in_same_lane && is_close) {
            should_change_lane = true;
            if (s < front_car_s) {
                front_car_speed = speed;
                front_car_s = s;
            }
        } else if (from_left) {
            car_from_left.push_back(vehicle);
        } else if (from_right) {
            car_from_right.push_back(vehicle);
        }
    }

    // try lane changing, if not slow down
    if (should_change_lane) {
        return TryChangingLane(car_from_left, car_from_right, car_s, car_d, front_car_speed);
    }
    // stay on the same lane
    else {
        return SetGoal(car_s + safe_dist, 2 + 4 * lane, UpdateSpeed());
    }
}

int BehaviorPlaner::DetectLaneFromCarPos(double car_d) {
    if (car_d < 4) {
        return 0;
    } else if (car_d >= 4 && car_d < 8) {
        return 1;
    } else {
        return 2;
    }
}

vector<double>
BehaviorPlaner::TryChangingLane(const vector<vector<double>> &car_from_left,
                                const vector<vector<double>> &car_from_right,
                                double car_s, double car_d,
                                double front_car_speed) {
    // current lane
    int lane = DetectLaneFromCarPos(car_d);

    // lanes check
    bool is_left_lane_safe = CheckLane(car_from_left, car_s, car_d);
    bool is_right_lane_safe = CheckLane(car_from_right, car_s, car_d);

    // if not left, try right
    if ((lane == 0 || (lane == 1 && !is_left_lane_safe)) && is_right_lane_safe) {
        return SetGoal(car_s + 1.5 * safe_dist, 2 + 4 * (lane + 1), UpdateSpeed());
    }
    // check best lane
    else if (lane == 1 && is_left_lane_safe && is_right_lane_safe) {
        int best = BestLane(car_from_left, car_from_right, car_s);
        return SetGoal(car_s + 1.5 * safe_dist, 2 + 4 * best, UpdateSpeed());
    }
    // if not go right, try left
    else if ((lane == 2 || ((lane == 1))) && is_left_lane_safe) {
        return SetGoal(car_s + 1.5 * safe_dist, 2 + 4 * (lane - 1), UpdateSpeed());
    }
    // slow down
    else {
        return SetGoal(car_s + safe_dist, 2 + 4 * lane, UpdateSpeed(front_car_speed));
    }
}

bool BehaviorPlaner::CheckLane(vector<vector<double>> sensor_fusion, double car_s, double car_d) {
    // check all cars of intended lane
    for (auto &i : sensor_fusion) {
        // get car state
        double x = i[3];
        double v = i[4];
        double s = i[5];

        double speed = sqrt(x * x + v * v);
        s += (double) speed * .02;

        // check cars in front of car
        if ((s >= car_s) && (s - car_s <= 2.0 * safe_dist)) {
            return false;
        }

        // check cars behind of car
        double min_dist = 0.25 * safe_dist;
        double max_dist = 1.0 * safe_dist;
        double dist = (1 - ref_vel / safe_dist) * max_dist + min_dist;
        if ((s <= car_s) && (car_s - s <= dist)) {
            return false;
        }
    }

    return true;
}

int BehaviorPlaner::BestLane(vector<vector<double>> sensor_fusion_left, vector<vector<double>> sensor_fusion_right,
                             double car_s) {
    double closest_left_car_dist = INFINITY;
    double closest_right_car_dist = INFINITY;

    // left lane closest car
    for (auto &i : sensor_fusion_left) {
        // get detected car state
        double x = i[3];
        double y = i[4];
        double s = i[5];

        double speed = sqrt(x * x + y * y);
        s += (double) speed * .02;

        if ((s > car_s) && (s - car_s < closest_left_car_dist)) {
            closest_left_car_dist = s - car_s;
        }
    }

    // right lane closest car
    for (auto &i : sensor_fusion_right) {
        // get detected car state
        double x = i[3];
        double y = i[4];
        double s = i[5];

        double speed = sqrt(x * x + y * y);
        s += (double) speed * .02;

        if ((s > car_s) && (s - car_s < closest_right_car_dist)) {
            closest_right_car_dist = s - car_s;
        }
    }

    // go left, if cars are far away in front of the car
    // when the left front car is faster than the right front car
    if ((closest_left_car_dist >= 3 * safe_dist && closest_right_car_dist >= 3 * safe_dist) ||
        closest_left_car_dist >= closest_right_car_dist) {
        return 0;
    }
        // Otherwise, go right
    else {
        return 2;
    }
}

double BehaviorPlaner::UpdateSpeed(double speed_to_match) {
    // default speedup
    double updated_speed = ref_vel + max_acc;

    // try to get lane speed
    if (speed_to_match != -1) {
        speed_to_match *= 2.24 * .98; // mph // .98 to prevent overshooting front car speed

        // slow down for matching lane speed
        if (max_acc <= ref_vel) {
            updated_speed = max(speed_to_match, ref_vel - 1.5 * max_acc);
        }
            // speedup to match lane speed
        else {
            updated_speed = min(speed_to_match, ref_vel + max_acc);
        }
    }

    // check for speed limit
    ref_vel = min(speed_limit, updated_speed);

    return ref_vel;
}

vector<double> BehaviorPlaner::SetGoal(double s, double d, double speed) {
    vector<double> end_goal;

    end_goal.push_back(s);
    end_goal.push_back(d);
    end_goal.push_back(speed);

    return end_goal;
}
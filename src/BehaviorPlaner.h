#ifndef PATH_PLANNING_BEHAVIORPLANER_H
#define PATH_PLANNING_BEHAVIORPLANER_H

#include <vector>

using namespace std;

class BehaviorPlaner {
public:
    BehaviorPlaner();

    virtual ~BehaviorPlaner();

    vector<double> CalcBehavior(vector<vector<double>> sensor_fusion, double car_s, double car_d);

private:

    static int DetectLaneFromCarPos(double car_d);
    double UpdateSpeed(double speed_to_match = -1);
    static vector<double> SetGoal(double s, double d, double speed);
    vector<double> TryChangingLane(const vector<vector<double>>& car_from_left, const vector<vector<double>>& car_from_right,
                                   double car_s, double car_d,
                                   double front_car_speed);
    int BestLane(vector<vector<double>> sensor_fusion_left, vector<vector<double>> sensor_fusion_right, double car_s);
    bool CheckLane(vector<vector<double>> sensor_fusion, double car_s, double car_d);

    // Reference velocity to target
    double ref_vel; //mph
    double speed_limit; // mph
    double safe_dist; // meters
    double max_acc; // m/s2
};


#endif //PATH_PLANNING_BEHAVIORPLANER_H

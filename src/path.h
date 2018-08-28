
#ifndef PATH_H_
#define PATH_H_
#include <math.h>
#include <vector>
#include "json.hpp"

using namespace std;

constexpr double pi() { return M_PI; }


class Path {
public:
    static constexpr  double sample_t = 0.02; // the simulator sample time is 20ms
    static constexpr  int lane_width = 4; // assume the lane width is 4 meters
    static constexpr  int half_lane_width = lane_width / 2;    
    static constexpr  double max_vel = 49.5  * 0.44704; // maximum velocity is 50 mile per hour; use 49.5 to add small margin, and convert to meter per second
    static constexpr  double vel_inc = 0.3; // the veloctiy increment for one path plan sample time; 
    static constexpr  double lane_num = 3;  // assume there are three lanes; the leftmost is 0, the middle is 1, and the rightmost is 2.
    static constexpr  double safe_dis = 30.0; // the safe distance used to control the car
    // Constructor.
    Path (vector<double>& map_x, vector<double>& map_y, vector<double>& map_s, vector<double>& map_dx, vector<double>& map_dy);
    // Destructor. 
    ~Path();
    void predict (nlohmann::basic_json<> sensor_fusion, double car_s, int prev_size);
    void computePath (vector<double>& next_x_vals, vector<double>& next_y_vals, vector<double>&& previous_path_x, vector<double>&& previous_path_y, int prev_size, 
        double car_x, double car_y, double car_yaw, double car_s);
    
private:
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy; 
    int curr_lane;   // the current lane the car is located at
    double ctrl_vel; // the controlled car velocity 

    // get the lane center d from the lane number
    inline int get_d (int lane) {
        return lane * lane_width + half_lane_width;
    }
    bool check_lane (auto sensor_fusion, int check_lane, double car_s, int prev_size); 
};

#endif
#include <iostream>
#include "path.h"
#include "spline.h"

using namespace std;
// For converting back and forth between radians and degrees.
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

double distance(double x1, double y1, double x2, double y2)
{
    return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}


int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

    double closestLen = 100000; //large number
    int closestWaypoint = 0;

    for(int i = 0; i < maps_x.size(); i++)
    {
        double map_x = maps_x[i];
        double map_y = maps_y[i];
        double dist = distance(x,y,map_x,map_y);
        if(dist < closestLen)
        {
            closestLen = dist;
            closestWaypoint = i;
        }

    }

    return closestWaypoint;
}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

    int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

    double map_x = maps_x[closestWaypoint];
    double map_y = maps_y[closestWaypoint];

    double heading = atan2((map_y-y),(map_x-x));

    double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
    int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

    int prev_wp;
    prev_wp = next_wp-1;
    if(next_wp == 0)
    {
        prev_wp  = maps_x.size()-1;
    }

    double n_x = maps_x[next_wp]-maps_x[prev_wp];
    double n_y = maps_y[next_wp]-maps_y[prev_wp];
    double x_x = x - maps_x[prev_wp];
    double x_y = y - maps_y[prev_wp];

    // find the projection of x onto n
    double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
    double proj_x = proj_norm*n_x;
    double proj_y = proj_norm*n_y;

    double frenet_d = distance(x_x,x_y,proj_x,proj_y);

    //see if d value is positive or negative by comparing it to a center point

    double center_x = 1000-maps_x[prev_wp];
    double center_y = 2000-maps_y[prev_wp];
    double centerToPos = distance(center_x,center_y,x_x,x_y);
    double centerToRef = distance(center_x,center_y,proj_x,proj_y);

    if(centerToPos <= centerToRef)
    {
        frenet_d *= -1;
    }

    // calculate s value
    double frenet_s = 0;
    for(int i = 0; i < prev_wp; i++)
    {
        frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
    }

    frenet_s += distance(0,0,proj_x,proj_y);

    return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
    int prev_wp = -1;

    while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
    {
        prev_wp++;
    }

    int wp2 = (prev_wp+1)%maps_x.size();

    double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
    // the x,y,s along the segment
    double seg_s = (s-maps_s[prev_wp]);

    double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
    double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

    double perp_heading = heading-pi()/2;

    double x = seg_x + d*cos(perp_heading);
    double y = seg_y + d*sin(perp_heading);

    return {x,y};

}

Path::Path (vector<double>& map_x, vector<double>& map_y, vector<double>& map_s, vector<double>& map_dx, vector<double>& map_dy) {
    // initial the map way points
    map_waypoints_x  = map_x;
    map_waypoints_y  = map_y;
    map_waypoints_s  = map_s;
    map_waypoints_dx = map_dx;
    map_waypoints_dy = map_dy;    

    // initial the car lane and velocity
    curr_lane = 1;
    ctrl_vel  = 0.0;
}

Path::~Path () {

}


/*
 * this function is to check whether this lane has cars
*/
bool Path::check_lane (auto sensor_fusion, int check_lane, double car_s, int prev_size) {
	
	double back_dist;
	if (check_lane == curr_lane) {
		back_dist = 0;
	} else {
		back_dist = -5;
	}
	// check whether this lane has car
	for (int i = 0; i < sensor_fusion.size(); i++) {
		float d = sensor_fusion[i][6];
        int lane_d =  get_d (check_lane);

        if ((d < (lane_d + half_lane_width)) && (d > (lane_d - half_lane_width))) {
        	double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];

            double check_speed = sqrt (vx * vx + vy * vy);
            double check_car_s = sensor_fusion[i][5];

            check_car_s += (double) prev_size * sample_t * check_speed;

            if ((check_car_s - car_s> back_dist) && (check_car_s - car_s < safe_dis)) {
                return false;
            }
        }
	}
	return true;
}

/*
 * this function is to predict the car velocity and lane according to the sensor fusion results
*/
void Path::predict (nlohmann::basic_json<> sensor_fusion, double car_s, int prev_size) {

	bool no_front_car = check_lane (sensor_fusion, curr_lane, car_s, prev_size);

    if (no_front_car) {
    	// no car in the front of the car lane; keep the same lane and increase the speed if not reach maximum.
    	if (ctrl_vel + vel_inc < max_vel) {
    		ctrl_vel += vel_inc;
    	}
    	return;
    }

    // check the left lane. if no car there, change the lane 
	int left_lane = curr_lane - 1;
	if (left_lane >= 0) {
		if (check_lane(sensor_fusion, left_lane, car_s, prev_size)) {
			curr_lane = left_lane;
			return;
		}
	}

	// check the right lane. if no car there, change the lane 
	int right_lane = curr_lane + 1;
	if (right_lane < lane_num) {
		if (check_lane(sensor_fusion, right_lane, car_s, prev_size)) {
			curr_lane = right_lane;
			return;
		}
	}

	// reduce the veloctiy is the last option
	ctrl_vel -= vel_inc;
	return;
}


void Path::computePath (vector<double>& next_x_vals, vector<double>& next_y_vals, vector<double>&& previous_path_x, vector<double>&& previous_path_y, int prev_size, 
	double car_x, double car_y, double car_yaw, double car_s) {

	vector<double> ptsx;
    vector<double> ptsy;

    double ref_x;
    double ref_y;
    double ref_yaw;

    if (prev_size < 2) {
    	ref_x = car_x;
    	ref_y = car_y;
    	ref_yaw = deg2rad(car_yaw);

        double prev_car_x = car_x - cos(car_yaw);
        double prev_car_y = car_y - sin(car_yaw);

        ptsx.push_back(prev_car_x);
        ptsx.push_back(car_x);

        ptsy.push_back(prev_car_y);
        ptsy.push_back(prev_car_y);
    } else {
        ref_x = previous_path_x[prev_size - 1];
        ref_y = previous_path_y[prev_size - 1];

        double ref_x_prev = previous_path_x[prev_size - 2];
        double ref_y_prev = previous_path_y[prev_size - 2];

        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

        ptsx.push_back(ref_x_prev);
        ptsx.push_back(ref_x);

        ptsy.push_back(ref_y_prev);
        ptsy.push_back(ref_y);
    }

    vector<double> next_wp0 = getXY(car_s + safe_dis,       get_d(curr_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp1 = getXY(car_s + 2.0 * safe_dis, get_d(curr_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp2 = getXY(car_s + 3.0 * safe_dis, get_d(curr_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);

    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);

    // convert the points from the global coordinate system to the car-center local coordinate system
    for (int i = 0; i < ptsx.size(); i ++) {
        double shift_x = ptsx[i] - ref_x;
        double shift_y = ptsy[i] - ref_y;

        ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
        ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
    }

    // create spline
    tk::spline s;
    s.set_points(ptsx, ptsy);

    // start with all of the previous path points 
    for (int i = 0; i < previous_path_x.size(); i++) {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }

    // calculate how to break up spline points so that the car can travel at the desired velocity
    double target_x = safe_dis;
    double target_y = s(target_x);
    double target_dist = sqrt(target_x * target_x + target_y * target_y);

    double x_add_on = 0;

    // fill up the rest of the path points 
    double N = target_dist / (sample_t * ctrl_vel);
    for (int i = 0; i < 30 - previous_path_x.size(); i ++) {
        double x_point = x_add_on + target_x / N;
        double y_point = s(x_point);

        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        // convert back to the global coordinate system before used as path point
        x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
        y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

        x_point += ref_x;
        y_point += ref_y;

        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);
    }
    return;
}


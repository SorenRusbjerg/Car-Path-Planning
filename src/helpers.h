#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>

// for convenience
using std::string;
using std::vector;

struct S_sf_Car {
  double lane;
  double lane_diff;
  double s_diff;
  double speed;
};


// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, 
                    const vector<double> &maps_y) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, 
                 const vector<double> &maps_y) {
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = std::min(2*pi() - angle, angle);

  if (angle > pi()/2) {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
      std::cout << "closestWaypoint = 0" << std::endl;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, 
                         const vector<double> &maps_x, 
                         const vector<double> &maps_y) {
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if (next_wp == 0) {
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

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, 
                     const vector<double> &maps_x, 
                     const vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) {
    ++prev_wp;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),
                         (maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}

// Get car lane from frenet d
int GetLane(double d, double laneWidth)
{
  int lane = 0;
  if (0 <= d && d < laneWidth)
    lane = 0;
  else if (laneWidth <= d && d < 2*laneWidth)
    lane = 1;
  else if (2*laneWidth <= d && d < 3*laneWidth)
    lane = 2;

  return lane;
}

// Get info if other car is too close
bool CarTooClose(double car_s, double sf_car_s, double dist)
{
  double diff = sf_car_s - car_s;
  if (diff > 0.0 and diff < dist)
    return true;
  else
    return false;

}

// Get cost for all lanes
vector<double> GetLaneCost(const vector<S_sf_Car> &sf_cars)
{
  double max_dist_pos = 100.0;
  double max_dist_neg = -20.0;
  double target_speed = 50.0/2.24;
  double cost_dist = 0.5; // per meter
  double cost_speed = 3.0; // 
  double cost_lanediff = 20.0;

  vector<double> laneCost = {0,0,0};

  // Loop over all cars in lane and add lane cost
  for (S_sf_Car car : sf_cars)
  {
    // within short distance calculate cost
    if ((car.s_diff < max_dist_pos) and (car.s_diff > max_dist_neg))
    {
      int lane = int(car.lane); 
      // Calculate lane speed cost
      laneCost[lane] += cost_speed*(target_speed - car.speed); 

      if (car.s_diff > 0) // car ahead    
        laneCost[lane] += cost_dist * pow(max_dist_pos - car.s_diff,1.5); 
      else // car behind
      {
        laneCost[lane] += 5.0*cost_dist * pow(car.s_diff - max_dist_neg,1.5); 
      }

      laneCost[lane] += cost_lanediff * car.lane_diff;       
    }
  }
  return laneCost;
}

// Get lane speed for slowest car in lane within immidiate distance 
double GetLaneSpeed(const vector<S_sf_Car> &sf_cars, int lane)
{
  double max_dist_pos = 30.0;
  double max_dist_neg = 0.0;
  double target_speed = 50.0/2.24;
  double laneSpeed = target_speed;

  // Loop over all cars in lane
  for (S_sf_Car car : sf_cars)
  {
    // within short distance calculate speed
    if ((car.s_diff < max_dist_pos) and (car.s_diff > max_dist_neg) and car.lane == lane)
    {
      // Get minimum lane speed
      if (car.speed < laneSpeed)
      {
        laneSpeed = car.speed;
      } 
    }
  }

  return laneSpeed;

}

// Calculate d-coordinates from lanes
vector<double> Get_d_ref(const int currentLane, const int newLane)
{
  double current_d = 2.0 + 4.0*currentLane;
  double new_d = 2.0 + 4.0*newLane;

  vector<double> d_ref(3);
  d_ref[0] = current_d + (new_d-current_d);
  d_ref[1] = current_d + (new_d-current_d);
  d_ref[2] = current_d + (new_d-current_d);
  return d_ref;
}


// Calculate s-coordinates from lanes
vector<double> Get_s_ref_turn()
{
  vector<double> s_ref(3);
  s_ref[0] = 80.0;
  s_ref[1] = 110.0;
  s_ref[2] = 120.0;
  return s_ref;
}

// Calculate s-coordinates from lanes
vector<double> Get_s_ref_straight()
{
  vector<double> s_ref(3);
  s_ref[0] = 40.0;
  s_ref[1] = 80.0;
  s_ref[2] = 120.0;
  return s_ref;
}

#endif  // HELPERS_H
#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::cout;
using std::endl;
using std::string;
using std::vector;

int main()
{
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line))
  {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
               &map_waypoints_dx, &map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                                                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {

      auto s = hasData(data);

      if (s != "")
      {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry")
        {
          // j[1] is the data JSON object

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          double Ts = 0.02; // SampleTime
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          // set target lane
          double lane = 1.0; // 0, 1, 2

          // Set car yaw to radians
          car_yaw = deg2rad(car_yaw);

          // Set car refence coordinates
          int prev_size = previous_path_x.size();
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = car_yaw;
          double last_ref_x;
          double last_ref_y;

          // Create next waypoint vectors
          vector<double> next_wp_x;
          vector<double> next_wp_y;
          // Create 2 starting points for car in xy-coordinates if previous_path is empty
          if (previous_path_x.size() < 2)
          {
            last_ref_x = car_x - cos(car_yaw);
            last_ref_y = car_y - sin(car_yaw);
            cout << "Creating new points" << endl;
          }
          else // Use previous waypoints as starting points
          {
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];

            last_ref_x = previous_path_x[prev_size - 2];
            last_ref_y = previous_path_y[prev_size - 2];
            ref_yaw = atan2(ref_y-last_ref_y, ref_x-last_ref_x);
          }
          next_wp_x.push_back(last_ref_x);
          next_wp_x.push_back(ref_x);

          next_wp_y.push_back(last_ref_y);
          next_wp_y.push_back(ref_y);

          // Create spline from Frenet waypoints along a given path
          vector<double> next_wp0 = getXY(car_s + 40.0, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 80.0, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 120.0, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          next_wp_x.push_back(next_wp0[0]);
          next_wp_x.push_back(next_wp1[0]);
          next_wp_x.push_back(next_wp2[0]);

          next_wp_y.push_back(next_wp0[1]);
          next_wp_y.push_back(next_wp1[1]);
          next_wp_y.push_back(next_wp2[1]);

          // Shift coordinates to car frame (x,y,angle)=(0,0,0)
          for (int i = 0; i < next_wp_x.size(); i++)
          {
            double dx = next_wp_x[i] - ref_x;
            double dy = next_wp_y[i] - ref_y;            
            next_wp_x[i] = dx * cos(-ref_yaw) - dy * sin(-ref_yaw);
            next_wp_y[i] = dx * sin(-ref_yaw) + dy * cos(-ref_yaw);
            cout << "New wp_x: " << next_wp_x[i] << endl;
          }

          // Make spline
          tk::spline spl;

          // Set spline points in car coordinates
          spl.set_points(next_wp_x, next_wp_y);

          // Put the previous path points in the start of the next path points vector
          for (int i = 0; i < prev_size; i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // Use spline to calculate new reference points at a target speed
          int N_newpoints = 50;
          double ref_speed = 22.0; // m/s
          double target_x = 30.0;
          double target_y = spl(target_x);
          double target_dist = sqrt(target_x * target_x + target_y * target_y);
          double x_last = 0.0;

          for (int i = 0; i < N_newpoints - prev_size; ++i)
          {
            double N = target_dist / (Ts * ref_speed);
            double x_point_cf = x_last + target_x / N;
            double y_point_cf = spl(x_point_cf);

            x_last = x_point_cf;

            // rotate points back to map frame and add car coordinates again
            double x_point = ref_x + x_point_cf * cos(ref_yaw) - y_point_cf * sin(ref_yaw);
            double y_point = ref_y + x_point_cf * sin(ref_yaw) + y_point_cf * cos(ref_yaw);

            // Add points to next coordinates list
            cout << "New x_point: " << x_point << endl;
            cout << "New y_point: " << y_point << endl;
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          // Insert into Jason package
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        } // end "telemetry" if
      }
      else
      {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    } // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}
#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/LU"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using Eigen::MatrixXd;

/*Helper static function. */

static void printVelAndAcc(vector<double> next_x_vals, vector<double> next_y_vals)
{
  vector<double> velocity;
  vector<double> acceleration;
              std::cout << "seg" << std::endl;
  if(next_x_vals.size() < 2)
    return;
  for(int i = 0; i < next_x_vals.size() - 1; i++)
  {
    double dist = distance(next_x_vals[i], next_y_vals[i], next_x_vals[i+1], next_y_vals[i+1]);
    velocity.push_back(dist/0.2);
  }
                std::cout << "seg" << std::endl;
  if(velocity.size() == 1)
    return;
  for(int i = 0; i < velocity.size() - 1; i++)
  {
    std::cout << velocity[i] << std::endl;
    acceleration.push_back((velocity[i] - velocity[i+1])/0.2);
  }
       std::cout << velocity[velocity.size()-1] << std::endl;
              std::cout << "seg" << std::endl;
  if(acceleration.size() == 1)
    return;
 for(int i = 0; i < acceleration.size(); i++)
  {
   std::cout << acceleration[i] << std::endl;
  }

}

static double slope(double x1,double y1,double x2,double y2)
{
  double slp = 0.0;
  
  slp = (y2-y1)/(x2-x1);
  
  return slp;
}

int main() {
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
  while (getline(in_map_, line)) {
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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
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

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          static double ref_speed = 0;

          static bool change_lanes = false;

          static bool lane_change_man_in_progress = false;
          
          static double global_car_d = car_d;

          int size = previous_path_x.size();
 
          vector<double> spline_x;
          vector<double> spline_y;

          double prev_x = car_x;
          double prev_y = car_y;
          double prev_yaw = deg2rad(car_yaw);
          
          vector<double> further_point1;
          vector<double> further_point2;
          vector<double> further_point3;
          
          /*Initialise the furthest points of the spline at car_s + 30, 60 and 90 metres.*/
          further_point1 =  getXY(car_s + 30, global_car_d, map_waypoints_s, 
                                  map_waypoints_x, 
                                  map_waypoints_y);

          further_point2 =  getXY(car_s + 60, global_car_d, map_waypoints_s, 
                                  map_waypoints_x, 
                                  map_waypoints_y);

          further_point3 =  getXY(car_s + 90, global_car_d, map_waypoints_s, 
                                  map_waypoints_x, 
                                  map_waypoints_y);

          static double obstacle_speed = -1; // Denotes the speed of the object 30m ahead of us.
          /* Check for objects ahead of us.*/
          for(int j = 0; j < sensor_fusion.size(); j++)
          {
            if((double(sensor_fusion[j][5]) <= (car_s + 30)) && (double(sensor_fusion[j][5]) > car_s) && (abs(double(sensor_fusion[j][6]) - global_car_d) < 2))
            {
              if(sqrt(double(sensor_fusion[j][3])*double(sensor_fusion[j][3]) + double(sensor_fusion[j][4])*double(sensor_fusion[j][4])) < ref_speed)
              {
                /*If object is slower than the car's speed, enable lane change maneuver check.*/
                obstacle_speed = sqrt(double(sensor_fusion[j][3])*double(sensor_fusion[j][3]) + double(sensor_fusion[j][4])*double(sensor_fusion[j][4]));
                change_lanes = true;
                break;
              }
              else
              {
                obstacle_speed = -1;
              }
            }
          }

          lane_change_man_in_progress = false;

          if(change_lanes)
          {
            if(global_car_d < 4)
            {
              /*Look for right lane overtake. */
              for(int j = 0; j < sensor_fusion.size(); j++)
              {
                  if((double(sensor_fusion[j][5]) <= (car_s + 30)) && (double(sensor_fusion[j][5]) >= (car_s - 5)) && ((double(sensor_fusion[j][6]) - global_car_d) > 2) && ((double(sensor_fusion[j][6]) - global_car_d) < 6))
                  {       
                    /*Can't overtake*/
                    change_lanes = false;
                    break;
                  }
              }
              
              if(change_lanes)
              {
                /*Furthest spline points progressing towards neighboring right lane.*/
                further_point1 =  getXY(car_s + 30, 2, map_waypoints_s, 
                                        map_waypoints_x, 
                                        map_waypoints_y);

                further_point2 =  getXY(car_s + 60, 4, map_waypoints_s, 
                                        map_waypoints_x, 
                                        map_waypoints_y);

                further_point3 =  getXY(car_s + 90, 6, map_waypoints_s, 
                                        map_waypoints_x, 
                                        map_waypoints_y);

                global_car_d = 6;
                change_lanes = false;
                lane_change_man_in_progress = true;
                obstacle_speed = -1;
          std::cout <<  "Left to Centre" << std::endl;
              }

            }
            else if(global_car_d < 8)
            {
              char lane_change = 'r'; // Key to denote right, left or straight lane
              for(int j = 0; j < sensor_fusion.size(); j++)
              {
                  if((double(sensor_fusion[j][5]) <= (car_s + 30)) && (double(sensor_fusion[j][5]) >= (car_s - 5)) && ((double(sensor_fusion[j][6]) - global_car_d) > 2))
                  {       
                    /*Can't overtake*/
                    change_lanes = false;
                    lane_change = 's';
                    break;
                  }
              }
              if(lane_change == 's')
              {
                lane_change = 'l';
                for(int j = 0; j < sensor_fusion.size(); j++)
                {
                    if((double(sensor_fusion[j][5]) <= (car_s + 30)) && (double(sensor_fusion[j][5]) >= (car_s - 5)) && ((double(sensor_fusion[j][6]) - global_car_d) < -2))
                    {       
                      /*Can't overtake*/
                      change_lanes = false;
                      lane_change = 's';
                      break;
                    }
                }
              }

              if(lane_change == 'l')
              {
                /*Furthest spline points progressing towards neighboring left lane.*/
                further_point1 =  getXY(car_s + 30, 6, map_waypoints_s, 
                                        map_waypoints_x, 
                                        map_waypoints_y);

                further_point2 =  getXY(car_s + 60, 4, map_waypoints_s, 
                                        map_waypoints_x, 
                                        map_waypoints_y);

                further_point3 =  getXY(car_s + 90, 2, map_waypoints_s, 
                                        map_waypoints_x, 
                                        map_waypoints_y);

                global_car_d = 2;
                change_lanes = false;
                lane_change_man_in_progress = true;
                obstacle_speed = -1;
          std::cout <<  "Centre to Left" << std::endl;
              }
              else if(lane_change == 'r')
              {
                /*Furthest spline points progressing towards neighboring right lane.*/
                further_point1 =  getXY(car_s + 30, 6, map_waypoints_s, 
                                        map_waypoints_x, 
                                        map_waypoints_y);

                further_point2 =  getXY(car_s + 60, 8, map_waypoints_s, 
                                        map_waypoints_x, 
                                        map_waypoints_y);

                further_point3 =  getXY(car_s + 90, 10, map_waypoints_s, 
                                        map_waypoints_x, 
                                        map_waypoints_y);

                global_car_d = 10;
                change_lanes = false;
                lane_change_man_in_progress = true;
                obstacle_speed = -1;
          std::cout <<  "Centre to Right" << std::endl;

              }
            }
            else
            {
              /*Look for left lane overtake. */
              for(int j = 0; j < sensor_fusion.size(); j++)
              {
                  if((double(sensor_fusion[j][5]) <= (car_s + 30)) && (double(sensor_fusion[j][5]) >= (car_s - 5)) && ((double(sensor_fusion[j][6]) - global_car_d) < -2) && ((double(sensor_fusion[j][6]) - global_car_d) > -6))
                  {       
                    /*Can't overtake*/
                    change_lanes = false;
                    break;
                  }
              }
              
              if(change_lanes)
              {
                /*Furthest spline points progressing towards neighboring left lane.*/
                further_point1 =  getXY(car_s + 30, 8, map_waypoints_s, 
                                        map_waypoints_x, 
                                        map_waypoints_y);

                further_point2 =  getXY(car_s + 60, 6, map_waypoints_s, 
                                        map_waypoints_x, 
                                        map_waypoints_y);

                further_point3 =  getXY(car_s + 90, 4, map_waypoints_s, 
                                        map_waypoints_x, 
                                        map_waypoints_y);
                
                global_car_d = 6;
                change_lanes = false;
                lane_change_man_in_progress = true;
                obstacle_speed = -1;

          std::cout <<  "Right to Centre" << std::endl;

              }
            }
          }


          /*Initialise spline points with car's position and its previous point (projected if not available.).*/
          if(previous_path_x.size() < 2)
          {
            prev_x = car_x - 0.3 * cos(car_yaw);
            prev_y = car_y - 0.3 * sin(car_yaw);

            spline_x.push_back(prev_x);
            spline_y.push_back(prev_y);

            spline_x.push_back(car_x);
            spline_y.push_back(car_y);

            prev_x = car_x;
            prev_y = car_y;
          }
          else
          {
            prev_x = previous_path_x[previous_path_x.size()-1];
            prev_y = previous_path_y[previous_path_y.size()-1];

            double prev_prev_x = previous_path_x[previous_path_x.size()-2];
            double prev_prev_y = previous_path_y[previous_path_y.size()-2];

            spline_x.push_back(previous_path_x[previous_path_x.size()-1]);
            spline_y.push_back(previous_path_y[previous_path_y.size()-1]);

            spline_x.push_back(car_x);
            spline_y.push_back(car_y);

            prev_yaw = atan2(prev_y-prev_prev_y, prev_x-prev_prev_x);
          }

         /*Append furthest points to spline.*/
          spline_x.push_back(further_point1[0]);
          spline_y.push_back(further_point1[1]);
          
          spline_x.push_back(further_point2[0]);
          spline_y.push_back(further_point2[1]);
          
          spline_x.push_back(further_point3[0]);
          spline_y.push_back(further_point3[1]);
          
          /*Shift spline points to car coordinates and sort them in ascending order.*/
          for(int i = 0; i < spline_x.size(); i++)
          {
            double shift_x = spline_x[i]-prev_x;
            double shift_y = spline_y[i]-prev_y;
            
            spline_x[i] = (shift_x * cos(0-prev_yaw) - shift_y * sin(0-prev_yaw));
            spline_y[i] = (shift_x * sin(0-prev_yaw) + shift_y * cos(0-prev_yaw));
          }
          
          for(int i = 0; i < spline_x.size()-1; i++)
          {
            for(int j = (i+1); j < spline_x.size(); j++)
            {
              if(spline_x[j] < spline_x[i])
              {
                double temp_x = spline_x[j];
                double temp_y = spline_y[j];
                
                spline_x[j] = spline_x[i];
                spline_x[i] = temp_x;
                
                spline_y[j] = spline_y[i];
                spline_y[i] = temp_y;
              }
            }
          }

          tk::spline spline_poly(spline_x, spline_y);

          for(int i = 0; i < previous_path_x.size(); i++)
          {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
          }

          double x_i_0 = 0;
          for(int i = 0; i <= (5-previous_path_x.size()); i++)
          {
            if(obstacle_speed == -1)
            {
              /*Increase speed until it reaches reference speed.*/
              if(ref_speed < 22)
              {
                ref_speed += 0.05;
              }
              else
              {
                ref_speed -= 0.05;
              }
            }
            else if(ref_speed > obstacle_speed)
            {
               /*If an obstacle is identified ahead, decrease the car's speed until it matches the obstacles' speed.*/
                ref_speed -= 0.1;
            }

            if(ref_speed > 22)
            {
                ref_speed -= 0.05;
            }

            double x = x_i_0 + (ref_speed * 0.02);

            double vel = distance(x_i_0, spline_poly(x_i_0), x, spline_poly(x))/0.02;

           /*Sanity check to ensure that increase in velocity due to the points from the spline doesn't cross the reference speed set by us..*/
            while(abs(vel - ref_speed) > 10)
            {
              //std::cout << distance(x_i_0, spline_poly(x_i_0), x, spline_poly(x))/0.02  << " " << ref_speed << std::endl;
              if((vel - ref_speed) > 0)
              {
                x = x * 0.75;
                vel = distance(x_i_0, spline_poly(x_i_0), x, spline_poly(x))/0.02;
              }
              else if((vel - ref_speed) < 0)
              {
                x = x * 1.25;
                vel = distance(x_i_0, spline_poly(x_i_0), x, spline_poly(x))/0.02;
                //std::cout <<  x - (ref_speed * 0.02) * cos(tangent) * 1.25  << " " << x << std::endl;
              }
              else
              {
                break;
              }
            }

            double x_ref = x;
            double y_ref = spline_poly(x);
            
            x_i_0 = x;
            
           /*Convert the path points to global coordinates and append them to vector.*/
            x = (x_ref * cos(prev_yaw) - y_ref * sin(prev_yaw)) + prev_x;
            double y = (x_ref * sin(prev_yaw) + y_ref * cos(prev_yaw)) + prev_y;
            
            next_x_vals.push_back(x);
            next_y_vals.push_back(y);

          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
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
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}
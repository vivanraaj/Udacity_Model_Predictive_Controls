#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}
#include "Eigen-3.3/Eigen/Dense"
Eigen::Vector3f  transform_points(vector<double> ptsx, vector<double> ptsy, vector<double> psis) {
    Eigen::Matrix3f T;
    Eigen::Vector3f P;
    Eigen::Vector3f trans_p;
    for (int i = 0; i < ptsx.size(); i++ )
    {
        double psi = psis[i];
        double px = ptsx[i];
        double py = ptsy[i];
        // Transform matrix from global to vehicle
        T << cos(psi), -sin(psi), px,
            sin(psi), cos(psi), py,
                    0,0,1;
        P << ptsx[i], ptsy[i], 1;
        // Transform matrix from vehicle to global x position in global map.
        trans_p = T.inverse()*P;
        ptsx[i] = trans_p[0];
        ptsy[i] = trans_p[1];
    }
    return trans_p;
}


// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// polynomial derivative.
double polyeval_derivative(Eigen::VectorXd coeffs, double x) {
    double result = 0.0;
    for (int i = 1; i < coeffs.size(); i++) {
        result += coeffs[i] * pow(x, i-1);
    }
    return result;
}


// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
//Eigen::VectorXd polyfit(vector<double>& xvals, vector<double>& yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  Eigen::VectorXd yvals_ = Eigen::VectorXd::Map(yvals.data(), yvals.size());
  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals_);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    //   cout << "sdata " << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
  
            
          double steer_angle = j[1]["steering_angle"];
          double acceleration = j[1]["throttle"];
          //Find state at 100ms in the future
          double lat = 0.1;

          const int num_pts = ptsx.size();
            
          //Transform the co-ords and convert to Eigen::VectorXd
          Eigen::VectorXd ptsx_trans(num_pts), ptsy_trans(num_pts);
          for(int i = 0; i < num_pts; i++) {
                ptsx_trans[i] = (ptsx[i] - px) * cos(-psi) - (ptsy[i] - py) * sin(-psi);
                ptsy_trans[i] = (ptsy[i] - py) * cos(-psi) + (ptsx[i] - px) * sin(-psi);
          }
          
          //fit a third degree polunomial to the waypoints
          //There will be 4 coefficients stored vector coeffs
          auto coeffs = polyfit(ptsx_trans, ptsy_trans, 3);
            
          // Estimate the car's future position after latency time
          
          psi = (-v* steer_angle * lat)/2.67;
          px = v*cos(psi) * lat;
          py = v*sin(psi) * lat;
          v = v + acceleration * lat;
            
          // calculate the cross track error by evaluating the polynomial at x = px
          // and subtracting y = 0.
          double cte = polyeval(coeffs, px);
            
          double epsi = psi - atan(coeffs[1] + 2*coeffs[2]*px + 3*coeffs[3]*px*px);//
            
          Eigen::VectorXd state(6);
          state << px, 0, psi, v, cte, epsi;

          //Display the waypoints/reference line
          // below for future points
          auto vars = mpc.Solve(state, coeffs);
            
          double steer_value = vars[0];
          double throttle_value = vars[1];


          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = -steer_value;// norrmalized in MPC.cpp
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory( its the green line)
          //add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;
    
          for (int i = 1; i < 2*num_pts - 1; i++) {
                mpc_x_vals.push_back(vars[2*i + 2]);
                mpc_y_vals.push_back(vars[2*i + 3]);
          }

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line   (its the yellow line)
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          for (int i = 1; i < num_pts ; ++i ) {
                next_x_vals.push_back(ptsx_trans[i]);
                next_y_vals.push_back(ptsy_trans[i]);
          }
            
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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
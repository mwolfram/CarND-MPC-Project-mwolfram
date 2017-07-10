#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <fstream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

constexpr double Lf() { return 2.67; }

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

// transform a point to car coordinate system
void transformPoint(double& ptx, double& pty, const double& refx, const double& refy, const double& refpsi) {
    double deltaX = ptx - refx;
    double deltaY = pty - refy;
    double cosdiff = cos(0-refpsi);
    double sindiff = sin(0-refpsi);
    ptx = (deltaX * cosdiff - deltaY * sindiff);
    pty = (deltaX * sindiff + deltaY * cosdiff);
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
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
    //cout << sdata << endl;
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

          // transform all points to car coordinate system
          for (auto i = 0; i < ptsx.size(); ++i) {
              transformPoint(ptsx[i], ptsy[i], px, py, psi);
          }

          Eigen::VectorXd xvals(6);
          Eigen::VectorXd yvals(6);
          xvals << ptsx[0], ptsx[1], ptsx[2], ptsx[3], ptsx[4], ptsx[5];
          yvals << ptsy[0], ptsy[1], ptsy[2], ptsy[3], ptsy[4], ptsy[5];
          Eigen::VectorXd coeffs = polyfit(xvals, yvals, 3);

          double cte = polyeval(coeffs, 0);
          double epsi = -atan(coeffs[1]);

          Eigen::VectorXd state(6);
          state << 0, 0, 0, v, cte, epsi;

          //std::ifstream config_file("./config.cfg");

          double ref_cte = 0.0;
          double ref_epsi = 0.0;
          double ref_vel = 100.0;
          double cte_w = 1700.0;
          double epsi_w = 1200.0;
          double vel_w = 1.0;
          double delta_w = 1.0;
          double acc_w = 12.0;
          double delta_change_w = 5.0;
          double acc_change_w = 1.0;

          /*
          config_file >> ref_cte;
          config_file >> ref_epsi;
          config_file >> ref_vel;
          config_file >> cte_w;
          config_file >> epsi_w;
          config_file >> vel_w;
          config_file >> delta_w;
          config_file >> acc_w;
          config_file >> delta_change_w;
          config_file >> acc_change_w;

          config_file.close();

          cout << ref_cte << " " << ref_epsi << " " << ref_vel << " " << cte_w << " " << epsi_w << " " << vel_w << " " << delta_w << " " << acc_w << " " << delta_change_w << " " << acc_change_w << endl;
          */

          /*
           * Calculate steering angle and throttle using MPC.
           * Both are in between [-1, 1].
           */
          auto result = mpc.Solve(state, coeffs,
                                  ref_cte, ref_epsi, ref_vel,
                                  cte_w, epsi_w, vel_w,
                                  delta_w, acc_w,
                                  delta_change_w, acc_change_w);

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          double angle = -result[0]/(deg2rad(25)*Lf());
          msgJson["steering_angle"] = angle;
          msgJson["throttle"] = result[1];

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          for (auto i = 2; i < result.size(); i = i + 2) {
              mpc_x_vals.push_back(result[i]);
              mpc_y_vals.push_back(result[i + 1]);
          }

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          double poly_inc = 2.5;
          int num_points = 25;
          for (auto i = 1; i < num_points; i++) {
              next_x_vals.push_back(poly_inc*i);
              next_y_vals.push_back(polyeval(coeffs, poly_inc*i));
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100)); // TODO enable
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual drivingwd
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

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

#define DEBUG 0


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
    cout << sdata << endl;
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
          
          // collect steering_angle from simulator
          //double steer_value = j[1]["steering_angle"] ;

          /*
          * TODO: Calculate steeering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */ 
          
           // 1) Convert map space to car space 


          // Use Eigen vector for polyfit() 
          Eigen::VectorXd   ptsx_car_space = Eigen::VectorXd( ptsx.size() ) ;
          Eigen::VectorXd   ptsy_car_space = Eigen::VectorXd( ptsx.size() ) ;

          for (int i = 0;   i < ptsx.size() ;   i++) {
            ptsx_car_space(i) = (ptsx[i] - px) * cos(psi) + (ptsy[i] - py) * sin(psi)  ;
            ptsy_car_space(i) = (ptsy[i] - py) * cos(psi) - (ptsx[i] - px) * sin(psi)  ;
            if(DEBUG){
            cout << " ptsx.size() " << ptsx.size() << endl ;
            cout << "ptsx[i] " << ptsx[i] << "\t px " << px << "\t  ptsx_car_space(i) " <<  ptsx_car_space(i) << endl ;
            cout << "ptsy[i] " << ptsy[i] << "\t py " << py << "\t  ptsy_car_space(i) " <<  ptsy_car_space(i) << endl ;
            }
          }
          
          //  local coordinate
          px = 0;
          py = 0;
          psi = 0;

          // 2)Fit (ptsx,ptsy) to a polynomial curve of oder 3
          
           auto coeffs =polyfit(ptsx_car_space, ptsy_car_space, 3) ;
           
           if(DEBUG) cout << "coeffs\t" << coeffs << endl ;


           
           // 3) Error calculation (cte and epsi) and state definition.( latency involved)
           
           auto cte = polyeval(coeffs, px) - py;
           // calculate the orientation error
           auto f1=coeffs[1] + (2 * coeffs[2] * px) + (3 * coeffs[3] * (px * px));
           auto f2= 2 * coeffs[2] + 6*coeffs[3] * px;
           auto epsi = psi - atan(f1);

           Eigen::VectorXd state(6);
           state << px, py, psi, v, cte, epsi;

           // 4) calculate the  reference road_curvature and set ref_v according to road_curvature ,
           // R=( 1 + y'^2)^(3/2)/|y''|

           auto road_curvature=pow(1.0+f1*f1,1.5)/fabs(f2);

            if(DEBUG)  {  //try runing for a whole round
                cout << "--------------------------------------------------  "<< endl;
                cout << "road curvature   " <<  road_curvature << endl;
                cout << "--------------------------------------------------  "<< endl;
            }

            if(road_curvature<40) ref_v =30;
            else if(road_curvature<100 && road_curvature>40) ref_v =50;
            else ref_v =80;

           //5) solve the output by mpc controller and send to simulator

           auto vars = mpc.Solve(state, coeffs) ;

           if(DEBUG)  cout << "steering_angle  " <<  - mpc.steering_angle<< "\t throttle  " << mpc.throttle << endl;

           json msgJson;

           msgJson["steering_angle"] = - mpc.steering_angle;// /deg2rad(25);
           msgJson["throttle"] = mpc.throttle ;
         
           //6)  Display MPC predicted trajectory in simulator  //green line

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
           msgJson["mpc_x"] = mpc.mpc_x_vals;
           msgJson["mpc_y"] = mpc.mpc_y_vals;

          // 7) Display the waypoints/reference line  in simulator  //yellow line

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          for (int i = 0;  i < ptsx_car_space.size();  i++) {
                     next_x_vals.push_back(ptsx_car_space[i] ) ;
                     next_y_vals.push_back(ptsy_car_space[i] ) ;
                   }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
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

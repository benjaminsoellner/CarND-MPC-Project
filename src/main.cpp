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
    for (int i = 0; i < coeffs.rows()*coeffs.cols(); i++) {
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
    Eigen::MatrixXd A(xvals.rows()*xvals.cols(), order + 1);
    for (int i = 0; i < xvals.rows()*xvals.cols(); i++) {
        A(i, 0) = 1.0;
    }
    for (int j = 0; j < xvals.rows()*xvals.cols(); j++) {
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
    // remember previous steering angle and throttle
    double prev_steer_value = 0.0;
    double prev_throttle_value = 0.0;
    
    // *** Telemetry Message Handling
    
    
    h.onMessage([&mpc, &prev_steer_value, &prev_throttle_value](
                        uWS::WebSocket<uWS::SERVER> ws,
                        char *data, size_t length,
                        uWS::OpCode opCode) {
        
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        string sdata = string(data).substr(0, length);
        if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
            string s = hasData(sdata);
            if (s != "") {
                auto j = json::parse(s);
                string event = j[0].get<string>();
                if (event == "telemetry") {
                    
                    // j[1] is the data JSON object
                    vector<double> wayptsx = j[1]["ptsx"];
                    vector<double> wayptsy = j[1]["ptsy"];
                    
                    double px = j[1]["x"];
                    double py = j[1]["y"];
                    double psi = j[1]["psi"];
                    double v_mph = j[1]["speed"];
                    double steering_value = j[1]["steering_angle"];
                    double throttle = j[1]["throttle"];
                    const double mph_to_mps = (1609. / 3600.);
                    double v = v_mph * mph_to_mps; // speed in m/s

                    // DONE.
                    // Calculate steering angle and throttle using MPC.
                    
                    // Take into account latency 
                    const double latency = 0.15 ; // ~150 ms
                    if (latency > 0) {
                        px = px + v * cos(psi) * latency;
                        py = py + v * sin(psi) * latency;
                        psi = psi - v * steering_value / mpc.Lf * latency;
                        v = v + throttle * latency;
                    }
                    
                    // Convert waypoints from global to car coordinate system
                    // Eigen::VectorXd representation needed for MPC
                    Eigen::VectorXd wayptsx_car(wayptsx.size());
                    Eigen::VectorXd wayptsy_car(wayptsy.size());
                    // vector<double> representation needed for output of polynom to simulator (later)
                    vector<double> wayptsx_car_v(wayptsx.size());
                    vector<double> wayptsy_car_v(wayptsx.size());
                    for (unsigned int i = 0; i < wayptsx.size(); i++) {
                        double x = (wayptsx[i]-px) * cos(psi) + (wayptsy[i]-py) * sin(psi);
                        double y = -(wayptsx[i]-px) * sin(psi) + (wayptsy[i]-py) * cos(psi);
                        wayptsx_car[i] = x;
                        wayptsx_car_v[i] = x;
                        wayptsy_car[i] = y;
                        wayptsy_car_v[i] = y;
                    }
                    
                    // Fit polynomial through waypoints
                    const int poly_order = 2;
                    Eigen::VectorXd coeff = polyfit(wayptsx_car, wayptsy_car, poly_order);
                    
                    // Prepare state vector, for cte and epsi use polynome
                    // we just created
                    Eigen::VectorXd state(6);
                    // x,y - car is at center of its own coordinate system in
                    //       initial state
                    state[0] = 0.0; 
                    state[1] = 0.0; 
                    // psi - car points straight forward
                    state[2] = 0.0; 
                    // v - speed, m/s
                    state[3] = v; 
                    // cte - is simply f(0.0)
                    state[4] = polyeval(coeff, 0.0); 
                    // epsi - angle btw. f(x) & x-axis at x=0 --> arctan(f'(0))
                    //        f=ax^2+bx+c --> f'(x)=2ax+b --> f'(0)=b
                    state[5] = -atan(coeff[1]);
                    
                    // calculate radius of curvature of polynomial f(x) at x=0
                    double radius;
                    // catch division by zero
                    if (fabs(coeff[2]) > 0.0001) {
                        // R(x) = ((1 + (dy/dx)^2)^1.5) / abs(d^2y/dx^2)
                        //      = ((1 + f'(x)^2)^1.5) / abs(f''(x))
                        //      = ((1 + (3ax^2+2bx+c)^2)^1.5 / abs(6ax+2b)
                        // R(0) = ((1 + c^2)^1.5 / abs(2b)
                        radius = pow(1.0+pow(coeff[1], 2), 1.5) / fabs(2.*coeff[2]);
                    } else {
                        radius = 1000.;
                    }
                    
                    // Set reference speed depending on radius
                    double v_ref; // in mph. MPC will convert to m/s2
                    if (v_mph < 40.) {
                        // at beginning, speed up
                        v_ref = 120;
                    } else {
                        if (radius < 80) {
                            // in a turn, slow down
                            v_ref = 40;
                        } else {
                            // in a straight road, accelerate!
                            v_ref = 70;
                        }
                    }
                    v_ref *= mph_to_mps;
        
                    // Output variables of MPC (success status, trajectory and actuators)
                    bool success;
                    vector<double> ptsx_car_pred_v;
                    vector<double> ptsy_car_pred_v;
                    double steer_value = -0.02;
                    double throttle_value = 0.1;
                    
                    // Solve MPC problem
                    vector<double> solution = mpc.Solve(state, coeff, v_ref, success, ptsx_car_pred_v, ptsy_car_pred_v);
                    if (success) {
                        // NOTE: Remember to divide by deg2rad(25) before you send 
                        // the steering value back. Otherwise the values will be 
                        // in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
                        steer_value = solution[0] / deg2rad(25); 
                        throttle_value = solution[1];
                        std::cout << "Steering: " << steer_value << ", Throttle: " << throttle_value << std::endl;
                    } else {
                        // if solver failed, keep old values 
                        steer_value = prev_steer_value;
                        throttle_value = prev_throttle_value;
                    }

                    json msgJson;
                    msgJson["steering_angle"] = steer_value;
                    msgJson["throttle"] = throttle_value;

                    // Return found trajectory from MPC solver
                    msgJson["mpc_x"] = ptsx_car_pred_v;
                    msgJson["mpc_y"] = ptsy_car_pred_v;

                    // Return next waypoints
                    msgJson["next_x"] = wayptsx_car_v;
                    msgJson["next_y"] = wayptsy_car_v;

                    // Update previous actuators
                    prev_steer_value = steer_value;
                    prev_throttle_value = throttle_value;
                    
                    // Latency
                    // The purpose is to mimic real driving conditions where
                    // the car does actuate the commands instantly.
                    // Feel free to play around with this value but should be to drive
                    // around the track with 100ms latency.
                    // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
                    // SUBMITTING.
                    this_thread::sleep_for(chrono::milliseconds(100));
          
                    std::string msg = "42[\"steer\"," + msgJson.dump() + "]";
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                    
                }
                
            } else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }
    });
    
    // *** end of: Telemetry Message Handling
    

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
    if (h.listen("0.0.0.0", port)) {
      std::cout << "Listening to port " << port << std::endl;
    } else {
      std::cerr << "Failed to listen to port" << std::endl;
      return -1;
    }
    h.run();
}

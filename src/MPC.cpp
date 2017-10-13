#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// DONE
// We set the number of timesteps to 15
// and the timestep evaluation frequency or evaluation
// period to 0.15.
size_t N = 15;
double dt = 0.15;


// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

// *****
// ** FG_eval class for ipopt's inner workings
// *****

class FG_eval {

public:

    // Coefficients of the fitted polynomial.
    Eigen::VectorXd coeffs;
    double v_ref;
    double Lf;
    const double cte_penalty = 5;
    const double steering_penalty = 2000;

    // Constructor
    FG_eval(Eigen::VectorXd coeffs, double v_ref, double Lf) {
        this->coeffs = coeffs;
        this->v_ref = v_ref;
        this->Lf = Lf;
    }

    // ADvector will be shorthand for CppAD::vector<double>
    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

    // Operator overloading, you can call an `FG_eval` object `o` like
    // `o(fg, vars)` where
    // `fg` is a vector containing the cost and constraints.
    // `vars` is a vector containing the variable values (state & actuators).
    void operator()(ADvector& fg, const ADvector& vars) {

        // *** Cost: fg[0]

        // The cost is stored is the first element of `fg`.
        // Any additions to the cost should be added to `fg[0]`.
        fg[0] = 0;

        // DONE
        // The part of the cost based on the reference state:
        // Iterate through all sampling points (N)
        for (unsigned int t = 0; t < N; t++) {
            // cte has square influence to cost
            fg[0] += cte_penalty * CppAD::pow(vars[cte_start+t], 2);
            // e_psi has square influence to cost
            fg[0] += CppAD::pow(vars[epsi_start+t], 2);
            // difference to reference velocity has square influence to cost
            fg[0] += CppAD::pow(vars[v_start+t] - v_ref, 2);
        }

        // Minimize the use of actuators.
        // Iterate through all sampling points except last one (N-1)
        for (unsigned int t = 0; t < N - 1; t++) {
            // steering delta should be minimal - square influence to cost
            fg[0] += steering_penalty * CppAD::pow(vars[delta_start+t], 2);
            // accelleration should be minimal - square influence to cost
            fg[0] += CppAD::pow(vars[a_start+t], 2);
        }

        // Minimize the value gap between sequential actuations.
        // Iterate through all sampling points except last two (N-1)
        for (unsigned int t = 0; t < N - 2; t++) {
            // difference between current and next steering should be minimal
            // square influence to cost
            fg[0] += CppAD::pow(vars[delta_start+t+1] - vars[delta_start+t], 2);
            // difference between current and next actuation should be minimal
            // square influence to cost
            fg[0] += CppAD::pow(vars[a_start+t+1] - vars[a_start+t], 2);
        }


        // *** Initial constraints: fg[1 + {variable}]
        // That's where we are right now! (constant constraint)

        // Bumping up by (fg[1 + ...]) because fg[0] contains our cost
        fg[1 + x_start] = vars[x_start];
        fg[1 + y_start] = vars[y_start];
        fg[1 + psi_start] = vars[psi_start];
        fg[1 + v_start] = vars[v_start];
        fg[1 + cte_start] = vars[cte_start];
        fg[1 + epsi_start] = vars[epsi_start];

        // *** Constraints based on motion model fg[1 + {variable} + {1..N}]
        // With "+ {1..N}" we "fill the gaps" in fg[...] that are open due to
        // the way the "..._start" variables are spaced apart.
        // For constraints based on motion model we only need to consider
        // starting from time t = 1
        for (unsigned int t = 1; t < N; t++) {

            // The state at time t
            AD<double> x1 = vars[x_start + t];
            AD<double> y1 = vars[y_start + t];
            AD<double> psi1 = vars[psi_start + t];
            AD<double> v1 = vars[v_start + t];
            AD<double> cte1 = vars[cte_start + t];
            AD<double> epsi1 = vars[epsi_start + t];

            // The state at time t-1
            AD<double> x0 = vars[x_start + t - 1];
            AD<double> y0 = vars[y_start + t - 1];
            AD<double> psi0 = vars[psi_start + t - 1];
            AD<double> v0 = vars[v_start + t - 1];
            AD<double> cte0 = vars[cte_start + t - 1];
            AD<double> epsi0 = vars[epsi_start + t - 1];

            // Only consider the actuation at time t-1.
            AD<double> delta0 = vars[delta_start + t - 1];
            AD<double> a0 = vars[a_start + t - 1];

            // Linear approximation of moving from x0 to x1
            AD<double> f0 = coeffs[0] + x0 * (coeffs[1] + x0 * (coeffs[2]));
            // Designated steering angle
            AD<double> psides0 = CppAD::atan(coeffs[1]);

            // We will set constraints A=B as fg[i]=A-B and will then
            // optimize fg[i] -> 0.
            // So from the various equations of the motion model follows...

            // x[t1] = x[t0] + v[t0] * cos(psi[t0]) * dt
            fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);

            // y[t1] = y[t0] + v[t0] * sin(psi[t]) * dt
            fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);

            // psi[t1] = psi[t0] + v[t0] / Lf * delta[t] * dt
            // but: simulator steering angle/delta is reversed, hence "- ... delta0"
            fg[1 + psi_start + t] = psi1 - (psi0 - v0 * delta0 / Lf * dt);

            // v[t1] = v[t0] + a[t0] * dt
            fg[1 + v_start + t] = v1 - (v0 + a0 * dt);

            // cte[t1] = f(x[t0]) - y[t0] + v[t0] * sin(epsi[t]) * dt
            fg[1 + cte_start + t] =
                cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));

            // epsi[t1] = psi[t0] - psides[t0] + v[t0] * delta[t0] / Lf * dt
            // but: simulator steering angle/delta is reversed, hence "- ... delta0"
            fg[1 + epsi_start + t] =
                epsi1 - (psi0 - psides0 - v0 * delta0 / Lf * dt);

        }
    }
};

// *****
// ** MPC class implementation
// *****

MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs, double v_ref,
                          bool& success, vector<double>& ptsx_car_pred, vector<double>& ptsy_car_pred) {

    typedef CPPAD_TESTVECTOR(double) Dvector;

    // initial state
    double x = state[0];
    double y = state[1];
    double psi = state[2];
    double v = state[3];
    double cte = state[4];
    double epsi = state[5];

    // number of independent variables
    // N timesteps == N - 1 actuations
    size_t n_vars = N * 6 + (N - 1) * 2;
    // Number of constraints
    size_t n_constraints = N * 6;

    // *** Setting up variables and constraints

    // Initial value of the independent variables.
    // Should be 0 except for the initial values.
    Dvector vars(n_vars);
    for (unsigned int i = 0; i < n_vars; i++) {
        vars[i] = 0.0;
    }

    // Set the initial variable values
    vars[x_start] = x;
    vars[y_start] = y;
    vars[psi_start] = psi;
    vars[v_start] = v;
    vars[cte_start] = cte;
    vars[epsi_start] = epsi;

    // Lower and upper limits for x
    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);

    // Set all non-actuators upper and lowerlimits to the max negative
    // and positive values.
    for (unsigned int i = 0; i < delta_start; i++) {
        vars_lowerbound[i] = -1.0e19;
        vars_upperbound[i] = 1.0e19;
    }

    // The upper and lower limits of delta are set to -25 and 25
    // degrees (values in radians).
    // NOTE: Feel free to change this to something else.
    for (unsigned int i = delta_start; i < a_start; i++) {
        vars_lowerbound[i] = -0.436332;
        vars_upperbound[i] = 0.436332;
    }

    // Acceleration/decceleration upper and lower limits.
    // NOTE: Feel free to change this to something else.
    for (unsigned int i = a_start; i < n_vars; i++) {
        vars_lowerbound[i] = -1.0;
        vars_upperbound[i] = 1.0;
    }

    // Lower and upper limits for the constraints
    // Should be 0 except initial state.
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    for (unsigned int i = 0; i < n_constraints; i++) {
      constraints_lowerbound[i] = 0;
      constraints_upperbound[i] = 0;
    }

    constraints_lowerbound[x_start] = x;
    constraints_lowerbound[y_start] = y;
    constraints_lowerbound[psi_start] = psi;
    constraints_lowerbound[v_start] = v;
    constraints_lowerbound[cte_start] = cte;
    constraints_lowerbound[epsi_start] = epsi;

    constraints_upperbound[x_start] = x;
    constraints_upperbound[y_start] = y;
    constraints_upperbound[psi_start] = psi;
    constraints_upperbound[v_start] = v;
    constraints_upperbound[cte_start] = cte;
    constraints_upperbound[epsi_start] = epsi;

    // *** Calling the solver

    // Object that computes objective and constraints
    FG_eval fg_eval(coeffs, v_ref, Lf);

    // options for IPOPT solver
    std::string options;
    // Uncomment this if you'd like more print information
    options += "Integer print_level  0\n";
    // NOTE: Setting sparse to true allows the solver to take advantage
    // of sparse routines, this makes the computation MUCH FASTER. If you
    // can uncomment 1 of these and see if it makes a difference or not but
    // if you uncomment both the computation time should go up in orders of
    // magnitude.
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
    // Change this as you see fit.
    options += "Numeric max_cpu_time          0.5\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(
            options, vars, vars_lowerbound, vars_upperbound,
            constraints_lowerbound, constraints_upperbound,
            fg_eval, solution
        );

    // Modify arguments passed by reference (return status + trajectory)
    success = (solution.status == CppAD::ipopt::solve_result<Dvector>::success);
    ptsx_car_pred.resize(N);
    ptsy_car_pred.resize(N);
    for (unsigned int t=0; t<N; t++) {
        ptsx_car_pred[t] = solution.x[x_start + t];
        ptsy_car_pred[t] = solution.x[y_start + t];
    }

    // DONE
    // Return the first actuator values. The variables can be accessed with
    // solution.x[i]`.
    // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
    // creates a 2 element double vector.
    return {solution.x[delta_start], solution.x[a_start]};
}

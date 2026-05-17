#ifndef ILQR_HPP
#define ILQR_HPP

#include <string>
#include <Eigen/Dense>
#include <vector>

#include "../../../model/src/pendulum_plant.hpp"
#include "../../../simulator/src/simulator.hpp"

class ilqr{

    static const int n_x = 2;
    static const int n_u = 1;


    double mass;
    double length;
    double damping;
    double coulomb_friction;
    double gravity;
    double inertia;
    double torque_limit;

    PendulumPlant plant;
    Simulator sim;

    std::string integrator;
    double dt;
    State pendulum_discrete_dynamics(State, Action);

    State x0, goal;
    double goal_energy;

    double sCu, sCp, sCv, sCen, fCp, fCv, fCen;
    double stage_cost(State, Action);
    double final_cost(State);
    double calculate_cost(bool);

    //Eigen::Matrix2d dyn_x; // (2,2)
    Eigen::Matrix<double, n_x, n_x> dyn_x; // (2,2)
    //Eigen::Matrix<double, 2, 1> dyn_u; // (1,2)
    Eigen::Matrix<double, n_x, n_u> dyn_u; // (1,2)
    //Eigen::Matrix<double, 2, 1> stage_x; // (1,2)
    Eigen::Matrix<double, n_x, 1> stage_x; // (1,2)
    double stage_u; // (1,1)
    //Eigen::Matrix2d stage_xx; // (2,2) (1,2,2)
    Eigen::Matrix<double, n_x, n_x> stage_xx; // (2,2) (1,2,2)
    //Eigen::Matrix<double, 1, 2> stage_ux; //(1,2)
    Eigen::Matrix<double, n_u, n_x> stage_ux; //(1,2)
    double stage_uu; // (1,1)
    //Eigen::Matrix<double, 1, 2> final_x; // (1,2)
    Eigen::Matrix<double, 1, n_x> final_x; // (1,2)
    //Eigen::Matrix2d final_xx; // (2,2) (1,2,2)
    Eigen::Matrix<double, n_x, n_x> final_xx; // (2,2) (1,2,2)

    void compute_dynamics_x(State, Action);
    void compute_dynamics_u(State, Action);
    void compute_stage_x(State, Action);
    void compute_stage_u(State, Action);
    void compute_stage_xx(State, Action);
    void compute_stage_ux(State, Action);
    void compute_stage_uu(State, Action);
    void compute_final_x(State);
    void compute_final_xx(State);
    void compute_derivatives(State, Action);


    void rollout();
    void calculate_Q_terms();
    void calculate_V_terms();
    void calculate_gains(double);
    double expected_cost_reduction();
    void forward_pass();
    double backward_pass(double);


    double Q_u, Q_uu, Q_uu_regu, k;
    Eigen::Matrix<double, 1, 2> Q_ux, K;
    Eigen::Matrix<double, 2, 1> V_x, Q_x;
    //Eigen::RowVector2d Q_x, V_x;
    Eigen::Matrix2d Q_xx, V_xx;

    // helper variables
    double en_diff;

    int verbose = 0;

    //static const int N = 1000;
    int N;
    //double k_traj[N];
    //Eigen::Vector2d K_traj[N];
    double* k_traj = new double[N];
    Eigen::Vector2d* K_traj = new Eigen::Vector2d[N];

    bool warm_start_x = false;
    bool warm_start_u = false;

    //double regu;
    double* u_traj_doubles = new double[N-1];
    double* p_traj_doubles = new double[N];
    double* v_traj_doubles = new double[N];

public:

    //ilqr() : dyn_u(2,1), stage_ux(1,2) {}
    ilqr();
    ilqr(int);
    ~ilqr();

    void set_parameters(int, int, int, double);
    void set_cost_parameters(double, double, double, double, double, double, double);
    void set_pendulum_parameters(double, double, double, double, double, double, double);
    void set_start(State);
    void set_start(double, double);
    void set_goal(State);
    void set_goal(double, double);
    void set_u_init_traj(double u[]);
    void set_x_init_traj(double p[], double v[]);
    void run_ilqr(int, double, double);

    //Action u_traj[N-1], u_traj_new[N-1];
    //State x_traj[N], x_traj_new[N];

    Action* u_traj = new Action[N-1];
    Action* u_traj_new = new Action[N-1];
    State* x_traj = new State[N];
    State* x_traj_new = new State[N];

    //std::vector<double> get_u_traj();
    int get_N();
    double* get_u_traj();
    double* get_p_traj();
    double* get_v_traj();

};

#endif // ILQR_HPP

#include <iostream>
#include <fstream>
#include <yaml-cpp/node/node.h>
#include <yaml-cpp/node/parse.h>
#include <yaml-cpp/yaml.h>


#include "ilqr.hpp"

int main(int argc, char *argv[], char *envp[]){

    std::string foldername = ".";
    if (argc >= 1){foldername = std::string(argv[1]);}

    // default parameters

    //pendulum parameters
    double mass = 0.57288;
    double length = 0.5;
    double inertia = mass*length*length;
    double damping = 0.1;
    double coulomb_friction = 0.0;
    double gravity = 9.81;
    double torque_limit = 2.0;

    // simulation parameters
    double dt = 0.01;
    int integrator_ind = 1;

    //swingup parameters
    double start_pos = 0.0;
    double start_vel = 0.0;
    double goal_pos = 3.1415;
    double goal_vel = 0.0;

    // cost parameters
    double sCu = 0.1;
    double sCp = 0.0;
    double sCv = 0.0;
    double sCen = 0.0;
    double fCp = 1000.;
    double fCv = 1.0;
    double fCen = 0.0;

    // other ilqr parameters
    int max_iter = 100;
    double break_cost_redu = 1e-6;
    double regu_init = 100.;
    int N=1000;

    //TODO fix yaml loading
    // read parameters from yaml file
    YAML::Node config = YAML::LoadFile(foldername + "/config.yml");
    if (config["mass"]) {mass=config["mass"].as<double>();}
    if (config["length"]) {length=config["length"].as<double>();}
    if (config["damping"]) {damping=config["damping"].as<double>();}
    if (config["coulomb_friction"]) {coulomb_friction=config["coulomb_friction"].as<double>();}
    if (config["gravity"]) {gravity=config["gravity"].as<double>();}
    if (config["inertia"]){
        inertia=config["inertia"].as<double>();
    }
    else{
        inertia = mass*length*length;
    }
    if (config["torque_limit"]) {torque_limit=config["torque_limit"].as<double>();}
    if (config["deltaT"]) {dt=config["deltaT"].as<double>();}
    if (config["integrator"]){integrator_ind = config["integrator"].as<int>();}
    if (config["start_pos"]) {start_pos=config["start_pos"].as<double>();}
    if (config["start_vel"]) {start_vel=config["start_vel"].as<double>();}
    if (config["goal_pos"]) {goal_pos=config["goal_pos"].as<double>();}
    if (config["goal_vel"]) {goal_vel=config["goal_vel"].as<double>();}
    if (config["sCu"]) {sCu=config["sCu"].as<double>();}
    if (config["sCp"]) {sCp=config["sCp"].as<double>();}
    if (config["sCv"]) {sCv=config["sCv"].as<double>();}
    if (config["sCen"]) {sCen=config["sCen"].as<double>();}
    if (config["fCp"]) {fCp=config["fCp"].as<double>();}
    if (config["fCv"]) {fCv=config["fCv"].as<double>();}
    if (config["fCen"]) {fCen=config["fCen"].as<double>();}
    if (config["max_iter"]) {max_iter=config["max_iter"].as<int>();}
    if (config["break_cost_redu"]) {break_cost_redu=config["break_cost_redu"].as<double>();}
    if (config["regu_init"]) {regu_init=config["regu_init"].as<double>();}
    if (config["N"]) {N=config["N"].as<int>();}

    std::cout << "mass " << mass << std::endl;
    std::cout << "length " << length << std::endl;
    std::cout << "damping " << damping << std::endl;
    std::cout << "coublomb_friction " << coulomb_friction << std::endl;
    std::cout << "gravity " << gravity << std::endl;
    std::cout << "inertia " << inertia << std::endl;
    std::cout << "torque_limit " << torque_limit << std::endl;
    std::cout << "dt " << dt << std::endl;
    std::cout << "integrator " << integrator_ind << std::endl;
    std::cout << "start_pos " << start_pos << std::endl;
    std::cout << "start_vel " << start_vel << std::endl;
    std::cout << "goal_pos " << goal_pos << std::endl;
    std::cout << "goal_vel " << goal_vel << std::endl;
    std::cout << "sCu " << sCu << std::endl;
    std::cout << "sCp " << sCp << std::endl;
    std::cout << "sCv " << sCv << std::endl;
    std::cout << "sCen " << sCen << std::endl;
    std::cout << "fCp " << fCp << std::endl;
    std::cout << "fCv " << fCv << std::endl;
    std::cout << "fCen " << fCen << std::endl;
    std::cout << "max_iter " << max_iter << std::endl;
    std::cout << "break_cost_redu " << break_cost_redu << std::endl;
    std::cout << "regu_init " << regu_init << std::endl;
    std::cout << "N " << N << std::endl;

    ilqr ilqr_calc(N);

    ilqr_calc.set_parameters(2, 1, integrator_ind, dt);
    ilqr_calc.set_pendulum_parameters(mass, length, damping, coulomb_friction, gravity, inertia, torque_limit);
    ilqr_calc.set_cost_parameters(sCu, sCp, sCv, sCen, fCp, fCv, fCen);


    State x0;
    x0.pos = start_pos;
    x0.vel = start_vel;
    State goal;
    goal.pos = goal_pos,
    goal.vel = goal_vel;

    ilqr_calc.set_start(x0);
    ilqr_calc.set_goal(goal);

    //double u_traj[N-1] = {0.0}; 
    //double p_traj[N] = {0.0}; 
    //double v_traj[N] = {0.0}; 
    //ilqr_calc.set_u_init_traj(u_traj);
    //ilqr_calc.set_x_init_traj(p_traj, v_traj);

    //State random_x;
    //random_x.pos = 0.0;
    //random_x.vel = 0.1;
    //Action random_u;
    //random_u.u1 = 0.4;
    //ilqr_calc.compute_derivatives(random_x, random_u);
    ilqr_calc.run_ilqr(max_iter, break_cost_redu, regu_init);



    std::ofstream traj_file;
    traj_file.open (foldername+"/trajectory.csv");
    traj_file << "pos, vel, tau\n";

    for (int i=0; i<N-1; i++){
        //printf("Final state (%f, %f, %f) ", ilqr_calc.x_traj[i].pos, ilqr_calc.x_traj[i].vel, ilqr_calc.u_traj[i].u1);
        traj_file << ilqr_calc.x_traj[i].pos << ", " << ilqr_calc.x_traj[i].vel << ", " << ilqr_calc.u_traj[i].u1 << "\n";
    }
    State xf = ilqr_calc.x_traj[N-1];
    printf("Final state %f %f \n", xf.pos, xf.vel);
    traj_file << ilqr_calc.x_traj[N-1].pos << ", " << ilqr_calc.x_traj[N-1].vel << ", " << 0.0;

}

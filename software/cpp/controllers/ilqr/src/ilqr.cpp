#include <stdlib.h>
#include <iostream>
#include <math.h>
#include <algorithm>

#include "ilqr.hpp"
#include "../../../model/src/pendulum_plant.hpp"
#include "../../../simulator/src/simulator.hpp"


//ilqr::ilqr(void){
//    u_traj_doubles = malloc(sizeof(double)*N);
//}
//
//ilqr::~ilqr(void){
//    delete[] u_traj_doubles;
//}


ilqr::ilqr() : N(1000){
    srand (time(NULL));
}

ilqr::ilqr(int n) : N(n){
    srand (time(NULL));
}

ilqr::~ilqr(){
    delete [] u_traj_doubles;
    delete [] p_traj_doubles;
    delete [] v_traj_doubles;
    delete [] u_traj;
    delete [] u_traj_new;
    delete [] x_traj;
    delete [] x_traj_new;
    delete [] k_traj;
    delete [] K_traj;
}

int ilqr::get_N(){
    return N;
}

void ilqr::set_parameters(int nx, int nu, int integrator_ind, double delta_t){
    //n_x = nx;
    //n_u = nu;
    if(integrator_ind == 0){
        integrator = "euler";
    }
    else{
        integrator = "runge_kutta";
    }
    dt = delta_t;
}


void ilqr::set_cost_parameters(double su, double sp, double sv, double sen,
                               double fp, double fv, double fen){
    sCu = su;
    sCp = sp;
    sCv = sv;
    sCen = sen;
    fCp = fp;
    fCv = fv;
    fCen = fen;
}

void ilqr::set_pendulum_parameters(double m, double l, double d, double cf, double g, double I, double tl){
    mass = m;
    length = l;
    damping = d;
    coulomb_friction = cf;
    gravity = g;
    inertia = I;
    torque_limit = tl;

    plant = PendulumPlant();
    plant.set_parameters(m, l, d, cf, g, I, tl);
    sim = Simulator();
    sim.set_plant(plant);
}

void ilqr::set_start(State x){
    x0 = x;
}

void ilqr::set_start(double pos, double vel){
    x0.pos = pos;
    x0.vel = vel;
}

void ilqr::set_goal(State x){
    goal = x;
    goal_energy = plant.calculate_total_energy(goal);
}

void ilqr::set_goal(double pos, double vel){
    goal.pos = std::fmod(pos, 2.*M_PI);
    goal.vel = vel;
    goal_energy = plant.calculate_total_energy(goal);
}

void ilqr::set_u_init_traj(double u[]){
    for(int i=0; i<N-1; i++){
        u_traj[i].u1 = u[i];
        u_traj_doubles[i] = u[i];
    }
    warm_start_u = true;
    //printf("enabled u warm start: %i\n", int(warm_start_u));
}

void ilqr::set_x_init_traj(double p[], double v[]){
    for(int i=0; i<N; i++){
        x_traj[i].pos = p[i];
        p_traj_doubles[i] = p[i];
        x_traj[i].vel = v[i];
        v_traj_doubles[i] = v[i];
    }
    warm_start_x = true;
    //printf("enabled x warm start: %i\n", int(warm_start_x));
}

State ilqr::pendulum_discrete_dynamics(State x, Action u){
    sim.simulate(0.0, x.pos, x.vel, dt, dt, u.u1, integrator);
    return sim.get_state();
}

double ilqr::stage_cost(State x, Action u){
    double eps, pos_error, vel_error, u_cost, en_error, scost;
    eps = 1e-6;
    pos_error = pow((std::fmod(x.pos, 2.*M_PI) - goal.pos + eps), 2.);
    vel_error = pow((x.vel - goal.vel + eps), 2.);
    u_cost = pow(u.u1, 2.);
    en_error = pow(plant.calculate_total_energy(x) - goal_energy + eps, 2.);
    scost = sCp*pos_error + sCv*vel_error + sCu*u_cost + sCen*en_error;
    //printf("pos_cost %e, vel_cost %e, u_cost %e, en_cost %e, stage cost %f\n", sCp*pos_error, sCv*vel_error, sCu*u_cost, sCen*en_error, scost);
    return scost; 
}

double ilqr::final_cost(State x){
    double eps, pos_error, vel_error, en_error, fcost;
    eps = 1e-6;
    pos_error = pow((std::fmod(x.pos, 2.*M_PI) - goal.pos + eps), 2.);
    vel_error = pow((x.vel - goal.vel + eps), 2.);
    en_error = pow(plant.calculate_total_energy(x) - goal_energy + eps, 2.);
    fcost = fCp*pos_error + fCv*vel_error + fCen*en_error;
    //printf("pos_cost %e, vel_cost %e, en_cost %e, stage cost %f\n", fCp*pos_error, fCv*vel_error, fCen*en_error, fcost);
    //printf("final cost %f\n", fcost);
    return fcost;
}

double ilqr::calculate_cost(bool new_traj){
    double total = 0.;
    if (new_traj){
        for (int i=0; i<N-1; i++){
            total += stage_cost(x_traj_new[i], u_traj_new[i]) / (1.*(N-1));
        }
        total += final_cost(x_traj_new[N-1]);
    }
    else{
        for (int i=0; i<N-1; i++){
            total += stage_cost(x_traj[i], u_traj[i]) / (1.*(N-1));
            //printf("stage cost %e\n", total);
        }
        total += final_cost(x_traj[N-1]);
    }
    return total;
}

void ilqr::compute_dynamics_x(State x, Action u){
    dyn_x(0,0) = 1.;
    dyn_x(0,1) = dt;
    dyn_x(1,0) = -dt*gravity*cos(x.pos)/length;
    dyn_x(1,1) = 1. + dt*damping/inertia;
    // TODO: runge_kutta integration
}

void ilqr::compute_dynamics_u(State x, Action u){
    dyn_u(0) = 0.;
    dyn_u(1) = dt/inertia;
}

void ilqr::compute_stage_x(State x, Action u){
    en_diff = plant.calculate_total_energy(x) - goal_energy;
    stage_x(0) = 2.*sCp*(std::fmod(x.pos, 2.*M_PI) - goal.pos) + 2.*sCen*mass*gravity*length*sin(x.pos)*en_diff;
    stage_x(1) = 2.*sCv*(x.vel - goal.vel) + 2.*sCen*mass*pow(length, 2.)*x.vel*en_diff;
}

void ilqr::compute_stage_u(State x, Action u){
    stage_u = 2*sCu*u.u1;
}

void ilqr::compute_stage_xx(State x, Action u){
    en_diff = plant.calculate_total_energy(x) - goal_energy;
    stage_xx(0,0) = 2.*sCp + 2*sCen*(mass*gravity*length*cos(x.pos)*en_diff + pow(mass*gravity*length*sin(x.pos), 2.));
    stage_xx(0,1) = 2.*sCen*pow(mass, 2.)*gravity*pow(length, 3.)*sin(x.pos)*x.vel;
    stage_xx(1,0) = 2.*sCen*pow(mass, 2.)*gravity*pow(length, 3.)*sin(x.pos)*x.vel;
    stage_xx(1,1) = 2.*sCv + 2*sCen*(mass*pow(length, 2.)*en_diff + pow(mass*x.vel, 2.)*pow(length, 4.));
}

void ilqr::compute_stage_ux(State x, Action u){
    stage_ux(0) = 0.;
    stage_ux(1) = 0.;
}

void ilqr::compute_stage_uu(State x, Action u){
    stage_uu = 2.*sCu;
}

void ilqr::compute_final_x(State x){
    en_diff = plant.calculate_total_energy(x) - goal_energy;
    final_x(0) = 2.*fCp*(std::fmod(x.pos, 2.*M_PI) - goal.pos) + 2.*fCen*mass*gravity*length*sin(x.pos)*en_diff;
    final_x(1) = 2.*fCv*(x.vel - goal.vel) + 2.*fCen*mass*pow(length, 2.)*x.vel*en_diff;
}

void ilqr::compute_final_xx(State x){
    en_diff = plant.calculate_total_energy(x) - goal_energy;
    final_xx(0,0) = 2.*fCp + 2*fCen*(mass*gravity*length*cos(x.pos)*en_diff + pow(mass*gravity*length*sin(x.pos), 2.));
    final_xx(0,1) = 2.*fCen*pow(mass, 2.)*gravity*pow(length, 3.)*sin(x.pos)*x.vel;
    final_xx(1,0) = 2.*fCen*pow(mass, 2.)*gravity*pow(length, 3.)*sin(x.pos)*x.vel;
    final_xx(1,1) = 2.*fCv + 2*fCen*(mass*pow(length, 2.)*en_diff + pow(mass*x.vel, 2.)*pow(length, 4.));
}

void ilqr::compute_derivatives(State x, Action u){
    //en_diff = plant.calculate_total_energy(x) - goal_energy;

    compute_dynamics_x(x, u);
    compute_dynamics_u(x, u);
    compute_stage_x(x, u);
    compute_stage_u(x, u);
    compute_stage_xx(x, u);
    compute_stage_ux(x, u);
    compute_stage_uu(x, u);
    compute_final_x(x);
    compute_final_xx(x);

    if (verbose > 2){
        std::cout << "derivative terms" << std::endl
                  << "dyn_x " << dyn_x << std::endl
                  << "dyn_u " << dyn_u << std::endl
                  << "stage_x " << stage_x << std::endl
                  << "stage_u " << stage_u << std::endl
                  << "stage_xx " << stage_xx << std::endl
                  << "stage_ux "<< stage_ux << std::endl
                  << "stage_uu " << stage_uu << std::endl
                  << "final_x " << final_x << std::endl
                  << "final_xx " << final_xx << std::endl;
    }
}

void ilqr::rollout(){
    x_traj[0] = x0;
    for (int i=0; i<N-1; i++){
        x_traj[i+1] = pendulum_discrete_dynamics(x_traj[i], u_traj[i]);
    }
}

void ilqr::calculate_Q_terms(){
    Q_x = stage_x + dyn_x.transpose()*V_x;
    Q_u = stage_u + dyn_u.transpose()*V_x;
    Q_xx = stage_xx + dyn_x.transpose()*(V_xx*dyn_x);
    Q_ux = stage_ux + dyn_u.transpose()*(V_xx*dyn_x);
    Q_uu = stage_uu + dyn_u.transpose()*(V_xx*dyn_u);
    if (verbose > 2){
        std::cout << "calculate Q terms" << std::endl
                  << "Q_x " << Q_x << std::endl
                  << "Q_u " << Q_u << std::endl
                  << "Q_xx " << Q_xx << std::endl
                  << "Q_ux " << Q_ux << std::endl
                  << "Q_uu " << Q_uu << std::endl;
    }
}

void ilqr::calculate_gains(double regu){
    Q_uu_regu = Q_uu + regu;
    k = -1.0/Q_uu_regu*Q_u;
    K = -1.0/Q_uu_regu*Q_ux;
    if (verbose > 2){
        std::cout << "Gains" << std::endl
                  << "k " << k << std::endl
                  << "K " << K << std::endl;
    }
}

void ilqr::calculate_V_terms(){
    V_x = Q_x - K.transpose()*Q_uu*k;
    V_xx = Q_xx - K.transpose()*Q_uu*K;
    if (verbose > 2){
        std::cout << "calculate V terms" << std::endl
                  << "V_x " << V_x << std::endl
                  << "V_xx " << V_xx << std::endl;
    }
}

double ilqr::expected_cost_reduction(){
    //printf("expected_cost_redu: Q_u %f, Q_uu %f, k %f\n", Q_u, Q_uu, k);
    return -Q_u*k - 0.5*k*Q_uu*k;
}

void ilqr::forward_pass(){
    Eigen::Vector2d xdiff;
    x_traj_new[0] = x_traj[0];
    for (int i=0; i<N-1; i++){
        xdiff[0] = x_traj_new[i].pos - x_traj[i].pos;
        xdiff[1] = x_traj_new[i].vel - x_traj[i].vel;
        u_traj_new[i].u1 = u_traj[i].u1 + k_traj[i] + K_traj[i].transpose()*xdiff;
        x_traj_new[i+1] = pendulum_discrete_dynamics(x_traj_new[i], u_traj_new[i]);
    }
}

double ilqr::backward_pass(double regu){
    double expected_cost_redu = 0.0;
    compute_final_x(x_traj[N-1]);
    compute_final_xx(x_traj[N-1]);
    V_x = final_x;
    V_xx = final_xx;
    for(int i=N-2; i>-1; i--){
        compute_derivatives(x_traj[i], u_traj[i]);
        calculate_Q_terms();
        calculate_gains(regu);
        k_traj[i] = k;
        K_traj[i] = K;
        calculate_V_terms();
        expected_cost_redu += expected_cost_reduction();
        if (verbose > 2){
            printf("bw pass step %d expected_cost_redu %f\n", i, expected_cost_redu);
        }
    }
    return expected_cost_redu;
}

void ilqr::run_ilqr(int max_iter, double break_cost_redu, double regu_init){
    if (not warm_start_u){
        for (int i=0; i<N-1; i++){
            u_traj[i].u1 = ((double) rand() / RAND_MAX)*0.0001;
        }
    }
    if (not warm_start_x){
        rollout();
    }
    //for (int i=0; i<N-2; i++){
    //    printf("%e", u_traj[i].u1);
    //}
    //printf("\n\n");

    //for (int i=0; i<N-1; i++){
    //    printf("%e", x_traj[i].pos);
    //}
    //printf("\n");

    //printf("mass %f", mass);
    //printf("length %f", length);
    //printf("damping %f", damping);
    //printf("coulomb_friction %f", coulomb_friction);
    //printf("gravity %f", gravity);
    //printf("inertia %f", inertia);
    //printf("torque_limit %f", torque_limit);

    if (verbose > 1){
        std::cout << "mass " << mass << std::endl;
        std::cout << "length " << length << std::endl;
        std::cout << "damping " << damping << std::endl;
        std::cout << "coublomb_friction " << coulomb_friction << std::endl;
        std::cout << "gravity " << gravity << std::endl;
        std::cout << "inertia " << inertia << std::endl;
        std::cout << "torque_limit " << torque_limit << std::endl;
        std::cout << "dt " << dt << std::endl;
        std::cout << "integrator " << integrator << std::endl;
        std::cout << "start_pos " << x0.pos << std::endl;
        std::cout << "start_vel " << x0.vel << std::endl;
        std::cout << "goal_pos " << goal.pos << std::endl;
        std::cout << "goal_vel " << goal.vel << std::endl;
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
    }

    double total_cost = calculate_cost(false);
    double last_cost = total_cost;

    double regu = regu_init;
    double max_regu = 10000.;
    double min_regu = 0.01;
    double expected_cost_redu = 0;

    for (int n=0; n<max_iter; n++){
        expected_cost_redu = backward_pass(regu);
        forward_pass();
        total_cost = calculate_cost(true);

        //printf("iteration %d, last_pos %f, last cost %e, total_cost %e, ", n, x_traj_new[999].pos, last_cost, total_cost);
        if ((last_cost - total_cost) > 0.){
            // improvement
            last_cost = total_cost;
            for (int i=0; i<N-1; i++){
                x_traj[i] = x_traj_new[i];
                u_traj[i] = u_traj_new[i];
                u_traj_doubles[i] = u_traj_new[i].u1;
                p_traj_doubles[i] = x_traj_new[i].pos;
                v_traj_doubles[i] = x_traj_new[i].vel;
            }
            x_traj[N-1] = x_traj_new[N-1];
            p_traj_doubles[N-1] = x_traj_new[N-1].pos;
            v_traj_doubles[N-1] = x_traj_new[N-1].vel;
            regu *= 0.7;
        }
        else{
            //no improvement
            if (regu >= max_regu){
                //printf("\nOptimization failed\n");
                break;
            }
            regu *= 2.0;
        }
        if (regu < min_regu){
            regu = min_regu;
        } 
        if (regu > max_regu){
            regu = max_regu;
        }
        //printf("expected cost redu %f, regu: %f\n", expected_cost_redu, regu);
        if (expected_cost_redu < break_cost_redu){
            break;
        }
    }
}

double* ilqr::get_u_traj(){
    return u_traj_doubles;
}

double* ilqr::get_p_traj(){
    return p_traj_doubles;
}

double* ilqr::get_v_traj(){
    return v_traj_doubles;
}


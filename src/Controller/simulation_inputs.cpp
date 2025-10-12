#include "src/controller/simulation_inputs.h"

// Definition of the constant air density
double rho = 1.225;

// Initializes the Vehicle with default values
Vehicle::Vehicle() : R(0.0), a(0.0), b(0.0), m(0.0), gamma_w(0.0), Cd(0.0), Af(0.0), f_r_F(0.001) {}

// Initializes the Vehicle with specific parameters
Vehicle::Vehicle(double r, double a_val, double b_val, double mi, double gamma, double cd, double af, double f_r)
    : R(r), a(a_val), b(b_val), m(mi), gamma_w(gamma), Cd(cd), Af(af), f_r_F(f_r) {}

// Definition of the Individual constructors

// Initializes all member variables to default values
Individual::Individual() : delta(0.0), alpha_F_guess(0.0), fitness(0.0),
    Fz_F(0.0), Fz_R(0.0), MF_Fx_F(0.0), MF_Fx_R(0.0),
    MF_Fy_F(0.0), MF_Fy_R(0.0), r(0.0), beta(0.0),
    ay(0.0), alpha_F(0.0), alpha_R(0.0), kappa_F(0.0), kappa_R(0.0), converged(false) {}

// Initializes delta and alpha_F_guess with provided values, others to default
Individual::Individual(double d, double guess) : delta(d), alpha_F_guess(guess), fitness(0.0),
    Fz_F(0.0), Fz_R(0.0), MF_Fx_F(0.0), MF_Fx_R(0.0),
    MF_Fy_F(0.0), MF_Fy_R(0.0), r(0.0), beta(0.0),
    ay(0.0), alpha_F(0.0), alpha_R(0.0), kappa_F(0.0), kappa_R(0.0), converged(false) {}

void Individual::defineGuesses(double alpha_F_g, double alpha_R_g, double kappa_F_g, double kappa_R_g, double V_g, double Vx_g, double Vy_g){
    alpha_F_guess = alpha_F_g;
    alpha_R_guess = alpha_R_g;
    kappa_F_guess = kappa_F_g;
    kappa_R_guess = kappa_R_g;
    V_guess = V_g;
    Vx_guess = Vx_g;
    Vy_guess = Vy_g;
}

// Default with tires from Girish's thesis
void setDefaultTires(PacejkaParams &frontTire, PacejkaParams &rearTire){

    frontTire = createTireParams (      "Default Front Tire",
                                        1.622, 1.275, -0.1237, 0,          // p_Cx1, p_Dx1, p_Dx2, p_Dx3
                                        -0.1048, 0.7129, 0.3907, 0,          // p_Ex1, p_Ex2, p_Ex3, p_Ex4
                                        23.755, 2.195, 0.3222,               // p_Kx1, p_Kx2, p_Kx3
                                        0, 0,                                // p_Hx1, p_Hx2
                                        0, 0,                                // p_Vx1, p_Vx2
                                        19.278, -14.019, 0.9819,             // r_Bx1, r_Bx2, r_Cx1
                                        0, 0,                                // r_Ex1, r_Ex2
                                        0,                                   // r_Hx1
                                        1.4872, 1.0488, -0.23, 0.8878,       // p_Cy1, p_Dy1, p_Dy2, p_Dy3
                                        -0.8996, -0.5536, 0, -5.2807,      // p_Ey1, p_Ey2, p_Ey3, p_Ey4
                                        16.859, 1.9348, 0.1695,              // p_Ky1, p_Ky2, p_Ky3
                                        0, 0, 0.0041,                        // p_Hy1, p_Hy2, p_Hy3
                                        0, 0, 0.5365, 0.4555,                // p_Vy1, p_Vy2, p_Vy3, p_Vy4
                                        6.9875, 7.2, 0, 1.0074,              // r_By1, r_By2, r_By3, r_Cy1
                                        0, 0,                                // r_Ey1, r_Ey2
                                        0, 0,                                // r_Hy1, r_Hy2
                                        0, 0, 0, 0, 0, 0,                    // r_Vy1, r_Vy2, r_Vy3, r_Vy4, r_Vy5, r_Vy6
                                        8.6458, -1.0905, -2.8235, 0, 0.4723, 9.4811, 0,    // q_Bz1, q_Bz2, q_Bz3, q_Bz4, q_Bz5, q_Bz9, q_Bz10
                                        1.1479,                                             // q_Cz1
                                        0.1232, -0.0086, 0, -0.2289, 0, 0, 0.2597, 0.0279,  // q_Dz1, q_Dz2, q_Dz3, q_Dz4, q_Dz6, q_Dz7, q_Dz8, q_Dz9
                                        -3.2802, -0.7523, 0, 0, -3.9464,                    // q_Ez1, q_Ez2, q_Ez3, q_Ez4, q_Ez5
                                        0, 0, 0.0329, 0.0465,                               // q_Hz1, q_Hz2, q_Hz3, q_Hz4
                                        0, 0.0458, -0.9372, 0.504,                          // S_Sz1, S_Sz2, S_Sz3, S_Sz4
                                        1, 1, 1.4, 1, 1, 1, 1, 1,           // lambda_gammax, lambda_Cx, lambda_mux, lambda_Ex, lambda_Kx, lambda_Hx, lambda_Vx, lambda_xalpha
                                        1.45, 2, 1, 1, 1, 1, 1, 1, 1,        // lambda_muy, lambda_Ky, lambda_gammay, lambda_Cy, lambda_Ey, lambda_Hy, lambda_Vy, lambda_Vykappa, lambda_ykappa
                                        1, 5782, 1,                         // lambda_Fz0, F_z0, lambda_S
                                        1, 1, 1,
                                        0.3160);

    rearTire = createTireParams (       "Default Rear Tire",
                                        1.622, 1.275, -0.1237, 0,          // p_Cx1, p_Dx1, p_Dx2, p_Dx3
                                        -0.1048, 0.7129, 0.3907, 0,          // p_Ex1, p_Ex2, p_Ex3, p_Ex4
                                        23.755, 2.195, 0.3222,               // p_Kx1, p_Kx2, p_Kx3
                                        0, 0,                                // p_Hx1, p_Hx2
                                        0, 0,                                // p_Vx1, p_Vx2
                                        19.278, -14.019, 0.9819,             // r_Bx1, r_Bx2, r_Cx1
                                        0, 0,                                // r_Ex1, r_Ex2
                                        0,                                   // r_Hx1
                                        1.4872, 1.0488, -0.23, 0.8878,       // p_Cy1, p_Dy1, p_Dy2, p_Dy3
                                        -0.8996, -0.5536, 0, -5.2807,      // p_Ey1, p_Ey2, p_Ey3, p_Ey4
                                        16.859, 1.9348, 0.1695,              // p_Ky1, p_Ky2, p_Ky3
                                        0, 0, 0.0041,                        // p_Hy1, p_Hy2, p_Hy3
                                        0, 0, 0.5365, 0.4555,                // p_Vy1, p_Vy2, p_Vy3, p_Vy4
                                        6.9875, 7.2, 0, 1.0074,              // r_By1, r_By2, r_By3, r_Cy1
                                        0, 0,                                // r_Ey1, r_Ey2
                                        0, 0,                                // r_Hy1, r_Hy2
                                        0, 0, 0, 0, 0, 0,                    // r_Vy1, r_Vy2, r_Vy3, r_Vy4, r_Vy5, r_Vy6
                                        8.6458, -1.0905, -2.8235, 0, 0.4723, 9.4811, 0,    // q_Bz1, q_Bz2, q_Bz3, q_Bz4, q_Bz5, q_Bz9, q_Bz10
                                        1.1479,                                             // q_Cz1
                                        0.1232, -0.0086, 0, -0.2289, 0, 0, 0.2597, 0.0279,  // q_Dz1, q_Dz2, q_Dz3, q_Dz4, q_Dz6, q_Dz7, q_Dz8, q_Dz9
                                        -3.2802, -0.7523, 0, 0, -3.9464,                    // q_Ez1, q_Ez2, q_Ez3, q_Ez4, q_Ez5
                                        0, 0, 0.0329, 0.0465,                               // q_Hz1, q_Hz2, q_Hz3, q_Hz4
                                        0, 0.0458, -0.9372, 0.504,                          // S_Sz1, S_Sz2, S_Sz3, S_Sz4
                                        1, 1, 1.4, 1, 1, 1, 1, 1,           // lambda_gammax, lambda_Cx, lambda_mux, lambda_Ex, lambda_Kx, lambda_Hx, lambda_Vx, lambda_xalpha
                                        1.6, 2.75, 1, 1, 1, 1, 1, 1, 1,        // lambda_muy, lambda_Ky, lambda_gammay, lambda_Cy, lambda_Ey, lambda_Hy, lambda_Vy, lambda_Vykappa, lambda_ykappa
                                        1, 5782, 1,                         // lambda_Fz0, F_z0, lambda_S
                                        1, 1, 1,
                                        0.3160);
}


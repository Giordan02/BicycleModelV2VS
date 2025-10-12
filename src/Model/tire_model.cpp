#include "src/Model/tire_model.h"
#include <cmath>

// Funtion to create a tire with its parameters
PacejkaParams createTireParams(  QString name,   
                                 double p_Cx1, double p_Dx1, double p_Dx2, double p_Dx3,
                                 double p_Ex1, double p_Ex2, double p_Ex3, double p_Ex4,
                                 double p_Kx1, double p_Kx2, double p_Kx3,
                                 double p_Hx1, double p_Hx2,
                                 double p_Vx1, double p_Vx2,
                                 double r_Bx1, double r_Bx2, double r_Cx1,
                                 double r_Ex1, double r_Ex2,
                                 double r_Hx1,
                                 double p_Cy1, double p_Dy1, double p_Dy2, double p_Dy3,
                                 double p_Ey1, double p_Ey2, double p_Ey3, double p_Ey4,
                                 double p_Ky1, double p_Ky2, double p_Ky3,
                                 double p_Hy1, double p_Hy2, double p_Hy3,
                                 double p_Vy1, double p_Vy2, double p_Vy3, double p_Vy4,
                                 double r_By1, double r_By2, double r_By3, double r_Cy1,
                                 double r_Ey1, double r_Ey2,
                                 double r_Hy1, double r_Hy2,
                                 double r_Vy1, double r_Vy2, double r_Vy3, double r_Vy4, double r_Vy5, double r_Vy6,
                                 double q_Bz1, double q_Bz2, double q_Bz3, double q_Bz4, double q_Bz5, double q_Bz9, double q_Bz10,
                                 double q_Cz1,
                                 double q_Dz1, double q_Dz2, double q_Dz3, double q_Dz4, double q_Dz6, double q_Dz7, double q_Dz8, double q_Dz9,
                                 double q_Ez1, double q_Ez2, double q_Ez3, double q_Ez4, double q_Ez5,
                                 double q_Hz1, double q_Hz2, double q_Hz3, double q_Hz4,
                                 double S_Sz1, double S_Sz2, double S_Sz3, double S_Sz4,
                                 double lambda_gammax, double lambda_Cx, double lambda_mux, double lambda_Ex,
                                 double lambda_Kx, double lambda_Hx, double lambda_Vx, double lambda_xalpha,
                                 double lambda_muy, double lambda_Ky, double lambda_gammay, double lambda_Cy,
                                 double lambda_Ey, double lambda_Hy, double lambda_Vy, double lambda_Vykappa, double lambda_ykappa,
                                 double lambda_Fz0, double F_z0, double lambda_S,
                                 double lambda_gammaz, double lambda_t, double lambda_r,
                                 double R_0){
    PacejkaParams p;
    p.name = name;
    p.R_0 = R_0;
    // Longitudinal
    p.p_Cx1 = p_Cx1; p.p_Dx1 = p_Dx1; p.p_Dx2 = p_Dx2; p.p_Dx3 = p_Dx3;
    p.p_Ex1 = p_Ex1; p.p_Ex2 = p_Ex2; p.p_Ex3 = p_Ex3; p.p_Ex4 = p_Ex4;
    p.p_Kx1 = p_Kx1; p.p_Kx2 = p_Kx2; p.p_Kx3 = p_Kx3;
    p.p_Hx1 = p_Hx1; p.p_Hx2 = p_Hx2;
    p.p_Vx1 = p_Vx1; p.p_Vx2 = p_Vx2;
    p.r_Bx1 = r_Bx1; p.r_Bx2 = r_Bx2; p.r_Cx1 = r_Cx1;
    p.r_Ex1 = r_Ex1; p.r_Ex2 = r_Ex2;
    p.r_Hx1 = r_Hx1;
    
    // Lateral
    p.p_Cy1 = p_Cy1; p.p_Dy1 = p_Dy1; p.p_Dy2 = p_Dy2; p.p_Dy3 = p_Dy3;
    p.p_Ey1 = p_Ey1; p.p_Ey2 = p_Ey2; p.p_Ey3 = p_Ey3; p.p_Ey4 = p_Ey4;
    p.p_Ky1 = p_Ky1; p.p_Ky2 = p_Ky2; p.p_Ky3 = p_Ky3;
    p.p_Hy1 = p_Hy1; p.p_Hy2 = p_Hy2; p.p_Hy3 = p_Hy3;
    p.p_Vy1 = p_Vy1; p.p_Vy2 = p_Vy2; p.p_Vy3 = p_Vy3; p.p_Vy4 = p_Vy4;
    p.r_By1 = r_By1; p.r_By2 = r_By2; p.r_By3 = r_By3; p.r_Cy1 = r_Cy1;
    p.r_Ey1 = r_Ey1; p.r_Ey2 = r_Ey2;
    p.r_Hy1 = r_Hy1; p.r_Hy2 = r_Hy2;
    p.r_Vy1 = r_Vy1; p.r_Vy2 = r_Vy2; p.r_Vy3 = r_Vy3; p.r_Vy4 = r_Vy4; p.r_Vy5 = r_Vy5; p.r_Vy6 = r_Vy6;

    // Aligning Moment
    p.q_Bz1 = q_Bz1; p.q_Bz2 = q_Bz2; p.q_Bz3 = q_Bz3; p.q_Bz4 = q_Bz4; p.q_Bz5 = q_Bz5; p.q_Bz9 = q_Bz9; p.q_Bz10 = q_Bz10;
    p.q_Cz1 = q_Cz1;
    p.q_Dz1 = q_Dz1; p.q_Dz2 = q_Dz2; p.q_Dz3 = q_Dz3; p.q_Dz4 = q_Dz4; p.q_Dz6 = q_Dz6; p.q_Dz7 = q_Dz7; p.q_Dz8 = q_Dz8; p.q_Dz9 = q_Dz9;
    p.q_Ez1 = q_Ez1; p.q_Ez2 = q_Ez2; p.q_Ez3 = q_Ez3; p.q_Ez4 = q_Ez4; p.q_Ez5 = q_Ez5;
    p.q_Hz1 = q_Hz1; p.q_Hz2 = q_Hz2; p.q_Hz3 = q_Hz3; p.q_Hz4 = q_Hz4;
    p.S_Sz1 = S_Sz1; p.S_Sz2 = S_Sz2; p.S_Sz3 = S_Sz3; p.S_Sz4 = S_Sz4;

    // Scaling Factors
    p.lambda_gammax = lambda_gammax; p.lambda_Cx = lambda_Cx; p.lambda_mux = lambda_mux; p.lambda_Ex = lambda_Ex;
    p.lambda_Kx = lambda_Kx; p.lambda_Hx = lambda_Hx; p.lambda_Vx = lambda_Vx; p.lambda_xalpha = lambda_xalpha;
    p.lambda_muy = lambda_muy; p.lambda_Ky = lambda_Ky; p.lambda_gammay = lambda_gammay; p.lambda_Cy = lambda_Cy;
    p.lambda_Ey = lambda_Ey; p.lambda_Hy = lambda_Hy; p.lambda_Vy = lambda_Vy; p.lambda_Vykappa = lambda_Vykappa; p.lambda_ykappa = lambda_ykappa;
    p.lambda_Fz0 = lambda_Fz0; p.F_z0 = F_z0; p.lambda_S = lambda_S;
    p.lambda_gammaz = lambda_gammaz; p.lambda_t = lambda_t; p.lambda_r = lambda_r;

    return p;
}





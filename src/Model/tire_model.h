#ifndef TIREMODEL_H
#define TIREMODEL_H

/* 
    tire_model implements the Magic Formula 5.2, when its functions 
    needs the Pacejka Params of the tire, its vertical force, inclination
    angle, slip angle or/and the slip ratio, and returns the force asked for.
*/

#include <ceres/ceres.h>
#include <QString>
#include <cmath>


/**
 * @brief A template function that returns the sign of a value.
 * @param x The input value.
 * @return 1 for positive, -1 for negative, and 0 for zero.
 * @tparam T The numeric type (e.g., double, ceres::Jet).
 */
template<typename T>
T sgn(const T& x) {
    return (x > T(0)) ? T(1) : ((x < T(0)) ? T(-1) : T(0));
}

/**
 * @struct PacejkaParams
 * @brief Holds all the coefficients for the Pacejka 'Magic Formula' 5.2 tire model.
 * The parameters are grouped by the force or moment they affect.
 */
struct PacejkaParams {
    QString name;
    // Longitudinal (x) parameters
    double p_Cx1, p_Dx1, p_Dx2, p_Dx3;
    double p_Ex1, p_Ex2, p_Ex3, p_Ex4;
    double p_Kx1, p_Kx2, p_Kx3;
    double p_Hx1, p_Hx2;
    double p_Vx1, p_Vx2;
    double r_Bx1, r_Bx2, r_Cx1;
    double r_Ex1, r_Ex2;
    double r_Hx1;

    // Lateral (y) parameters
    double p_Cy1, p_Dy1, p_Dy2, p_Dy3;
    double p_Ey1, p_Ey2, p_Ey3, p_Ey4;
    double p_Ky1, p_Ky2, p_Ky3;
    double p_Hy1, p_Hy2, p_Hy3;
    double p_Vy1, p_Vy2, p_Vy3, p_Vy4;
    double r_By1, r_By2, r_By3, r_Cy1;
    double r_Ey1, r_Ey2;
    double r_Hy1, r_Hy2;
    double r_Vy1, r_Vy2, r_Vy3, r_Vy4, r_Vy5, r_Vy6;

    // Aligning Moment (z) parameters
    double q_Bz1, q_Bz2, q_Bz3, q_Bz4, q_Bz5, q_Bz9, q_Bz10;
    double q_Cz1;
    double q_Dz1, q_Dz2, q_Dz3, q_Dz4, q_Dz6, q_Dz7, q_Dz8, q_Dz9;
    double q_Ez1, q_Ez2, q_Ez3, q_Ez4, q_Ez5;
    double q_Hz1, q_Hz2, q_Hz3, q_Hz4;
    double S_Sz1, S_Sz2, S_Sz3, S_Sz4;

    // Scaling Factors
    // Longitudinal
    double lambda_gammax, lambda_Cx, lambda_mux, lambda_Ex, lambda_Kx, lambda_Hx, lambda_Vx, lambda_xalpha;
    // Lateral
    double lambda_muy, lambda_Ky, lambda_gammay, lambda_Cy, lambda_Ey, lambda_Hy, lambda_Vy, lambda_Vykappa, lambda_ykappa;
    // Aligning Moment
    double lambda_gammaz, lambda_t, lambda_r;
    // General
    double lambda_Fz0, F_z0, lambda_S;

    double R_0;
};

// Funtion to create a tire with its parameters
PacejkaParams createTireParams( QString name,
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
                                double R_0);

/*
        ------------------
        MAGIC FORMULA 5.2
        ------------------
The description of functions and coefficients are available
at Tire and Vehicle Dynamics 2 ed., Hans B. Pacejka.

*/

/**
 * @brief Calculates the PURE longitudinal tire force (Fx) for a given slip ratio.
 * @param params The PacejkaParams struct for the tire.
 * @param F_z The current vertical load on the tire in Newtons.
 * @param kappa The longitudinal slip ratio (dimensionless).
 * @param gamma The inclination (camber) angle in radians.
 * @return The calculated pure longitudinal force in Newtons.
 */
template <typename T>
T calculatePureLongitudinalForce(const PacejkaParams& params, const T& F_z, const T& kappa, const T& gamma) {
    T F_z0_prime = T(params.lambda_Fz0) * T(params.F_z0);
    T df_z = (F_z - F_z0_prime) / F_z0_prime;
    T gamma_x = gamma * T(params.lambda_gammax);
    T C_x = T(params.p_Cx1) * T(params.lambda_Cx);
    T mu_x = (T(params.p_Dx1) + T(params.p_Dx2) * df_z) * (T(1.0) - T(params.p_Dx3) * gamma_x * gamma_x) * T(params.lambda_mux);
    T D_x = mu_x * F_z;
    T K_x = F_z * (T(params.p_Kx1) + T(params.p_Kx2) * df_z) * ceres::exp(T(params.p_Kx3) * df_z) * T(params.lambda_Kx);
    T B_x = K_x / (C_x * D_x);
    T S_Hx = (T(params.p_Hx1) + T(params.p_Hx2) * df_z) * T(params.lambda_Hx);
    T kappa_x = kappa + S_Hx;
    T E_x = (T(params.p_Ex1) + T(params.p_Ex2) * df_z + T(params.p_Ex3) * df_z * df_z) * (T(1.0) - T(params.p_Ex4) * sgn(kappa_x)) * T(params.lambda_Ex);
    T S_Vx = F_z * (T(params.p_Vx1) + T(params.p_Vx2) * df_z) * T(params.lambda_Vx) * T(params.lambda_mux);
    T arg = B_x * kappa_x - E_x * (B_x * kappa_x - ceres::atan(B_x * kappa_x));
    T F_xo = D_x * ceres::sin(C_x * ceres::atan(arg)) + S_Vx;
    return F_xo;
}

/**
 * @brief Calculates the PURE lateral tire force (Fy) for a given slip angle.
 * @param params The PacejkaParams struct for the tire.
 * @param F_z The current vertical load on the tire in Newtons.
 * @param alpha The slip angle in radians.
 * @param gamma The inclination (camber) angle in radians.
 * @return The calculated pure lateral force in Newtons.
 */
template <typename T>
T calculatePureLateralForce(const PacejkaParams& params, const T& F_z, const T& alpha, const T& gamma) {
    T F_z0_prime = T(params.lambda_Fz0) * T(params.F_z0);
    T df_z = (F_z - F_z0_prime) / F_z0_prime;
    T gamma_y = gamma * T(params.lambda_gammay);
    T C_y = T(params.p_Cy1) * T(params.lambda_Cy);
    T mu_y = (T(params.p_Dy1) + T(params.p_Dy2) * df_z) * (T(1.0) - T(params.p_Dy3) * gamma_y * gamma_y) * T(params.lambda_muy);
    T D_y = mu_y * F_z;
    T K_y = T(params.p_Ky1) * F_z0_prime * ceres::sin(T(2.0) * ceres::atan(F_z / (T(params.p_Ky2) * F_z0_prime))) * (T(1.0) - T(params.p_Ky3) * ceres::abs(gamma_y)) * T(params.lambda_Ky); 
    T B_y = K_y / (C_y * D_y);
    T S_Hy = (T(params.p_Hy1) + T(params.p_Hy2) * df_z) * T(params.lambda_Hy) + T(params.p_Hy3) * gamma_y;
    T alpha_y = alpha + S_Hy;
    T E_y = (T(params.p_Ey1) + T(params.p_Ey2) * df_z) * (T(1.0) - (T(params.p_Ey3) + T(params.p_Ey4) * gamma_y) * sgn(alpha_y)) * T(params.lambda_Ey);
    T arg = B_y * alpha_y - E_y * (B_y * alpha_y - ceres::atan(B_y * alpha_y));
    T S_Vy = F_z * ((T(params.p_Vy1) + T(params.p_Vy2) * df_z) * T(params.lambda_Vy) + (T(params.p_Vy3) + T(params.p_Vy4) * df_z) * gamma_y) * T(params.lambda_muy);
    T F_yo = D_y * ceres::sin(C_y * ceres::atan(arg)) + S_Vy;
    return F_yo;
}

/**
 * @brief Calculates the PURE self-aligning moment (Mz) for a given slip angle.
 * @param params The PacejkaParams struct for the tire.
 * @param F_z The current vertical load on the tire in Newtons.
 * @param alpha The slip angle in radians.
 * @param gamma The inclination (camber) angle in radians.
 * @return The calculated pure self-aligning moment in Newton-meters.
 */
template <typename T>
T calculatePureAligningMoment(const PacejkaParams& params, const T& F_z, const T& alpha, const T& gamma) {
    double PI = 3.14159265358979323846;
    T F_z0_prime = T(params.lambda_Fz0) * T(params.F_z0);
    T df_z = (F_z - F_z0_prime) / F_z0_prime;
    T gamma_y = gamma * T(params.lambda_gammay);
    T C_y = T(params.p_Cy1) * T(params.lambda_Cy);
    T mu_y = (T(params.p_Dy1) + T(params.p_Dy2) * df_z) * (T(1.0) - T(params.p_Dy3) * gamma_y * gamma_y) * T(params.lambda_muy);
    T D_y = mu_y * F_z;
    T K_y = T(params.p_Ky1) * F_z0_prime * ceres::sin(T(2.0) * ceres::atan(F_z / (T(params.p_Ky2) * F_z0_prime))) * (T(1.0) - T(params.p_Ky3) * ceres::abs(gamma_y)) * T(params.lambda_Ky); 
    T B_y = K_y / (C_y * D_y);
    T S_Hy = (T(params.p_Hy1) + T(params.p_Hy2) * df_z) * T(params.lambda_Hy) + T(params.p_Hy3) * gamma_y;
    T alpha_y = alpha + S_Hy;
    T E_y = (T(params.p_Ey1) + T(params.p_Ey2) * df_z) * (T(1.0) - (T(params.p_Ey3) + T(params.p_Ey4) * gamma_y) * sgn(alpha_y)) * T(params.lambda_Ey);
    T arg = B_y * alpha_y - E_y * (B_y * alpha_y - ceres::atan(B_y * alpha_y));
    T S_Vy = F_z * ((T(params.p_Vy1) + T(params.p_Vy2) * df_z) * T(params.lambda_Vy) + (T(params.p_Vy3) + T(params.p_Vy4) * df_z) * gamma_y) * T(params.lambda_muy);
    T F_yo = D_y * ceres::sin(C_y * ceres::atan(arg)) + S_Vy;

    T S_Hf = S_Hy + S_Vy / K_y;
    T gamma_z = gamma * T(params.lambda_gammaz);
    T S_Ht = T(params.q_Hz1) + T(params.q_Hz2) * df_z + (T(params.q_Hz3) + T(params.q_Hz4) * df_z) * gamma_z;
    T alpha_t = alpha + S_Ht;
    T B_t = (T(params.q_Bz1) + T(params.q_Bz2) * df_z + T(params.q_Bz3) * df_z * df_z) * (T(1.0) + T(params.q_Bz4) * gamma_z + T(params.q_Bz5) * abs(gamma_z)) * T(params.lambda_Ky) / T(params.lambda_muy);
    T C_t = T(params.q_Cz1);
    T Et_factor = T(1.0) + (T(params.q_Ez4) + T(params.q_Ez5) * gamma_z) * (T(2.0) / T(PI)) * ceres::atan(B_t * C_t * alpha_t);
    T E_t = (T(params.q_Ez1) + T(params.q_Ez2) * df_z + T(params.q_Ez3) * df_z * df_z) * std::min(T(1.0), Et_factor);
    T D_t = F_z * (T(params.q_Dz1) + T(params.q_Dz2) * df_z) * (T(1.0) + T(params.q_Dz3) * gamma_z + T(params.q_Dz4) * gamma_z * gamma_z) * (T(params.R_0) / T(params.F_z0)) * T(params.lambda_t);
    T B_t_alpha_t = B_t * alpha_t;
    T term_inside_arctan = B_t_alpha_t - E_t * (B_t_alpha_t - ceres::atan(B_t_alpha_t));
    T t = D_t * ceres::cos(C_t * ceres::atan(term_inside_arctan)) * ceres::cos(alpha);
    T B_r = T(params.q_Bz9) * T(params.lambda_Ky) / T(params.lambda_muy) + T(params.q_Bz10) * B_y * C_y;
    T D_r = F_z * ((T(params.q_Dz6) + T(params.q_Dz7) * df_z) * T(params.lambda_r) + (T(params.q_Dz8) + T(params.q_Dz9) * df_z) * gamma_z) * T(params.R_0) * T(params.lambda_muy);
    T alpha_r = alpha + S_Hf;
    T M_zr = D_r * ceres::cos(ceres::atan(B_r * alpha_r)) * ceres::cos(alpha);
    T M_zo = -t * F_yo + M_zr;
    return M_zo;
}

/**
 * @brief Calculates the COMBINED slip longitudinal force (Fx).
 * This function accounts for the interaction between slip angle and slip ratio.
 * @return The calculated combined longitudinal force in Newtons.
 */
template <typename T>
T calculateCombinedLongitudinalForce(const PacejkaParams& params, const T& F_z, const T& alpha, const T& kappa, const T& gamma) {
    T F_x0 = calculatePureLongitudinalForce(params, F_z, kappa, gamma);
    T F_z0_prime = T(params.lambda_Fz0) * T(params.F_z0);
    T df_z = (F_z - F_z0_prime) / F_z0_prime;
    T S_Hxa = T(params.r_Hx1);
    T a_s = alpha + S_Hxa;
    T B_xa = T(params.r_Bx1) * ceres::cos(ceres::atan(T(params.r_Bx2) * kappa)) * T(params.lambda_xalpha);
    T C_xa = T(params.r_Cx1);
    T E_xa = T(params.r_Ex1) + T(params.r_Ex2) * df_z;
    T denom = ceres::cos(C_xa * ceres::atan(B_xa * S_Hxa - E_xa * (B_xa * S_Hxa - ceres::atan(B_xa * S_Hxa))));
    T D_xa = (ceres::abs(denom) > T(1e-10)) ? F_x0 / denom : F_x0;
    T arg = B_xa * a_s - E_xa * (B_xa * a_s - ceres::atan(B_xa * a_s));
    T F_x = D_xa * ceres::cos(C_xa * ceres::atan(arg));
    return F_x;
}

/**
 * @brief Calculates the COMBINED slip lateral force (Fy).
 * This function accounts for the interaction between slip ratio and slip angle.
 * @return The calculated combined lateral force in Newtons.
 */
template <typename T>
T calculateCombinedLateralForce(const PacejkaParams& params, const T& F_z, const T& alpha, const T& kappa, const T& gamma) {
    T F_y0 = calculatePureLateralForce(params, F_z, alpha, gamma);
    T F_z0_prime = T(params.lambda_Fz0) * T(params.F_z0);
    T df_z = (F_z - F_z0_prime) / F_z0_prime;
    T gamma_y = gamma * T(params.lambda_gammay);
    T mu_y = (T(params.p_Dy1) + T(params.p_Dy2) * df_z) * (T(1.0) - T(params.p_Dy3) * gamma_y * gamma_y) * T(params.lambda_muy);
    T S_Hyk = T(params.r_Hy1) + T(params.r_Hy2) * df_z;
    T k_s = kappa + S_Hyk;
    T B_yk = T(params.r_By1) * ceres::cos(ceres::atan(T(params.r_By2) * (alpha - T(params.r_By3)))) * T(params.lambda_ykappa);
    T C_yk = T(params.r_Cy1);
    T E_yk = T(params.r_Ey1) + T(params.r_Ey2) * df_z;
    T D_Vyk = mu_y * F_z * (T(params.r_Vy1) + T(params.r_Vy2) * df_z + T(params.r_Vy3) * gamma_y) * ceres::cos(ceres::atan(T(params.r_Vy4) * alpha));
    T S_Vyk = D_Vyk * ceres::sin(T(params.r_Vy5) * ceres::atan(T(params.r_Vy6) * kappa)) * T(params.lambda_Vykappa);
    T denom = ceres::cos(C_yk * ceres::atan(B_yk * S_Hyk - E_yk * (B_yk * S_Hyk - ceres::atan(B_yk * S_Hyk))));
    T D_yk = (ceres::abs(denom) > T(1e-10)) ? F_y0 / denom : F_y0;
    T arg = B_yk * k_s - E_yk * (B_yk * k_s - ceres::atan(B_yk * k_s));
    T F_y = D_yk * ceres::cos(C_yk * ceres::atan(arg)) + S_Vyk;
    return F_y;
}

/**
 * @brief Calculates the COMBINED slip self-aligning moment (Mz).
 * This function accounts for interactions from both Fx and Fy.
 * @return The calculated combined self-aligning moment in Newton-meters.
 */
template <typename T>
T calculateCombinedAligningMoment(const PacejkaParams& params, const T& F_z, const T& alpha, const T& kappa, const T& gamma) {
    double PI = 3.14159265358979323846;
    T F_z0_prime = T(params.lambda_Fz0) * T(params.F_z0);
    T df_z = (F_z - F_z0_prime) / F_z0_prime;
    T gamma_y = gamma * T(params.lambda_gammay);
    T C_y = T(params.p_Cy1) * T(params.lambda_Cy);
    T mu_y = (T(params.p_Dy1) + T(params.p_Dy2) * df_z) * (T(1.0) - T(params.p_Dy3) * gamma_y * gamma_y) * T(params.lambda_muy);
    T D_y = mu_y * F_z;
    T K_y = T(params.p_Ky1) * F_z0_prime * ceres::sin(T(2.0) * ceres::atan(F_z / (T(params.p_Ky2) * F_z0_prime))) * (T(1.0) - T(params.p_Ky3) * ceres::abs(gamma_y)) * T(params.lambda_Ky); 
    T B_y = K_y / (C_y * D_y);
    T S_Hy = (T(params.p_Hy1) + T(params.p_Hy2) * df_z) * T(params.lambda_Hy) + T(params.p_Hy3) * gamma_y;
    T alpha_y = alpha + S_Hy;
    T E_y = (T(params.p_Ey1) + T(params.p_Ey2) * df_z) * (T(1.0) - (T(params.p_Ey3) + T(params.p_Ey4) * gamma_y) * sgn(alpha_y)) * T(params.lambda_Ey);
    T arg = B_y * alpha_y - E_y * (B_y * alpha_y - ceres::atan(B_y * alpha_y));
    T S_Vy = F_z * ((T(params.p_Vy1) + T(params.p_Vy2) * df_z) * T(params.lambda_Vy) + (T(params.p_Vy3) + T(params.p_Vy4) * df_z) * gamma_y) * T(params.lambda_muy);

    T S_Hf = S_Hy + S_Vy / K_y;
    T gamma_z = gamma * T(params.lambda_gammaz);
    T S_Ht = T(params.q_Hz1) + T(params.q_Hz2) * df_z + (T(params.q_Hz3) + T(params.q_Hz4) * df_z) * gamma_z;
    T alpha_t = alpha + S_Ht;
    T B_t = (T(params.q_Bz1) + T(params.q_Bz2) * df_z + T(params.q_Bz3) * df_z * df_z) * (T(1.0) + T(params.q_Bz4) * gamma_z + T(params.q_Bz5) * abs(gamma_z)) * T(params.lambda_Ky) / T(params.lambda_muy);
    T C_t = T(params.q_Cz1);
    T Et_factor = T(1.0) + (T(params.q_Ez4) + T(params.q_Ez5) * gamma_z) * (T(2.0) / T(PI)) * ceres::atan(B_t * C_t * alpha_t);
    T E_t = (T(params.q_Ez1) + T(params.q_Ez2) * df_z + T(params.q_Ez3) * df_z * df_z) * std::min(T(1.0), Et_factor);
    T D_t = F_z * (T(params.q_Dz1) + T(params.q_Dz2) * df_z) * (T(1.0) + T(params.q_Dz3) * gamma_z + T(params.q_Dz4) * gamma_z * gamma_z) * (T(params.R_0) / T(params.F_z0)) * T(params.lambda_t);
    T B_t_alpha_t = B_t * alpha_t;
    T term_inside_arctan = B_t_alpha_t - E_t * (B_t_alpha_t - ceres::atan(B_t_alpha_t));
    T B_r = T(params.q_Bz9) * T(params.lambda_Ky) / T(params.lambda_muy) + T(params.q_Bz10) * B_y * C_y;
    T D_r = F_z * ((T(params.q_Dz6) + T(params.q_Dz7) * df_z) * T(params.lambda_r) + (T(params.q_Dz8) + T(params.q_Dz9) * df_z) * gamma_z) * T(params.R_0) * T(params.lambda_muy);
    T alpha_r = alpha + S_Hf;

    T K_x = F_z * (T(params.p_Kx1) + T(params.p_Kx2) * df_z) * ceres::exp(T(params.p_Kx3) * df_z) * T(params.lambda_Kx);

    T tan_alpha_t = ceres::tan(alpha_t);
    T Kx_div_Ky = K_x / K_y; 
    T term_under_sqrt_t = tan_alpha_t * tan_alpha_t + Kx_div_Ky * Kx_div_Ky * kappa * kappa;
    T alpha_t_eq = ceres::atan(sqrt(term_under_sqrt_t)) * sgn(alpha_t);

    T tan_alpha_r = ceres::tan(alpha_r);
    T term_under_sqrt_r = tan_alpha_r * tan_alpha_r + Kx_div_Ky * Kx_div_Ky * kappa * kappa;
    T alpha_r_eq = ceres::atan(sqrt(term_under_sqrt_r)) * sgn(alpha_r);

    T D_Vyk = mu_y * F_z * (T(params.r_Vy1) + T(params.r_Vy2) * df_z + T(params.r_Vy3) * gamma_y) * ceres::cos(ceres::atan(T(params.r_Vy4) * alpha));
    T S_Vyk = D_Vyk * ceres::sin(T(params.r_Vy5) * ceres::atan(T(params.r_Vy6) * kappa)) * T(params.lambda_Vykappa);
    T F_y = calculateCombinedLateralForce(params, F_z, alpha, kappa, gamma);
    T F_y_prime = F_y - S_Vyk; 

    T s = (T(params.S_Sz1) + T(params.S_Sz2) * (F_y / T(params.F_z0)) + (T(params.S_Sz3) + T(params.S_Sz4) * df_z) * gamma) * T(params.R_0) * T(params.lambda_S);

    T B_t_alpha_t_eq = B_t * alpha_t_eq;
    T term_inside_arctan_t = B_t_alpha_t_eq - E_t * (B_t_alpha_t_eq - ceres::atan(B_t_alpha_t_eq));
    T t = D_t * ceres::cos(C_t * ceres::atan(term_inside_arctan_t)) * ceres::cos(alpha);

    T M_zr = D_r * ceres::cos(ceres::atan(B_r * alpha_r_eq)) * ceres::cos(alpha);

    T F_x = calculateCombinedLongitudinalForce(params, F_z, alpha, kappa, gamma);

    T M_z = -t * F_y_prime + M_zr + s * F_x;

    return M_z;
}

#endif // TIREMODEL_H

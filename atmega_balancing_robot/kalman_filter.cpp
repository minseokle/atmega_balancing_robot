/*
 * kalman_filter.cpp
 *
 * Created: 2024-06-21 오후 7:30:18
 *  Author: 0311b
 */
#include "kalman_filter.h"

kalman_filter::kalman_filter() : x{ 0, 0 }, P{ { 0.02, 0 }, { 0, 0.02 } }, R{ 0.01, 0.01 }, Q{ 0.001, 0.001 }, dt(0.004)
{
}

kalman_filter::~kalman_filter()
{
}

void kalman_filter::update(double z[2], double res_x[2])
{
  // predict x
  double x_hat[2];
  x_hat[0] = x[0] + dt * x[1];
  x_hat[1] = x[1];

  // predict P
  double p_hat[2][2];

  p_hat[1][1] = P[1][1] + Q[1];
  p_hat[0][1] = P[0][1] + dt * P[1][1];
  p_hat[1][0] = P[1][0] + dt * P[1][1];
  p_hat[0][0] = P[0][0] + dt * P[0][1] + dt * p_hat[1][0] + Q[0];

  double p_hat_p_R[2][2];
  p_hat_p_R[0][0] = p_hat[0][0] + R[0];
  p_hat_p_R[0][1] = p_hat[0][1];
  p_hat_p_R[1][0] = p_hat[1][0];
  p_hat_p_R[1][1] = p_hat[1][1] + R[1];

  double det_p_hat_p_R = p_hat_p_R[0][0] * p_hat_p_R[1][1] - p_hat_p_R[1][0] * p_hat_p_R[0][1];
  double inv_p_hat_p_R[2][2];

  inv_p_hat_p_R[0][0] = p_hat_p_R[1][1] / det_p_hat_p_R;
  inv_p_hat_p_R[0][1] = -p_hat_p_R[0][1] / det_p_hat_p_R;
  inv_p_hat_p_R[1][0] = -p_hat_p_R[1][0] / det_p_hat_p_R;
  inv_p_hat_p_R[1][1] = p_hat_p_R[0][0] / det_p_hat_p_R;

  double K[2][2];
  K[0][0] = p_hat[0][0] * inv_p_hat_p_R[0][0] + p_hat[0][1] * inv_p_hat_p_R[1][0];
  K[0][1] = p_hat[0][0] * inv_p_hat_p_R[0][1] + p_hat[0][1] * inv_p_hat_p_R[1][1];
  K[1][0] = p_hat[1][0] * inv_p_hat_p_R[0][0] + p_hat[1][1] * inv_p_hat_p_R[1][0];
  K[1][1] = p_hat[1][0] * inv_p_hat_p_R[0][1] + p_hat[1][1] * inv_p_hat_p_R[1][1];

  double z_m_x_hat[2];
  z_m_x_hat[0] = z[0] - x[0];
  z_m_x_hat[1] = z[1] - x[1];

  x[0] = x_hat[0] + K[0][0] * z_m_x_hat[0] + K[0][1] * z_m_x_hat[1];
  x[1] = x_hat[1] + K[1][0] * z_m_x_hat[0] + K[1][1] * z_m_x_hat[1];

  P[0][0] = (1 - K[0][0]) * p_hat[0][0] - K[0][1] * p_hat[1][0];
  P[0][1] = (1 - K[0][0]) * p_hat[0][1] - K[0][1] * p_hat[1][1];
  P[1][0] = -K[1][0] * p_hat[0][0] + (1 - K[1][1]) * p_hat[1][0];
  P[1][1] = -K[1][0] * p_hat[0][1] + (1 - K[1][1]) * p_hat[1][1];
  
  res_x[0]=x[0];
  res_x[1]=x[1];
}
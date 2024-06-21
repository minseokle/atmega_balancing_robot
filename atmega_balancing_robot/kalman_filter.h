/*
 * kalman_filter.h
 *
 * Created: 2024-06-21 오후 7:30:00
 *  Author: 0311b
 */

#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

class kalman_filter
{
private:
  double x[2];
  double P[2][2];

  double R[2];
  double Q[2];

  double dt;

public:
  kalman_filter();
  ~kalman_filter();

  void update(double z[2], double res_x[2]);
};

#endif /* KALMAN_FILTER_H_ */
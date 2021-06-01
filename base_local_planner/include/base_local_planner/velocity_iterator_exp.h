#ifndef DWA_LOCAL_PLANNER_VELOCITY_ITERATOR_H_
#define DWA_LOCAL_PLANNER_VELOCITY_ITERATOR_H_
#include <algorithm>
#include <cmath>
#include <vector>
#include <ros/ros.h>

namespace base_local_planner {

  /**
   * We use the class to get samples between min and max, inluding zero if it is not included (and range goes from negative to positive)
   * We sample more points around the current value.
   */
  class VelocityIteratorExp {
    public:
      VelocityIteratorExp(double lower, double upper, int num_samples, double min_norm, double exp):
        VelocityIteratorExp((upper+lower)/2, lower, upper, num_samples, min_norm, exp) {};

      VelocityIteratorExp(double ref, double lower, double upper, int num_samples, double min_norm, double exp):
        current_index(0)
      {
        exp_ = exp;

        if (lower == upper) {
          samples_.push_back(lower);
        } else {
          num_samples = std::max(2, num_samples);

          // e.g. for 4 samples, split distance in 3 even parts


          // solving the quadratic function y=ax²+bx+c 
          // the apex
          double apex = interp(num_samples-1, ref, lower, upper); // the step at which the quadratic polynome has its extremum
          double d1 = ref - min_norm;
          double d2 = ref + min_norm;
          double a1 = (lower-d1) /  std::pow(apex, exp_); // coeff for left quadratic polynome 
          double a2 = (upper-d2) /  std::pow(num_samples-1-apex, exp_); // coeff for right quadratic polynome 

          if (std::isnan(a1)) { a1 = -1; }
          if (std::isnan(a2)) { a2 = 1; }


          // we make sure to avoid rounding errors around lower and upper.
          double current;
          double next = lower;
          bool is_zero_sampled = false;

        for (int j = 0; j < num_samples - 1; ++j) {
            current = next;
            samples_.push_back(current);

            if (j+1 < apex) {
                // calc step of left polynome
                next = poly(a1, apex-(j+1), exp_, d1);
            } 
            else if (j+1 == apex) {
                if (abs(d1 - d2) > 1e-14) {
                    // at the apex, we have to sample multiple points, if d1 != d2
                    // here we want to sample exactly the minimum value
                    samples_.push_back(d1);
                    samples_.push_back(0.0);
                    next = d2;
                } else {
                    next = ref;
                }
            } else {
                next = poly(a2, j+1-apex, exp_, d2);
            }

            if (current < d1 && next > d2 && next < upper) {
                // Add ref value whenever crossing the reference value
                // at this point, current should be equal to ref

                if (abs(d1 - d2) > 1e-14) {
                    // at the apex, we have to sample multiple points, if d1 != d2
                    samples_.push_back(d1);
                    samples_.push_back(0.0);
                    next = d2;
                } else {
                    next = ref;
                }

                j--;
            } 
            

            if (samples_.back() == 0.0) {
                is_zero_sampled = true;
            }
        }

        samples_.push_back(upper);
                
        // if 0 is among samples, this is never true. Else it inserts a 0 between the positive and negative samples
        if ((lower < 0) && (upper > 0) && !is_zero_sampled) {
            samples_.push_back(0.0);

            // Let's sort again, because we added the 0 at the end
            std::sort (samples_.begin(), samples_.end()); 
        }

        }
      }

      inline double poly(double const& a, double const& x, double const& exp, double const& d) {
          return a * std::pow(x, exp) + d;
      }

      double interp(int n, double x, double lower, double upper) {
          return n * ((x - lower) / (upper - lower));
      }

      double getVelocity(){
        return samples_.at(current_index);
      }

      VelocityIteratorExp& operator++(int){
        current_index++;
        return *this;
      }

      void reset(){
        current_index = 0;
      }

      bool isFinished(){
        return current_index >= samples_.size();
      }

    private:
      std::vector<double> samples_;
      unsigned int current_index;
      double exp_ = 2;
  };
};
#endif

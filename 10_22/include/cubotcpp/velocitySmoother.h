#ifndef VEL_SMOOTHER_H
#define VEL_SMOOTHER_H

#include <iostream>
#include <vector>
#include <algorithm>
#include <boost/date_time.hpp>
#include <boost/thread.hpp>
#include "boost/function.hpp"
#include "boost/bind.hpp"
#include <boost/date_time/gregorian/gregorian.hpp>
#include "msg.h"

namespace cubot
{

using namespace std;
using namespace boost::posix_time;

#define PERIOD_RECORD_SIZE    5
#define ZERO_VEL_COMMAND      Twist()
#define IS_ZERO_VEOCITY(a)   ((a.linear.x == 0.0) && (a.angular.z == 0.0))

class velocitysmoother
{
    typedef  boost::function<void(const Twist)> VelSmootherCallback;

    public:
    velocitysmoother(VelSmootherCallback velcacllback):
              velsmootherCB_(velcacllback),
              speed_lim_v  (1),   speed_lim_w  (1),
              accel_lim_v  (0.6), accel_lim_w  (0.3),
              decel_factor (1.0), frequency(20),
              pr_next(0),
              quiet(false), input_active(false)
          {
              decel_lim_v = decel_factor*accel_lim_v;
              decel_lim_w = decel_factor*accel_lim_w;
              thread_spin = boost::thread(boost::bind(&velocitysmoother::spin,this));
          }

          ~velocitysmoother()
          {
            thread_spin.join();
          }

          void spin();
          void twistin(const Twist & msg);
    private:

          std::string name;
          bool      quiet;        /**< Quieten some warnings that are unavoidable because of velocity multiplexing. **/
          double    speed_lim_v, accel_lim_v, decel_lim_v;
          double    speed_lim_w, accel_lim_w, decel_lim_w;
          double    decel_factor;
          double    frequency;

          bool                      input_active;
          double                    cb_avg_time;
          std::vector<double>       period_record; /**< Historic of latest periods between velocity commands */
          unsigned int              pr_next; /**< Next position to fill in the periods record buffer */

          Twist           target_vel;
          Twist           current_vel;
          Twist           last_cmd_vel;

          ptime   last_cb_time;
          boost::thread   thread_spin;
          VelSmootherCallback velsmootherCB_;

          double median(std::vector<double> values)
          {
            nth_element(values.begin(), values.begin() + values.size()/2, values.end());
            return values[values.size()/2];
          }
          double sign(double x)
          {
              return x < 0.0 ? -1.0 : +1.0;
          }

};


}
#endif

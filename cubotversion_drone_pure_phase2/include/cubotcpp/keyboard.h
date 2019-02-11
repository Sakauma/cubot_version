#ifndef KEYBOARD_H
#define KEYBOARD_H

#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>

#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>

#include "msg.h"


namespace cubot {
    using namespace std;

    typedef boost::function<void(const Twist)> KeyboardCallback;

    class keyboard
    {

    public:
        keyboard(KeyboardCallback callback) :
                keycallback_(callback),
                walk_vel_(0.5),
                run_vel_(1.0),
                yaw_rate_(1.0),
                yaw_rate_run_(1.5),
                kfd(0)
        {
            keyboard_thread = boost::thread(boost::bind(&keyboard::keyboardLoop, this));
        }

        ~
        keyboard() {
            keyboard_thread.join();
            tcsetattr(kfd, TCSANOW, &cooked);
        }

        void keyboardLoop();

    private:
        void stopRobot();

        double walk_vel_;
        double run_vel_;
        double yaw_rate_;
        double yaw_rate_run_;

        bool done;
        int kfd;
        struct termios cooked, raw;
        Twist cmdvel_;
        boost::thread keyboard_thread;
        KeyboardCallback keycallback_;

    };


}


#endif
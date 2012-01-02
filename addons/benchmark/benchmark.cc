#include <iostream>

#include <ctime>
#include <sys/time.h>
#include <algorithm>
#include <string>
#include <vector>
#include <cstdlib>

#include "model_generator.h"

using namespace std;

struct TimerInfo {
	/// time stamp when timer_start() gets called
	struct timeval clock_start_value;
	/// time stamp when the timer was stopped
	struct timeval clock_end_value;

	/// duration between clock_start_value and clock_end_value in seconds
	double duration_sec;
};

void timer_start (TimerInfo *timer) {
	gettimeofday (&(timer->clock_start_value), NULL);
}

double timer_stop (TimerInfo *timer) {
	gettimeofday(&(timer->clock_end_value), NULL);

	timer->duration_sec = static_cast<double> (
			timer->clock_end_value.tv_sec - timer->clock_start_value.tv_sec)
		+ static_cast<double>(timer->clock_end_value.tv_usec - timer->clock_start_value.tv_usec) * 1.0e-6;

	return timer->duration_sec;
}

int main (int argc, char *argv[]) {
	TimerInfo tinfo;

	timer_start (&tinfo);

	cout << "ping" << endl;

	sleep (2);
	timer_stop (&tinfo);

	cout << "pong: ";

	cout << tinfo.duration_sec << endl;

	return 0;
}

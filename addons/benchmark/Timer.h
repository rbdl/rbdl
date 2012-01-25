#ifndef _TIMER_H
#define _TIMER_H

#include <ctime>
#include <sys/time.h>

struct TimerInfo {
	/// time stamp when timer_start() gets called
	struct timeval clock_start_value;
	/// time stamp when the timer was stopped
	struct timeval clock_end_value;

	/// duration between clock_start_value and clock_end_value in seconds
	double duration_sec;
};

inline void timer_start (TimerInfo *timer) {
	gettimeofday (&(timer->clock_start_value), NULL);
}

inline double timer_stop (TimerInfo *timer) {
	gettimeofday(&(timer->clock_end_value), NULL);

	timer->duration_sec = static_cast<double> (
			timer->clock_end_value.tv_sec - timer->clock_start_value.tv_sec)
		+ static_cast<double>(timer->clock_end_value.tv_usec - timer->clock_start_value.tv_usec) * 1.0e-6;

	return timer->duration_sec;
}

#endif

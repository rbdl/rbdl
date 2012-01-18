#include <iostream>

#include <ctime>
#include <sys/time.h>
#include <algorithm>
#include <string>
#include <vector>
#include <cstdlib>
#include <iomanip>
#include <sstream>

#include "rbdl.h"
#include "model_generator.h"

using namespace std;
using namespace RigidBodyDynamics;

int benchmark_sample_count = 1000;
int benchmark_model_max_depth = 5;

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

void fill_random_vector_data (VectorNd &values, int count) {
	values.resize(count);
	int j;
	for (j = 0; j < count; j++) {
		values[j] = (rand() / RAND_MAX) * 2. -1.;
	}
}

double run_forward_dynamics_benchmark (Model *model, int sample_count) {
	VectorNd q = VectorNd::Zero (model->dof_count);
	VectorNd qdot = VectorNd::Zero (model->dof_count);
	VectorNd qddot = VectorNd::Zero (model->dof_count);
	VectorNd tau = VectorNd::Zero (model->dof_count);

	VectorNd *q_data = new VectorNd[sample_count];
	VectorNd *qdot_data = new VectorNd[sample_count];
	VectorNd *qddot_data = new VectorNd[sample_count];
	VectorNd *tau_data = new VectorNd[sample_count];

	for (int i = 0; i < sample_count; i++) {
	 	fill_random_vector_data (q_data[i], model->dof_count);
	 	fill_random_vector_data (qdot_data[i], model->dof_count);
		qddot_data[i].resize (model->dof_count);
		qddot_data[i].setZero();
	 	fill_random_vector_data (tau_data[i], model->dof_count);
	}

	TimerInfo tinfo;
	timer_start (&tinfo);

	for (int i = 0; i < sample_count; i++) {
		ForwardDynamics (*model, q_data[i], qdot_data[i], tau_data[i], qddot_data[i]);
	}

	double duration = timer_stop (&tinfo);

	cout << "#DOF: " << setw(3) << model->dof_count 
		<< " #samples: " << sample_count 
		<< " duration = " << setw(10) << duration << "(s)"
		<< " (~" << setw(10) << duration / sample_count << "(s) per call)" << endl;
	
	delete[] q_data;
	delete[] qdot_data;
	delete[] qddot_data;
	delete[] tau_data;

	return duration;
}

void print_usage () {
	cout << "Usage: benchmark [--count|-c <sample_count>] [--depth|-d <depth>]" << endl;
	cout << "Simple benchmark tool for the Rigid Body Dynamics Library." << endl;
	cout << "  --count | -c <sample_count> : sets the number of sample states that should" << endl;
	cout << "                be calculated (default: 1000)" << endl;
	cout << "  --depth | -d <depth>        : sets maximum depth for the branched test model" << endl;
	cout << "                which is created increased from 1 to <depth> (default: 5)." << endl;
}

void parse_args (int argc, char* argv[]) {
	int argi = 1;
	while (argi < argc) {
		string arg = argv[argi];

		if (arg == "--help" || arg == "-h") {
			print_usage();
			exit (1);
		} else if (arg == "--count" || arg == "-c" ) {
			if (argi == argc - 1) {
				print_usage();

				cerr << "Error: missing number of samples!" << endl;
				exit (1);
			}

			argi++;
			stringstream count_stream (argv[argi]);

			count_stream >> benchmark_sample_count;
		} else if (arg == "--depth" || arg == "-d" ) {
			if (argi == argc - 1) {
				print_usage();

				cerr << "Error: missing number for model depth!" << endl;
				exit (1);
			}

			argi++;
			stringstream depth_stream (argv[argi]);

			depth_stream >> benchmark_model_max_depth;
		} else {
			print_usage();
			cerr << "Invalid argument '" << arg << "'." << endl;
			exit(1);
		}
		argi++;
	}
}

int main (int argc, char *argv[]) {
	parse_args (argc, argv);

	Model *model = NULL;

	for (int depth = 1; depth <= benchmark_model_max_depth; depth++) {
		model = new Model();
		model->Init();
		model->gravity = Vector3d (0., -9.81, 0.);

		generate_planar_tree (model, depth);

		VectorNd q = VectorNd::Zero (model->dof_count);
		VectorNd qdot = VectorNd::Zero (model->dof_count);
		VectorNd qddot = VectorNd::Zero (model->dof_count);
		VectorNd tau = VectorNd::Zero (model->dof_count);

		run_forward_dynamics_benchmark (model, benchmark_sample_count);

		delete model;
	}

	return 0;
}

#include <iostream>

#include <algorithm>
#include <string>
#include <vector>
#include <cstdlib>
#include <iomanip>
#include <sstream>

#include "rbdl.h"
#include "model_generator.h"
#include "SampleData.h"
#include "Timer.h"

using namespace std;
using namespace RigidBodyDynamics;

int benchmark_sample_count = 1000;
int benchmark_model_max_depth = 5;

double run_forward_dynamics_ABA_benchmark (Model *model, int sample_count) {
	SampleData sample_data;
	sample_data.fill_random_data(model->dof_count, sample_count);

	TimerInfo tinfo;
	timer_start (&tinfo);

	for (int i = 0; i < sample_count; i++) {
		ForwardDynamics (*model,
				sample_data.q_data[i],
				sample_data.qdot_data[i],
				sample_data.tau_data[i],
				sample_data.qddot_data[i]);
	}

	double duration = timer_stop (&tinfo);

	cout << "#DOF: " << setw(3) << model->dof_count 
		<< " #samples: " << sample_count 
		<< " duration = " << setw(10) << duration << "(s)"
		<< " (~" << setw(10) << duration / sample_count << "(s) per call)" << endl;
	
	return duration;
}

double run_forward_dynamics_lagrangian_benchmark (Model *model, int sample_count) {
	SampleData sample_data;
	sample_data.fill_random_data(model->dof_count, sample_count);

	TimerInfo tinfo;
	timer_start (&tinfo);

	for (int i = 0; i < sample_count; i++) {
		ForwardDynamicsLagrangian (*model,
				sample_data.q_data[i],
				sample_data.qdot_data[i],
				sample_data.tau_data[i],
				sample_data.qddot_data[i]);
	}

	double duration = timer_stop (&tinfo);

	cout << "#DOF: " << setw(3) << model->dof_count 
		<< " #samples: " << sample_count 
		<< " duration = " << setw(10) << duration << "(s)"
		<< " (~" << setw(10) << duration / sample_count << "(s) per call)" << endl;
	
	return duration;
}

double run_inverse_dynamics_RNEA_benchmark (Model *model, int sample_count) {
	SampleData sample_data;
	sample_data.fill_random_data(model->dof_count, sample_count);

	TimerInfo tinfo;
	timer_start (&tinfo);

	for (int i = 0; i < sample_count; i++) {
		InverseDynamics (*model,
				sample_data.q_data[i],
				sample_data.qdot_data[i],
				sample_data.qddot_data[i],
				sample_data.tau_data[i]
				);
	}

	double duration = timer_stop (&tinfo);

	cout << "#DOF: " << setw(3) << model->dof_count 
		<< " #samples: " << sample_count 
		<< " duration = " << setw(10) << duration << "(s)"
		<< " (~" << setw(10) << duration / sample_count << "(s) per call)" << endl;
	
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

	cout << "= ForwardDynamics ABA =" << endl;
	for (int depth = 1; depth <= benchmark_model_max_depth; depth++) {
		model = new Model();
		model->Init();
		model->gravity = Vector3d (0., -9.81, 0.);

		generate_planar_tree (model, depth);

		run_forward_dynamics_ABA_benchmark (model, benchmark_sample_count);

		delete model;
	}
	cout << endl;

	cout << "= ForwardDynamics Lagrangian =" << endl;
	for (int depth = 1; depth <= benchmark_model_max_depth; depth++) {
		model = new Model();
		model->Init();
		model->gravity = Vector3d (0., -9.81, 0.);

		generate_planar_tree (model, depth);

		run_forward_dynamics_lagrangian_benchmark (model, benchmark_sample_count);

		delete model;
	}
	cout << endl;

	cout << "= InverseDynamics RNEA =" << endl;
	for (int depth = 1; depth <= benchmark_model_max_depth; depth++) {
		model = new Model();
		model->Init();
		model->gravity = Vector3d (0., -9.81, 0.);

		generate_planar_tree (model, depth);

		run_inverse_dynamics_RNEA_benchmark (model, benchmark_sample_count);

		delete model;
	}

	return 0;
}

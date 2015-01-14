#include <iostream>

#include <algorithm>
#include <string>
#include <vector>
#include <cstdlib>
#include <iomanip>
#include <sstream>

#include "rbdl/rbdl.h"
#include "model_generator.h"
#include "Human36Model.h"
#include "SampleData.h"
#include "Timer.h"

#ifdef RBDL_BUILD_ADDON_LUAMODEL
#include "../addons/luamodel/luamodel.h"
bool have_luamodel = true;
#else
bool have_luamodel = false;
#endif

#ifdef RBDL_BUILD_ADDON_URDFREADER
#include "../addons/urdfreader/urdfreader.h"
bool have_urdfreader = true;
#else
bool have_urdfreader = false;
#endif

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

int benchmark_sample_count = 1000;
int benchmark_model_max_depth = 5;

bool benchmark_run_fd_aba = true;
bool benchmark_run_fd_lagrangian = true;
bool benchmark_run_id_rnea = true;
bool benchmark_run_crba = true;
bool benchmark_run_nle = true;
bool benchmark_run_contacts = false;

string model_file = "";

enum ContactsMethod {
	ContactsMethodLagrangian = 0,
	ContactsMethodRangeSpaceSparse,
	ContactsMethodNullSpace,
	ContactsMethodKokkevis
};

double run_forward_dynamics_ABA_benchmark (Model *model, int sample_count) {
	SampleData sample_data;
	sample_data.fillRandom(model->dof_count, sample_count);

	TimerInfo tinfo;
	timer_start (&tinfo);

	for (int i = 0; i < sample_count; i++) {
		ForwardDynamics (*model,
				sample_data.q[i],
				sample_data.qdot[i],
				sample_data.tau[i],
				sample_data.qddot[i]);
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
	sample_data.fillRandom(model->dof_count, sample_count);

	TimerInfo tinfo;
	timer_start (&tinfo);

	MatrixNd H (MatrixNd::Zero(model->dof_count, model->dof_count));
	VectorNd C (VectorNd::Zero(model->dof_count));

	for (int i = 0; i < sample_count; i++) {
		ForwardDynamicsLagrangian (*model,
				sample_data.q[i],
				sample_data.qdot[i],
				sample_data.tau[i],
				sample_data.qddot[i],
				Math::LinearSolverPartialPivLU,
				NULL,
				&H,
				&C
);
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
	sample_data.fillRandom(model->dof_count, sample_count);

	TimerInfo tinfo;
	timer_start (&tinfo);

	for (int i = 0; i < sample_count; i++) {
		InverseDynamics (*model,
				sample_data.q[i],
				sample_data.qdot[i],
				sample_data.qddot[i],
				sample_data.tau[i]
				);
	}

	double duration = timer_stop (&tinfo);

	cout << "#DOF: " << setw(3) << model->dof_count 
		<< " #samples: " << sample_count 
		<< " duration = " << setw(10) << duration << "(s)"
		<< " (~" << setw(10) << duration / sample_count << "(s) per call)" << endl;

	return duration;
}

double run_CRBA_benchmark (Model *model, int sample_count) {
	SampleData sample_data;
	sample_data.fillRandom(model->dof_count, sample_count);

	Math::MatrixNd H = Math::MatrixNd::Zero(model->dof_count, model->dof_count);

	TimerInfo tinfo;
	timer_start (&tinfo);

	for (int i = 0; i < sample_count; i++) {
		CompositeRigidBodyAlgorithm (*model, sample_data.q[i], H, true);
	}

	double duration = timer_stop (&tinfo);

	cout << "#DOF: " << setw(3) << model->dof_count 
		<< " #samples: " << sample_count 
		<< " duration = " << setw(10) << duration << "(s)"
		<< " (~" << setw(10) << duration / sample_count << "(s) per call)" << endl;

	return duration;
}

double run_nle_benchmark (Model *model, int sample_count) {
	SampleData sample_data;
	sample_data.fillRandom(model->dof_count, sample_count);

	TimerInfo tinfo;
	timer_start (&tinfo);

	for (int i = 0; i < sample_count; i++) {
		NonlinearEffects (*model,
				sample_data.q[i],
				sample_data.qdot[i],
				sample_data.tau[i]
				);
	}

	double duration = timer_stop (&tinfo);

	cout << "#DOF: " << setw(3) << model->dof_count 
		<< " #samples: " << sample_count 
		<< " duration = " << setw(10) << duration << "(s)"
		<< " (~" << setw(10) << duration / sample_count << "(s) per call)" << endl;

	return duration;
}

double run_contacts_lagrangian_benchmark (Model *model, ConstraintSet *constraint_set, int sample_count) {
	SampleData sample_data;
	sample_data.fillRandom(model->dof_count, sample_count);

	TimerInfo tinfo;
	timer_start (&tinfo);

	for (int i = 0; i < sample_count; i++) {
		ForwardDynamicsContactsDirect (*model, sample_data.q[i], sample_data.qdot[i], sample_data.tau[i], *constraint_set, sample_data.qddot[i]); 
	}

	double duration = timer_stop (&tinfo);

	return duration;
}

double run_contacts_lagrangian_sparse_benchmark (Model *model, ConstraintSet *constraint_set, int sample_count) {
	SampleData sample_data;
	sample_data.fillRandom(model->dof_count, sample_count);

	TimerInfo tinfo;
	timer_start (&tinfo);

	for (int i = 0; i < sample_count; i++) {
		ForwardDynamicsContactsRangeSpaceSparse (*model, sample_data.q[i], sample_data.qdot[i], sample_data.tau[i], *constraint_set, sample_data.qddot[i]); 
	}

	double duration = timer_stop (&tinfo);

	return duration;
}

double run_contacts_null_space (Model *model, ConstraintSet *constraint_set, int sample_count) {
	SampleData sample_data;
	sample_data.fillRandom(model->dof_count, sample_count);

	TimerInfo tinfo;
	timer_start (&tinfo);

	for (int i = 0; i < sample_count; i++) {
		ForwardDynamicsContactsNullSpace (*model, sample_data.q[i], sample_data.qdot[i], sample_data.tau[i], *constraint_set, sample_data.qddot[i]); 
	}

	double duration = timer_stop (&tinfo);

	return duration;
}

double run_contacts_kokkevis_benchmark (Model *model, ConstraintSet *constraint_set, int sample_count) {
	SampleData sample_data;
	sample_data.fillRandom(model->dof_count, sample_count);

	TimerInfo tinfo;
	timer_start (&tinfo);

	for (int i = 0; i < sample_count; i++) {
		ForwardDynamicsContactsKokkevis (*model, sample_data.q[i], sample_data.qdot[i], sample_data.tau[i], *constraint_set, sample_data.qddot[i]); 
	}

	double duration = timer_stop (&tinfo);

	return duration;
}

double contacts_benchmark (int sample_count, ContactsMethod contacts_method) {
	// initialize the human model
	Model *model = new Model();
	generate_human36model(model);

	// initialize the constraint sets
	unsigned int foot_r = model->GetBodyId ("foot_r");
	unsigned int foot_l = model->GetBodyId ("foot_l");
	unsigned int hand_r = model->GetBodyId ("hand_r");
	unsigned int hand_l = model->GetBodyId ("hand_l");

	ConstraintSet one_body_one_constraint;
	ConstraintSet two_bodies_one_constraint;
	ConstraintSet four_bodies_one_constraint;

	ConstraintSet one_body_four_constraints;
	ConstraintSet two_bodies_four_constraints;
	ConstraintSet four_bodies_four_constraints;

	LinearSolver linear_solver = LinearSolverPartialPivLU;

	one_body_one_constraint.linear_solver = linear_solver;
	two_bodies_one_constraint.linear_solver = linear_solver;
	four_bodies_one_constraint.linear_solver = linear_solver;
	one_body_four_constraints.linear_solver = linear_solver;
	two_bodies_four_constraints.linear_solver = linear_solver;
	four_bodies_four_constraints.linear_solver = linear_solver;

	// one_body_one
	one_body_one_constraint.AddConstraint (foot_r, Vector3d (0.1, 0., -0.05), Vector3d (1., 0., 0.));
	one_body_one_constraint.Bind (*model);

	// two_bodies_one
	two_bodies_one_constraint.AddConstraint (foot_r, Vector3d (0.1, 0., -0.05), Vector3d (1., 0., 0.));
	two_bodies_one_constraint.AddConstraint (foot_l, Vector3d (0.1, 0., -0.05), Vector3d (1., 0., 0.));
	two_bodies_one_constraint.Bind (*model);

	// four_bodies_one
	four_bodies_one_constraint.AddConstraint (foot_r, Vector3d (0.1, 0., -0.05), Vector3d (1., 0., 0.));
	four_bodies_one_constraint.AddConstraint (foot_l, Vector3d (0.1, 0., -0.05), Vector3d (1., 0., 0.));
	four_bodies_one_constraint.AddConstraint (hand_r, Vector3d (0.1, 0., -0.05), Vector3d (1., 0., 0.));
	four_bodies_one_constraint.AddConstraint (hand_l, Vector3d (0.1, 0., -0.05), Vector3d (1., 0., 0.));
	four_bodies_one_constraint.Bind (*model);

	// one_body_four
	one_body_four_constraints.AddConstraint (foot_r, Vector3d (0.1, 0., -0.05), Vector3d (1., 0., 0.));
	one_body_four_constraints.AddConstraint (foot_r, Vector3d (0.1, 0., -0.05), Vector3d (0., 1., 0.));
	one_body_four_constraints.AddConstraint (foot_r, Vector3d (0.1, 0., -0.05), Vector3d (0., 0., 1.));
	one_body_four_constraints.AddConstraint (foot_r, Vector3d (-0.1, 0., -0.05), Vector3d (1., 0., 0.));
	one_body_four_constraints.Bind (*model);	

	// two_bodies_four
	two_bodies_four_constraints.AddConstraint (foot_r, Vector3d (0.1, 0., -0.05), Vector3d (1., 0., 0.));
	two_bodies_four_constraints.AddConstraint (foot_r, Vector3d (0.1, 0., -0.05), Vector3d (0., 1., 0.));
	two_bodies_four_constraints.AddConstraint (foot_r, Vector3d (0.1, 0., -0.05), Vector3d (0., 0., 1.));
	two_bodies_four_constraints.AddConstraint (foot_r, Vector3d (-0.1, 0., -0.05), Vector3d (1., 0., 0.));
	
	two_bodies_four_constraints.AddConstraint (foot_l, Vector3d (0.1, 0., -0.05), Vector3d (1., 0., 0.));
	two_bodies_four_constraints.AddConstraint (foot_l, Vector3d (0.1, 0., -0.05), Vector3d (0., 1., 0.));
	two_bodies_four_constraints.AddConstraint (foot_l, Vector3d (0.1, 0., -0.05), Vector3d (0., 0., 1.));
	two_bodies_four_constraints.AddConstraint (foot_l, Vector3d (-0.1, 0., -0.05), Vector3d (1., 0., 0.));

	two_bodies_four_constraints.Bind (*model);

	// four_bodies_four
	four_bodies_four_constraints.AddConstraint (foot_r, Vector3d (0.1, 0., -0.05), Vector3d (1., 0., 0.));
	four_bodies_four_constraints.AddConstraint (foot_r, Vector3d (0.1, 0., -0.05), Vector3d (0., 1., 0.));
	four_bodies_four_constraints.AddConstraint (foot_r, Vector3d (0.1, 0., -0.05), Vector3d (0., 0., 1.));
	four_bodies_four_constraints.AddConstraint (foot_r, Vector3d (-0.1, 0., -0.05), Vector3d (1., 0., 0.));
	
	four_bodies_four_constraints.AddConstraint (foot_l, Vector3d (0.1, 0., -0.05), Vector3d (1., 0., 0.));
	four_bodies_four_constraints.AddConstraint (foot_l, Vector3d (0.1, 0., -0.05), Vector3d (0., 1., 0.));
	four_bodies_four_constraints.AddConstraint (foot_l, Vector3d (0.1, 0., -0.05), Vector3d (0., 0., 1.));
	four_bodies_four_constraints.AddConstraint (foot_l, Vector3d (-0.1, 0., -0.05), Vector3d (1., 0., 0.));

	four_bodies_four_constraints.AddConstraint (hand_r, Vector3d (0.1, 0., -0.05), Vector3d (1., 0., 0.));
	four_bodies_four_constraints.AddConstraint (hand_r, Vector3d (0.1, 0., -0.05), Vector3d (0., 1., 0.));
	four_bodies_four_constraints.AddConstraint (hand_r, Vector3d (0.1, 0., -0.05), Vector3d (0., 0., 1.));
	four_bodies_four_constraints.AddConstraint (hand_r, Vector3d (-0.1, 0., -0.05), Vector3d (1., 0., 0.));
	
	four_bodies_four_constraints.AddConstraint (hand_l, Vector3d (0.1, 0., -0.05), Vector3d (1., 0., 0.));
	four_bodies_four_constraints.AddConstraint (hand_l, Vector3d (0.1, 0., -0.05), Vector3d (0., 1., 0.));
	four_bodies_four_constraints.AddConstraint (hand_l, Vector3d (0.1, 0., -0.05), Vector3d (0., 0., 1.));
	four_bodies_four_constraints.AddConstraint (hand_l, Vector3d (-0.1, 0., -0.05), Vector3d (1., 0., 0.));

	four_bodies_four_constraints.Bind (*model);

	cout << "= #DOF: " << setw(3) << model->dof_count << endl;
	cout << "= #samples: " << sample_count << endl;
	cout << "= No constraints (Articulated Body Algorithm):" << endl;
	run_forward_dynamics_ABA_benchmark (model, sample_count);
	cout << "= Constraints:" << endl;
	double duration;

	// one body one
	if (contacts_method == ContactsMethodLagrangian) {
		duration = run_contacts_lagrangian_benchmark (model, &one_body_one_constraint, sample_count);
	} else if (contacts_method == ContactsMethodRangeSpaceSparse) {
		duration = run_contacts_lagrangian_sparse_benchmark (model, &one_body_one_constraint, sample_count);
	} else if (contacts_method == ContactsMethodNullSpace) {
		duration = run_contacts_null_space (model, &one_body_one_constraint, sample_count);
	} else {
		duration = run_contacts_kokkevis_benchmark (model, &one_body_one_constraint, sample_count);
	}

	cout << "ConstraintSet: 1 Body 1 Constraint   : "
		<< " duration = " << setw(10) << duration << "(s)"
		<< " (~" << setw(10) << duration / sample_count << "(s) per call)" << endl;

	// two_bodies_one
	if (contacts_method == ContactsMethodLagrangian) {
		duration = run_contacts_lagrangian_benchmark (model, &two_bodies_one_constraint, sample_count);
	} else if (contacts_method == ContactsMethodRangeSpaceSparse) {
		duration = run_contacts_lagrangian_sparse_benchmark (model, &two_bodies_one_constraint, sample_count);
	} else if (contacts_method == ContactsMethodNullSpace) {
		duration = run_contacts_null_space (model, &two_bodies_one_constraint, sample_count);
	} else {
		duration = run_contacts_kokkevis_benchmark (model, &two_bodies_one_constraint, sample_count);
	}

	cout << "ConstraintSet: 2 Bodies 1 Constraint : "
		<< " duration = " << setw(10) << duration << "(s)"
		<< " (~" << setw(10) << duration / sample_count << "(s) per call)" << endl;


	// four_bodies_one
	if (contacts_method == ContactsMethodLagrangian) {
		duration = run_contacts_lagrangian_benchmark (model, &four_bodies_one_constraint, sample_count);
	} else if (contacts_method == ContactsMethodRangeSpaceSparse) {
		duration = run_contacts_lagrangian_sparse_benchmark (model, &four_bodies_one_constraint, sample_count);
	} else if (contacts_method == ContactsMethodNullSpace) {
		duration = run_contacts_null_space (model, &four_bodies_one_constraint, sample_count);
	} else {
		duration = run_contacts_kokkevis_benchmark (model, &four_bodies_one_constraint, sample_count);
	}

	cout << "ConstraintSet: 4 Bodies 1 Constraint : "
		<< " duration = " << setw(10) << duration << "(s)"
		<< " (~" << setw(10) << duration / sample_count << "(s) per call)" << endl;

	// one_body_four
	if (contacts_method == ContactsMethodLagrangian) {
		duration = run_contacts_lagrangian_benchmark (model, &one_body_four_constraints, sample_count);
	} else if (contacts_method == ContactsMethodRangeSpaceSparse) {
		duration = run_contacts_lagrangian_sparse_benchmark (model, &one_body_four_constraints, sample_count);
	} else if (contacts_method == ContactsMethodNullSpace) {
		duration = run_contacts_null_space (model, &one_body_four_constraints, sample_count);
	} else {
		duration = run_contacts_kokkevis_benchmark (model, &one_body_four_constraints, sample_count);
	}

	cout << "ConstraintSet: 1 Body 4 Constraints  : "
		<< " duration = " << setw(10) << duration << "(s)"
		<< " (~" << setw(10) << duration / sample_count << "(s) per call)" << endl;

	// two_bodies_four
	if (contacts_method == ContactsMethodLagrangian) {
		duration = run_contacts_lagrangian_benchmark (model, &two_bodies_four_constraints, sample_count);
	} else if (contacts_method == ContactsMethodRangeSpaceSparse) {
		duration = run_contacts_lagrangian_sparse_benchmark (model, &two_bodies_four_constraints, sample_count);
	} else if (contacts_method == ContactsMethodNullSpace) {
		duration = run_contacts_null_space (model, &two_bodies_four_constraints, sample_count);
	} else {
		duration = run_contacts_kokkevis_benchmark (model, &two_bodies_four_constraints, sample_count);
	}

	cout << "ConstraintSet: 2 Bodies 4 Constraints: "
		<< " duration = " << setw(10) << duration << "(s)"
		<< " (~" << setw(10) << duration / sample_count << "(s) per call)" << endl;


	// four_bodies_four
	if (contacts_method == ContactsMethodLagrangian) {
		duration = run_contacts_lagrangian_benchmark (model, &four_bodies_four_constraints, sample_count);
	} else if (contacts_method == ContactsMethodRangeSpaceSparse) {
		duration = run_contacts_lagrangian_sparse_benchmark (model, &four_bodies_four_constraints, sample_count);
	} else if (contacts_method == ContactsMethodNullSpace) {
		duration = run_contacts_null_space (model, &four_bodies_four_constraints, sample_count);
	} else {
		duration = run_contacts_kokkevis_benchmark (model, &four_bodies_four_constraints, sample_count);
	}

	cout << "ConstraintSet: 4 Bodies 4 Constraints: "
		<< " duration = " << setw(10) << duration << "(s)"
		<< " (~" << setw(10) << duration / sample_count << "(s) per call)" << endl;

	delete model;

	return duration;
}

void print_usage () {
#if defined (RBDL_BUILD_ADDON_LUAMODEL) || defined (RBDL_BUILD_ADDON_URDFREADER)
	cout << "Usage: benchmark [--count|-c <sample_count>] [--depth|-d <depth>] <model.lua>" << endl;
#else
	cout << "Usage: benchmark [--count|-c <sample_count>] [--depth|-d <depth>]" << endl;
#endif
	cout << "Simple benchmark tool for the Rigid Body Dynamics Library." << endl;
	cout << "  --count | -c <sample_count> : sets the number of sample states that should" << endl;
	cout << "                be calculated (default: 1000)" << endl;
	cout << "  --depth | -d <depth>        : sets maximum depth for the branched test model" << endl;
	cout << "                which is created increased from 1 to <depth> (default: 5)." << endl;
	cout << "  --no-fd                     : disables benchmarking of forward dynamics." << endl;
	cout << "  --no-fd-aba                 : disables benchmark for forwards dynamics using" << endl;
	cout << "                                the Articulated Body Algorithm" << endl;
	cout << "  --no-fd-lagrangian          : disables benchmark for forward dynamics via" << endl;
	cout << "                                solving the lagrangian equation." << endl;
	cout << "  --no-id-rnea                : disables benchmark for inverse dynamics using" << endl;
	cout << "                                the recursive newton euler algorithm." << endl;
	cout << "  --no-crba                   : disables benchmark for joint space inertia" << endl;
	cout << "                                matrix computation using the composite rigid." << endl;
	cout << "  --no-nle                    : disables benchmark for the nonlinear effects." << endl;
	cout << "                                body algorithm." << endl;
	cout << "  --only-contacts | -C        : only runs contact model benchmarks." << endl;
	cout << "  --help | -h                 : prints this help." << endl;
}

void disable_all_benchmarks () {
	benchmark_run_fd_aba = false;
	benchmark_run_fd_lagrangian = false;
	benchmark_run_id_rnea = false;
	benchmark_run_crba = false;
	benchmark_run_nle = false;
	benchmark_run_contacts = false;
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
		} else if (arg == "--no-fd" ) {
			benchmark_run_fd_aba = false;
			benchmark_run_fd_lagrangian = false;
		} else if (arg == "--no-fd-aba" ) {
			benchmark_run_fd_aba = false;
		} else if (arg == "--no-fd-lagrangian" ) {
			benchmark_run_fd_lagrangian = false;
		} else if (arg == "--no-id-rnea" ) {
			benchmark_run_id_rnea = false;
		} else if (arg == "--no-crba" ) {
			benchmark_run_crba = false;
		} else if (arg == "--no-nle" ) {
			benchmark_run_nle = false;
		} else if (arg == "--only-contacts" || arg == "-C") {
			disable_all_benchmarks();
			benchmark_run_contacts = true;
#if defined (RBDL_BUILD_ADDON_LUAMODEL) || defined (RBDL_BUILD_ADDON_URDFREADER)
		} else if (model_file == "") {
			model_file = arg;
#endif
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

	model = new Model();

	if (model_file != "") {
		if (model_file.substr (model_file.size() - 4, 4) == ".lua") {
#ifdef RBDL_BUILD_ADDON_LUAMODEL
			RigidBodyDynamics::Addons::LuaModelReadFromFile (model_file.c_str(), model);
#else
			cerr << "Could not load Lua model: LuaModel addon not enabled!" << endl;
			abort();
#endif
		}
		if (model_file.substr (model_file.size() - 5, 5) == ".urdf") {
#ifdef RBDL_BUILD_ADDON_URDFREADER
			RigidBodyDynamics::Addons::URDFReadFromFile(model_file.c_str(), model);
#else
			cerr << "Could not load URDF model: urdfreader addon not enabled!" << endl;
			abort();
#endif
		}

		if (benchmark_run_fd_aba) {
			cout << "= Forward Dynamics: ABA =" << endl;
			run_forward_dynamics_ABA_benchmark (model, benchmark_sample_count);
		}

		if (benchmark_run_fd_lagrangian) {
			cout << "= Forward Dynamics: Lagrangian (Piv. LU decomposition) =" << endl;
			run_forward_dynamics_lagrangian_benchmark (model, benchmark_sample_count);
		}

		if (benchmark_run_id_rnea) {
			cout << "= Inverse Dynamics: RNEA =" << endl;
			run_inverse_dynamics_RNEA_benchmark (model, benchmark_sample_count);
		}

		if (benchmark_run_crba) {
			cout << "= Joint Space Inertia Matrix: CRBA =" << endl;
			run_CRBA_benchmark (model, benchmark_sample_count);
		}

		if (benchmark_run_nle) {
			cout << "= Nonlinear effects  =" << endl;
			run_nle_benchmark (model, benchmark_sample_count);
		}

		delete model;

		return 0;
	}

	rbdl_print_version();
	cout << endl;

	if (benchmark_run_fd_aba) {
		cout << "= Forward Dynamics: ABA =" << endl;
		for (int depth = 1; depth <= benchmark_model_max_depth; depth++) {
			model = new Model();
			model->gravity = Vector3d (0., -9.81, 0.);

			generate_planar_tree (model, depth);

			run_forward_dynamics_ABA_benchmark (model, benchmark_sample_count);

			delete model;
		}
		cout << endl;
	}

	if (benchmark_run_fd_lagrangian) {
		cout << "= Forward Dynamics: Lagrangian (Piv. LU decomposition) =" << endl;
		for (int depth = 1; depth <= benchmark_model_max_depth; depth++) {
			model = new Model();
			model->gravity = Vector3d (0., -9.81, 0.);

			generate_planar_tree (model, depth);

			run_forward_dynamics_lagrangian_benchmark (model, benchmark_sample_count);

			delete model;
		}
		cout << endl;
	}

	if (benchmark_run_id_rnea) {
		cout << "= Inverse Dynamics: RNEA =" << endl;
		for (int depth = 1; depth <= benchmark_model_max_depth; depth++) {
			model = new Model();
			model->gravity = Vector3d (0., -9.81, 0.);

			generate_planar_tree (model, depth);

			run_inverse_dynamics_RNEA_benchmark (model, benchmark_sample_count);

			delete model;
		}
		cout << endl;
	}

	if (benchmark_run_crba) {
		cout << "= Joint Space Inertia Matrix: CRBA =" << endl;
		for (int depth = 1; depth <= benchmark_model_max_depth; depth++) {
			model = new Model();
			model->gravity = Vector3d (0., -9.81, 0.);

			generate_planar_tree (model, depth);

			run_CRBA_benchmark (model, benchmark_sample_count);

			delete model;
		}
		cout << endl;
	}

	if (benchmark_run_nle) {
		cout << "= Nonlinear Effects =" << endl;
		for (int depth = 1; depth <= benchmark_model_max_depth; depth++) {
			model = new Model();
			model->gravity = Vector3d (0., -9.81, 0.);

			generate_planar_tree (model, depth);

			run_nle_benchmark (model, benchmark_sample_count);

			delete model;
		}
		cout << endl;
	}

	if (benchmark_run_contacts) {
		cout << "= Contacts: ForwardDynamicsContactsLagrangian" << endl;
		contacts_benchmark (benchmark_sample_count, ContactsMethodLagrangian);

		cout << "= Contacts: ForwardDynamicsContactsRangeSpaceSparse" << endl;
		contacts_benchmark (benchmark_sample_count, ContactsMethodRangeSpaceSparse);

		cout << "= Contacts: ForwardDynamicsContactsNullSpace" << endl;
		contacts_benchmark (benchmark_sample_count, ContactsMethodNullSpace);

		cout << "= Contacts: ForwardDynamicsContactsKokkevis" << endl;
		contacts_benchmark (benchmark_sample_count, ContactsMethodKokkevis);
	}

	return 0;
}

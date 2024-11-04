/*
1) Based on time-varying A,B matrix from the previous time step 
and the LQR weights Q and R compute the stabilizing feedback gain
by solving a discrete algebraic ricatti equation (DARE).
2) Use these gains and the MPC solution from the previous time step
to compute the propagated control variance.
3) Use the propagated control variance and the contact plan this iteration
to compute the friction cone constraint tightening
4) Alternatively, return zero constraint tightening for nominal mpc and 
constant constraint tightening for heuristic MPC
*/

#include <vector>
#include <unordered_map>
#include <iostream>
#include <algorithm>
#include <numeric>
#include <assert.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/Cholesky>
#include <Eigen/LU>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/stl.h>

#include "compute_feedback_gain.h"

// Define constants
namespace constants{
	constexpr int full_state_dimension = 13;
	constexpr int reduced_state_dimension = 6;
	constexpr int num_friction_cone_surfaces = 4;
	constexpr int k3dim = 3;
	constexpr int num_robot_parameters = 16;
	constexpr int num_legs = 4;
	constexpr int force_dimension = 3;
constexpr int control_dimension = num_legs * force_dimension;
} //constants

class ChanceConstraints
{
public:

	/**
	 * @brief Construct a new ChanceConstraints object
	 * 
	 * @param dare_Q Eigen matrix of state weights for computing DARE solution for stabilizing feedback gain
	 * 
	 * @param dare_R Eigen matrix of control weights for computing DARE solution for stabilizing feedback gain
	 * 
	 * @param constraint_tightening_multiplier_map Unordered map from number of feet in contact 
	 * 																						 to the constraint tightening pre-multiplier

	 * @param sigma_theta Eigen matrix of tunable parameteric uncertainties, 
	 											inertias, mass and foot locations
	 *  
	 * @param sigma_w Eigen matrix of tunable additive uncertainties in 
	 * 								angular and linear velocities
	 * 
	 * @param mpc_horizon integer representing the length of the MPC horizon
	 * 
	 * @param mpc_timestep double representing the discretization in seconds for MPC timesteps
	 * 
	 * @param nominal_friction_coefficient double representing the coefficient of nominal friction
	 * 
	 * @param robot_inertias std vector representing the nominal robot inertias i.e. Ixx, Iyy, Izz
	 * 
	 * @param robot_mass double representing the nominal robot mass
	 *
	 * @param friction_cone_base_matrix Eigen matrix representing the surfaces of the friction cone 
	 * 
	 */ 
	ChanceConstraints(const Eigen::MatrixXd& dare_Q,
										const Eigen::MatrixXd& dare_R,
										const std::unordered_map<int,double>& constraint_tightening_multiplier_map,
										const Eigen::MatrixXd& sigma_theta,
										const Eigen::MatrixXd& sigma_w,
										const int mpc_horizon,
										const double mpc_timestep,
										const double nominal_friction_coefficient,
										const std::vector<double>& robot_inertias,
										const double robot_mass,
										const Eigen::MatrixXd& friction_cone_base_matrix);

	/**
	 * @brief Compute the adjustments to friction cone constraints based on the propagated control variance
	 * 				Account for the contact location, inertial, mass uncertainties and additive uncertainties in 
	 * 				robot dynamics. The constraint tightening for the current MPC solve is found using the values
	 * 				computed at the previous mpc solve. 
	 * 
	 * @param reduced_B Eigen Matrix representing the dynamics sub-B matrix used in MPC at the previous MPC solve
	 * 
	 * @param foot_positions_world_flattened std::vector of flattened foot positions in world frame at the start of the previous MPC solve
	 * 
	 * @param previous_mpc_solution std::vector of flattened forces computed at the previous MPC solve for each leg
	 * 															Contains zeros for legs not in contact
	 * 
	 * @param current_contact_plan std::vector flattened representing the contact plan per foot per timestep
	 * 												     at the current MPC solve
	 */
	std::vector<double> ChanceConstraintTightening(const Eigen::MatrixXd& reduced_B,
																								 const std::vector<double>& foot_positions_world_flattened,
																								 const std::vector<double>& previous_mpc_solution,
																								 const std::vector<bool>& current_contact_plan) const;
private:

	// Default constructor, not implemented
	ChanceConstraints() = delete;

	// Dynamics subspace A matrix
	const Eigen::MatrixXd reduced_A_;

	// State weights used in DARE solver
	const Eigen::MatrixXd dare_Q_;

	// Control weights used in DARE solver
	const Eigen::MatrixXd dare_R_;

	// Map from number of legs in contact 
	// to the constraint tightening pre-multiplier
	const std::unordered_map<int,double> constraint_tightening_multiplier_map_;

	// Covariance in robot parameters
	const Eigen::MatrixXd sigma_theta_;

	// Covariance in additive disturbance
	const Eigen::MatrixXd sigma_w_;

	// MPC Horizon
	const int mpc_horizon_;

	// MPC discretization time step
	const double mpc_timestep_;

	// Nominal friction coefficient
	const double friction_coefficient_;

	// Robot inertias
	const std::vector<double> robot_inertias_;

	// Extract out inertias 
	const double Ixx_, Iyy_, Izz_;

	// Robot mass
	const double robot_mass_;

	// Linearized friction cone constraint matrix. Replicate this into
	// the augmented constraint matrix for all feet which are in contact
	// We hardcode this to four faces of the friction cone as is common in practice
	Eigen::MatrixXd friction_cone_base_matrix_;

	// Number of constraints to be tightened
	const double num_constraints_per_foot_;

	/**
	 * @brief Helper function to hand compute the gradient
	 * 				of the dynamics wrt the parameters (inertia, mass and foot positions)
	 *  			We hand computed the gradients of the dynamics wrt the 16 parameters 
	 * 				and fill in the respective eigen matrix.
	 * 				This was to avoid having an external library as a dependency
	 * 				The results were sanity checked using casadi in python
	 * 
	 * @param previous_mpc_solution MPC output at the previous time step
	 * 
	 * @param control_start_idx Based on the current iteration, where to start extracting
	 * 													previous MPC solution from for computing parameter gradients
	 * 
	 * @param foot_positions_world Foot positions in world frame flattened
	 * 
	 * @returns gradients_mat Fill out this matrix with the required gradients
	 */
	Eigen::MatrixXd ComputeParameterGradients(const std::vector<double> previous_mpc_solution,
																						const int control_start_idx,
																						const std::vector<double>& foot_positions_world) const;
};

//////////////// Class Implementation ////////////////


ChanceConstraints::ChanceConstraints(const Eigen::MatrixXd& dare_Q,
										const Eigen::MatrixXd& dare_R,
										const std::unordered_map<int,double>& constraint_tightening_multiplier_map,
										const Eigen::MatrixXd& sigma_theta,
										const Eigen::MatrixXd& sigma_w,
										const int mpc_horizon,
										const double mpc_timestep,
										const double nominal_friction_coefficient,
										const std::vector<double>& robot_inertias,
										const double robot_mass,
										const Eigen::MatrixXd& friction_cone_base_matrix)
	:
	reduced_A_(Eigen::MatrixXd::Identity(constants::reduced_state_dimension, constants::reduced_state_dimension)),
	dare_Q_(dare_Q),
	dare_R_(dare_R),
	constraint_tightening_multiplier_map_(constraint_tightening_multiplier_map),
	sigma_theta_(sigma_theta),
	sigma_w_(sigma_w),
	mpc_horizon_(mpc_horizon),
	mpc_timestep_(mpc_timestep),
	friction_coefficient_(nominal_friction_coefficient),
	robot_inertias_(robot_inertias), Ixx_(robot_inertias[0]), Iyy_(robot_inertias[1]),	Izz_(robot_inertias[2]),
	robot_mass_(robot_mass),				
	friction_cone_base_matrix_(friction_cone_base_matrix),
	num_constraints_per_foot_(constants::num_friction_cone_surfaces + 1)
	{
		// Sanity checks
		assert(dare_Q_.rows() == dare_Q_.cols() &&  dare_Q_.rows() == constants::reduced_state_dimension);

		assert(dare_R_.rows() == dare_R_.cols() && dare_R_.rows() == constants::control_dimension);
		
		assert(sigma_w_.rows() == sigma_w_.cols() && sigma_w_.rows() == constants::reduced_state_dimension);

		assert(sigma_theta_.rows() == sigma_theta_.cols() && sigma_theta_.rows() == constants::num_robot_parameters);

		assert(constraint_tightening_multiplier_map_.size() == constants::num_legs + 1);

		assert(robot_inertias_.size() == constants::k3dim);

		// Four sides to the friction cone and one more for unilateral contact forces
		assert(friction_cone_base_matrix_.rows() == num_constraints_per_foot_
				   && friction_cone_base_matrix_.cols() == constants::force_dimension);

	}


Eigen::MatrixXd ChanceConstraints::ComputeParameterGradients(const std::vector<double> previous_mpc_solution,
																														 const int control_start_idx,
																														 const std::vector<double>& foot_positions_world) const
{
	// Fill out this matrix with the required gradients (size 6*16)
	// We only fill out the non-zero elements of this matrix
	// Instead of using an auto-diff tool like casadi, we computed the gradients by hand
	// to keep the implementation simple and not have dependence on any additional libraries
	// Think of this as d/dθ (Ax + B(θ)u) where, θ is as defined as:
	// Ix, Iy, Iz, m, (r_1^x,r_1^y,r_1^z),(r_2^x,r_2^y,r_2^z),(r_3^x,r_3^y,r_3^z),(r_4^x,r_4^y,r_4^z)

	// Extract the control action
	const std::vector<double>::const_iterator mpc_solution_begin_iter = previous_mpc_solution.begin();

	const std::vector<double> control_action (mpc_solution_begin_iter + control_start_idx,mpc_solution_begin_iter + control_start_idx + constants::control_dimension);

	// Populate this matrix with gradients
	Eigen::MatrixXd gradients = Eigen::MatrixXd::Zero(constants::reduced_state_dimension, constants::num_robot_parameters);

	// Begin by extracting out the foot positions and the previous MPC solution
	const std::vector<double> r_x = {foot_positions_world[0],foot_positions_world[3],foot_positions_world[6],foot_positions_world[9] };
	const std::vector<double> r_y = {foot_positions_world[1],foot_positions_world[4],foot_positions_world[7],foot_positions_world[10] };
	const std::vector<double> r_z = {foot_positions_world[2],foot_positions_world[5],foot_positions_world[8],foot_positions_world[11] };

	const std::vector<double> f_x = {control_action[0],control_action[3],control_action[6],control_action[9]};
	const std::vector<double> f_y = {control_action[1],control_action[4],control_action[7],control_action[10]};
	const std::vector<double> f_z = {control_action[2],control_action[5],control_action[8],control_action[11]};

	// Initialize derivatives wrt Ixx,Iyy and Izz to zero 
	double element_ixx = 0.0; 
	double element_iyy = 0.0;
	double element_izz = 0.0;

	// Start index of sub matrix used to compute derivative wrt contact location
	const int contact_submatrix_row_start = 0;
	int contact_submatrix_column_start = 4;

	// Create non-zero element sub-matrices
	for (int leg_idx=0;leg_idx<constants::num_legs;++leg_idx)
	{
		// Accumulate the terms related to inertia derivatives
		element_ixx += (-r_z[leg_idx]*f_y[leg_idx] + r_y[leg_idx]*f_z[leg_idx]) * (-mpc_timestep_ / (Ixx_*Ixx_) );
		element_iyy += (-r_x[leg_idx]*f_z[leg_idx] + r_z[leg_idx]*f_x[leg_idx]) * (-mpc_timestep_ / (Iyy_*Iyy_) );
		element_izz += (-r_y[leg_idx]*f_x[leg_idx] + r_x[leg_idx]*f_y[leg_idx]) * (-mpc_timestep_ / (Izz_*Izz_) );

		// Plug in the sub matrix corresponding to the derivatives with contact locations
		Eigen::Matrix3d contact_submatrix = Eigen::Matrix3d::Zero();
		contact_submatrix << 0.0, (mpc_timestep_/Ixx_)*f_z[leg_idx], (-mpc_timestep_/Ixx_)*f_y[leg_idx],
													(-mpc_timestep_/Iyy_)*f_z[leg_idx], 0.0, (mpc_timestep_/Iyy_)*f_x[leg_idx],
													(mpc_timestep_/Izz_)*f_y[leg_idx], (-mpc_timestep_/Izz_)*f_x[leg_idx],0.0;

		gradients.block(contact_submatrix_row_start,contact_submatrix_column_start,  3,3) = contact_submatrix;
		contact_submatrix_column_start += 3;
	}

	// Plug in the sub matrix corresponding to the derivatives with inertias
	Eigen::Matrix3d inertia_submatrix = Eigen::Matrix3d::Zero();
	inertia_submatrix.diagonal() << element_ixx,element_iyy,element_izz;

	gradients.block(0,0,  3,3) = inertia_submatrix;

	// Plug in the sub matrix corresponding to the derivatives with mass
	Eigen::Vector3d mass_submatrix;

	mass_submatrix<<std::accumulate(f_x.begin(),f_x.end(),0.0) * (-mpc_timestep_ / (robot_mass_*robot_mass_) ),
									std::accumulate(f_y.begin(),f_y.end(),0.0) * (-mpc_timestep_ / (robot_mass_*robot_mass_) ),
									std::accumulate(f_z.begin(),f_z.end(),0.0) * (-mpc_timestep_ / (robot_mass_*robot_mass_) );

	gradients.block(3,3,  3,1) = mass_submatrix;

	return gradients;
}


std::vector<double> ChanceConstraints::ChanceConstraintTightening(const Eigen::MatrixXd& reduced_B,
																																	const std::vector<double>& foot_positions_world_flattened,
																																	const std::vector<double>& previous_mpc_solution,
																																	const std::vector<bool>& current_contact_plan) const
{
	// Flattened constraint tightening output
	std::vector<double> constraint_tightening_flattened;
	constraint_tightening_flattened.reserve(constants::num_legs * num_constraints_per_foot_ * mpc_horizon_);

	// LQR feedback gain based on A,B matrices at the previous MPC solve
	Eigen::MatrixXd K = feedback_gain::ComputeFeedbackGain(reduced_A_,reduced_B, dare_Q_, dare_R_);

	// Closed loop dynamics based on the computed gain
	Eigen::MatrixXd A_Cl = reduced_A_ - reduced_B * K;

	// Instantiate the state covariance matrix to all zeros
	// Equivalently, state covariance at first time step is zero
	Eigen::MatrixXd sigma_x = Eigen::MatrixXd::Zero(constants::reduced_state_dimension,constants::reduced_state_dimension);

	// Instantiate control variance to all zeros
	// Propagated control variance is of dimension control_dimension* control_dimension
	// However to minimize conservativeness of the controller, only the feet in contact
	// should have their respective constraint tightening set to non zero values
	// We do this by zeroing out the effects of these feet not in contact
	Eigen::MatrixXd sigma_u = Eigen::MatrixXd::Zero(constants::control_dimension, constants::control_dimension);

	// Create an augmented linearized friction cone constraints matrix with the feet not in contact set to zero
	// Subsequently loop through all the legs, check if they are in contact as per the contact schedule
	// If so, add a friction cone base matrix to the augmented matrix
	// We also include the unilateral contact force constraint in the augmented_constraint_matrix
	Eigen::MatrixXd augmented_constraint_matrix = 
			Eigen::MatrixXd::Zero(constants::num_legs * num_constraints_per_foot_, constants::control_dimension);

	// Matrix form of the constraint tightening
	// The true values lie on the diagonal
	Eigen::MatrixXd constraint_tightening_mat =
			Eigen::MatrixXd::Zero( constants::num_legs * num_constraints_per_foot_, constants::num_legs * num_constraints_per_foot_ );

	// Loop through all the mpc timesteps and compute the friction cone constraint tightening
	for (size_t mpc_iter = 0; mpc_iter<mpc_horizon_; ++mpc_iter)
	{
		// Propagate control variance
		sigma_u = K * sigma_x * K.transpose();

		// Reset augmented constraint matrix to all zeros
		augmented_constraint_matrix.setZero();

		// If a foot is in contact assign constraint tightening to it
		int num_feet_in_contact = 0;

		int constraint_matrix_row_idx = 0;

		int constraint_matrix_col_idx = 0;

		for (size_t leg_no=0; leg_no<constants::num_legs;++leg_no)
		{
			bool in_contact = current_contact_plan[mpc_iter*constants::num_legs + leg_no];

			if (in_contact == true)
			{
				// Update the number of feet in contact
				num_feet_in_contact += 1;

				// Add the friction cone constraint to the corresponding augmented constraint submatrix
				augmented_constraint_matrix.block(constraint_matrix_row_idx,constraint_matrix_col_idx,num_constraints_per_foot_,constants::force_dimension) = 
					friction_cone_base_matrix_;
			}

			// Progress the submatrix			
			constraint_matrix_row_idx += num_constraints_per_foot_;
			constraint_matrix_col_idx += constants::force_dimension;
		}

		// Pre multiplier for the constraint adjustment based on number of feet in contact
		const double constraint_tightening_multiplier = constraint_tightening_multiplier_map_.at(num_feet_in_contact);

		// Finally compute the constraint tightening factors
		// The factors are in the diagonal of the matrix
		constraint_tightening_mat = augmented_constraint_matrix * sigma_u * augmented_constraint_matrix.transpose();
		
		// Extract the diagonal elements of the matrix into a temporary eigen vector
		Eigen::VectorXd tmp_diagonal = constraint_tightening_multiplier * constraint_tightening_mat.diagonal().array().sqrt();

		// Finally add the constraint adjustments to the flattened array
		std::copy(tmp_diagonal.data(), tmp_diagonal.data() + tmp_diagonal.size(), std::back_inserter(constraint_tightening_flattened));

		/////// Setup for next iteration ///////

		// Extract MPC solution for the current timestep
		const int control_start_idx = mpc_iter * constants::control_dimension;

		// Compute grdient of dynamics function wrt parameters inertia, mass and foot locations
		const Eigen::MatrixXd C = ComputeParameterGradients(previous_mpc_solution, control_start_idx,foot_positions_world_flattened);

		// Finally update state covariance update for next time step
		sigma_x = A_Cl * sigma_x * A_Cl.transpose() 
						+ C * sigma_theta_ * C.transpose() 
						+ sigma_w_;
	}

	return constraint_tightening_flattened;
	
}

/////PyBind Setup/////

namespace py = pybind11;

PYBIND11_MODULE(chance_constraints_cpp, m) {
	// Module Purpose
	m.doc() = "Chance Constraints implementation in C++";

	// ChanceConstraints class setup
	py::class_<ChanceConstraints>(m, "ChanceConstraints")
		// Class constructor
		.def(py::init<
									const Eigen::MatrixXd&,
									const Eigen::MatrixXd&,
									const std::unordered_map<int,double>&,
									const Eigen::MatrixXd&,
									const Eigen::MatrixXd&,
									const int,
									const double,
									const double,
									const std::vector<double>&,
									const double,
									const Eigen::MatrixXd&>())

		// Compute constraint tightening
		.def("chance_constraint_tightening",&ChanceConstraints::ChanceConstraintTightening);
}
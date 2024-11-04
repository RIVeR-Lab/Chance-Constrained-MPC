/* 
Based on a discretized linear dynamics x_(i+1) = A_i*x_(i) + B_i*u_(i), 
state feedback gains Q and control gains R, solve the resultant 
Discrete Algebraic Ricatti Equation (DARE) and compute the LQR feedback
gain K(i). Derived from Drake (https://drake.mit.edu/) implementation of 
discrete time linear quadratic regulator.
*/

#include <vector>
#include <iostream>
#include <algorithm>
#include <numeric>
#include <assert.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/Cholesky>
#include <Eigen/LU>

namespace feedback_gain{

bool IsStabilizable(const Eigen::Ref<const Eigen::MatrixXd>& A, const Eigen::Ref<const Eigen::MatrixXd>& B)
{
  // (A, B) is stabilizable if [(λI - A) B] is full rank for all unstable eigenvalues

  // Instantiate solver for computing eigen values of A
  Eigen::EigenSolver<Eigen::MatrixXd> es(A,false);
  assert(es.info() == Eigen::Success);


  for (int i = 0; i < es.eigenvalues().size(); ++i)
  {
    // Check for unstable eigen values i.e. norm outside the unit circle
    bool stable_mode = false;
    stable_mode = std::norm(es.eigenvalues()(i)) < 1;

    // For unstable eigen values do the full rank check
    if (stable_mode == false)
    {
      // The matrix [(λI - A) B]
      Eigen::MatrixXcd mat(A.rows(), A.cols() + B.cols());
      mat.leftCols(A.cols()) = es.eigenvalues()(i) * Eigen::MatrixXd::Identity(A.rows(), A.cols()) - A;
      mat.rightCols(B.cols()) = B;

      // Computing the QR factorization makes rank computation faster
      Eigen::ColPivHouseholderQR<Eigen::MatrixXcd> qr(mat);
      assert(qr.info() == Eigen::Success);

      // Check the rank condition
      if (qr.rank() < A.rows()) 
      {
        return false;
      }
    }
  }

  return true;

}


bool IsDetectable(const Eigen::Ref<const Eigen::MatrixXd>& A, const Eigen::Ref<const Eigen::MatrixXd>& C)
{
  // The system (C,A) is detectable iff (A',C') is stabilizable
  return feedback_gain::IsStabilizable(A.transpose(),C.transpose());
}


Eigen::MatrixXd ComputeFeedbackGain(const Eigen::Ref<const Eigen::MatrixXd>& A,
                                    const Eigen::Ref<const Eigen::MatrixXd>& B,
                                    const Eigen::Ref<const Eigen::MatrixXd>& Q,
                                    const Eigen::Ref<const Eigen::MatrixXd>& R)
{
  /////////// Begin by solving the Discrete Algebraic Ricatti Equation ///////////
  
  // LDLᵀ decomposition of state weight matrix Q
  auto Q_ldlt = Q.ldlt();
  assert(Q_ldlt.info() == Eigen::Success);

  // LLᵀ decomposition of the control weights matrix R & corresponding sanity check
  Eigen::LLT<Eigen::MatrixXd> R_llt{R};
  assert(R_llt.info() == Eigen::Success);

  // Ensure the pair (A,B) is stabilizable
  bool stabilizable_check = feedback_gain::IsStabilizable(A,B);
  assert(stabilizable_check == true);

  // Ensure (A, C) pair where Q = CᵀC is detectable
  // Q = CᵀC = PᵀLDLᵀP
  // Cᵀ = PᵀL√(D)
  // C = (PᵀL√(D))ᵀ

  Eigen::MatrixXd C = (Q_ldlt.transpositionsP().transpose() *
                       // NOLINTNEXTLINE(whitespace/braces)
                       Eigen::MatrixXd{Q_ldlt.matrixL()} *
                       Q_ldlt.vectorD().cwiseSqrt().asDiagonal()).transpose();

  bool detectable_check = feedback_gain::IsDetectable(A,C);
  assert(detectable_check == true);

  // Implements the SDA algorithm on page 5 of [1].
  //
  // [1] E. K.-W. Chu, H.-Y. Fan, W.-W. Lin & C.-S. Wang "Structure-Preserving
  //     Algorithms for Periodic Discrete-Time Algebraic Riccati Equations",
  //     International Journal of Control, 77:8, 767-788, 2004.
  //     DOI: 10.1080/00207170410001714988

  // A₀ = A
  Eigen::MatrixXd A_k = A;

  // G₀ = BR⁻¹Bᵀ
  // 
  // See equation (4) of [1].
  Eigen::MatrixXd G_k = B * R_llt.solve(B.transpose());

  // H₀ = Q
  // 
  // See equation (4) of [1].
  Eigen::MatrixXd H_k;
  Eigen::MatrixXd H_k1 = Q;

  do {
    H_k = H_k1;

    // W = I + GₖHₖ
    Eigen::MatrixXd W =
        Eigen::MatrixXd::Identity(H_k.rows(), H_k.cols()) + G_k * H_k;

    auto W_solver = W.lu();

    // Solve WV₁ = Aₖ for V₁
    Eigen::MatrixXd V_1 = W_solver.solve(A_k);

    // Solve V₂Wᵀ = Gₖ for V₂
    //
    // We want to put V₂Wᵀ = Gₖ into Ax = b form so we can solve it more
    // efficiently.
    //
    // V₂Wᵀ = Gₖ
    // (V₂Wᵀ)ᵀ = Gₖᵀ
    // WV₂ᵀ = Gₖᵀ
    //
    // The solution of Ax = b can be found via x = A.solve(b).
    //
    // V₂ᵀ = W.solve(Gₖᵀ)
    // V₂ = W.solve(Gₖᵀ)ᵀ
    Eigen::MatrixXd V_2 = W_solver.solve(G_k.transpose()).transpose();

    // Gₖ₊₁ = Gₖ + AₖV₂Aₖᵀ
    G_k += A_k * V_2 * A_k.transpose();

    // Hₖ₊₁ = Hₖ + V₁ᵀHₖAₖ
    H_k1 = H_k + V_1.transpose() * H_k * A_k;

    // Aₖ₊₁ = AₖV₁
    A_k *= V_1;

    // while |Hₖ₊₁ − Hₖ| > ε |Hₖ₊₁|
  } while ((H_k1 - H_k).norm() > 1e-10 * H_k1.norm());

  // Assign solution of the DARE to new variable S
  // to keep convention as in underactuated note
  Eigen::MatrixXd S = H_k1;

  // Compute feedback gain as in
  // Section 8.3.1 of the underactuated notes: 
  // https://underactuated.csail.mit.edu/lqr.html#section3

  Eigen::MatrixXd tmp = B.transpose() * S * B + R;
  Eigen::MatrixXd K = tmp.llt().solve(B.transpose() * S * A);

  return K;
  
}

} // namespace feedback_gain
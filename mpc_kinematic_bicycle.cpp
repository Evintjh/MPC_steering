#include <cmath>

#include <acado_code_generation.hpp>
#include <acado_optimal_control.hpp>

constexpr bool CONTROL_VEL = false;  // Whether to control the velocity of the vehicle on top of the steering angle

int main() {
  USING_NAMESPACE_ACADO

  // Wheelbase of the vehicle
  const double vehicle_wheelbase = 3.1;  // From custom_parameters_PSA8374.yaml

  // Optimal Control problem
  double tStart = 0.0;  // Start of the MPC
  double tEnd = 2.0;    // End of the MPC
  int N = 20;           // Time horizon of the MPC
  OCP ocp(tStart, tEnd, N);

  if (CONTROL_VEL) {
    // Variables
    DifferentialState x;
    DifferentialState y;
    DifferentialState psi;
    Control vR;
    Control delta;

    OnlineData xR, yR, psiR;
    ocp.setNOD(3);

    IntermediateState lat_acc = vR * vR * tan(delta) / vehicle_wheelbase;
    IntermediateState eX = (x - xR) * cos(psiR) + (y - yR) * sin(psiR);
    IntermediateState eY = -(x - xR) * sin(psiR) + (y - yR) * cos(psiR);
    IntermediateState ePsi = psi - psiR;

    // Dynamics
    DifferentialEquation f;
    f << dot(x) == vR * cos(psi);
    f << dot(y) == vR * sin(psi);
    f << dot(psi) == vR * tan(delta) / vehicle_wheelbase;

    // Reference functions
    Function h, hN;
    h << eX << eY << ePsi << vR << delta << lat_acc;
    hN << eX << eY << ePsi;

    // Weighting matrices
    DMatrix W = eye<double>(h.getDim());
    W(0, 0) = 1.0;
    W(1, 1) = 1.0;
    W(5, 5) = 1.0;
    DMatrix WN = eye<double>(hN.getDim());
    WN *= 2.0;

    // Cost
    ocp.minimizeLSQ(W, h);
    ocp.minimizeLSQEndTerm(WN, hN);

    // Constraints
    ocp.subjectTo(f);
    ocp.subjectTo(-5.56 <= vR <= 5.56);     // Up to 20 km/h
    ocp.subjectTo(-0.65 <= delta <= 0.65);  // Steering angle for Terberg APMs
    // ocp.subjectTo(-0.2 <= eX <= 0.2);
    // ocp.subjectTo(-0.4 <= eY <= 0.4); // this is  too small; consider x0 not on path
    ocp.subjectTo(-M_PI / 4 <= ePsi <= M_PI / 4);
    ocp.subjectTo(
        -pow(36 / 5.56, 2) <= lat_acc <=
        pow(36 / 5.56, 2));  // Adapted from https://www.fhwa.dot.gov/publications/research/safety/08019/08019.pdf p.58

  } else {
    // Variables
    DifferentialState x, y, psi;
    Control delta;

    OnlineData xR, yR, psiR, vR;
    ocp.setNOD(4);

    IntermediateState lat_acc = vR * vR * tan(delta) / vehicle_wheelbase;
    IntermediateState eX = (x - xR) * cos(psiR) + (y - yR) * sin(psiR);
    IntermediateState eY = -(x - xR) * sin(psiR) + (y - yR) * cos(psiR);
    IntermediateState ePsi = psi - psiR;

    // Dynamics
    DifferentialEquation f;
    f << dot(x) == vR * cos(psi);
    f << dot(y) == vR * sin(psi);
    f << dot(psi) == vR * tan(delta) / vehicle_wheelbase;

    // Reference functions
    Function h, hN;
    h << eX << eY << ePsi << delta << lat_acc;
    hN << eX << eY << ePsi;

    // Weighting matrices
    DMatrix W = eye<double>(h.getDim());
    W(0, 0) = 2.0;
    W(1, 1) = 2.0;
    W(2, 2) = 2.0;
    W(3, 3) = 3.0;
    W(4, 4) = 1.0;
    DMatrix WN = eye<double>(hN.getDim());
    WN(0, 0) = 2.0;
    WN(1, 1) = 2.0;
    WN(2, 2) = 2.0;
    WN *= 3.0;

    // Cost
    ocp.minimizeLSQ(W, h);
    ocp.minimizeLSQEndTerm(WN, hN);

    // Constraints
    ocp.subjectTo(f);
    ocp.subjectTo(-0.65 <= delta <= 0.65);  // Steering angle for Terberg APMs
    // ocp.subjectTo(-2.0 <= eX <= 2.0);
    // ocp.subjectTo(-2.0 <= eY <= 2.0);
    ocp.subjectTo(-M_PI / 4 <= ePsi <= M_PI / 4);
    ocp.subjectTo(
        -pow(36 / 5.56, 2) <= lat_acc <=
        pow(36 / 5.56, 2));  // Adapted from https://www.fhwa.dot.gov/publications/research/safety/08019/08019.pdf p.58
  }

  // Export the code
  OCPexport mpc(ocp);

  mpc.set(HESSIAN_APPROXIMATION, GAUSS_NEWTON);
  mpc.set(DISCRETIZATION_TYPE, SINGLE_SHOOTING);
  mpc.set(INTEGRATOR_TYPE, INT_RK4);
  mpc.set(NUM_INTEGRATOR_STEPS, 30);

  mpc.set(QP_SOLVER, QP_QPOASES);
  // mpc.set( HOTSTART_QP,                 YES             );
  // mpc.set( LEVENBERG_MARQUARDT,         1.0e-4          );
  mpc.set(GENERATE_TEST_FILE, YES);
  mpc.set(GENERATE_MAKE_FILE, NO);
  mpc.set(GENERATE_MATLAB_INTERFACE, NO);
  mpc.set(GENERATE_SIMULINK_INTERFACE, NO);
  // 	mpc.set( USE_SINGLE_PRECISION,        YES             );

  if (mpc.exportCode("navigation/mpc_steering/lib/generated") != SUCCESSFUL_RETURN) {
    exit(EXIT_FAILURE);
  }

  mpc.printDimensionsQP();

  return EXIT_SUCCESS;
}

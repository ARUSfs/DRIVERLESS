#include <acado_code_generation.hpp>

USING_NAMESPACE_ACADO

int main(int argc, char * const argv[ ])
{
	//
	// Variables
	//

	DifferentialState	s, mu, phi, vx, D, delta;
	Control				Dp, deltap;

	IntermediateState 	coef, delta_corrected;
	IntermediateState	sp, ax, r, vy;
	IntermediateState	T;

	OnlineData k;

	const double		m = 300.0;
	const double		Cm = 230*45/11/0.202;
	const double 		Cr = 0;
	const double 		Cd = 0.5;
	const double 		WB = 1.535;
	const double 		l_R = 0.5*WB;
	const double 		pi = 3.14;

	const double p1 = 1449;
	const double q1 = 146.8;
	const double q2 = -2438;
	const double q3 = -0.5052;
	const double q4 = 1457;

	coef = ((p1) / (pow(k,4) + q1*pow(k,3) + q2*pow(k,2) + q3*k + q4));
	delta_corrected = delta / coef;

	r = tan(delta_corrected)*vx/WB;
	vy = r*l_R;
	sp = (vx*cos(phi) - vy*sin(phi))/(1 - mu*k);
	ax = (Cm*D - Cr - Cd*vx*vx)/m;
	T = 1/sp;

	//
	// Differential algebraic equation
	//

	DifferentialEquation f;

	f << dot( s ) == sp;
	f << dot( mu ) == vx*sin(phi) + vy*cos(phi);
	f << dot( phi ) == r - sp*k;
	f << dot( vx ) == ax;
	f << dot( D ) == Dp;
	f << dot( delta ) == deltap;
	
	//
	// Weighting matrices and reference functions
	//

	Function rf;
	Function rfN;

	rf << mu << phi << deltap << Dp << T;
	rfN << T << mu << phi;

	DMatrix W = eye<double>( rf.getDim() );
	W(0,0) = 50; // Era 50 en la version estable
	W(1,1) = 1;
	W(2,2) = 100; // Era 100 en la version estable
	W(3,3) = 1;
	W(4,4) = 100; // Era 100 en la version estable


	DMatrix WN = eye<double>( rfN.getDim() );
	WN(0, 0) = 100;
	WN(1, 1) = 3;
	WN(2, 2) = 1;
	
	//
	// Optimal Control Problem
	//

	const int N  = 50;
	const int Ni = 4;
	const double Ts = 0.1;

	OCP ocp(0, N * Ts, N);

	ocp.subjectTo( f );

	ocp.minimizeLSQ(W, rf);
	ocp.minimizeLSQEndTerm(WN, rfN);

	// Box restrictions
	ocp.subjectTo(0.0 <= vx <= 20.0);
	ocp.subjectTo(-0.6 <= mu <= 0.6);
	ocp.subjectTo(-pi/4 <= delta <= pi/4);
	ocp.subjectTo(-1.0 <= D <= 1.0);
	ocp.subjectTo(-30.0 <= deltap <= 30.0);
	ocp.subjectTo(-300 <= Dp <= 300);
	ocp.subjectTo(0.0 <= sp);
	ocp.subjectTo(-15<= vx*vx*k <= 15);	
	ocp.subjectTo( AT_END, 0.0 <= vx <= 10.0);


	//
	// Export the code:
	//
	OCPexport mpc( ocp );

	mpc.set(HESSIAN_APPROXIMATION, GAUSS_NEWTON);
	mpc.set(DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);

	mpc.set(INTEGRATOR_TYPE, INT_RK4);
	mpc.set(NUM_INTEGRATOR_STEPS, N * Ni);

//	mpc.set(SPARSE_QP_SOLUTION, FULL_CONDENSING);
//	mpc.set(SPARSE_QP_SOLUTION, CONDENSING);
	mpc.set(QP_SOLVER, QP_QPOASES);
//	mpc.set(MAX_NUM_QP_ITERATIONS, 20);
	mpc.set(HOTSTART_QP, YES);

//	mpc.set(SPARSE_QP_SOLUTION, SPARSE_SOLVER);
//	mpc.set(QP_SOLVER, QP_FORCES);

//	mpc.set(LEVENBERG_MARQUARDT, 1.0e-10);

	mpc.set(GENERATE_TEST_FILE, YES);
	mpc.set(GENERATE_MAKE_FILE, YES);
	mpc.set(GENERATE_MATLAB_INTERFACE, NO);

//	mpc.set(USE_SINGLE_PRECISION, YES);
//	mpc.set(CG_USE_OPENMP, YES);

	if (mpc.exportCode( "prueba_kin_export" ) != SUCCESSFUL_RETURN)
		exit( EXIT_FAILURE );

	mpc.printDimensionsQP( );

	return EXIT_SUCCESS;
}


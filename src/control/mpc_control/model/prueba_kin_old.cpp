/*
 *    This file is part of ACADO Toolkit.
 *
 *    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
 *    Copyright (C) 2008-2014 by Boris Houska, Hans Joachim Ferreau,
 *    Milan Vukov, Rien Quirynen, KU Leuven.
 *    Developed within the Optimization in Engineering Center (OPTEC)
 *    under supervision of Moritz Diehl. All rights reserved.
 *
 *    ACADO Toolkit is free software; you can redistribute it and/or
 *    modify it under the terms of the GNU Lesser General Public
 *    License as published by the Free Software Foundation; either
 *    version 3 of the License, or (at your option) any later version.
 *
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *    Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with ACADO Toolkit; if not, write to the Free Software
 *    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include <acado_code_generation.hpp>

USING_NAMESPACE_ACADO

int main(int argc, char * const argv[ ])
{
	//
	// Variables
	//

	DifferentialState	s, mu, phi, vx, vy, r, D, delta;
	Control				Dp, deltap;

	IntermediateState	sp, ax;
	IntermediateState	T;

	OnlineData k;

	const double		m = 300.0;
	const double		Cm=230*45/11;
	const double 		Cr=0;
	const double 		Cd=0.5;
	const double 		WB=1.535;
	const double 		l_R=0.7;
	const double 		pi=3.14;


	sp = (vx*cos(phi) - vy*sin(phi))/(1 - mu*k);
	ax= (Cm*D + Cr + Cd*vx*vx)/m;
	T=1/sp;

	//
	// Differential algebraic equation
	//

	DifferentialEquation f;

	f << dot( s ) == sp;
	f << dot( mu ) == vx*sin(phi) + vy*cos(phi);
	f << dot( phi ) == r - sp*k;
	f << dot( vx ) == ax;
	f << dot( vy ) == (deltap*vx + delta*ax)*l_R/WB;
	f << dot( r ) == (deltap*vx + delta*ax)*WB;
	f << dot( D ) == Dp;
	f << dot( delta ) == deltap;
	
	//
	// Weighting matrices and reference functions
	//

	Function rf;
	Function rfN;

	rf << mu << phi << deltap << Dp;
	rfN << T << mu << phi;

	DMatrix W = eye<double>( rf.getDim() );
	W(0,0) = 5;
	W(1,1) = 1;
	W(2,2) = 100;
	W(3,3) = 1;


	DMatrix WN = eye<double>( rfN.getDim() );
	WN(0, 0) = 100;
	WN(1, 1) = 3;
	WN(2, 2) = 1;
	
	//
	// Optimal Control Problem
	//

	const int N  = 50;
	const int Ni = 4;
	const double Ts = 0.05;

	OCP ocp(0, N * Ts, N);

	ocp.subjectTo( f );

	ocp.minimizeLSQ(W, rf);
	ocp.minimizeLSQEndTerm(WN, rfN);

	// Box restrictions
	ocp.subjectTo(0.0 <= vx <= 10.0);
	ocp.subjectTo(-1.0 <= mu <= 1.0);
	ocp.subjectTo(-pi/5 <= delta <= pi/5);
	ocp.subjectTo(-1.0 <= D <= 1.0);
	ocp.subjectTo(-30.0 <= deltap <= 30.0);
	ocp.subjectTo(-300 <= Dp <= 300);
	ocp.subjectTo(-6<= vx*vx*k <= 6);	


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

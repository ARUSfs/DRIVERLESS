
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <string>
#include <iomanip>
#include <cstring>
#include <cmath>
#include <cstdlib>

using namespace std;
// #include "acado_common.h"
// #include "acado_auxiliary_functions.h"

#include <stdio.h>

#include "ros/ros.h"
#include <std_msgs/String.h>
#include <common_msgs/Controls.h>
#include <common_msgs/Trajectory.h>
#include <geometry_msgs/Point.h>
#include <common_msgs/MPCCurrentState.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud2.h>
#include "ros/console.h"
#include "utrilla_mpc/utrilla_wrapper.h"

using namespace utrilla_mpc;

/* Some convenient definitions. */
#define NX          ACADO_NX  /* Number of differential state variables.  */
#define NXA         ACADO_NXA /* Number of algebraic variables. */
#define NU          ACADO_NU  /* Number of control inputs. */
#define NOD         ACADO_NOD  /* Number of online data values. */

#define NY          ACADO_NY  /* Number of measurements/references on nodes 0..N - 1. */
#define NYN         ACADO_NYN /* Number of measurements/references on node N. */

#define N           ACADO_N   /* Number of intervals in the horizon. */

#define NUM_STEPS   1       /* Number of real-time iterations. */
#define VERBOSE     1         /* Show iterations: 1, silent: 0.  */


double prediccion[NX*(N+1)];

double u_prediccion[NU*N];

bool acado_inicializado = false;
bool acado_preparado = false;
bool primera_iter=true;

bool track_received = false;

double i_mu;

vector< double > k;
vector< double > s;


double interpolate( vector<double> &xData, vector<double> &yData, double x, bool extrapolate )
{
   int size = xData.size();

   int i = 0;                                                                  // find left end of interval for interpolation
   if ( x >= xData[size - 2] )                                                 // special case: beyond right end
   {
      i = size - 2;
   }
   else
   {
      while ( x > xData[i+1] ) i++;
   }
   double xL = xData[i], yL = yData[i], xR = xData[i+1], yR = yData[i+1];      // points on either side (unless beyond ends)
   if ( !extrapolate )                                                         // if beyond ends of array and not extrapolating
   {
      if ( x < xL ) yR = yL;
      if ( x > xR ) yL = yR;
   }

   double dydx = ( yR - yL ) / ( xR - xL );                                    // gradient

   return yL + dydx * ( x - xL );                                              // linear interpolation
}


/* A template for testing of the solver. */
void timerCallback(const ros::TimerEvent& event, ros::Publisher& pub, ros::ServiceClient& client){
	
	// cout << track_received << endl;
	if(track_received){
		// Create a request message
		common_msgs::MPCCurrentState srv;
		//req.field_name = value; // Populate the request fields as needed

		// Call the service
		if (client.call(srv)) {
			// Handle the response
			printf("Service call successful");
			cout << " " << srv.response.si << " " << srv.response.dist << " " << srv.response.phi << " " << srv.response.vx << " " << srv.response.delta << " " << srv.response.dc << endl;
		} else {
			printf("Failed to call service");
		}

		/* Some temporary variables. */
		int i;

		// Reset all solver memory
		memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
		memset(&acadoVariables, 0, sizeof( acadoVariables ));

		//Initialize x0
		acadoVariables.x0[ 0 ] = srv.response.si;
		acadoVariables.x0[ 1 ] = srv.response.dist;
		acadoVariables.x0[ 2 ] = srv.response.phi;
		acadoVariables.x0[ 3 ] = srv.response.vx;
		acadoVariables.x0[ 4 ] = srv.response.delta;
		acadoVariables.x0[ 5 ] = srv.response.dc;
		
		if (acadoVariables.x0[3]<2.0){
			common_msgs::Controls cmd;
			cmd.accelerator = 0.2;
			cmd.steering = 0.0;//-0.2*acadoVariables.x0[1]-0.2*i_mu;
			printf("cmd: %f, delta: %f\n", cmd.accelerator, cmd.steering );
			cout << "v_real " << acadoVariables.x0[3] << "\n";
			cout << "i_mu " << i_mu << "\n";
			pub.publish(cmd);
			primera_iter=true;
		}
		else{
			/* Initialize the solver. */
		if( !acado_inicializado ) {
			acado_initializeSolver();
			acado_inicializado = true;
		}
		// acado_initializeSolver();
		if( primera_iter ) {
			acadoVariables.x0[5] = 0.0;
			for (i = 0; i < (N+1); ++i) {
				for (int j=0; j<NX;++j) prediccion[i*NX +j]=acadoVariables.x0[j];
			}
			primera_iter = false;
		}
		

		/* Initialize the states and controls. */
		for (i = 0; i < NX * (N + 1); ++i)  acadoVariables.x[ i ] = prediccion[ i ];
		for (i = 0; i < NU * N; ++i)  acadoVariables.u[ i ] = u_prediccion[ i ];

		/* Initialize the measurements/reference. */
		for (i = 0; i < NY * N; ++i)  acadoVariables.y[ i ] = 0.0;
		for (i = 0; i < NYN; ++i)  acadoVariables.yN[ i ] = 0.0;
		
		
		for (i=0; i<NX+1;++i) acadoVariables.od[i]= interpolate(s,k,acadoVariables.x[i*NX],true);
		
		/* Prepare first step */
		// if(!acado_preparado) {
		// 	acado_preparationStep();
		// 	acado_preparado = true;
		// }
		acado_preparationStep();

		/* Perform the feedback step. */
		acado_feedbackStep( );

		
		if( VERBOSE ) printf("\tKKT Tolerance = %.3e\n\n", acado_getKKT() );

		// acado_printDifferentialVariables();
		/* Optional: shift the initialization (look at acado_common.h). */
		acado_shiftStates(2, 0, 0); 
		acado_shiftControls( 0 ); 

		/* Apply the new control immediately to the process, first NU components. */
		common_msgs::Controls cmd;
		cmd.accelerator = acadoVariables.x[4];
		double delta = acadoVariables.x[5];
		cmd.steering = 180*delta/3.14159265358979323846;

		if (cmd.accelerator*cmd.accelerator < 1) {
			printf("cmd: %f, delta: %f\n", cmd.accelerator, cmd.steering );
			pub.publish(cmd);	
		}
		else{
			cmd.accelerator = 0.0;
			cmd.steering = 0.0; 
			pub.publish(cmd);
		}
		for (i = 0; i < NX * (N + 1); ++i) prediccion[i] = acadoVariables.x[i];
		for (i = 0; i < NU * N; ++i) u_prediccion[i] = acadoVariables.u[i];

		// std_msgs::Float32MultiArray pred;
		// std_msgs::Float32 pred_v;
		// pred_v.data=prediccion[3];
		// pred.data.resize(2*N+2);
		// for (i=0; i<N+1; ++i){
		// 	pred.data.push_back(prediccion[i*NX]);
		// 	pred.data.push_back(prediccion[i*NX+1]);
		// }
		// pub_pred.publish(pred);
		// pub_pred_v.publish(pred_v);

		// acado_printDifferentialVariables();
		// acado_printControlVariables();
		/* Prepare for the next step. */
		acado_preparationStep();
		cout << "---\n";
		}
	}	
}

void firstCallback(const common_msgs::Trajectory::ConstPtr& msg){
	for (int i = 1; i < (msg->trajectory).size(); i++) {
		geometry_msgs::Point p = (msg->trajectory)[i];
		s.push_back(p.x);
		k.push_back(p.y);
		cout << p.y << endl ;
	}
	track_received = true;
}

int main(int argc, char **argv){

	for (int i = 0; i < NX*(N+1); ++i) prediccion[i] = 0.0;
	for (int i = 0; i < N+1; ++i) {
		prediccion[i*NX+3] = 2;
		prediccion[i*NX+4] = 0.2;
		prediccion[i*NX]= 3;
	}
	for (int i = 0; i < NU*N; ++i) u_prediccion[i] = 0.0;

	i_mu=0;

    ros::init(argc,argv,"utrilla_mpc");
    ros::NodeHandle n;
    
    ros::Publisher pub = n.advertise<common_msgs::Controls>("controls_mpc", 1);
	// ros::Publisher pub_pred = n.advertise<std_msgs::Float32MultiArray>("pred_ruta",1);
	// ros::Publisher pub_pred_v = n.advertise<std_msgs::Float32>("pred_v",1);

	ros::Subscriber sub = n.subscribe("/controller/sk", 1, firstCallback);

	string service = "get_x_msg";
	ros::ServiceClient client = n.serviceClient<common_msgs::MPCCurrentState>(service);

	ros::Timer timer = n.createTimer(ros::Duration(0.1), boost::bind(timerCallback, _1, boost::ref(pub), boost::ref(client)));

	    
    ros::spin();

    return 0;
}

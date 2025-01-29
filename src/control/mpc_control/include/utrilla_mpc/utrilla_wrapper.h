
#pragma once

//#include <Eigen/Eigen>
#include <ros/ros.h>

namespace utrilla_mpc {

#include "acado_auxiliary_functions.h"
#include "acado_common.h"

static constexpr int kSamples = ACADO_N;      // number of samples
static constexpr int kStateSize = ACADO_NX;   // number of states
static constexpr int kRefSize = ACADO_NY;     // number of reference states
static constexpr int kEndRefSize = ACADO_NYN; // number of end reference states
static constexpr int kInputSize = ACADO_NU;   // number of inputs
static constexpr int kCostSize = ACADO_NY - ACADO_NU; // number of state costs
static constexpr int kOdSize = ACADO_NOD;     // number of online data

ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;


template <typename T>
class UtrillaWrapper
{
 public:


  UtrillaWrapper();
  
 private:
  
};



} // namespace MPC
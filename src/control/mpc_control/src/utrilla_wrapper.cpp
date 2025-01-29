#include "utrilla_mpc/utrilla_wrapper.h"



namespace utrilla_mpc {

// Default Constructor.
template <typename T>
UtrillaWrapper<T>::UtrillaWrapper()
{
//   // Clear solver memory.
//   memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
//   memset(&acadoVariables, 0, sizeof( acadoVariables ));
  
  // Initialize the solver.
//   acado_initializeSolver();
  std::cout << "acado inicializado" << std::endl;

}
}

#define S_FUNCTION_LEVEL 2 
#define S_FUNCTION_NAME NDOB
#include "simstruc.h" 
#include <math.h> 

#define U(element) (*uPtrs[element]) /*Pointer to Input Port0*/ 

static void mdlInitializeSizes(SimStruct *S){ 
ssSetNumContStates(S, 1);						//terdapat 1 state 
if (!ssSetNumInputPorts(S, 1)) return; 
ssSetInputPortWidth(S, 0, 2); 					//terdapat 2 input
ssSetInputPortDirectFeedThrough(S, 0, 1); 
ssSetInputPortOverWritable(S, 0, 1); 
if (!ssSetNumOutputPorts(S, 1)) return; 
ssSetOutputPortWidth(S, 0, 1); 					//terdapat 1 output
ssSetNumSampleTimes(S, 1); 

ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE); } 

static void mdlInitializeSampleTimes(SimStruct *S) { 
ssSetSampleTime(S, 0, CONTINUOUS_SAMPLE_TIME); 
ssSetOffsetTime(S, 0, 0.0); } 

#define MDL_INITIALIZE_CONDITIONS 
static void mdlInitializeConditions(SimStruct *S) { 

real_T *X0 = ssGetContStates(S); 
int_T nStates = ssGetNumContStates(S); 
int_T i; 

/* initialize the states to 0.0 */ 
for (i=0; i < nStates; i++) {X0[i] = 0.0;} } 

static void mdlOutputs(SimStruct *S, int_T tid) { 
real_T *Y = ssGetOutputPortRealSignal(S,0); 
real_T *X = ssGetContStates(S); 
InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0); 

real_T w_m = U(0);

//state variable
real_T p = X[0];

//inisialisasi variabel bantu
real_T l = 200;

//disturbance estimated
real_T d_hat = p + l*w_m;

//output
Y[0] = d_hat;
} 

#define MDL_DERIVATIVES 
static void mdlDerivatives(SimStruct *S) { 
real_T *dX = ssGetdX(S); 
real_T *X = ssGetContStates(S); 
InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0); 

//inisialisasi parameter model
real_T R = 1.86; 		//resistansi stator
real_T L = 2.8e-3; 		//induktansi stator
real_T P = 4;			//jumlah pole pair
real_T psi = 0.109;		//flux permanent magnet
real_T J = 2.95e-4;		//momen inersia bare rotor
real_T B = 0.001; 		//koefisien gaya gesek

//inisialisasi variabel input
real_T w_m = U(0);
real_T iq = U(1);

//inisialisasi variabel bantu
real_T x2 = w_m;
real_T x3 = iq;
real_T l = 200;
real_T theta_1 = (3*P*psi)/(2*J);
real_T theta_2 = B/J;

//state variable
real_T p = X[0];

//state equation
real_T p_dot = (-1)*l*p - l*(l*x2 - theta_2*x2 + theta_1*x3);

//state derivative
dX[0] = p_dot;
} 

static void mdlTerminate(SimStruct *S) 
{} /*Keep this function empty since no memory is allocated*/ 

#ifdef MATLAB_MEX_FILE 
/* Is this file being compiled as a MEX-file? */ 
#include "simulink.c" /* MEX-file interface mechanism */ 
#else 
#include "cg_sfun.h" /*Code generation registration function*/ 
#endif 

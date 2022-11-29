#define S_FUNCTION_LEVEL 2 
#define S_FUNCTION_NAME derivative_d_hat
#include "simstruc.h" 
#include <math.h> 

#define U(element) (*uPtrs[element]) /*Pointer to Input Port0*/ 

static void mdlInitializeSizes(SimStruct *S){ 
ssSetNumDiscStates(S, 2); 						//menggunakan 2 state
if (!ssSetNumInputPorts(S, 1)) return; 
ssSetInputPortWidth(S, 0, 1); 					//terdapat 1 input
ssSetInputPortDirectFeedThrough(S, 0, 1); 
ssSetInputPortOverWritable(S, 0, 1); 
if (!ssSetNumOutputPorts(S, 1)) return; 
ssSetOutputPortWidth(S, 0, 1); 					//terdapat 1 output
ssSetNumSampleTimes(S, 1); 

ssSetOptions(S, (SS_OPTION_EXCEPTION_FREE_CODE 
| SS_OPTION_DISCRETE_VALUED_OUTPUT));} 

static void mdlInitializeSampleTimes(SimStruct *S){ 
ssSetSampleTime(S, 0, 1e-6); 
ssSetOffsetTime(S, 0, 0.0);} 

#define MDL_INITIALIZE_CONDITIONS 
static void mdlInitializeConditions(SimStruct *S){ 
real_T *X0 = ssGetRealDiscStates(S); 
int_T nXStates = ssGetNumDiscStates(S); 
InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0); 
int_T i; 

/* initialize the states to 0.0 */ 
for (i=0; i < nXStates; i++) { 
X0[i] = 0.0; } } 

static void mdlOutputs(SimStruct *S, int_T tid){ 
real_T *Y = ssGetOutputPortRealSignal(S,0); 
real_T *X = ssGetRealDiscStates(S); 

//output
Y[0] = X[1];

} 

#define MDL_UPDATE 
static void mdlUpdate(SimStruct *S, int_T tid) { 
real_T *X = ssGetRealDiscStates(S); 
InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0); 

real_T dt = 1e-6;

//inisialisasi input
real_T d_hat = U(0);

//inisialisasi state variable
real_T d_hat_old = X[0];

//menghitung derivative dari d_hat
real_T d_hat_dot = (d_hat - d_hat_old)/dt;

//update state variable
X[0] = d_hat;
X[1] = d_hat_dot;

}

static void mdlTerminate(SimStruct *S) 
{ } /*Keep this function empty since no memory is allocated*/ 

#ifdef MATLAB_MEX_FILE 
/* Is this file being compiled as a MEX-file? */ 
#include "simulink.c" /*MEX-file interface mechanism*/ 
#else 
#include "cg_sfun.h" /*Code generation registration function*/ 
#endif 


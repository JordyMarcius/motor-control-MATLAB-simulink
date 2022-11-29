#define S_FUNCTION_LEVEL 2 
#define S_FUNCTION_NAME PMSM
#include "simstruc.h" 
#include <math.h> 

#define U(element) (*uPtrs[element]) /*Pointer to Input Port0*/ 

static void mdlInitializeSizes(SimStruct *S){ 
ssSetNumContStates(S, 4);						//terdapat 4 state 
if (!ssSetNumInputPorts(S, 1)) return; 
ssSetInputPortWidth(S, 0, 4); 					//terdapat 4 input
ssSetInputPortDirectFeedThrough(S, 0, 1); 
ssSetInputPortOverWritable(S, 0, 1); 
if (!ssSetNumOutputPorts(S, 1)) return; 
ssSetOutputPortWidth(S, 0, 4);					//terdapat 4 output 
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

//state variable
real_T x1 = X[0];		//x1 = theta_m
real_T x2 = X[1];		//x2 = w_m
real_T x3 = X[2];		//x3 = iq
real_T x4 = X[3];		//x4 = id

//output
Y[0] = x4;
Y[1] = x3;
Y[2] = x1;
Y[3] = x2;

} 

#define MDL_DERIVATIVES 
static void mdlDerivatives(SimStruct *S) { 
real_T *dX = ssGetdX(S); 
real_T *X = ssGetContStates(S); 
InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0); 

//inisialisasi parameter model
real_T R = 1.86; 		//resistansi stator
real_T Ls = 2.8e-3; 	//induktansi stator
real_T P = 4;			//jumlah pole pair
real_T psi = 0.109;		//flux permanent magnet
real_T J = 2.95e-4;		//momen inersia bare rotor
real_T B = 0.001; 		//koefisien gaya gesek

//inisialisasi variabel input
real_T Va = U(0);
real_T Vb = U(1);
real_T Vc = U(2);
real_T TL = U(3);

//inisialisasi variabel bantu
//real_T x1 = theta_m;
//real_T x2 = w_m;
//real_T x3 = iq;
//real_T x4 = id;

real_T g1 = R/Ls;
real_T g2 = P;
real_T g3 = (P*psi)/Ls;
real_T g4 = 1/Ls;

real_T theta_1 = (3*P*psi)/(2*J);
real_T theta_2 = B/J;
real_T theta_3 = TL/J;

real_T h1 = 20;			//nilai batas f1 (uncertainties)
real_T h2 = 20;			//nilai batas f2 (unvertainties)

real_T K = 0.816497; 	//akar(2/3)
real_T L = 0.866025;	//(akar3)/2

//state variable
real_T x1 = X[0];		//x1 = theta_m
real_T x2 = X[1];		//x2 = w_m
real_T x3 = X[2];		//x3 = iq
real_T x4 = X[3];		//x4 = id

//transformasi abc to alpha-beta (Clarke Transformation)
real_T V_alpha = K*(Va - 0.5*Vb - 0.5*Vc);
real_T V_beta = K*(L*Vb - L*Vc);

real_T theta_e = x1 * P;
//transormasi alpha-beta to dq (Park Transformation)
real_T Vd = (V_alpha * cos(theta_e)) + (V_beta * sin(theta_e));
real_T Vq = ((-1)*V_alpha * sin(theta_e)) + (V_beta * cos(theta_e));

//state equation
real_T x1_dot = x2;
real_T x2_dot = theta_1*x3 - theta_2*x2 -theta_3;
real_T x3_dot = (-1)*g1*x3 - g2*x2*x4 - g3*x4 + g4*Vq;//+h1
real_T x4_dot = (-1)*g1*x4 + g2*x2*x3 + g4*Vd; //+h2

//state derivative
dX[0] = x1_dot;
dX[1] = x2_dot;
dX[2] = x3_dot;
dX[3] = x4_dot;
} 

static void mdlTerminate(SimStruct *S) 
{} /*Keep this function empty since no memory is allocated*/ 

#ifdef MATLAB_MEX_FILE 
/* Is this file being compiled as a MEX-file? */ 
#include "simulink.c" /* MEX-file interface mechanism */ 
#else 
#include "cg_sfun.h" /*Code generation registration function*/ 
#endif 

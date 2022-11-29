#define S_FUNCTION_LEVEL 2 
#define S_FUNCTION_NAME RBC
#include "simstruc.h" 
#include <math.h> 

#define U(element) (*uPtrs[element]) /*Pointer to Input Port0*/ 


static void mdlInitializeSizes(SimStruct *S){ 
if (!ssSetNumInputPorts(S, 1)) return; 
ssSetInputPortWidth(S, 0, 11); 					//terdapat 12 input
ssSetInputPortDirectFeedThrough(S, 0, 1); 
ssSetInputPortOverWritable(S, 0, 1); 
if (!ssSetNumOutputPorts(S, 1)) return; 
ssSetOutputPortWidth(S, 0, 3); 					//terdapat 3 output
ssSetNumSampleTimes(S, 1); 

ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE); } 

static void mdlInitializeSampleTimes(SimStruct *S) { 
ssSetSampleTime(S, 0, 1e-6); 
ssSetOffsetTime(S, 0, 0.0); } 

static void mdlOutputs(SimStruct *S, int_T tid) { 
real_T *Y = ssGetOutputPortRealSignal(S,0); 
InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0); 
real_T t = ssGetT(S);

//inisialisasi parameter model
real_T R = 1.86; 		//resistansi stator
real_T Ls = 0.0028; 	//induktansi stator
real_T P = 4;			//jumlah pole pair
real_T psi = 0.109;		//flux permanent magnet
real_T J = 0.000295;	//momen inersia bare rotor
real_T B = 0.001; 		//koefisien gaya gesek

//inisialisasi variabel nilai batas disturbance error
real_T zeta = 10;

//inisialisasi variabel Robust Backstepping Control (RBC) parameter
real_T k1 = 50;			
real_T k2 = 100;
real_T k3 = 500;
real_T k4 = 200;
real_T eps1 = 100;
real_T eps2 = 200;
real_T eps2r = 0.01;
real_T eps3 = 0.01;
real_T h1 = 20;
real_T h2 = 20;

//inisialisasi variabel input
real_T id_r 			= U(0);
real_T theta_r 			= U(1);
real_T theta_r_dot 		= U(2);
real_T theta_r_dot2 	= U(3);
real_T theta_r_dot3 	= U(4);
real_T id 				= U(5);
real_T iq 				= U(6);
real_T theta_m 			= U(7);
real_T w_m 				= U(8);
real_T d_hat 			= U(9);
real_T d_hat_dot 		= U(10);


//inisialisasi variabel bantu
real_T x1 = theta_m;
real_T x2 = w_m;
real_T x3 = iq;
real_T x4 = id;

real_T g1 = R/Ls;
real_T g2 = P;
real_T g3 = (P*psi)/Ls;
real_T g4 = 1/Ls;

real_T theta_1 = (3*P*psi)/(2*J);
real_T theta_2 = B/J;


//inisialisasi variabel eror 
real_T z1 = x1 - theta_r;					//eror theta

real_T alpha_1 = (-1)*k1*z1 + theta_r_dot;	//virtual control signal (w) referens
real_T z2 = x2 - alpha_1;					//eror w

real_T alpha_2a = (-1)*(1/theta_1)*(z1 - theta_2*x2 + d_hat + k1*x2 - k1*theta_r_dot - theta_r_dot2 + k2*z2);
real_T alpha_2r = (-1)*((zeta*zeta)/(4*theta_1*eps1))*z2;
real_T alpha_2 = alpha_2a + alpha_2r; 		//virtual control signal (iq) referens

real_T z3 = x3 - alpha_2;					//eror iq
real_T z4 = x4;								//eror id

//inisialisasi variabel bantu
real_T fi_1 = (1/theta_1)*((k1+k2)*theta_r_dot2 + theta_r_dot3 - d_hat_dot - (1+k1*k2)*(x2-theta_r_dot) + ((zeta*zeta)/(4*eps1))*z2);
real_T fi_2 = (1/theta_1)*((zeta*zeta)/(4*eps1) + k1 + k2 - theta_2);

//nilai Vq = V_qa + V_qr
real_T V_qa = (-1)*(1/g4)*(theta_1*z2 - g1*x3 - g3*x2 - fi_1 + fi_2*theta_1*x3 - fi_2*theta_2*x2 + fi_2*d_hat + k3*z3);	//model compensation control
real_T V_qr = (-1)*(z3/g4)*((h1*h1)/(4*eps2) + (fi_2*fi_2*zeta*zeta)/(4*eps2r)); //robust control rate
real_T Vq = V_qa + V_qr;

//nilai Vd = V_da + V_dr
real_T V_da = (-1)*(1/g4)*((-1)*g2*x2*z3 - g1*x4 + g2*x2*x3 + k4*z4); //model compensation control
real_T V_dr = (-1)*((h2*h2)/(4*g4*eps3))*z4; //robust control rate
real_T Vd = V_da + V_dr;

//menghitung theta elektris
real_T theta_e = theta_m * P;

//transformasi dq to alpha-beta (inverse Park Transformation)
real_T V_alpha = Vd*cos(theta_e) - Vq*sin(theta_e);
real_T V_beta = Vd*sin(theta_e) + Vq*cos(theta_e);


//transformasi alfa-beta to abc (inverse Clarke Transformation)
real_T K = 0.816497; 	//akar(2/3)
real_T L = 0.866025;	//(akar3)/2

real_T Va = K*V_alpha;
real_T Vb = K*((-1)*0.5*V_alpha + L*V_beta);
real_T Vc = K*((-1)*0.5*V_alpha - L*V_beta);

Y[0] = Va;
Y[1] = Vb;
Y[2] = Vc;
}


static void mdlTerminate(SimStruct *S) 
{ } /*Keep this function empty since no memory is allocated*/ 

#ifdef MATLAB_MEX_FILE 
/* Is this file being compiled as a MEX-file? */ 
#include "simulink.c" /* MEX-file interface mechanism */ 
#else 
#include "cg_sfun.h" /*Code generation registration function*/ 
#endif

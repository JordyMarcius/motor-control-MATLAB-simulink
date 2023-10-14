# Motor-Control-MATLAB-Simulink
The main goal of this project is to build a simulation of electrical motor control using CMEX S-Function. Mathematical model of electrical motor and control system is obtained from IEEE journals with the title is **Robust Position Anti-Interference Control for PMSM Servo System With Uncertain Disturbance** by Longfei Li, Jie Xiao, Yun Zhao, Kan Liu, Senior Member, IEEE, Xiaoyan Peng, Haozhe Luan, and Kaiqing Li.

<h1>Mathematical Model</h1>

<h2>Permanent Magnet Synchronous Motor (PMSM)</h2>

Mathematical model of Permanent Magnet Synchronous Motor (PMSM) on the $dq$-axis:
- $\dot{x_1} = x_2$
- $\dot{x_2} = \theta_1 x_3 - \theta_2 x_2 -\theta_3$
- $\dot{x_3} = -g_1 x_3 - g_2 x_2 x_4 -g_3 x_4 + g_4 u_q + f_1$
- $\dot{x_4} = -g_1 x_4 + g_2 x_2 x_3 + g_4 u_d + f_2$

where

$x_1 = \theta_m , x_2 = \omega_m , x_3 = i_q , x_4 = i_d$

$g_1 = \frac{R_s}{L} , g_2 = P_n , g_3 = \frac{P_n \psi_f}{L} , g_4 = \frac{1}{L}$

$\theta_1 = \frac{3 P_n \psi_f}{2J} , \theta_2 = \frac{B}{J} , \theta_3 = \frac{T_L}{J}$

where

| Symbol | Details  |
| ------- | --- |
| $i_d$ | current in $d$-axis |
| $i_q$ | current in $q$-axis |
| $R_s$ | stator resistance |
| $L$ | $dq$-axis inductance |
| $\psi_f$ | permanent magnet flux |
| $P_n$ | number of pole pairs |
| $J$ | moment of inertia |
| $T_L$ | load torque |
| $B$ | friction coefficient |
| $\theta_m$ | mechanical position angle |
| $\omega_m$ | rotor angular speed |
| $f_1, f_2$ | other uncertaities (external disturbance, modelling errors, etc) |


<h2>Nonlinear Disturbance Observer (NDOB)</h2>

- $\hat{d} = p + lx_2$
- $\dot{p} = -lp - l[lx_2 - \theta_{2n} x_2+\theta_{1n} x_3]$

where

| Symbol | Details  |
| ------- | --- |
| $\hat{d}$ | lumped system disturbance |
| $p$ | intermediate variables of NDOB |
| $l$ | gains of NDOB |

<h2>Robust Backstepping Controller</h2>

Control signal output ($u_q$) is expressed as follows:
- $u_q = u_{qa} + u_{qr}$
- $u_{qa} = -\frac{1}{g_4}(\theta_{1n}z_2-g_1x_3-g_3x_2-\varphi_1+\varphi_2\theta_{1n}x_3-\varphi_2\theta_{2n}x_2+\varphi_2\hat{d}+k_3z_3)$
- $u_{qr} = -\frac{z_3}{g_4}(\frac{(h_1)^2}{4\varepsilon_2} + \frac{(\varphi_2\zeta)^2}{4\epsilon_{2r}})$

Control signal output ($u_d$) is expressed as follows:
- $u_d = u_{da} + u_{dr}$
- $u_{da} = -\frac{1}{g_4}(-g_2x_2z_3-g_1x_4+g_2x_2x_3+k_4z_4)$
- $u_{dr} = -\frac{(h_2)^2}{4g_4\varepsilon_3}z_4$

where

$\varphi_1 = \frac{1}{\theta_{1n}}[(k_1 + k_2)\ddot{x_r} + \dddot{x_r} - \dot{\hat{d}} - (1 + k_1 k_2)(x_2 - \dot{x_r}) + \frac{(\zeta)^2}{4\varepsilon_1}z_2]$

$\varphi_2 = \frac{1}{\theta_{1n}}(\frac{(\zeta)^2}{4\varepsilon_1} + k_1 + k_2 - \theta_{2n})$

where

| Symbol | Details  |
| ------- | --- |
| $u_{da}, u_{qa}$ | model compensation control |
| $u_{dr}, u_{qr}$ | robust control rate |

<h1>Simulink Block Function</h1>

![1](https://user-images.githubusercontent.com/65435469/204591409-bd9fcabf-2c58-47ee-9cc0-757800b3595c.PNG)

| Symbol | Value  |
| ------- | --- |
| $R_s$ | $1.86 \Omega$ |
| $L$ | $2.8mH$ |
| $\psi_f$ | $0.109Wb$ |
| $P_n$ | $4$ |
| $J$ | $2.95 \times 10^{-4} kg \dot{} m^2$  |
| $B$ | $0.001$ |
| $l$ | $200$ |
| $h_1$ | $20$ |
| $h_2$ | $20$ |
| $zeta$ | $10$ |
| $k_1$ | $50$ |
| $k_2$ | $100$ |
| $k_3$ | $500$ |
| $k_4$ | $200$ |
| $\varepsilon_1$ | $100$ |
| $\varepsilon_2$ | $200$ |
| $\varepsilon_{2r}$ | $0.01$ |
| $\varepsilon_3$ | $0.01$ |

<h1>Experiment 1</h1>

In the first experiment, reference rotor position or tetha reference is $(10t-5) rad$ at $t = 0.5s$ and the motor is loaded with $1Nm$ at $t = 1.5s$. The result is shown in figure 1 and 2. (Figure 2 is the "zoom in" version of figure 1).

The yellow line inidicates tetha reference or reference angle position and the blue line indicates tetha mechanic or actual angle position for PSMM servo. Figure 3 shows the difference between these parameters.

<h1>Results of Experiment 1</h1>

![3](https://user-images.githubusercontent.com/65435469/204592439-019ad4f1-351d-4868-a94c-5b7a70983037.PNG)
Figure 1. Comparison between $\theta_r$ and $\theta_m$

![2](https://user-images.githubusercontent.com/65435469/204592643-daa152ca-153a-43e5-93d8-9dc984cae65e.PNG)
Figure 2. Comparison between $\theta_r$ and $\theta_m$ (zoom in version)

![4](https://user-images.githubusercontent.com/65435469/204592916-1e11250e-df77-4c51-837a-8acce30108ed.PNG)
Figure 3. Error angle position

<h1>Experiment 2</h1>

In the second experiment, the reference rotor position or tetha reference is a sinusoidal function $\theta_r = 3sin(2πt)rad$ and the motor is loaded with $1Nm$ at $t = 1.5s$. The result is shown in figure 4 and 5. (Figure 5 is the "zoom in" version of figure 4). 

The yellow line inidicates tetha reference or reference angle position and the blue line indicates tetha mechanic or actual angle position for PSMM servo. Figure 3 shows the difference between these parameters.

<h1>Results of Experiment 2</h1>

![5](https://user-images.githubusercontent.com/65435469/204593221-523bdf6a-81a0-4d4c-b423-8d610da75760.PNG)
Figure 4. Comparison between $\theta_r$ and $\theta_m$

![6](https://user-images.githubusercontent.com/65435469/204593233-9689f313-6739-4af5-adf9-cea962da6824.PNG)
Figure 5. Comparison between $\theta_r$ and $\theta_m$ (zoom in version)

![7](https://user-images.githubusercontent.com/65435469/204593247-0975d9cd-2f0d-418b-8f12-5f8724cf5a89.PNG)
Figure 6. Error angle position

<h1>Conclusion</h1>

From the results above, the actual angle position can track the reference angle position with addition of external load to the PMSM servo. It shows a high tracking accuracy with small value of error. So, it can be concluded that the PMSM servo can be controlled using a robust backstepping controller system with addition of nonlinear disturbance observer.

<h1>References</h1>

- Yusivar, Feri & Wakao, Shinji. (2001). Minimum requirements of motor vector control modeling and simulation utilizing C MEX S-function in MATLAB/SIMULINK. 315 - 321 vol.1. 10.1109/PEDS.2001.975333.

- L. Li et al., "Robust position anti-interference control for PMSM servo system with uncertain disturbance," in CES Transactions on Electrical Machines and Systems, vol. 4, no. 2, pp. 151-160, June 2020, doi: 10.30941/CESTEMS.2020.00020.

- Sun, Xiaofei & Yu, Haisheng & Yu, Jinpeng & Liu, Xudong. (2019). Design and implementation of a novel adaptive backstepping control scheme for aPMSM with unknown load torque. IET Electric Power Applications. 13. 10.1049/iet-epa.2018.5656

- W. Yin, X. Wu and X. Rui, "Adaptive Robust Backstepping Control of the Speed Regulating Differential Mechanism for Wind Turbines," in IEEE Transactions on Sustainable Energy, vol. 10, no. 3, pp. 1311-1318, July 2019, doi: 10.1109/TSTE.2018.2865631.

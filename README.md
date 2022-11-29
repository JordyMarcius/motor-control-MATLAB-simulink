# motor-control-MATLAB-simulink
The main goal of this project is to build a simulation of electrical motor control using CMEX S-Function. Mathematical model of electrical motor and control system is obtained from IEEE journals with title **Robust Position Anti-Interference Control for PMSM Servo System With Uncertain Disturbance** by Longfei Li, Jie Xiao, Yun Zhao, Kan Liu, Senior Member, IEEE, Xiaoyan Peng, Haozhe Luan, and Kaiqing Li.

<h1>Simulink Block Function</h1>

![1](https://user-images.githubusercontent.com/65435469/204591409-bd9fcabf-2c58-47ee-9cc0-757800b3595c.PNG)

<h1>Experiment 1</h1>
In experiment 1, reference rotor position or tetha reference is (10t-5)rad at t = 0.5s and the motor is loaded with 1Nm at t = 1.5s. The result is shown in a figure below. Yellow line inidicate tetha reference and blue line indicate tetha mechanic or actual. As can be seen that motor can be controlled and has high tracking accuracy with small error value.

<h2>Tetha Mechanic and Tetha Reference Comparison</h2>

![3](https://user-images.githubusercontent.com/65435469/204592439-019ad4f1-351d-4868-a94c-5b7a70983037.PNG)

![2](https://user-images.githubusercontent.com/65435469/204592643-daa152ca-153a-43e5-93d8-9dc984cae65e.PNG)

<h2>Error of Tetha Mechanic and Tetha Reference</h2>

![4](https://user-images.githubusercontent.com/65435469/204592916-1e11250e-df77-4c51-837a-8acce30108ed.PNG)

<h1>Experiment 2</h1>
In experiment 2, reference rotor position or tetha reference is sinusoidal function 3sin(2πt)rad and the motor is loaded with 1Nm at t = 1.5s. The result is shown in a figure below. Yellow line inidicate tetha reference and blue line indicate tetha mechanic or actual. As can be seen that motor can be controlled and has high tracking accuracy with small error value.

<h2>Tetha Mechanic and Tetha Reference Comparison</h2>

![5](https://user-images.githubusercontent.com/65435469/204593221-523bdf6a-81a0-4d4c-b423-8d610da75760.PNG)

![6](https://user-images.githubusercontent.com/65435469/204593233-9689f313-6739-4af5-adf9-cea962da6824.PNG)

<h2>Error of Tetha Mechanic and Tetha Reference</h2>

![7](https://user-images.githubusercontent.com/65435469/204593247-0975d9cd-2f0d-418b-8f12-5f8724cf5a89.PNG)

<h1>References</h1>

- Yusivar, Feri & Wakao, Shinji. (2001). Minimum requirements of motor vector control modeling and simulation utilizing C MEX S-function in MATLAB/SIMULINK. 315 - 321 vol.1. 10.1109/PEDS.2001.975333.

- L. Li et al., "Robust position anti-interference control for PMSM servo system with uncertain disturbance," in CES Transactions on Electrical Machines and Systems, vol. 4, no. 2, pp. 151-160, June 2020, doi: 10.30941/CESTEMS.2020.00020.

- Sun, Xiaofei & Yu, Haisheng & Yu, Jinpeng & Liu, Xudong. (2019). Design and implementation of a novel adaptive backstepping control scheme for aPMSM with unknown load torque. IET Electric Power Applications. 13. 10.1049/iet-epa.2018.5656

- W. Yin, X. Wu and X. Rui, "Adaptive Robust Backstepping Control of the Speed Regulating Differential Mechanism for Wind Turbines," in IEEE Transactions on Sustainable Energy, vol. 10, no. 3, pp. 1311-1318, July 2019, doi: 10.1109/TSTE.2018.2865631.

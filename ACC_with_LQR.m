
% Adaptive Cruise Control System

%%%%%%%%%%%%%%%%%% Continuous time Plant Specification %%%%%%%%%%%%%%%%%%%%

A=[0 1 0;0 0 1;-6.0476 -5.2856 -0.238];
B=[0;0;2.4767];
C=[1 0 0];
D=0;

dimension=3;

 %%% This should generate same A,B,C,D matrices as given as input

ACC_open_loop_sys = ss(A, B, C, D);

%%%%%%%%%%%%%% Derive Open loop Poles %%%%%%%%%%%%%%%%%%%%%%

open_loop_poles= pole(ACC_open_loop_sys);

%%% This should be same as open_loop_poles
 
eig(A); % two poles are at right-half plane. Hence the system is unstable in open loop. 


%%%%%%%%%%%%%%%%%% Discrete time Plant Design--State space Model only %%%%%%%%%%%%%%%%%%%%

Ts=0.1;  % sampling period % change sampling period and check response 

ACC_discrete_sys = c2d(ACC_open_loop_sys, Ts,'zoh');
 
[A_d, B_d, C_d, D_d] = ssdata(ACC_discrete_sys)

discrete_open_loop_poles=abs(eig(A_d));  % two poles are on unit circle. Hence unstable

figure; step(ACC_discrete_sys,50);   % open loop step response

%%%%%%%%%%%--- controller design ---%%%%%%%%%%

Q=10^3*(C'*C); % reducing Q increases settling time and vice versa
R=10^1;  % reducing R reduces settling time and vice versa

%%%%% Discrete LQR Controller for ACC_discrete_sys

[K,S,e] = dlqr(A_d, B_d, Q, R);     % Get OptimalGain K, S matrix, and poles e

%%%%%%% Cross checking the gain %%%%%%%

K1=place(A_d,B_d,e); % K and K1 should be same

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

discrete_closed_loop_poles=abs(e)

A_cl = (A_d-B_d*K);
B_cl = B_d;
C_cl = C_d;
D_cl = D_d;

ACC_closed_loop_sys = ss(A_cl,B_cl,C_cl,D_cl,Ts);

figure;step(ACC_closed_loop_sys,5);  % Closed loop step response upto 5 sec
 
stepinfo(ACC_closed_loop_sys)


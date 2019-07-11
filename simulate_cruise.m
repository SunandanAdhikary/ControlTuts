% Adaptive Cruise Control System

execution_pattern='111111111'; 
%  execution_pattern='11101110111';
 
%%%%%%%%%%%%%%%%%% Plant-Controller Design %%%%%%%%%%%%%%%%%%%%

A=[0 1 0;0 0 1;-6.0476 -5.2856 -0.238];
B=[0;0;2.4767];
C=[1 0 0];
D=0;

dimension=3;
Ts=0.04;

plant = ss(A, B, C, D);
[dim_s,dim_i]=size(B);       

ZOH = c2d(plant, Ts,'zoh');               % State space equation of plant-- discrete time
[Apd, Bpd, Cpd, Dpd] = ssdata(ZOH);

open_loop_poles=abs(eig(Apd));
% figure; step(ZOH,1);   % open loop step response

%%%%%%%%%%%--- controller design ---%%%%%%%%%%

Q_1c_1=10^3*(C'*C);
Q_12c_1=[0;0;0];
Q_2c_1=10^-4;
R_1c_1=0.005*(B*B');
R_2_1=0.005;


QXU=[Q_1c_1 Q_12c_1;Q_12c_1' Q_2c_1];
QWU=[R_1c_1 zeros(dimension,1);zeros(1,dimension) R_2_1];
        
lqg_reg=lqg(ZOH,QXU,QWU);
[Acd,Bcd,Ccd,Dcd]=ssdata(lqg_reg) ;

Ic=eye(dimension);
Z=zeros(dimension,dimension);   

A1=[Apd Bpd*Ccd;Bcd*Cpd Acd];  %non-delayed system
A0=[Apd Bpd*Ccd;Z Ic];

K=(-1)*lqg_reg.c;
x=abs(eig(A1));
closed_loop_poles=x(1:3);

Q=Q_1c_1;
R=Q_2c_1;

%{
[K,S,e] = dlqr(Apd, Bpd, Q, R);     % Get Optimal S matrix
K
closed_loop_poles=abs(e)

Z=zeros(dimension,dimension);

A1=[Apd Bpd;-K zeros(dim_i,dim_i)]; % one interval delayed system
A0=[Apd Bpd;zeros(dim_i,dim_s) eye(dim_i)];
%}


% figure;step(lqg_reg,1);  % Closed loop step response
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%{

d_nom=100;          % nominal disturbance
d_max=160;            % maximum disturbance
desired_norm=30;

ST_dsr=1.2;         % desirable settling time
ST_mrg=2.4;         % marginal settling time

m1=2;
m2=4;

L1=ST_dsr/Ts;
l1=ceil(L1/m1)

L2=ST_mrg/Ts;
l2=ceil(L2/m2)

epsilon_ext=(desired_norm/d_max)^(1/m1)
epsilon_nom=(desired_norm/d_nom)^(1/m1)
epsilon_mrg=(desired_norm/d_nom)^(1/m2)
%}
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% START SIMULATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

total_running_time=5;
simulation_sampling_time=1*(10^(-3));

control_sampling_time=Ts;
total_samples=ceil(total_running_time/simulation_sampling_time);
plot_vector_1=zeros(1,total_samples);
time_axis_1=zeros(1,total_samples);

initial_value=[0;0;0];
v=initial_value;
r=[30;0;0];  % ref speed limit is 50

quadratic_cost=0;
u=0;
pointer=0;

[temp length]=size(execution_pattern);
length;
[temp number_of_empty_slots]=size(strfind(execution_pattern,'0'));
number_of_empty_slots;

for i=1:total_samples
%    dv=(A*v+B*(u)+state_noise(B))*simulation_sampling_time;
   dv=(A*(v)+B*(u))*simulation_sampling_time;
% v
% dv
% norm(v)
   v=v+dv;
  if(~mod(i,(control_sampling_time/simulation_sampling_time)))
	  pointer=mod(pointer,length);
	  pointer=pointer+1;
	  if((execution_pattern(pointer)=='1'))
		u=(-1)*K*(v-r);
	  end
	  quadratic_cost=quadratic_cost+(v'*Q*v)+(u'*R*u); 
  end
    time_axis_1(i)=total_running_time*i/total_samples;
    plot_vector_1(i)=Cpd*v;
end
figure;
plot(time_axis_1,plot_vector_1,'r'),xlabel('Time(s)'),ylabel('Voltage');grid on;

quadratic_cost 



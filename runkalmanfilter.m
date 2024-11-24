%% Overview
% The model is configured with a Gaussian process and its output Y_k 
% connected to a Kalman Filter. To directly use this model, We
% only needs to provide model prarameters including parameters of the
% Gaussian process, which are state space matrices, A, B, C, and D, initial
% state, x0, and covariance matrices, Q and R; and similar parameters for
% the Kalman Filter, which can be in different values to mimic the model
% mismatch, plus the state covariance, P. 

%%  A 2-input 2-output 4-state system with non-zero D
clear all,
close all,
% dt = 0.1;
randn('seed',11);


% Model parameters 
F = [0.8110   -0.0348    0.0499    0.3313
     0.0038    0.8412    0.0184    0.0399
     0.1094    0.4094    0.6319    0.1080
    -0.3186   -0.0254   -0.1446    0.8391];
G = [-0.0130    0.0024
     -0.0011    0.0100
     -0.0781    0.0009
      0.092    0.0138];
H = [0.1685   -0.9595   -0.0755   -0.3771
     0.6664    -0.0835    0.6260    -0.6609];
% process noise variance
Q=diag([0.8^2 0.5^2]);
% measurement noise variance
R= .1*eye(2);
% initial state
x0 = 4*randn(4,1);


% Kalman filter set up

F1 =F;
G1 = G ;
H1 = H +10;
Q1 = Q;
R1 = R;
% eros initial state
x1 = x0; % zeros(4,1);
%changing the initial state
%x1 = x0+5;
% Initial state covariance
P1 = 4*eye(4);
% Simulation set up
% time span 400 samples
span = 400;
span2 = 20;

% simulation

[State, Obs] = model_sim(F,G, H,Q, R,x0,span); %State evolution and observation evolution
%  Kalman Filtering
[StateEst, ObsPred] = KalmanFilt(Obs,F1,G1, H1,Q1, R1,x1,P1,span);

t=0:span;
figure

for k=1:4
    subplot(4,1,k), hold
    p1 = plot(t, State(k,:),'b','linewidth',3); 
    p2 = plot(t, StateEst(k,:),'r','linewidth',2); 
    legend([p1, p2], {'Actual State', '1 step Predicted State'}, 'Location', 'best');
    title(sprintf('state %i',k))
end
figure(2)
for k=1:2
    subplot(2,1,k), hold
    p3 = plot(t,Obs(k,:),'b','linewidth',2);
    p4 = plot(t, ObsPred(k,:),'r','linewidth',2);
    legend([p3, p4],{'Actual Observation','1-step predicted observation'},'Location','best');
    title(sprintf('Obs %i',k))
end

%Effect on variance on convergence rate: 
%When the initial variance is higher, we place more uncertainty on the
%initial  state values, therefore the bayesian model bases its estimation
%more on the observations and less on the a priori law which leads to
%slower convergence. 
%If the initial states are the real states the filter gives precise
%estimations faster because it already starts from true values. In real
%applications we always know the initial state of our system.

[State, Obs] = model_sim(F,G, H,Q, R,x0,span2); %State evolution and observation evolution
%  Kalman Filtering
[StateEst, ObsPred] = KalmanFilt(Obs,F1,G1, H1,Q1, R1,x1,P1,span2);
[StateEst_10, ObsPred_10] = KalmanFilt(Obs,F1,G1, H1,Q1, R1,x1,10 * P1,span2);
[StateEst_100, ObsPred_100] = KalmanFilt(Obs,F1,G1, H1,Q1, R1,x1,100* P1,span2);
[StateEst_1000, ObsPred_1000] = KalmanFilt(Obs,F1,G1, H1,Q1, R1,x1,1000 *P1,span2);
t =0:span2; 
figure(3)
for k=1:4
    subplot(4,1,k), hold
    p1 = plot(t, State(k,:),'b','linewidth',3); 
    p2 = plot(t, StateEst(k,:),'r','linewidth',2); 
    p10 = plot(t, StateEst_10(k,:),'c','linewidth',2);
    p100= plot(t, StateEst_100(k,:),'y','linewidth',2);
    legend([p2, p10, p100], {'K0=P0','K0=10*P0','K0=100*P0'}, 'Location', 'best');
    title(sprintf('state %i',k))
end

xlabel('time, s')

figure(4)
for k=1:2
    subplot(2,1,k), hold
    p1 = plot(t, Obs(k,:),'b','linewidth',3); 
    p2 = plot(t, ObsPred(k,:),'r','linewidth',2); 
    p10 = plot(t, ObsPred_10(k,:),'c','linewidth',2);
    p100= plot(t, ObsPred_100(k,:),'y','linewidth',2);
    legend([p2, p10, p100], {'K0=P0','K0=10*P0','K0=100*P0'}, 'Location', 'best');
    title(sprintf('Obs %i',k))
end



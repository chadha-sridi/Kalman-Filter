function [StateEst, ObsPred] = KalmanFilt(Obs,F,G, H,Q, R,x0,K0,span);
% the Kalman filter implementation
StateEst = zeros(size(F,1), span+1);
ObsPred = zeros(size(H,1), span+1);

StateEst(:,1) = x0;
K_n = K0;


for i = 1:span
  %% Gamma update
  % Riccati for Gamma update 
  K_np1 = F * K_n * F' + G * Q * G';
  Sigma = R+ H * K_np1 * H' ;
  Gamma = K_np1 * H' / Sigma;
  Kn = (eye(size(K_n)) - Gamma * H) * K_np1;
  K_n = Kn;
  %% Kalman recursive update
  ObsPred(:,i+1) = H * F * StateEst(:,i);
  alpha = Obs(:,i) - ObsPred(:,i+1); %innovation error
  StateEst(:,i+1) = F * StateEst(:,i) + Gamma * alpha;
  
end 


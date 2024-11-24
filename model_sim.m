function [State, Obs] = model_sim(F, G, H, Q, R, x0, span)
    % Initialize process and observation noise
    %The Cholesky decomposition is used to generate noise with a specific covariance.It decomposes the covariance matrix Q or R into a lower triangular matrix L. 
    % By multiplying standard Gaussian noise (randn()) with L, the resulting noise has the desired covariance (ie R for B and Q for U).
    
    U = chol(Q)' * randn(size(G, 2), span + 1);  % Process noise
    U = U - mean(U, 'all');                      %centering the noise
    B = chol(R)' * randn(size(H, 1), span + 1);  % Observation noise
    B = B - mean(B, 'all'); 
   
    % Initialize state and observation arrays
    State = zeros(size(F, 1), span + 1);
    Obs = zeros(size(H, 1), span + 1);

    % Initial state and observation
    State(:, 1) = x0;
    Obs(:, 1) = H * x0 + B(:, 1);

    % Simulate state evolution and observations
    for i = 1:span
        State(:, i + 1) = F * State(:, i) + G * U(:, i + 1);  % State update
        Obs(:, i + 1) = H * State(:, i + 1) + B(:, i + 1);    % Observation update
    end
end

%% TCC LMI
% This code takes the closed loop model and determines
% via an iterative LMI optimization procedure the values our
% controller gains (K_i and K_p) should have.
%
% VERSION HISTORY
% v1.0 - Initial (incomplete) formulation
% v2.0 - Fixed formulation

%% Input
it_max = 50; % # of optimization attempts
min_p = 0;

%% Initialization

% Dimensions
nx = 3; % State vector
ny = 1; % Output vector
nu = 1; % Control input vector

nn = nx + ny; % Augmented state vector

% Pharmacokinetic model
gamma_mb = 0.095;
gamma_bo = 0.095;
gamma_ob = [0.037, 0.04];
gamma_bm = 0.03;
mu_m = [0.1, 0.2];
mu_o = [0.1, 0.15];
mu_b = 0.2;

% Number of vertices
N = length(gamma_ob) * length(mu_m) * length(mu_o);

% Open Loop Variables
% We iterate as to create the N different A(a_k) matrizes.
% Since I already know the number of vertices I have, I simply run it.
A_vertices = {};
it_A = 1;

for it_g = gamma_ob
    for it_m= mu_m
        for it_o = mu_o
            A_temp = [1-gamma_mb-it_m, gamma_bm, 0;
                 gamma_mb, 1-gamma_bo-gamma_bm-mu_b, it_g;
                 0, gamma_bo, 1-it_g-it_o];
            A_vertices{it_A} = A_temp;
            it_A = it_A + 1;
        end
    end
end
% Create a rolmip variable with N vertices and 1 degree
% -> This is a polytopic matrix
A = rolmipvar(A_vertices, 'A', N, 1);

% Input Matrix
D_m = 0.01; % Should I use D_m = 100?
B = [D_m;
     0;
     0];

% Output Matrix
c_m = 0;
c_b = 0;
c_o = 1;
C = [c_m, c_b, c_o];

% Closed Loop Variables
Kp = rolmipvar(nu, ny, 'Kp', 'full', N, 0); % Since its degree 0, its just an sdpvar. Making it rolmip makes my life easier.
Ki = rolmipvar(nu, ny, 'Ki', 'full', N, 0);
Ks = [Kp Ki]; % nu x nn / Is this properly ordered?

% Since A is already defined as a rolmipvar, apparently this command
% runs directly and creates for me ANOTHER rolmipvar.
A_r = [A+B*Kp*C, B*Ki;
          -C, eye(ny)];

%% LMI Algorithm
% Here we implement the algorithm proposed in both the article and the
% conference paper.

% Lyapunov Matrix
P = rolmipvar(nn, nn, 'symmetric', N, 1);

% Relevant Variables
% 
% Notice their dimensions
% Y (nn x 3*nn)
% X (3*nn x nn)
% M (3*nn x 3*nn)

% Arbitrary matrix acquired from applying Finsler
X_1 = rolmipvar(nn, nn, 'symmetric', N, 1);
X_2 = rolmipvar(nn, nn, 'symmetric', N, 1);
X_3 = rolmipvar(nn, nn, 'symmetric', N, 1);
X = [X_1;
    X_2; 
    X_3];

% Secondary matrix acquired from applying Finsler
Y_1 = zeros(nn, nn);
Y_2 = eye(nn);
Y_3 = -eye(nn);
Y = [Y_1, Y_2, Y_3];

%Infinite loop
m = 0;
p = sdpvar();
while (m<it_max)% still lacking a condition here (if ending, stop)
    
    % Increase iteration count
    m = m + 1;
    
    % Redefine LMI
    A_p = [A*(1-p)+B*Kp*C, B*Ki;
          (-C)*(1-p), (ny)*(1-p)];
    Q = [-P, zeros(nn,nn), A_p';
        zeros(nn,nn), P, -eye(nn);
        A_p, -eye(nn), zeros(nn,nn)];
    M = X*Y;
    LMIs = [];
    LMIs = [LMIs, P>=0, Q+M+M'<=0];
    
    % Optimize
    options = sdpsettings('solver','sedumi','verbose',0);
    sol = optimize(LMIs, p, options);
    % double(p)    
    % Swap both Y and X once more
    Y = double(X'); % Loop I do every iteration
    
    % Check whether I reached my stop condition
    if(value(p) < min_p)
        m = it_max; % End our loops
    end
end

%double(p)

%% Output
if double(p) < 0
    p_value  = value(p);
    Kp_value = value(Kp);
    Ki_value = value(Ki);

    fprintf('p = %f\n',p_value);

    disp('Kp = '); 
    disp(Kp_value/(1-p_value));

    disp('Ki = ');
    disp(Ki_value/(1-p_value));
else
    disp('Optimization failed.');
    disp(sol.info);
end


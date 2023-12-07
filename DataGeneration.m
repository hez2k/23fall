clc
clear
close all

%% Input parametes

% Number of simualtion steps
duration = 250;                                      % Specify the duration of simulation

% Boundary box specification
B_x = 40;                    % for open boundary case, set this value to inf
                             % for closed boundary case, specify the boundary value, 
B_y = 40;                    % for open boundary case, set this value to inf
                             % for closed boundary case, specify the boundary value,
                  
% Number of targets
num_tar = 3;

% Initial state of the targets
% Specify the intial sates in the following order 
% x position, y position, x velocity, y velcoity
s0 = [ 0  5  0.25 0.5;            % Target 1, x position, y position, x velocity, y velcoity
      15  0  0.5  0.25;           % Target 2, x position, y position, x velocity, y velcoity
      10  0  0.5  0.5;            % Target 3, x position, y position, x velocity, y velcoity
      15 15  0.75 1;              % Target 4, x position, y position, x velocity, y velcoity
       0 10  0.5  1];             % Target 5, x position, y position, x velocity, y velcoity

% Process and Observation noise covariances
dt = 01;                                    % Time Interval 
t = [0:dt:duration];                              
n = length(t);

R = diag([0, 0, 0.01, 0.02]);               % process noise variance
u = [randn(n,1)*sqrt(R(1,1))... 
     randn(n,1)*sqrt(R(2,2))...
     randn(n,1)*sqrt(R(3,3))...
     randn(n,1)*sqrt(R(4,4))];              % process noise, 0 mean Gaussian

Q = [0.2   0;
      0   0.25];                            % measurement noise variance      
w = [randn(n,1)*sqrt(Q(1,1))... 
     randn(n,1)*sqrt(Q(2,2))];              % measurement noise 0 mean Gaussian

%% System model parameters

% Discrete time ss model

% s[n+1] = A*s[n] + x[n] + u[n]    u[n]~N(0,R[n])
% y[n] = C*s[n] + w[n]             w[n]~N(0,Q[n])
A = [1 0 1 0;
     0 1 0 1;
     0 0 1 0;
     0 0 0 1];
C = [1 0 0 0;
     0 1 0 0];

%% Target simulation 

% Simulating all the targets from 1 to num_tar

Store_st_mm = zeros(2, n, num_tar);

for tar = 1:num_tar
    
    s = s0(tar,:)';
    y = zeros(2, n);    
    
    x = zeros(4,n);        % control input
    
    for k=1:(n-1),
        
        s(:,k+1) = A*s(:,k) + x(:,k) + [u(k,:)]';         % with process noise
        y(:,k+1) = C*s(:,k+1) + [w(k+1,:)]';
    
        if s(1,k+1) > B_x

            x(1,k) = -2 * ((s(1,k)+s(3,k))-B_x);
            x(3,k) = -2 *s(3,k);
            s(:,k+1) = A*s(:,k) + x(:,k) + [u(k,:)]';      % with process noise
            y(:,k+1) = C*s(:,k+1) + [w(k+1,:)]';

        end
        
        if s(2,k+1) > B_y  

            x(2,k) = -2 * ((s(2,k)+s(4,k))-B_y);
            x(4,k) = -2 * s(4,k);
            s(:,k+1) = A*s(:,k) + x(:,k) + [u(k,:)]';      % with process noise
            y(:,k+1) = C*s(:,k+1) + [w(k+1,:)]';

        end
    
        if s(1,k+1) < 0

            x(1,k) = -2 * ((s(1,k)+s(3,k)));
            x(3,k) = -2 * s(3,k);
            s(:,k+1) = A*s(:,k) + x(:,k) + [u(k,:)]';      % with process noise
            y(:,k+1) = C*s(:,k+1) + [w(k+1,:)]';

        end

        if s(2,k+1) < 0

            x(2,k) = -2 * (s(2,k)+s(4,k));
            x(4,k) = -2 * s(4,k);
            s(:,k+1) = A*s(:,k) + x(:,k) + [u(k,:)]';      % with process noise
            y(:,k+1) = C*s(:,k+1) + [w(k+1,:)]';

        end
    
    end

    % Storing the xtrue and z values from different targets
    Store_st_mm(1:4,:, tar) = s;        % First four rows are states
    Store_st_mm(5:6,:, tar) = y;        % Next two rows are the measurements

end

%% Your Kalman Filter starts here



% Your Kalman filter code ends here

%% Plotting

figure(1)
set(gcf, 'position', [800 100 1000 1000])
plot([0, B_x, B_x, 0, 0],[0, 0, B_y, B_y, 0], 'k-', ...
                                'linewidth', 2)
hold on
col = [1,0,0;
       0,1,0;
       0,0,1;
       0,1,1;
       1,0,1;
       1,1,0];

for tar = 1: num_tar
    plot(Store_st_mm(1,:,tar), Store_st_mm(2,:,tar), 'linewidth',2, 'Color',col(tar,:))
    plot(Store_st_mm(5,:,tar), Store_st_mm(6,:,tar), '.' ,'linewidth',2, 'Color',col(tar,:))
end

title('Target trajectory with noisy measurement')
xlabel('x position, [m]')
ylabel('y position, [m]')
legend('Boundary','Target 1 with process noise','Target 1 measurement',...
       'Target 2 with process noise','Target 2 measurement',...
       'Target 3 with process noise','Target 3 measurement',...
       'Target 4 with process noise','Target 4 measurement',...
       'Target 5 with process noise','Target 5 measurement',...
       'Target 6 with process noise','Target 6 measurement',...
       'location','best')
grid on



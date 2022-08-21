%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MP-273: Sliding mode control
% Author: Prof. Dr. Davi Ant�nio dos Santos / davists@ita.br
% Affiliation: Aeronautics Institute of Technology (ITA/Brazil)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description: 2nd order system.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function MAIN(T)
%DSP = ['Exemplo 1.2 com T = ', num2str(T), ', dmax = ', num2str(dmax)];
%disp(DSP)


%% Simulation parameters

tf = 10;                % simulation time in seconds

%% Objects

% Plant object

sPlant.x = -1;          % initial state
sPlant.v = -1;          % initial speed
sPlant.t = 0;          % continuous time
sPlant.L = 1;        % diturbance magnitude
sPlant.f = 3;        % disturbance frequency
sPlant.T = T;          % sampling time and integration step

oPlant = cPlant( sPlant );




% Regulator object


sRegulator.T  = T;
sRegulator.fc = 3;

oRegulator = cRegulator( sRegulator );




%% Time Loop

x(1)     = oPlant.x; 
xp(1)    = oPlant.v;
u(1)     = 0;
d(1)     = 0;




for k=1:tf/T

    %% Control law
    
    oRegulator.t  = oPlant.t;
    oRegulator.x  = oPlant.x;
    oRegulator.v  = oPlant.v;
    oRegulator    = switchingcontrol( oRegulator );
    %oRegulator    = lpf( oRegulator );
    
    
    %% Plant simulation
    
    oPlant.u = oRegulator.u;
    oPlant   = disturbance( oPlant );
    oPlant   = propagation( oPlant );
    
    
    %% Store variables to exhibit
    
    x(k+1)     = oPlant.x;
    xp(k+1)    = oPlant.v;
    u(k)       = oPlant.u;
    d(k)       = oPlant.d;
    de(k)      = oRegulator.de;
   


end


%% Plots
% Control input

figure('Renderer', 'painters', 'Position', [10 10 450 300]); hold on; grid; box;
plot( (1:k)*T, u,'LineWidth',1.0 );
set(gca, 'FontSize',18);
xlabel('time (s)','interpreter','latex','FontSize',20);
ylabel('$u$','interpreter','latex','FontSize',20);




% State

figure('Renderer', 'painters', 'Position', [10 10 450 300]); hold on; grid; box;
plot( (1:k+1)*T, x,'b','LineWidth',1.0 );
plot( (1:k+1)*T, xp, 'r', 'LineWidth',1.0 );
set(gca, 'FontSize',18);
xlabel('time (s)','interpreter','latex','FontSize',20);
legend('$x$','$\dot{x}$','interpreter','latex','FontSize',20);



figure('Renderer', 'painters', 'Position', [10 10 450 300]); hold on; grid; box;
plot(x, xp, 'b');
set(gca, 'FontSize',18);
xlabel('$x$','interpreter','latex','FontSize',20);
ylabel('$\dot{x}$','interpreter','latex','FontSize',20);

end








%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MP-273: Sliding mode control
% Author: Prof. Dr. Davi Ant�nio dos Santos / davists@ita.br
% Affiliation: Aeronautics Institute of Technology (ITA/Brazil)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description: 2nd order system.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function MAIN(T,obs)
%DSP = ['Exemplo 1.2 com T = ', num2str(T), ', dmax = ', num2str(dmax)];
%disp(DSP)


%% Simulation parameters

tf = 10;                % simulation time in seconds

%% Objects

% Plant object

sPlant.x = 0.1;          % initial state
sPlant.xp = 0.1;          % initial speed
sPlant.t = 0;          % continuous time
sPlant.L = 0.5;        % diturbance magnitude
sPlant.f = 2;        % disturbance frequency
sPlant.T = T;          % sampling time and integration step
sPlant.m = 1;
sPlant.b = 0.1;
sPlant.k0 = 0.05;
sPlant.k1 = 0.05;

oPlant = cPlant( sPlant );

% Observer object


sObserver.T  = T;

sObserver.xHat = 0;
sObserver.xHatp = 0;

sObserver.m = sPlant.m;
sObserver.b = sPlant.b;
sObserver.k0 = sPlant.k0;
sObserver.k1 = sPlant.k1;
            
sObserver.a1 = 0.2;
sObserver.a2 = 1;
sObserver.kappa1 = 1;
sObserver.kappa2 = 10;   
sObserver.c1 = 0.54;
sObserver.c2 = 0.66;

oObserver = cObserver( sObserver );
  


%% Time Loop

x(1)     = oPlant.x; 
xp(1)    = oPlant.xp;
y(1)     = oObserver.xHat;
yp(1)    = oObserver.xHatp;

for k=1:tf/T

    %% Control law
    oObserver.t  = oPlant.t;
    oObserver.x  = oPlant.x;
    
    if obs == 1
        oObserver    = slotine( oObserver );
    elseif obs == 2
        oObserver    = superTwisting( oObserver );
    end
    
    %% Plant simulation
    
    oPlant   = disturbance( oPlant );
    oPlant   = propagation( oPlant );
    
    
    %% Store variables to exhibit
    
    x(k+1)     = oPlant.x;
    xp(k+1)    = oPlant.xp;
    y(k+1)       = oObserver.xHat;
    yp(k+1)       = oObserver.xHatp;

end


%% Plots

cd '..'
cd 'imagens'

figure;
t = 0:T:tf;
plot(t, x - y)
hold on;
plot(t, xp - yp)
set(gca,'TickLabelInterpreter','latex','XLim',[0,tf]);
ylabel('Erro', 'FontSize', 12,'Interpreter','latex');
xlabel('Tempo $t$ $[s]$','Interpreter','latex', 'FontSize', 12);
legend('$\tilde{x}_1$', '$\tilde{x}_2$', 'FontSize', 12, 'Interpreter', 'latex'); 
grid on;
if obs == 1
    print(gcf,['error_T_slotine_',mat2str(T),'.png'],'-dpng','-r500');
elseif obs == 2
    print(gcf,['error_T_st_',mat2str(T),'.png'],'-dpng','-r500');
end

figure;
plot(t,x)
hold on;
plot(t,y)
set(gca,'TickLabelInterpreter','latex','XLim',[0,tf]);
ylabel('$y$, $[m]$', 'FontSize', 12,'Interpreter','latex');
xlabel('Tempo $t$ $[s]$','Interpreter','latex', 'FontSize', 12);
legend('${x}_1$', '$\hat{x}_1$', 'FontSize', 12, 'Interpreter', 'latex'); 
grid on;
if obs == 1
    print(gcf,['x1_T_slotine_',mat2str(T),'.png'],'-dpng','-r500'); 
elseif obs == 2
    print(gcf,['x1_T_st_',mat2str(T),'.png'],'-dpng','-r500'); 
end


figure;
plot(t,xp)
hold on;
plot(t,yp)
set(gca,'TickLabelInterpreter','latex','XLim',[0,tf]);
ylabel('$\dot{y}$', 'FontSize', 12,'Interpreter','latex');
xlabel('Tempo $t$ $[s]$','Interpreter','latex', 'FontSize', 12);
legend('${x}_2$', '$\hat{x}_2$', 'FontSize', 12, 'Interpreter', 'latex'); 
grid on;
if obs == 1
    print(gcf,['x2_T_slotine_',mat2str(T),'.png'],'-dpng','-r500'); 
elseif obs == 2
    print(gcf,['x2_T_st_',mat2str(T),'.png'],'-dpng','-r500'); 
end

cd '..'
cd 'matlab'

end








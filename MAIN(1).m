clear all;
close all;
clc;


%% Simulation parameters

T = 10^(-3);
tf = 16;                % simulation time in seconds

%% Objects

% Plant object

sPlant.x1 = zeros(3,1);          % initial state
sPlant.x2 = zeros(3,1);          % initial speed
sPlant.t = 0;          % continuous time
sPlant.L = [0.3; 0.4; 0.5];        % diturbance magnitude
sPlant.f = [1/8; 1/8; 1/8];        % disturbance frequency
sPlant.phi = [0; pi/2; pi];        % disturbance phases
sPlant.T = T;          % sampling time and integration step
sPlant.ddRef = [ - (pi/2) ^ 2 * sin( pi * 0 / 2);...
                 - (pi/2) ^ 2 * sin( pi * 0 / 2 + pi / 2);...
                 0];

oPlant = mDimensionalPlant( sPlant );

% Regulator object


sRegulator.T  = T;
sRegulator.L = sPlant.L;
sRegulator.ddRef = sPlant.ddRef;
sRegulator.kappa = 0.2;
sRegulator.gamma = 1;

oRegulator = mDimensionalRegulator( sRegulator );




%% Time Loop

x1 = cell(tf/T+1,1);
x2 = cell(tf/T+1,1);
x1bar = cell(tf/T+1,1);
x2bar = cell(tf/T+1,1);
u  = cell(tf/T+1,1);
d  = cell(tf/T+1,1);
x1ref = cell(tf/T+1,1);
x2ref = cell(tf/T+1,1);
kappa = cell(tf/T+1,1);

x1{1}     = oPlant.x1; 
x2{1}    = oPlant.x2;
x1bar{1}     = oPlant.x1; 
x2bar{1}    = oPlant.x2;
u{1}     = zeros(3,1);
d{1}     = zeros(3,1);
x1ref{1}     = [0; 1; 0];
x2ref{1}     = [ pi/2; 0; 1/4];
kappa{1}  = sRegulator.kappa;

%%
for k=1:tf/T

    %% Ref
    Ref = [ sin( pi * oPlant.t / 2);...
            sin( pi * oPlant.t / 2 + pi / 2);...
            oPlant.t / 4];
        
    dRef = [ pi/2 * cos( pi * oPlant.t / 2); ...
             pi/2 * cos( pi * oPlant.t / 2 + pi / 2);...
             1 / 4];

    ddRef = [ - (pi/2) ^ 2 * sin( pi * oPlant.t / 2);...
                 - (pi/2) ^ 2 * sin( pi * oPlant.t / 2 + pi / 2);...
                 0];

    
    %% Control law
    oRegulator.t  = oPlant.t;
    oRegulator.x1  = oPlant.x1;
    oRegulator.x2  = oPlant.x2;
    oRegulator.ddRef = ddRef;
    oRegulator    = switchingcontrol( oRegulator );
    %oRegulator    = lpf( oRegulator );
    
    
    %% Plant simulation
    
    oPlant.u = oRegulator.u;
    oRegulator.u;
    oPlant.ddRef = oRegulator.ddRef;
    oPlant   = disturbance( oPlant );
    oPlant   = propagation( oPlant );
    
    
    %% Store variables to exhibit
    
    
    x1{k+1}     = oPlant.x1 + Ref;
    x2{k+1}    = oPlant.x2 + dRef;
    x1bar{k+1}     = oPlant.x1;
    x2bar{k+1}    = oPlant.x2;
    u{k+1}       = oPlant.u;
    d{k+1}       = oPlant.d;
    x1ref{k+1} = Ref;
    x2ref{k+1} = dRef;
    kappa{k+1} = oRegulator.kappa;

end

%% Plots
% Control input

uF = cell2mat(u')';
u1 = uF(:,1);
u2 = uF(:,2);
u3 = uF(:,3);

x1F = cell2mat(x1')';
x11 = x1F(:,1);
x12 = x1F(:,2);
x13 = x1F(:,3);

x2F = cell2mat(x2')';
x21 = x2F(:,1);
x22 = x2F(:,2);
x23 = x2F(:,3);

x1barF = cell2mat(x1bar')';
x11bar = x1barF(:,1);
x12bar = x1barF(:,2);
x13bar = x1barF(:,3);

x2barF = cell2mat(x2bar')';
x21bar = x2barF(:,1);
x22bar = x2barF(:,2);
x23bar = x2barF(:,3);
% C = eye(3);

x1refF = cell2mat(x1ref')';
x11ref = x1refF(:,1);
x12ref = x1refF(:,2);
x13ref = x1refF(:,3);

x2refF = cell2mat(x2ref')';
x21ref = x2refF(:,1);
x22ref = x2refF(:,2);
x23ref = x2refF(:,3);

s1 = x11bar + x21bar - exp(-(0:T:tf))';
s2 = x12bar + x22bar - exp(-(0:T:tf))';
s3 = x13bar + x23bar - exp(-(0:T:tf))';

kappa = cell2mat(kappa')';

%% X por T

fig = figure;
subplot(3,1,1);
plot(0:T:tf, x11)
hold on
plot(0:T:tf, x11ref)
set(gca,'TickLabelInterpreter','latex','XLim',[0,tf]);
ylabel('$x_1^{(1)}$ $[m]$', 'FontSize', 12,'Interpreter','latex');
legend('$r_1^{(1)}$', '$\bar{r}_1^{(1)}$', 'FontSize', 12, 'Interpreter', 'latex'); 
grid on;


subplot(3,1,2);
plot(0:T:tf, x12)
hold on
plot(0:T:tf, x12ref)
set(gca,'TickLabelInterpreter','latex','XLim',[0,tf]);
ylabel('$x_1^{(2)}$ $[m]$', 'FontSize', 12,'Interpreter','latex');
legend('$r_1^{(2)}$', '$\bar{r}_1^{(2)}$', 'FontSize', 12, 'Interpreter', 'latex'); 
grid on;


subplot(3,1,3);
plot(0:T:tf, x13)
hold on
plot(0:T:tf, x13ref)
set(gca,'TickLabelInterpreter','latex','XLim',[0,tf]);
ylabel('$x_1^{(3)}$ $[m]$', 'FontSize', 12,'Interpreter','latex');
legend('$r_1^{(3)}$', '$\bar{r}_1^{(3)}$', 'FontSize', 12, 'Interpreter', 'latex'); 
grid on;


han=axes(fig,'visible','off'); 
han.XLabel.Visible='on';
han.YLabel.Visible='on';
xlabel('Tempo $t$ $[s]$','Interpreter','latex', 'FontSize', 12);

        print(gcf,['x1.png'],'-dpng','-r500'); 
%% X2 por T

fig = figure;
subplot(3,1,1);
plot(0:T:tf, x21)
hold on
plot(0:T:tf, x21ref)
set(gca,'TickLabelInterpreter','latex','XLim',[0,tf]);
ylabel('$r_2^{(1)}$ $[m]$', 'FontSize', 12,'Interpreter','latex');
legend('$r_2^{(1)}$', '$\bar{r}_2^{(1)}$', 'FontSize', 12, 'Interpreter', 'latex'); 
grid on;


subplot(3,1,2);
plot(0:T:tf, x22)
hold on
plot(0:T:tf, x22ref)
set(gca,'TickLabelInterpreter','latex','XLim',[0,tf]);
ylabel('$r_2^{(2)}$ $[m]$', 'FontSize', 12,'Interpreter','latex');
legend('$r_2^{(2)}$', '$\bar{r}_2^{(2)}$', 'FontSize', 12, 'Interpreter', 'latex'); 
grid on;


subplot(3,1,3);
plot(0:T:tf, x23)
hold on
plot(0:T:tf, x23ref)
set(gca,'TickLabelInterpreter','latex','XLim',[0,tf]);
ylabel('$r_2^{(3)}$ $[m]$', 'FontSize', 12,'Interpreter','latex');
legend('$r_2^{(3)}$', '$\bar{r}_2^{(3)}$', 'FontSize', 12, 'Interpreter', 'latex'); 
grid on;


han=axes(fig,'visible','off'); 
han.XLabel.Visible='on';
han.YLabel.Visible='on';
xlabel('Tempo $t$ $[s]$','Interpreter','latex', 'FontSize', 12);
        print(gcf,['x2.png'],'-dpng','-r500'); 
%% U por T

fig = figure;
subplot(3,1,1);
plot(0:T:tf, u1)
set(gca,'TickLabelInterpreter','latex','XLim',[0,tf]);
ylabel('$u^{(1)}$', 'FontSize', 12,'Interpreter','latex');
grid on;


subplot(3,1,2);
plot(0:T:tf, u2)
set(gca,'TickLabelInterpreter','latex','XLim',[0,tf]);
ylabel('$u^{(2)}$', 'FontSize', 12,'Interpreter','latex');
grid on;


subplot(3,1,3);
plot(0:T:tf, u3)
set(gca,'TickLabelInterpreter','latex','XLim',[0,tf]);
ylabel('$u^{(3)}$', 'FontSize', 12,'Interpreter','latex');
grid on;


han=axes(fig,'visible','off'); 
han.XLabel.Visible='on';
han.YLabel.Visible='on';
xlabel('Tempo $t$ $[s]$','Interpreter','latex', 'FontSize', 12);

        print(gcf,['u.png'],'-dpng','-r500'); 
%% S por T

fig = figure;
%subplot(3,1,1);

sMod = sqrt(s1.^2 + s2.^2 + s3.^2);

plot(0:T:tf, sMod)
set(gca,'TickLabelInterpreter','latex','XLim',[0,tf]);
ylabel('$||s||$', 'FontSize', 12,'Interpreter','latex');
grid on;


%subplot(3,1,2);
%plot(0:T:tf, s2)
%set(gca,'TickLabelInterpreter','latex','XLim',[0,tf]);
%ylabel('$s^{(2)}$', 'FontSize', 12,'Interpreter','latex');
%grid on;


%subplot(3,1,3);
%plot(0:T:tf, s3)
%set(gca,'TickLabelInterpreter','latex','XLim',[0,tf]);
%ylabel('$s^{(3)}$', 'FontSize', 12,'Interpreter','latex');
%grid on;


han=axes(fig,'visible','off'); 
han.XLabel.Visible='on';
han.YLabel.Visible='on';
xlabel('Tempo $t$ $[s]$','Interpreter','latex', 'FontSize', 12);

        print(gcf,['s.png'],'-dpng','-r500'); 
        
        %% KAPPA por T
figure;

plot(0:T:tf, kappa)
set(gca,'TickLabelInterpreter','latex','XLim',[0,tf]);
ylabel('$\kappa$', 'FontSize', 12,'Interpreter','latex');
grid on;
xlabel('Tempo $t$ $[s]$','Interpreter','latex', 'FontSize', 12);
print(gcf,['kappa.png'],'-dpng','-r500'); 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MP-273: Sliding mode control
% Autor: Reynaldo Santos de Lima
% Baseado em trabalho do professor
%           Prof. Dr. Davi Ant�nio dos Santos / davists@ita.br

classdef mDimensionalRegulator
    %MDIMENSIONALREGULATOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % Signals
        
        x1      % x = (x1,x2)
        x2      % 
        u       % control input
        t       % continuous time
        ddRef   % aceleração da referência comandada
        
        % Parameters
        
        L       % disturbance bounds
        T       % sampling time
        kappa
        gamma
        
    end
    
    methods
        function obj = mDimensionalRegulator( sRegulator )
            
            % Parameters
            
            obj.T  = sRegulator.T; 
            obj.L = sRegulator.L;
            obj.ddRef = sRegulator.ddRef;
            obj.kappa = sRegulator.kappa;
            obj.gamma = sRegulator.gamma;
                       
        end
        
        function obj = switchingcontrol( obj )
            
            % Do sistema em particular
            g = 9.81;
            m = 1;
            I3 = eye(3);
            e3 = I3(:,3);
            
            % Control law
            
            C = eye(3);
            P = eye(3) * exp( - obj.t );
            
            epsilon = 10^(-2);
            s = C * obj.x1 + obj.x2;        
            
            if norm(s) < epsilon
                id_eps = 0;
            else
                id_eps = norm(s);
            end
            
            obj.kappa = obj.kappa + obj.T * obj.gamma * norm(s) * ( id_eps ) ;
            df1dx2B = 1 / m; % norma
            
            if s == [0; 0; 0]
               a = obj.kappa .* [1; 1; 1];
            else
               a = obj.kappa .* s ./ norm(s); 
            end
            
            
            obj.u = - 1 / df1dx2B .* ( a / m + C * obj.x2 ...
                                      - obj.ddRef - g * e3 );
                     
        end
                       
    end
end


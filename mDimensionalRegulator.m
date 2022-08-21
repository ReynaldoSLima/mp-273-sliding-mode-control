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
        
    end
    
    methods
        function obj = mDimensionalRegulator( sRegulator )
            
            % Parameters
            
            obj.T  = sRegulator.T; 
            obj.L = sRegulator.L;
            obj.ddRef = sRegulator.ddRef;
                       
        end
        
        function obj = switchingcontrol( obj )
            
            % Do sistema em particular
            g = 9.81;
            m = 1;
            I3 = eye(3);
            e3 = I3(:,3);
            
            % Control law
            
            C = eye(3);
            kappa = 1.2 * sqrt(3) * max(obj.L);
            df1dx2B = 1 / m; % norma
            s = C * obj.x1 + obj.x2;        
            
            if s == [0; 0; 0]
               a = kappa .* [1; 1; 1];
            else
               a = kappa .* s ./ norm(s); 
            end
            
            
            obj.u = - 1 / df1dx2B .* ( a + C * obj.x2 ...
                                      - obj.ddRef - g * e3 );
                     
        end
                       
    end
end


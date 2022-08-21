%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MP-273: Sliding mode control
% Autor: Reynaldo Santos de Lima
% Baseado em trabalho do professor
%           Prof. Dr. Davi Ant�nio dos Santos / davists@ita.br


classdef mDimensionalPlant
    
    properties
        % Signals
        
        x1           % x = (x1, x2)
        x2           % 
        u            % input
        d            % disturbance
        t            % continuous time
        ddRef        % aceleração da referência comandada
        
        % Parameters
                
        L            % disturbance magnitude
        f            % disturbance frequency
        phi          % disturbance phase
        T            % sampling time/ integration step
    end
    
    methods
        function obj = mDimensionalPlant( sPlant )
            
            
            % Parameters
            
            obj.x1 = sPlant.x1;
            obj.x2 = sPlant.x2;
            obj.t = sPlant.t;
            obj.L = sPlant.L;
            obj.f = sPlant.f;
            obj.phi = sPlant.phi;
            obj.T = sPlant.T;
            obj.ddRef = sPlant.ddRef;
            obj.u = zeros(length(sPlant.x1),1);
            
            % Initializations
            
            obj.d = zeros(length(sPlant.x1),1);
            obj.t = 0;
            
        end
        
        %% Disturbance generator
        
        function obj = disturbance( obj )
            
            for i = 1:length(obj.d)
                obj.d(i) = obj.L(i) * sin( 2 * pi * obj.f(i) * obj.t ...
                                         + obj.phi(i) );
            end
                        
        end
        
        
        %% One-step-ahead simulator
        
        function obj = propagation( obj )
            
               % Auxiliares para planta, particular para cada sistema
               
               m = 1;
               g = 9.81;
               I3 = eye(3);
               e3 = I3(:,3);
               
               % Euler:
                          
               obj.x1 = obj.x1 + obj.T*( obj.x2 );
               
               %g * e3 
               %obj.ddRef
               %obj.u 
               %obj.d
               
               obj.x2 = obj.x2 + obj.T*( - g .* e3 - obj.ddRef  +...
                                            1 / m * ( obj.u + obj.d ) );               
               obj.t = obj.t + obj.T;
                    
        end
    end
end


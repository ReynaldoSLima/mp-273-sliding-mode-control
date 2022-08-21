%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MP-273: Sliding mode control
% Author: Prof. Dr. Davi Ant�nio dos Santos / davists@ita.br
% Affiliation: Aeronautics Institute of Technology (ITA/Brazil)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description: 1st order system
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Class implementation: cPlant - It simulates the plant.
classdef cPlant
    
    properties
        
        % Signals
        
        x            % state
        v            % speed, auxiliar
        u            % input
        d            % disturbance
        t            % continuous time
        
        % Parameters
                
        L            % disturbance magnitude
        f            % disturbance frequency
        T            % sampling time/ integration step
       
        
    
    end
    
    
    methods
        
        function obj = cPlant( sPlant )
            
            % Parameters
            
            obj.x = sPlant.x;
            obj.v = sPlant.v;
            obj.t = sPlant.t;
            obj.L = sPlant.L;
            obj.f = sPlant.f;
            obj.T = sPlant.T;
            
            
            % Initializations
            
            obj.d = 0;
            obj.t = 0;
           
            
        end
        
        
        %% Disturbance generator
        
        function obj = disturbance( obj )
            
            obj.d = obj.L*sin( 2*pi*obj.f*obj.t );
            
        end
        
        
        %% One-step-ahead simulator
        
        function obj = propagation( obj )
                       
               % Euler 1
               % v = x', v' = u + d
                    
               obj.x = obj.x + obj.T*( obj.v );
               
               obj.v = obj.v + obj.T*( obj.x * obj.x + (abs(obj.x) + abs(obj.v) + 1) * obj.u + obj.d );
               
               obj.t = obj.t + obj.T;
                    
        end
           
        
    end
    
end


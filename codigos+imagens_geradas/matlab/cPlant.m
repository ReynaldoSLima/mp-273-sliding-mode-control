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
        xp            % speed, auxiliar
        u            % input
        w
        d            % disturbance
        t            % continuous time
        
        % Parameters
                
        m
        b
        k0
        k1
        L            % disturbance magnitude
        f            % disturbance frequency
        T            % sampling time/ integration step
       
        
    
    end
    
    
    methods
        
        function obj = cPlant( sPlant )
            
            % Parameters
            
            obj.x = sPlant.x;
            obj.xp = sPlant.xp;
            obj.t = sPlant.t;
            obj.L = sPlant.L;
            obj.f = sPlant.f;
            obj.T = sPlant.T;
            obj.m = sPlant.m;
            obj.b = sPlant.b;
            obj.k0 = sPlant.k0;
            obj.k1 = sPlant.k1;
            obj.w = 0;
            
            
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
               % xp = x', xp' = u + d
                        
               obj.x = obj.x + obj.T*( obj.xp );    
               
               obj.xp = obj.xp + obj.T*( -obj.b/obj.m*obj.xp*abs(obj.xp)...
                                         - obj.k0/obj.m*obj.x - ...
                                           obj.k1/obj.m*(obj.x)^3 + obj.d/obj.m );
               
               obj.t = obj.t + obj.T;
                    
        end
           
        
    end
    
end


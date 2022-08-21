%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MP-273: Sliding mode control
% Author: Prof. Dr. Davi Ant�nio dos Santos / davists@ita.br
% Affiliation: Aeronautics Institute of Technology (ITA/Brazil)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description: 1st order system
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Class implementation: cRegulator - It implements some regulator con-
% trollers.
classdef cRegulator
    
    properties
        
        % Signals
        
        x       % state
        v       % speed
        u       % control input
        t       % continuous time
        de      % estimate of d
        
        % Parameters
        
        L       % disturbance bounds
        T       % sampling time
        fc      % cut-off frequency of the LPF
        tau     % time constant of the LPF
        
        
    end
    
    methods
        
        
        %% Class constructor
        
        function obj = cRegulator( sRegulator )
        
            % Parameters
            
            obj.T  = sRegulator.T; 
            obj.fc = sRegulator.fc;

          
            % Pre-computations and initializations
            
            obj.de  = 0;
            obj.tau = 1/(2*pi*obj.fc);
          
            
            
            
        end
    
        
      
        
        
        %% Switching control law
        
        function obj = switchingcontrol( obj )
            
            % Control law
                   
            obj.u = -sign( obj.x + obj.v ) - obj.v;
            
         
            
        end
        
        
         %% Switching control law
        
        function obj = lpf( obj )
            
            % Estimation of d by LP filtering -u
                   
            obj.de = obj.de + obj.T*( -1/obj.tau*obj.de + ...
                                     1/obj.tau*(-obj.u));
            
         
            
        end
       
        
      
        
      
        
        
        
    end
    
    
end


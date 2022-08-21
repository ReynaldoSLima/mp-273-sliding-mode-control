%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MP-273: Sliding mode control
% Author: Prof. Dr. Davi Ant�nio dos Santos / davists@ita.br
% Affiliation: Aeronautics Institute of Technology (ITA/Brazil)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description: 1st order system
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Class implementation: cRegulator - It implements some regulator con-
% trollers.
classdef cObserver
    
    properties
        
        % Signals
        
        x
        xHat       % state
        xHatp       % speed
        u       % control input
        t       % continuous time
        de      % estimate of d
        
        % Parameters
            
        m
        b
        k0
        k1
        
        a1
        a2
        kappa1
        kappa2
        c1
        c2
     
        
        L       % disturbance bounds
        T       % sampling time
        fc      % cut-off frequency of the LPF
        tau     % time constant of the LPF
        
        
    end
    
    methods
        
        
        %% Class constructor
        
        function obj = cObserver( sObserver )
        
            % Parameters
            
            obj.T  = sObserver.T; 
            
            obj.xHat = sObserver.xHat;
            obj.xHatp = sObserver.xHatp;
                  
            obj.m = sObserver.m;
            obj.b = sObserver.b;
            obj.k0 = sObserver.k0;
            obj.k1 = sObserver.k1;
            
            obj.a1 = sObserver.a1;
            obj.a2 = sObserver.a2;
            obj.kappa1 = sObserver.kappa1;
            obj.kappa2 = sObserver.kappa2;        
            
            obj.c1 = sObserver.c1;
            obj.c2 = sObserver.c2;
            
        end
    
        
      
        
        
        %% Slotine observer
        
        function obj = slotine( obj )
            
            
            xError = obj.x - obj.xHat; % só x é mensurável
            fXisHat = - obj.b/obj.m * obj.xHatp * abs(obj.xHatp) ...
                      - obj.k0/obj.m * obj.xHat - obj.k1/obj.m*(obj.xHat)^3;  
            obj.xHat = obj.xHat + obj.T * (- obj.a1 * xError +...
                                                obj.xHatp +...
                                                obj.kappa1*sign(xError));
            obj.xHatp = obj.xHatp + obj.T * (- obj.a2 *xError + fXisHat ...
                                                + obj.kappa2 * sign(xError));
            
            
        end
        
        function obj = superTwisting( obj )
            
            xError = obj.x - obj.xHat; % só x é mensurável
            fXisHat = - obj.b/obj.m * obj.xHatp * abs(obj.xHatp) ...
                      - obj.k0/obj.m * obj.xHat - obj.k1/obj.m*(obj.xHat)^3;  
            
            obj.xHat = obj.xHat + obj.T * ( obj.xHatp +...
                                    obj.c1*sqrt(abs(xError))*sign(xError));
            obj.xHatp = obj.xHatp + obj.T * ( fXisHat ...
                                            + obj.c2 * sign(xError));
            
        end
        
    end
    
    
end


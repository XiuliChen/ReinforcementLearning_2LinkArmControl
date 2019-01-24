function [Theta1,Theta2,dTheta1,dTheta2]=getTheta(Theta1t,Theta2t,dTheta1t,dTheta2t,tau1,tau2)
% this function takes shoulder angle and velocity and torque applied
% (Theta1,dTheta1, tau1), and elbow angle and velocity and torque applied
% (Theta2,dTheta2, tau2).

% the function outputs new shoulder angle and velocity, and new elbow angle and velocity
% rounded into 2 decimal.

global K1 K2 K3 K4 dt
    % Equations solved for angles; 
    % mutau added joint friction 
    C2 = cos(Theta2t);
    S2 = sin(Theta2t); 
    M11 = (K1 + K2*C2);
    M12 = (K3 + K4*C2); 
    M21 = M12; 
    M22 = K3; 
    H1 = -K2*S2*dTheta1t*dTheta2t - 1/2*K2*S2*dTheta2t^2; 
    H2 = 1/2*K2*S2*dTheta1t^2; 

    ddTheta2 = (H2*M11 - H1*M21 - M11*tau2 + M21*tau1) / (M12^2 - M11*M22); 
    ddTheta1 = (-H2 + tau2 - M22*ddTheta2) / M21; 
    
    dTheta2 = round(dTheta2t+ddTheta2*dt,2); 
    dTheta1 = round(dTheta1t+ddTheta1*dt,2); 
    
    Theta1 = round(Theta1t+dTheta1*dt,2); 
    Theta2 = round(Theta2t+dTheta2*dt,2); 
    

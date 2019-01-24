function [x1,y1,x2,y2]=endEffector(Theta1,Theta2,L1,L2)
    %%Plot arm 
    x1 = L1*cos(Theta1); 
    y1 = L1*sin(Theta1); %xy of elbow 
    
    x2 = x1+L2*cos(Theta2+Theta1); 
    y2 = y1+L2*sin(Theta2+Theta1); %xy of hand 
end
    
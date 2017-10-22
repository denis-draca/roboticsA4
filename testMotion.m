function [] = testMotion( start, robot )

qmatrix = [];
close all;
velocity = [0.1,0.2,-0.4, 0 0 0]';
steps = 100;

time = 1.5;

deltaT = time/steps;

qmatrix(1,:) = start;

W = diag(1:7);


for i = 2:steps 
    J = robot.jacob0(qmatrix(i-1,:));
    
%     J = J(1:3,1:3);
    
%     jV = inv(J) * velocity;
    
    jV = (inv(W)*J')*inv(J*inv(W)*J')*velocity;
    
    qmatrix(i,:) = qmatrix(i-1,:) + (jV*deltaT)';
end

robot.plot(qmatrix, 'trail','-r', 'fps',60)

end


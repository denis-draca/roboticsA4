%% Only VisualServoing
%Will visual servo to the object, move down, pick the object up and move
%using jtraj to a joint position I randomly selected. No torque or torque
%plot will be checked here
clear all;
camObj = performVS(1);

%% Mass = 2kg
% Visual Servo to object, move down, pick up the object, then maintain a
% safe torque (and speed) throughout the process while trying to complete
% the task at the same time as the 5kg scenario
%Three motions once the object is grasped
%   Move up 1 and along x -0.4
%   move along y -3
%   move down 1 and along x 0.4
% Torques will be graphed at the completion of all motions. Seperate figure
% for each motion. Each joint is on seperate subplot. Blue line indicates
% joint torque. Orange Line indicates max joint torque.
clear all;
camObj = performVS(2);

%% Mass = 5kg
% Visual Servo to object, move down, pick up the object, a safe speed will
% be maintained but not a safe torque while trying to complete the task at
%the same time as the 2kg scenario. It will fail while moving along y
%Three motions once the object is grasped
%   Move up 1 and along x -0.4
%   move along y -3
%   move down 1 and along x 0.4
% Torques will be graphed at the completion of all motions. Seperate figure
% for each motion. Each joint is on seperate subplot. Blue line indicates
% joint torque. Orange Line indicates max joint torque.
clear all;
camObj = performVS(3);
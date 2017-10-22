robot = importrobot('fetch.urdf');

homeConfig = homeConfiguration(robot);

transform1 = getTransform(robot,homeConfig,'shoulder_pan_link');
transform2 = getTransform(robot,homeConfig,'shoulder_lift_link');
transform3 = getTransform(robot,homeConfig,'upperarm_roll_link');
transform4 = getTransform(robot,homeConfig,'elbow_flex_link');
transform5 = getTransform(robot,homeConfig,'forearm_roll_link');
transform6 = getTransform(robot,homeConfig,'wrist_flex_link');
transform7 = getTransform(robot,homeConfig,'wrist_roll_link');
transform8 = getTransform(robot,homeConfig,'gripper_link');

transformList = [];

transformList(:,:,1) = transform1;
transformList(:,:,2) = transform2;
transformList(:,:,3) = transform3;
transformList(:,:,4) = transform4;
transformList(:,:,5) = transform5;
transformList(:,:,6) = transform6;
transformList(:,:,7) = transform7;
transformList(:,:,8) = transform8;

distanceList = [];

for i = 1:(length(transformList) - 1)
    differenceTransform = transformList(:,:,i+1) - transformList(:,:,i);
    
    distanceList(1,i) = differenceTransform(1,4);
end

body1 = getBody(robot,'shoulder_pan_link');
body2 = getBody(robot,'shoulder_lift_link');
body3 = getBody(robot,'upperarm_roll_link');
body4 = getBody(robot,'elbow_flex_link');
body5 = getBody(robot,'forearm_roll_link');
body6 = getBody(robot,'wrist_flex_link');
body7 = getBody(robot,'wrist_roll_link');

%% Draw Fetch

close all;

jointLim1 = deg2rad([-92 92]);
jointLim2 = deg2rad([-70 87]);
jointLim3 = [-2*pi 2*pi];
jointLim4 = deg2rad([-129 129]);
jointLim5 = [-2*pi 2*pi];
jointLim6 = deg2rad([-125 125]);
jointLim7 = [-2*pi 2*pi];


l1 = Link('d',0.06,'a',0.1170,'alpha',pi/2, ...
    'offset', 0, 'qlim', jointLim1);
l1.I = body1.Inertia;
l1.r = body1.CenterOfMass;

l2 = Link('d',0,'a',0,'alpha',-pi/2,...
    'offset',-pi/2, 'qlim', jointLim2);
l2.I = body2.Inertia;
% l2.r = body1.CenterOfMass;

l3 = Link('d',0.3520,'a',0,'alpha',pi/2,'offset',0, ...
    'offset', 0, 'qlim', jointLim3);
l3.I = body3.Inertia;

l4 = Link('d',0,'a',0,'alpha',-pi/2,'offset',0, 'qlim', jointLim4);
l4.I = body4.Inertia;


l5 = Link('d',0.3215,'a',0,'alpha',pi/2,'offset',0,...
    'offset', 0, 'qlim', jointLim5);
l5.I = body5.Inertia;


l6 = Link('d',0,'a',0,'alpha',-pi/2,'offset',0, 'qlim', jointLim6);
l6.I = body6.Inertia;


l7 = Link('d',0.3049,'a',0,'alpha',0,'offset',0, 'qlim', jointLim7);
l7.I = body7.Inertia;


links = [l1 l2 l3 l4 l5 l6 l7];

robot = SerialLink(links, 'name', 'test');

robot.plot([0 0 0 0 0 0 0]);


%% RMRC TEST
close all;
qmatrix = [];
velocity = [-0.2,0.2,0.4,0,0,0]';
steps = 100;

time = 1;   

deltaT = time/steps;
qmatrix(1,:) = robot.ikcon(transl(0.702,-0.007,0.384)*rpy2tr(-7.669*180/pi, 6.221*180/pi, 141*180/pi));

c = [10 100 1000 10000 100000 100000 1000000];

% W = diag(1:7);
W = eye(7);
% W = calcW(W, robot,qmatrix(1,:),c);

WList = [];
WList(:,:,1) = W;

j1Weights = [];
j2Weights = [];
j3Weights = [];
j4Weights = [];
j5Weights = [];
j6Weights = [];
j7Weights = [];

j1V = [];
j2V = [];
j3V = [];
j4V = [];
j5V = [];
j6V = [];
j7V = [];


j1Weights(1,1) = W(1,1);
j2Weights(1,1) = W(2,2);
j3Weights(1,1) = W(3,3);
j4Weights(1,1) = W(4,4);
j5Weights(1,1) = W(5,5);
j6Weights(1,1) = W(6,6);
j7Weights(1,1) = W(7,7);

performPlot = 1;
    

for i = 2:steps
    J = robot.jacob0(qmatrix(i-1,:));
    
    jV = (inv(W)*J')*inv(J*inv(W)*J')*velocity;
    
    if(i == 2 || i == 3)
       disp(jV); 
       J
       W
    end
    
    qmatrix(i,:) = qmatrix(i-1,:) + (jV*deltaT)';
    
    W = calcW(W, robot,qmatrix(i-1,:),c);
    WList(:,:,i) = W;
    j1Weights(1,i) = W(1,1);
    j2Weights(1,i) = W(2,2);
    j3Weights(1,i) = W(3,3);
    j4Weights(1,i) = W(4,4);
    j5Weights(1,i) = W(5,5);
    j6Weights(1,i) = W(6,6);
    j7Weights(1,i) = W(7,7);
    
    j1V(1,i-1) = jV(1);
    j2V(1,i-1) = jV(2);
    j3V(1,i-1) = jV(3);
    j4V(1,i-1) = jV(4);
    j5V(1,i-1) = jV(5);
    j6V(1,i-1) = jV(6);
    j7V(1,i-1) = jV(7);
    
    if(performPlot)
        figure(1);
        plot(1:i,j1Weights);
        hold on;
        plot(1:i,j2Weights);
        hold on;
        plot(1:i,j3Weights);
        hold on;
        plot(1:i,j4Weights);
        hold on;
        plot(1:i,j5Weights);
        hold on;
        plot(1:i,j6Weights);
        hold on;
        plot(1:i,j7Weights);
        hold on;

%         drawnow;

        figure(2);
        plot(1:i-1,j1V);
        hold on;
        plot(1:i-1,j2V);
        hold on;
        plot(1:i-1,j3V);
        hold on;
        plot(1:i-1,j4V);
        hold on;
        plot(1:i-1,j5V);
        hold on;
        plot(1:i-1,j6V);
        hold on;
        plot(1:i-1,j7V);
        hold on;
    end
    
end
figure(3)
robot.plot(qmatrix, 'trail','-r', 'fps',60)

%% Draw Random

close all;
% figure(2);

l1 = Link('d',0.06,'a',0.1170,'alpha',pi/2, ...
    'offset', 0);

l2 = Link('offset',pi/2 , 'd',0,'a',0.2190,'alpha',0);

l3 = Link('d',0.1330,'a',0,'alpha',0,'offset',0, ...
    'offset', 0);


links = [l1 l2 l3 ];

robot = SerialLink(links, 'name', 'test2');

robot.plot([0 0 0]);

view([-0.2 -0.2 0.1]);
transforms = linkPoses(robot, [0 0 0 ]);

printPlots(transforms,[2]);

%% VS fetch


function [ robot ] = drawFetch( startPos )
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
l2.r = body2.CenterOfMass + body1.CenterOfMass;

l3 = Link('d',0.3520,'a',0,'alpha',pi/2,'offset',0, ...
    'offset', 0, 'qlim', jointLim3);
l3.I = body3.Inertia;
l3.r = body3.CenterOfMass;

l4 = Link('d',0,'a',0,'alpha',-pi/2,'offset',0, 'qlim', jointLim4);
l4.I = body4.Inertia;
l4.r = body3.CenterOfMass + body4.CenterOfMass;

l5 = Link('d',0.3215,'a',0,'alpha',pi/2,'offset',0,...
    'offset', 0, 'qlim', jointLim5);
l5.I = body5.Inertia;
l5.r = body5.CenterOfMass;

l6 = Link('d',0,'a',0,'alpha',-pi/2,'offset',0, 'qlim', jointLim6);
l6.I = body6.Inertia;
l6.r = body6.CenterOfMass + body5.CenterOfMass;


l7 = Link('d',0.3049,'a',0,'alpha',0,'offset',0, 'qlim', jointLim7);
l7.I = body7.Inertia;
l7.r = body7.CenterOfMass;


links = [l1 l2 l3 l4 l5 l6 l7];

robot = SerialLink(links, 'name', 'test');

robot.plot(startPos);

end


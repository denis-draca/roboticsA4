function [cam] =  performVS(status)
%% Draw Fetch and staircase model
close all;
clc;
% clear;

tau_max = [33.82 131.76 76.94 66.18 29.35 25.70 7.36]';
maxSpeed = [1.25 1.45 1.57 1.52  1.57 2.26 2.26];
startPos = [1.4772    1.0687    0.1256   -0.4053   -0.0001   -2.1380    1.1257];
bot = drawFetch(startPos);

modelLocation = transl(0.45,  0.5, 0);
hold on;
helixModel = PartLoader('helix3.ply', modelLocation);

mod1 = modelLocation * transl(0.15,0,0); 
mod2 = modelLocation * transl(-0.15,0,0);
mod3 = modelLocation * transl(0,0.15,0); 
mod4 = modelLocation * transl(0,-0.15,0);

pS1 = [(512 + 170);512];
pS2 = [(512 - 170);512];
pS3 = [512;(512 - 170)];
pS4 = [512;(512 + 170)];

pStar = [pS2 pS1 pS4  pS3];

% pStemp = pStar;
% 
% pStar(1,:) = pStemp(2,:);
% pStar(2,:) = pStemp(1,:);
% P = [mod1(1:3,4) mod2(1:3,4) mod3(1:3,4) mod4(1:3,4)];

P = [mod1(1:3,4) mod2(1:3,4) mod3(1:3,4) mod4(1:3,4)];

depth = mean (P(1,:));
T1 = bot.fkine(bot.getpos);
depth = norm(T1(1:3,4) - modelLocation(1:3,4));
% depth = [];


%These two lines
% pStar = [512 512]';
% P = modelLocation(1:3,4);


axis([-1.5 1.5 -1.5 1.5 -0.5 1.5])
%% Setup Cam

cam = CentralCamera('focal', 0.08, 'pixel', 10e-5, ...
'resolution', [1024 1024], 'centre', [512 512],'name', 'FetchCam');

% frame rate
fps = 25;

%Define values
%gain of the controler
lambda = 1;

%% Draw inital State

Tc0= bot.fkine(bot.getpos);
cam.T = Tc0;
% P = modelLocation(1:3,4);
% plot_sphere(P, 0.05, 'b')
cam.plot_camera(P, 'label','scale',0.15);

%% Projection
p = cam.plot(P, 'Tcam', Tc0);

%camera view and plotting
cam.clf()
cam.plot(pStar, '*'); % create the camera view
cam.hold(true);
cam.plot(P, 'Tcam', Tc0, 'o'); % create the camera view
pause(2)
cam.hold(true);
cam.plot(P);    % show initial view



%% Loop

ksteps = 0;
W = eye(7);
c = [1 1 1 1 1 1 1];

while true
%     T1 = bot.fkine(bot.getpos);
%     depth = norm(T1(1:3,4) - modelLocation(1:3,4))
%     depth = bot.fkine(bot.getpos);
%     depth = (depth(3,4) + 1) * 1000;
    ksteps = ksteps + 1;
%     depth = [];
    
    % compute the view of the camera
    uv = cam.plot(P);
    
    % compute image plane error as a column
    e = pStar-uv;   % feature error
    e = e(:);
    Zest = [];
    
    % compute the Jacobian
    if isempty(depth)
        % exact depth from simulation (not possible in practice)
        pt = homtrans(inv(cam.T), P);
        J = cam.visjac_p(uv, pt(3,:) );
    elseif ~isempty(Zest)
        J = cam.visjac_p(uv, Zest);
    else
%         uv
%         depth
        J = cam.visjac_p(uv, depth );
    end
    
    % compute the velocity of camera in camera frame
    try
        v = lambda * pinv(J) * e;
    catch
        status = -1;
        return
    end
%     fprintf('v: %.3f %.3f %.3f %.3f %.3f %.3f\n', v);
    
    
    if(length(pStar(1,:)) == 1)
        v(1) = -v(1);
        v(2) = v(2);
        v(3) = v(3);
        v(4) = -v(4);
        v(5) = v(5);
        v(6) = v(6);
    else

        v(1) = -v(1);
        v(2) = v(2);
        v(3) = -v(3);
        v(4) = -v(4);
        v(5) = v(5);
        v(6) = -v(6);
    end
    
    
    J2 = bot.jacob0(bot.getpos);
    
    jV = (inv(W)*J2')*inv(J2*inv(W)*J2')*v;
    
    for z = 1:length(jV)
        if(jV(z) > maxSpeed(z))
            ratio = maxSpeed(z)/jV(z);
            jV = jV*ratio;
        elseif (jV(z) < -maxSpeed(z))
            ratio = -maxSpeed(z)/jV(z);
            jV = jV*ratio;
        end
    end
    
    qp = jV;
    q0 = bot.getpos;
%     qp = bot.getPos + (jV*deltaT)';
    
    W = calcW(W, bot,bot.getpos,c);
    
    %Update joints
    q = q0' + (1/fps)*qp;
   	bot.animate(q');
    
    
    %Get camera location
    Tc = bot.fkine(q);
    cam.T = Tc;
    drawnow

    pause(1/fps)
    
%     if ~isempty(200) && (ksteps > 200)
%         break;
%     end

    test = (abs(e) < 3);
    if(test)
        e
        test
        break
    end
    
    %update current joint position
%     q0 = q;
end %loop finishese

%% Move along Z
% pause;
%Camera is no longer needed, lets just move it away
cam.T = transl(0,0,10);
drawnow;


height = bot.fkine(bot.getpos);
height = (height(3,4));
W = eye(7);
c = [1 1 1 1 1 1 1];
fps = 25;
while(height ~= 0)
    distance = bot.fkine(bot.getpos) - modelLocation;
    
    height = bot.fkine(bot.getpos);
    height = (height(3,4));
    
    test = norm(distance(1:3,4));
    if(test < 0.09)
        break;
    end
    
    velocity = [0 0 -1 0 0 0]';
    
     J2 = bot.jacob0(bot.getpos);
    
    jV = (inv(W)*J2')*inv(J2*inv(W)*J2')*velocity;
    
    
    for z = 1:length(jV)
        if(jV(z) > maxSpeed(z))
            ratio = maxSpeed(z)/jV(z);
            jV = jV*ratio;
        elseif (jV(z) < -maxSpeed(z))
            ratio = -maxSpeed(z)/jV(z);
            jV = jV*ratio;
        end
    end
    
    
    qp = jV;
    q0 = bot.getpos;
%     qp = bot.getPos + (jV*deltaT)';
    
    W = calcW(W, bot,bot.getpos,c);
    
    %Update joints
    q = q0' + (1/fps)*qp;
   	bot.animate(q');
    
    pause(1/fps);
end
%% Move piece to random spot

% delete(cam);

% status = 3;
if(status == 1)
    qMatrix = jtraj(bot.getpos, [0.9313    0.9591    0.2513   -0.4053   -0.1257   0   1],40);

    for i = 1:length(qMatrix(:,1))

        bot.animate(qMatrix(i,:));

        helixModel.MovePart(bot.fkine(qMatrix(i,:)) * troty(pi));
    end
end

if(status == 2)
    disp("Mass = 2kg");
    mass = 2;
    time = 1;
    overallTorque = dynamicTorque(bot,helixModel,1, mass, time,tau_max, maxSpeed);
    disp("Sucessfully Completed Drop");
    
end


if(status == 3)
    disp("Mass = 5kg");
    mass = 5;
    time = 1;
    overallTorque = dynamicTorque(bot,helixModel,0, mass, time,tau_max, maxSpeed);
    
end
try
    plotTorques(overallTorque,tau_max);
end

end
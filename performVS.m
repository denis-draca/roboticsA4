function [bot] =  performVS(moveToStart)
%% Draw Fetch and staircase model
close all;
clc;
% clear;

bot = drawFetch([0.9313    0.9591    0.2513   -0.4053   -0.1257   -2.0944    1]);
modelLocation = transl(0.5,  0, -0.8);
hold on;
helixModel = PartLoader('helix3.ply', modelLocation);


mod1 = modelLocation * transl(0.1,0,0);
mod2 = modelLocation * transl(-0.1,0,0);
mod3 = modelLocation * transl(0,0.1,0);
mod4 = modelLocation * transl(0,-0.1,0);

pStar = [(512 + 50) (512 - 50) 512 512; 512 512 (512 - 50) (512 + 50)];
P = [mod1(1:3,4) mod2(1:3,4) mod3(1:3,4) mod4(1:3,4)];

depth = mean (P(1,:));
% depth = [];

pStar = [512 512]';
P = modelLocation(1:3,4);
%% Setup Cam

cam = CentralCamera('focal', 0.08, 'pixel', 10e-5, ...
'resolution', [1024 1024], 'centre', [512 512],'name', 'FetchCam');

% frame rate
fps = 25;

%Define values
%gain of the controler
lambda = 3;

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
    fprintf('v: %.3f %.3f %.3f %.3f %.3f %.3f\n', v);
    
    
    v(1) = -v(1);
    v(2) = v(2);
    v(3) = v(3);
    v(4) = -v(4) * 0.1;
    v(5) = v(5) * 0.1;
    v(6) = v(6) * 0.1;
    
    J2 = bot.jacob0(bot.getpos);
    
    jV = (inv(W)*J2')*inv(J2*inv(W)*J2')*v;
    
    slowDown = 0;
    
    for x = 1:length(jV)
        if(jV(x) > 20)
            slowDown = 1;
        end
    end
    
    if(slowDown)
        jV = jV.*0.01;
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

    if(mean(e) < 1)
        break
    end
    
    %update current joint position
    q0 = q;
end %loop finishese

%% Move along Z
height = bot.fkine(bot.getpos);
height = (height(3,4) + 0.8);
W = eye(7);
c = [1 1 1 1 1 1 1];
fps = 25;
while(height ~= 0)
    distance = bot.fkine(bot.getpos) - modelLocation;
    
    height = bot.fkine(bot.getpos);
    height = (height(3,4) + 1);
    
    if(height < 0.3)
        break;
    end
    
    velocity = [0 0 -1 0 0 0]';
    
     J2 = bot.jacob0(bot.getpos);
    
    jV = (inv(W)*J2')*inv(J2*inv(W)*J2')*velocity;
    
    slowDown = 0;
    
    for x = 1:length(jV)
        if(jV(x) > 20)
            slowDown = 1;
        end
    end
    
    if(slowDown)
        jV = jV.*0.01;
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
if(moveToStart)
    qMatrix = jtraj(bot.getpos, [0.9313    0.9591    0.2513   -0.4053   -0.1257   0   1],40);

    for i = 1:length(qMatrix(:,1))

        bot.animate(qMatrix(i,:));

        helixModel.MovePart(bot.fkine(qMatrix(i,:)) * troty(pi));
    end
end

pause;
end
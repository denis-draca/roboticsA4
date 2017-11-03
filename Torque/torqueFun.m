function [ torques, qTotal ] = torqueFun( bot, T1, T2, time, mass, massPos, torqueSafe, maxSpeed )


tau_max = [33.82 131.76 76.94 66.18 29.35 25.70 7.36]';


dt = 1/50;                                                                
steps = time/dt;                                                           
q1 = bot.getpos;
                                                 
q(1,:) = q1;

x1 = zeros(6,1);
x2 = zeros(6,1);

x1(1:3,1) = T1(1:3,4);
x2(1:3,1) = T2(1:3,4);

x = zeros(length(x1),steps);
s = lspb(0,1,steps);                                 % Create interpolation scalar
for i = 1:steps
    x(:,i) = x1*(1-s(i)) + s(i)*x2;                  % Create trajectory in x-y plane
end


qd = zeros(steps,7);                                                        % Array of joint velocities
qdd = nan(steps,7);                                                         % Array of joint accelerations
tau = nan(steps,7);                                                         % Array of joint torques                                                                 % Payload mass (kg)
bot.payload(mass, massPos);                                               % Set payload mass in Puma 560 model: offset 0.1m in x-direction

W = eye(7);
c = [1 1 1 1 1 1 1];
for i = 2:steps-1
    
    xdot = (x(:,i) - x(:,i-1))/dt;
    
    J = bot.jacob0(q(i - 1,:));
    
    jV = (inv(W)*J')*inv(J*inv(W)*J')*xdot;
    
    for z = 1:length(jV)
        if(jV(z) > maxSpeed(z))
            ratio = maxSpeed(z)/jV(z);
            jV = jV*ratio;
        elseif (jV(z) < -maxSpeed(z))
            ratio = -maxSpeed(z)/jV(z);
            jV = jV*ratio;
        end
    end
     
    qd(i,:) = jV;    
    q(i,:) = q(i-1,:) + (jV*dt)';   
    W = calcW(W, bot,q(i-1,:),c); 
    qdd(i,:) = (1/dt)^2 * (q(i,:) - q(i,:) - dt*qd(i-1,:));                 % Calculate joint acceleration to get to next set of joint angles
    M = bot.inertia(q(i,:));                                               % Calculate inertia matrix at this pose
    C = bot.coriolis(q(i,:),qd(i,:));                                      % Calculate coriolis matrix at this pose
    g = bot.gravload(q(i,:));                                              % Calculate gravity vector at this pose
    tau(i,:) = (M*qdd(i,:)' + C*qd(i,:)' + g')';                            % Calculate the joint torque needed
    torques(i,:) = tau(i,:);
    for j = 1:7
        if abs(tau(i,j)) > tau_max(j)                                       % Check if torque exceeds limits
            tau(i,j) = sign(tau(i,j))*tau_max(j);                           % Cap joint torque if above limits
        end
    end
    
    if(torqueSafe)
        torques(i,:) = tau(i,:);
    end
    
    qdd(i,:) = (inv(M)*(tau(i,:)' - C*qd(i,:)' - g'))'; 
end
qTotal = q;
end


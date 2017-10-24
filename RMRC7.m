function [ output_args ] = RMRC7( bot, T1, T2, steps)

qMatrix(1,:) = bot.ikcon(T1);

x1 = zeros(6,1);
x2 = zeros(6,1);

x1(1:3,1) = T1(1:3,4);
x2(1:3,1) = T2(1:3,4);

x = zeros(length(x1),steps);
s = lspb(0,1,steps);                                 % Create interpolation scalar
for i = 1:steps
    x(:,i) = x1*(1-s(i)) + s(i)*x2;                  % Create trajectory in x-y plane
end



end


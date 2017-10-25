function [ W ] = calcW( WPrev, robot, pose, c)

W = eye(length(pose));
Links = robot.links;

for i = 1:length(pose)
    try
        qMin = Links(i).qlim(1);
        qMax = Links(i).qlim(2);
        q = pose(i);

        value = ((qMax - qMin)*(2*q - qMax - qMin))/(c(i)*((qMax - q)^2)*((q - qMin)^2));
        
        
        if(isnan(value))
            c(i)
            q
            qMax
            qMin
        end
        
        if(value - WPrev(i,i) > 0)
            W(i,i) = 1 + abs(value);
        else
        
            W(i,i) = 1;
        end
    catch
        W(i,i) = 1;
    end
    
end


end


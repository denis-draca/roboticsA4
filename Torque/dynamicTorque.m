function [ overallTorque ] = dynamicTorque( bot, helixModel, safeSteps, mass, time,tau_max,maxSpeed)
    masPos = [0 0 0.1];
 
    % Move piece up
    T1 = bot.fkine(bot.getpos);
    T2 = T1 * transl(0,0,-0.5);
    
    
    
    [torqueList, qMatrix] = torqueFun(bot, T1, T2, time, mass, masPos, safeSteps, maxSpeed);
    
    overallTorque{1} = torqueList;
    
    
    for q = 1:length(qMatrix(:,1))
       
        Q = qMatrix(q,:);
        T = torqueList(q,:);
        
        
        for jT = 1:length(T)
            
            if(T(jT) > tau_max(jT))
               disp("Joint Failed")
               disp(jT);
               disp(T(jT));
               return;
            end
        end
        
        bot.animate(Q);
        helixModel.MovePart(bot.fkine(Q)* troty(pi));
        drawnow;
    end
    
     % Move along y
    T1 = bot.fkine(bot.getpos);
    T2 = T1 * transl(0,-0.5,0);
    
    
    [torqueList, qMatrix] = torqueFun(bot, T1, T2, time, mass, masPos, safeSteps,maxSpeed);
    overallTorque{2} = torqueList;
    
    for q = 1:length(qMatrix(:,1))
       
        Q = qMatrix(q,:);
        T = torqueList(q,:);
        
        
        for jT = 1:length(T)
            
            if(T(jT) > tau_max(jT))
               disp("Joint Failed")
               disp(jT);
               disp(T(jT));
               return;
            end
        end
        
        bot.animate(Q);
        helixModel.MovePart(bot.fkine(Q)* troty(pi));
        drawnow;
    end
    
    
    
     % place piece down
    T1 = bot.fkine(bot.getpos);
    T2 = T1 * transl(0,0,0.5);
    
    
    [torqueList, qMatrix] = torqueFun(bot, T1, T2, time, mass, masPos, safeSteps,maxSpeed);
    overallTorque{3} = torqueList;
    
    for q = 1:length(qMatrix(:,1))
       
        Q = qMatrix(q,:);
        T = torqueList(q,:);
        
        
        for jT = 1:length(T)
            
            if(T(jT) > tau_max(jT))
               disp("Joint Failed")
               disp(jT);
               disp(T(jT));
               return;
            end
        end
        
        bot.animate(Q);
        helixModel.MovePart(bot.fkine(Q)* troty(pi));
        drawnow;
    end
    
    
     %Move Away
    T1 = bot.fkine(bot.getpos);
    T2 = T1 * transl(0,0,-0.5);
    
    
    [torqueList, qMatrix] = torqueFun(bot, T1, T2, time, 0, masPos, safeSteps,maxSpeed);
    overallTorque{4} = torqueList;
    
    for q = 1:length(qMatrix(:,1))
       
        Q = qMatrix(q,:);
        T = torqueList(q,:);
        
        
        for jT = 1:length(T)
            
            if(T(jT) > tau_max(jT))
               disp("Joint Failed")
               disp(jT);
               disp(T(jT));
               return;
            end
        end
        
        bot.animate(Q);
        drawnow;
    end

end


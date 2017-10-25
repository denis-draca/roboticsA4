function [] = plotTorques( overallTorque, tauMax )
for i = 1:length(overallTorque)
    
    figure('Name', ['Motion ', num2str(i)]);
    
    motionTorques = overallTorque{i};
    
    for T = 1:length(motionTorques(1,:))
        hold on;
        subplot(length(motionTorques(1,:)),1,T);
        
        plot(motionTorques(:,T));
        
        hold on;
        plot([1 length(motionTorques(:,1))], [tauMax(T), tauMax(T)]);
        
        test = (motionTorques(:,T) > tauMax(T));
        
        xlabel("Step");
        ylabel("Torque (Nm)");
        
        title(['Motion ', num2str(i), ' - Joint ', num2str(T)]);
        
    end
end

end


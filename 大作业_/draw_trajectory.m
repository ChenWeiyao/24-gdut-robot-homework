
function draw_trajectory(theta, speed, acceleration, T, time_gap, t)

    x = 0:time_gap:T;
    t = roundn(t,-log(time_gap)/log(0.1));
    num = (t / time_gap) + 1;
    num = round(num);
    
    for i = 1:5
        figure(i+1)
        
        % 角度图
        subplot(3, 1, 1)
        plot(x, theta(:,i), 'r-');
        hold on;        
        plot(t, theta(num,i), 'r*');
        hold on;
        % 角速度图
        subplot(3, 1, 2)
        plot(x, speed(:,i), 'r-');
        hold on;
        plot(t, speed(num,i), 'r*');
        hold on;
        % 角加速度图
        subplot(3, 1, 3)
        plot(x, acceleration(:,i), 'r-');
        hold on;
        plot(t, acceleration(num,i), 'r*');
        hold on;
        % 坐标轴描述
        % 角度图
        subplot(3, 1, 1)
        xlabel('时间（单位：s）')  
        ylabel(sprintf('关节%i角度 (单位:°)',i))
        % 角速度图
        subplot(3, 1, 2)
        xlabel('时间（单位：s）')   
        ylabel(sprintf('关节%i角速度 (单位:°/s)',i)) 
        % 角加速度图
        subplot(3, 1, 3)
        xlabel('时间（单位：s）')   
        ylabel(sprintf('关节%i角加速度 (单位:°/s^2)',i)) 
    end
end
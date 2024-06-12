
function [result_theta, result_speed, result_acceleration, t] = Trajectory4334(route_theta, T, joint, gap)


    [route_num, ~] = size(route_theta);
    if route_num <= 3
        error('error?')
    end

    matrix_size = (route_num - 3)*4 + 10;
 
    matrix = zeros(matrix_size, matrix_size);
    joint_num = joint;
    
    theta = [route_theta(1,joint_num);
                       0             ;
                       0             ];
    for i = 1:(route_num - 2)
        theta_temp = [route_theta(i+1,joint_num);
                      route_theta(i+1,joint_num);
                                  0             ;
                                  0             ];
        theta = [   theta  ;
                 theta_temp];
    end
    theta = [             theta              ;
             route_theta(route_num,joint_num);
                            0                ;
                            0                ];
                                
    for j = 1:(route_num - 1)
        theta_between(j,:) = abs(route_theta(j+1,:) - route_theta(j,:));
    end
    max_theta = max(theta_between, [], 2);
    delta_theta = sum(max_theta);
    for k = 1:(route_num - 1)
        t(k) = (max_theta(k)/delta_theta)*T;
    end
    
    


    matrix1 = [1  0  0;
               0  1  0;
               0  0  2];
    matrix(1:3, 1:3) = matrix1;


    matrix2 = [1  t(1)  t(1)^2   t(1)^3     t(1)^4   0  0  0;
               0   0       0        0          0     1  0  0;
               0   2    4*t(1)  6*t(1)^2   8*t(1)^3  0  -1 0;
               0   0       4     12*t(1)   24*t(1)^2  0  0 -4];
    matrix(4:7,1:8) = matrix2;
    row_start_num = 8;
    column_start_num = 6;
    
    for n = 2:(route_num - 3 + 1)
        matrix_temp = [1  t(n)  t(n)^2   t(n)^3   0  0  0;
                       0   0       0        0     1  0  0;
                       0   2    4*t(n)  6*t(n)^2  0  -1 0;
                       0   0       4     12*t(n)   0  0 -4];
          
        matrix(row_start_num:row_start_num+3,column_start_num:column_start_num+6) = matrix_temp;
        
        row_start_num = row_start_num + 4;
        column_start_num = column_start_num + 4;
    end
    
    m = route_num - 1;
    matrix_end = [1  t(m)  t(m)^2   t(m)^3    t(m)^4 ;
                  0   1    2*t(m)  3*t(m)^2  4*t(m)^3;
                  0   0      2      6*t(m)  12*t(m)^2];
    matrix(matrix_size-2:matrix_size, matrix_size-4:matrix_size) = matrix_end;
    
    solve = matrix \ theta;
    
    for h = 2:(route_num - 1)
        t(h) = t(h-1) + t(h);
    end
    t = [0 t];
    time_gap = gap;
    result_theta = [];
    result_speed = [];
    result_acceleration = [];
    for g = 1:(route_num - 1)
        if g == 1
            x = roundn(t(g),-log(time_gap)/log(0.1)):time_gap:roundn(t(g+1),-log(time_gap)/log(0.1));
            y_theta = solve(1) + solve(2)*x + solve(3)*x.^2 + solve(4)*x.^3 + solve(5)*x.^4;  %
            y_speed = solve(2) + 2*solve(3)*x + 3*solve(4)*x.^2 + 4*solve(5)*x.^3;  %
            y_acceleration = 2*solve(3) + 6*solve(4)*x + 12*solve(5)*x.^2;  % 
            
            result_theta = [result_theta;
                              y_theta'];
            result_speed = [result_speed;
                              y_speed'];
            result_acceleration = [result_acceleration;
                                      y_acceleration'];
        end
        if g>1 && g<(route_num - 1) 
            x = roundn(t(g)+time_gap,-log(time_gap)/log(0.1)):time_gap:roundn(t(g+1),-log(time_gap)/log(0.1));
            y_theta = solve(6+(g-2)*4) + solve(6+(g-2)*4+1)*(x-t(g)) + solve(6+(g-2)*4+2)*(x-t(g)).^2 + solve(6+(g-2)*4+3)*(x-t(g)).^3;  %
            y_speed = solve(6+(g-2)*4+1) + 2*solve(6+(g-2)*4+2)*(x-t(g)) + 3*solve(6+(g-2)*4+3)*(x-t(g)).^2;  % 
            y_acceleration = 2*solve(6+(g-2)*4+2) + 6*solve(6+(g-2)*4+3)*(x-t(g));  % 
            
    
            result_theta = [result_theta;
                              y_theta'];
            result_speed = [result_speed;
                              y_speed'];
            result_acceleration = [result_acceleration;
                                      y_acceleration'];
        end
        if g == (route_num - 1)
            x = roundn(t(g)+time_gap,-log(time_gap)/log(0.1)):time_gap:roundn(t(g+1),-log(time_gap)/log(0.1));
            y_theta =  solve(matrix_size-4) + solve(matrix_size-3)*(x-t(g)) + solve(matrix_size-2)*(x-t(g)).^2 + solve(matrix_size-1)*(x-t(g)).^3 + solve(matrix_size)*(x-t(g)).^4;  % 关节�?
            y_speed =  solve(matrix_size-3) + 2*solve(matrix_size-2)*(x-t(g)) + 3*solve(matrix_size-1)*(x-t(g)).^2 + 4*solve(matrix_size)*(x-t(g)).^3;  % 关节速度
            y_acceleration =  2*solve(matrix_size-2) + 6*solve(matrix_size-1)*(x-t(g)) + 12*solve(matrix_size)*(x-t(g)).^2;  
            result_theta = [result_theta;
                              y_theta'];
            result_speed = [result_speed;
                              y_speed'];
            result_acceleration = [result_acceleration;
                                      y_acceleration'];
        end
    end
end


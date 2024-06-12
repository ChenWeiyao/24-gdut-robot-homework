function[th_finger2_1,th_finger2_2,th_finger2_3,th_finger2_4,th_finger2_5] = IK_finger2(Tpos,rpos)
    global Link_finger2
    num=1;
    ToDeg = 180/pi;
    ToRad = pi/180;
    %
    th_finger2_1=0;
    th_finger2_2=0;
    th_finger2_3=0;
    th_finger2_4=0;
    th_finger2_5=0;
    DHfk_finger2_Lnya_calculate(th_finger2_1,th_finger2_2,th_finger2_3,th_finger2_4,th_finger2_5); %

    th1_min = 0;
    th1_max = 40;
    th2_min = -90;
    th2_max = 90;
    th3_min = -90;
    th3_max = 0;
    th4_min = -150;
    th4_max = 0;
    th5_min = -150;
    th5_max = 0;
    %%
    tic
    while (1)
        if toc > 1
            th_finger2_1=0;
            th_finger2_2=0;
            th_finger2_3=0;
            th_finger2_4=0;
            th_finger2_5=-20;
            break;
        end
        DHfk_finger2_Lnya_calculate(th_finger2_1,th_finger2_2,th_finger2_3,th_finger2_4,th_finger2_5);
        ex=Link_finger2(6).p(1);
        ey=Link_finger2(6).p(2);
        ez=Link_finger2(6).p(3);
        %% 
        R_now=Link_finger2(6).R;
        p_err =[Tpos(1)-ex, Tpos(2)-ey, Tpos(3)-ez]' ;%
        R_err=R_now'*rpos;%R_err   
        if R_err==eye(3)
            w_err=[0,0,0]'; 
            Loss = norm(p_err) + norm(w_err) ; % 
            Loss_pos = norm(p_err);
        else
            theta_R_err=acos((R_err(1,1)+R_err(2,2)+R_err(3,3)-1)/2);
            w_err=(theta_R_err/(2*sin(theta_R_err)))*[R_err(3,2)-R_err(2,3),R_err(1,3)-R_err(3,1),R_err(2,1)-R_err(1,2)]';
            Loss = norm(p_err) + norm(w_err) ; % 
            Loss_pos = norm(p_err);
        end
        %% 
        if Loss<5 && Loss_pos<1
            break;
        end
        %
        J=Jacobian5DoF_finger2(th_finger2_1,th_finger2_2,th_finger2_3,th_finger2_4,th_finger2_5);    %
            learning_rate = min(Loss/10, 0.8); %
    %         learning_rate =0.2;
            dth = learning_rate * pinv(J) * [p_err; w_err];  %
        th_finger2_1=th_finger2_1+dth(1)*ToDeg;
        th_finger2_1 = max(th1_min, min(th1_max, th_finger2_1));
        th_finger2_2=th_finger2_2+dth(2)*ToDeg;
        th_finger2_2 = max(th2_min, min(th2_max, th_finger2_2));
        th_finger2_3=th_finger2_3+dth(3)*ToDeg;
        th_finger2_3 = max(th3_min, min(th3_max, th_finger2_3));
        th_finger2_4=th_finger2_4+dth(4)*ToDeg;
        th_finger2_4 = max(th4_min, min(th4_max, th_finger2_4));
        th_finger2_5=th_finger2_5+dth(5)*ToDeg;
        th_finger2_5 = max(th5_min, min(th5_max, th_finger2_5));
        x(num)=ex;
        y(num)=ey;
        z(num)=ez;
        num=num+1;  
        for i = 1:length(x)
%         fprintf('Step %d: Link(6).p(1) = %f, Link(6).p(2) = %f, Link(6).p(3) = %f, Loss=%f\n', i, Link_finger0(6).p(1), Link_finger0(6).p(2), Link_finger0(6).p(3),Loss); 
          fprintf('快收敛了快收敛了\n');
        end
    end
    DHfk_finger2_Lnya_calculate(th_finger2_1,th_finger2_2,th_finger2_3,th_finger2_4,th_finger2_5);







    
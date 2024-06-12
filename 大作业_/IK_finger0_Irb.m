function[th1_finger0,th2_finger0,th3_finger0,th4_finger0,th5_finger0] = IK_finger0_Irb(Tpos,rpos,th1,th2,th3,th4,th5,th6,th1_finger0,th2_finger0,th3_finger0,th4_finger0,th5_finger0,th1_finger1,th2_finger1,th3_finger1,th4_finger1,th5_finger1,th1_finger2,th2_finger2,th3_finger2,th4_finger2,th5_finger2,th1_finger3,th2_finger3,th3_finger3,th4_finger3,th5_finger3,th1_finger4,th2_finger4,th3_finger4,th4_finger4,th5_finger4,th1_finger5,th2_finger5,th3_finger5,th4_finger5,th5_finger5,th1_finger6,th2_finger6,th3_finger6,th4_finger6,th5_finger6,th1_finger7,th2_finger7,th3_finger7,th4_finger7,th5_finger7)
    global Link_finger0
    num=1;
    ToDeg = 180/pi;
    ToRad = pi/180;
    %

    DHfk_fingers_Lnya_Mov_calculate_every_finger(th1,th2,th3,th4,th5,th6,th1_finger0,th2_finger0,th3_finger0,th4_finger0,th5_finger0,th1_finger1,th2_finger1,th3_finger1,th4_finger1,th5_finger1,th1_finger2,th2_finger2,th3_finger2,th4_finger2,th5_finger2,th1_finger3,th2_finger3,th3_finger3,th4_finger3,th5_finger3,th1_finger4,th2_finger4,th3_finger4,th4_finger4,th5_finger4,th1_finger5,th2_finger5,th3_finger5,th4_finger5,th5_finger5,th1_finger6,th2_finger6,th3_finger6,th4_finger6,th5_finger6,th1_finger7,th2_finger7,th3_finger7,th4_finger7,th5_finger7); %

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
            th1_finger0=0;
            th2_finger0=0;
            th3_finger0=0;
            th4_finger0=0;
            th5_finger0=-20;
            break;
        end
        DHfk_fingers_Lnya_Mov_calculate_every_finger(th1,th2,th3,th4,th5,th6,th1_finger0,th2_finger0,th3_finger0,th4_finger0,th5_finger0,th1_finger1,th2_finger1,th3_finger1,th4_finger1,th5_finger1,th1_finger2,th2_finger2,th3_finger2,th4_finger2,th5_finger2,th1_finger3,th2_finger3,th3_finger3,th4_finger3,th5_finger3,th1_finger4,th2_finger4,th3_finger4,th4_finger4,th5_finger4,th1_finger5,th2_finger5,th3_finger5,th4_finger5,th5_finger5,th1_finger6,th2_finger6,th3_finger6,th4_finger6,th5_finger6,th1_finger7,th2_finger7,th3_finger7,th4_finger7,th5_finger7);
        ex=Link_finger0(6).p(1);
        ey=Link_finger0(6).p(2);
        ez=Link_finger0(6).p(3);
        %% 
        R_now=Link_finger0(6).R;
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
        if Loss<4 && Loss_pos<1
            break;
        end
        %
        J=Jacobian5DoF_finger0(th1_finger0,th2_finger0,th3_finger0,th4_finger0,th5_finger0);    %
        learning_rate = min(Loss, 0.8); %
        %         learning_rate =0.2;
        dth = learning_rate * pinv(J) * [p_err; w_err];  %
        th1_finger0=th1_finger0+dth(1)*ToDeg;
        th1_finger0 = max(th1_min, min(th1_max, th1_finger0));
        th2_finger0=th2_finger0+dth(2)*ToDeg;
        th2_finger0 = max(th2_min, min(th2_max, th2_finger0));
        th3_finger0=th3_finger0+dth(3)*ToDeg;
        th3_finger0 = max(th3_min, min(th3_max, th3_finger0));
        th4_finger0=th4_finger0+dth(4)*ToDeg;
        th4_finger0 = max(th4_min, min(th4_max, th4_finger0));
        th5_finger0=th5_finger0+dth(5)*ToDeg;
        th5_finger0 = max(th5_min, min(th5_max, th5_finger0));
        
        x(num)=ex;
        y(num)=ey;
        z(num)=ez;
        num=num+1;  
        for i = 1:length(x)
          fprintf('Loss_pos_finger0: %f\n', Loss_pos);
        end
    end
    DHfk_fingers_Lnya_Mov_calculate_every_finger(th1,th2,th3,th4,th5,th6,th1_finger0,th2_finger0,th3_finger0,th4_finger0,th5_finger0,th1_finger1,th2_finger1,th3_finger1,th4_finger1,th5_finger1,th1_finger2,th2_finger2,th3_finger2,th4_finger2,th5_finger2,th1_finger3,th2_finger3,th3_finger3,th4_finger3,th5_finger3,th1_finger4,th2_finger4,th3_finger4,th4_finger4,th5_finger4,th1_finger5,th2_finger5,th3_finger5,th4_finger5,th5_finger5,th1_finger6,th2_finger6,th3_finger6,th4_finger6,th5_finger6,th1_finger7,th2_finger7,th3_finger7,th4_finger7,th5_finger7);







    
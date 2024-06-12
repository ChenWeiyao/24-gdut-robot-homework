function DHfk_fingers_Lnya_Mov_every_finger(th1,th2,th3,th4,th5,th6,th1_finger0,th2_finger0,th3_finger0,th4_finger0,th5_finger0,th1_finger1,th2_finger1,th3_finger1,th4_finger1,th5_finger1,th1_finger2,th2_finger2,th3_finger2,th4_finger2,th5_finger2,th1_finger3,th2_finger3,th3_finger3,th4_finger3,th5_finger3,th1_finger4,th2_finger4,th3_finger4,th4_finger4,th5_finger4,th1_finger5,th2_finger5,th3_finger5,th4_finger5,th5_finger5,th1_finger6,th2_finger6,th3_finger6,th4_finger6,th5_finger6,th1_finger7,th2_finger7,th3_finger7,th4_finger7,th5_finger7,fcla)
    global Link_IRB120
    global Link_finger0
    global Link_finger1
    global Link_finger2
    global Link_finger3
    global Link_finger4
    global Link_finger5
    global Link_finger6
    global Link_finger7
    IRB120_DH;
    finger0_DH;
    finger1_DH;
    finger2_DH;
    finger3_DH;
    finger4_DH;
    finger5_DH;
    finger6_DH;
    finger7_DH;
    radius_f    = 5;  %25
    len_f       = 20;  %60
    joint_col_f = 0;
    radius    = 10;  %25
    len       = 20;  %60
    joint_col = 0;
    plot3(0,0,0,'ro'); 
    Link_IRB120(2).th=Link_IRB120(2).th+th1*pi/180;
    Link_IRB120(3).th=Link_IRB120(3).th+th2*pi/180;
    Link_IRB120(4).th=Link_IRB120(4).th+th3*pi/180;
    Link_IRB120(5).th=Link_IRB120(5).th+th4*pi/180;
    Link_IRB120(6).th=Link_IRB120(6).th+th5*pi/180;
    Link_IRB120(7).th=Link_IRB120(7).th+th6*pi/180;    %for initial position
    
    Link_finger0(2).th=Link_finger0(2).th+th1_finger0*pi/180;
    Link_finger0(3).th=Link_finger0(3).th+th2_finger0*pi/180;
    Link_finger0(4).th=Link_finger0(4).th+th3_finger0*pi/180;
    Link_finger0(5).th=Link_finger0(5).th+th4_finger0*pi/180;
    Link_finger0(6).th=Link_finger0(6).th+th5_finger0*pi/180;
    Link_finger1(2).th=Link_finger1(2).th+th1_finger1*pi/180;
    Link_finger1(3).th=Link_finger1(3).th+th2_finger1*pi/180;
    Link_finger1(4).th=Link_finger1(4).th+th3_finger1*pi/180;
    Link_finger1(5).th=Link_finger1(5).th+th4_finger1*pi/180;
    Link_finger1(6).th=Link_finger1(6).th+th5_finger1*pi/180;
    Link_finger2(2).th=Link_finger2(2).th+th1_finger2*pi/180;
    Link_finger2(3).th=Link_finger2(3).th+th2_finger2*pi/180;
    Link_finger2(4).th=Link_finger2(4).th+th3_finger2*pi/180;
    Link_finger2(5).th=Link_finger2(5).th+th4_finger2*pi/180;
    Link_finger2(6).th=Link_finger2(6).th+th5_finger2*pi/180;
    Link_finger3(2).th=Link_finger3(2).th+th1_finger3*pi/180;
    Link_finger3(3).th=Link_finger3(3).th+th2_finger3*pi/180;
    Link_finger3(4).th=Link_finger3(4).th+th3_finger3*pi/180;
    Link_finger3(5).th=Link_finger3(5).th+th4_finger3*pi/180;
    Link_finger3(6).th=Link_finger3(6).th+th5_finger3*pi/180;
    Link_finger4(2).th=Link_finger4(2).th+th1_finger4*pi/180;
    Link_finger4(3).th=Link_finger4(3).th+th2_finger4*pi/180;
    Link_finger4(4).th=Link_finger4(4).th+th3_finger4*pi/180;
    Link_finger4(5).th=Link_finger4(5).th+th4_finger4*pi/180;
    Link_finger4(6).th=Link_finger4(6).th+th5_finger4*pi/180;
    Link_finger5(2).th=Link_finger5(2).th+th1_finger5*pi/180;
    Link_finger5(3).th=Link_finger5(3).th+th2_finger5*pi/180;
    Link_finger5(4).th=Link_finger5(4).th+th3_finger5*pi/180;
    Link_finger5(5).th=Link_finger5(5).th+th4_finger5*pi/180;
    Link_finger5(6).th=Link_finger5(6).th+th5_finger5*pi/180;
    Link_finger6(2).th=Link_finger6(2).th+th1_finger6*pi/180;
    Link_finger6(3).th=Link_finger6(3).th+th2_finger6*pi/180;
    Link_finger6(4).th=Link_finger6(4).th+th3_finger6*pi/180;
    Link_finger6(5).th=Link_finger6(5).th+th4_finger6*pi/180;
    Link_finger6(6).th=Link_finger6(6).th+th5_finger6*pi/180;
    Link_finger7(2).th=Link_finger7(2).th+th1_finger7*pi/180;
    Link_finger7(3).th=Link_finger7(3).th+th2_finger7*pi/180;
    Link_finger7(4).th=Link_finger7(4).th+th3_finger7*pi/180;
    Link_finger7(5).th=Link_finger7(5).th+th4_finger7*pi/180;
    Link_finger7(6).th=Link_finger7(6).th+th5_finger7*pi/180;
    
    p0=[0,0,0]';
    
    for i=1:7
    Matrix_DH_IRB120(i);
    end
    for i=1:6
    Matrix_DH_finger0(i);
    Matrix_DH_finger1(i);
    Matrix_DH_finger2(i);
    Matrix_DH_finger3(i);
    Matrix_DH_finger4(i);
    Matrix_DH_finger5(i);
    Matrix_DH_finger6(i);
    Matrix_DH_finger7(i);
    end
    
    for i=2:7

        Link_IRB120(i).A=Link_IRB120(i-1).A*Link_IRB120(i).A;
        Link_IRB120(i).p= Link_IRB120(i).A(:,4);
        Link_IRB120(i).n= Link_IRB120(i).A(:,1);
        Link_IRB120(i).o= Link_IRB120(i).A(:,2);
        Link_IRB120(i).a= Link_IRB120(i).A(:,3);
        Link_IRB120(i).R=[Link_IRB120(i).n(1:3),Link_IRB120(i).o(1:3),Link_IRB120(i).a(1:3)];
        Connect3D(Link_IRB120(i-1).p,Link_IRB120(i).p,'b',2); 
        hold on;
        plot3(Link_IRB120(i).p(1),Link_IRB120(i).p(2),Link_IRB120(i).p(3),'rx');
        hold on;
        if i<=7
            DrawCylinder(Link_IRB120(i-1).p, Link_IRB120(i-1).R * Link_IRB120(i).az, radius,len, joint_col); 
            hold on;
        end 
    end
    Link_finger0(1).A= Link_IRB120(7).A*Link_finger0(1).A;
    Link_finger0(1).p= Link_finger0(1).A(:,4);
    Link_finger0(1).n= Link_finger0(1).A(:,1);
    Link_finger0(1).o= Link_finger0(1).A(:,2);
    Link_finger0(1).a= Link_finger0(1).A(:,3);
    Link_finger0(1).R=[Link_finger0(1).n(1:3),Link_finger0(1).o(1:3),Link_finger0(1).a(1:3)];

    Link_finger1(1).A= Link_IRB120(7).A*Link_finger1(1).A;
    Link_finger1(1).p= Link_finger1(1).A(:,4);
    Link_finger1(1).n= Link_finger1(1).A(:,1);
    Link_finger1(1).o= Link_finger1(1).A(:,2);
    Link_finger1(1).a= Link_finger1(1).A(:,3);
    Link_finger1(1).R=[Link_finger1(1).n(1:3),Link_finger1(1).o(1:3),Link_finger1(1).a(1:3)];

    Link_finger2(1).A= Link_IRB120(7).A*Link_finger2(1).A;
    Link_finger2(1).p= Link_finger2(1).A(:,4);
    Link_finger2(1).n= Link_finger2(1).A(:,1);
    Link_finger2(1).o= Link_finger2(1).A(:,2);
    Link_finger2(1).a= Link_finger2(1).A(:,3);
    Link_finger2(1).R=[Link_finger2(1).n(1:3),Link_finger2(1).o(1:3),Link_finger2(1).a(1:3)];

    Link_finger3(1).A= Link_IRB120(7).A*Link_finger3(1).A;
    Link_finger3(1).p= Link_finger3(1).A(:,4);
    Link_finger3(1).n= Link_finger3(1).A(:,1);
    Link_finger3(1).o= Link_finger3(1).A(:,2);
    Link_finger3(1).a= Link_finger3(1).A(:,3);
    Link_finger3(1).R=[Link_finger3(1).n(1:3),Link_finger3(1).o(1:3),Link_finger3(1).a(1:3)];

    Link_finger4(1).A= Link_IRB120(7).A*Link_finger4(1).A;
    Link_finger4(1).p= Link_finger4(1).A(:,4);
    Link_finger4(1).n= Link_finger4(1).A(:,1);
    Link_finger4(1).o= Link_finger4(1).A(:,2);
    Link_finger4(1).a= Link_finger4(1).A(:,3);
    Link_finger4(1).R=[Link_finger4(1).n(1:3),Link_finger4(1).o(1:3),Link_finger4(1).a(1:3)];

    Link_finger5(1).A= Link_IRB120(7).A*Link_finger5(1).A;
    Link_finger5(1).p= Link_finger5(1).A(:,4);
    Link_finger5(1).n= Link_finger5(1).A(:,1);
    Link_finger5(1).o= Link_finger5(1).A(:,2);
    Link_finger5(1).a= Link_finger5(1).A(:,3);
    Link_finger5(1).R=[Link_finger5(1).n(1:3),Link_finger5(1).o(1:3),Link_finger5(1).a(1:3)];

    Link_finger6(1).A= Link_IRB120(7).A*Link_finger6(1).A;
    Link_finger6(1).p= Link_finger6(1).A(:,4);
    Link_finger6(1).n= Link_finger6(1).A(:,1);
    Link_finger6(1).o= Link_finger6(1).A(:,2);
    Link_finger6(1).a= Link_finger6(1).A(:,3);
    Link_finger6(1).R=[Link_finger6(1).n(1:3),Link_finger6(1).o(1:3),Link_finger6(1).a(1:3)];

    Link_finger7(1).A= Link_IRB120(7).A*Link_finger7(1).A;
    Link_finger7(1).p= Link_finger7(1).A(:,4);
    Link_finger7(1).n= Link_finger7(1).A(:,1);
    Link_finger7(1).o= Link_finger7(1).A(:,2);
    Link_finger7(1).a= Link_finger7(1).A(:,3);
    Link_finger7(1).R=[Link_finger7(1).n(1:3),Link_finger7(1).o(1:3),Link_finger7(1).a(1:3)];

    Connect3D(Link_IRB120(7).p,Link_finger0(1).p,'b',2);
    Connect3D(Link_IRB120(7).p,Link_finger1(1).p,'b',2);
    Connect3D(Link_IRB120(7).p,Link_finger2(1).p,'b',2);
    Connect3D(Link_IRB120(7).p,Link_finger3(1).p,'b',2);
    Connect3D(Link_IRB120(7).p,Link_finger4(1).p,'b',2);
    Connect3D(Link_IRB120(7).p,Link_finger5(1).p,'b',2);
    Connect3D(Link_IRB120(7).p,Link_finger6(1).p,'b',2);
    Connect3D(Link_IRB120(7).p,Link_finger7(1).p,'b',2);
    hold on;

    for i=2:6
    
        Link_finger0(i).A=Link_finger0(i-1).A*Link_finger0(i).A;
        Link_finger0(i).p= Link_finger0(i).A(:,4);
        Link_finger0(i).n= Link_finger0(i).A(:,1);
        Link_finger0(i).o= Link_finger0(i).A(:,2);
        Link_finger0(i).a= Link_finger0(i).A(:,3);
        Link_finger0(i).R=[Link_finger0(i).n(1:3),Link_finger0(i).o(1:3),Link_finger0(i).a(1:3)];

        Link_finger1(i).A=Link_finger1(i-1).A*Link_finger1(i).A;
        Link_finger1(i).p= Link_finger1(i).A(:,4);
        Link_finger1(i).n= Link_finger1(i).A(:,1);
        Link_finger1(i).o= Link_finger1(i).A(:,2);
        Link_finger1(i).a= Link_finger1(i).A(:,3);
        Link_finger1(i).R=[Link_finger1(i).n(1:3),Link_finger1(i).o(1:3),Link_finger1(i).a(1:3)];

        Link_finger2(i).A=Link_finger2(i-1).A*Link_finger2(i).A;
        Link_finger2(i).p= Link_finger2(i).A(:,4);
        Link_finger2(i).n= Link_finger2(i).A(:,1);
        Link_finger2(i).o= Link_finger2(i).A(:,2);
        Link_finger2(i).a= Link_finger2(i).A(:,3);
        Link_finger2(i).R=[Link_finger2(i).n(1:3),Link_finger2(i).o(1:3),Link_finger2(i).a(1:3)];

        Link_finger3(i).A=Link_finger3(i-1).A*Link_finger3(i).A;
        Link_finger3(i).p= Link_finger3(i).A(:,4);
        Link_finger3(i).n= Link_finger3(i).A(:,1);
        Link_finger3(i).o= Link_finger3(i).A(:,2);
        Link_finger3(i).a= Link_finger3(i).A(:,3);
        Link_finger3(i).R=[Link_finger3(i).n(1:3),Link_finger3(i).o(1:3),Link_finger3(i).a(1:3)];

        Link_finger4(i).A=Link_finger4(i-1).A*Link_finger4(i).A;
        Link_finger4(i).p= Link_finger4(i).A(:,4);
        Link_finger4(i).n= Link_finger4(i).A(:,1);
        Link_finger4(i).o= Link_finger4(i).A(:,2);
        Link_finger4(i).a= Link_finger4(i).A(:,3);
        Link_finger4(i).R=[Link_finger4(i).n(1:3),Link_finger4(i).o(1:3),Link_finger4(i).a(1:3)];

        Link_finger5(i).A=Link_finger5(i-1).A*Link_finger5(i).A;
        Link_finger5(i).p= Link_finger5(i).A(:,4);
        Link_finger5(i).n= Link_finger5(i).A(:,1);
        Link_finger5(i).o= Link_finger5(i).A(:,2);
        Link_finger5(i).a= Link_finger5(i).A(:,3);
        Link_finger5(i).R=[Link_finger5(i).n(1:3),Link_finger5(i).o(1:3),Link_finger5(i).a(1:3)];

        Link_finger6(i).A=Link_finger6(i-1).A*Link_finger6(i).A;
        Link_finger6(i).p= Link_finger6(i).A(:,4);
        Link_finger6(i).n= Link_finger6(i).A(:,1);
        Link_finger6(i).o= Link_finger6(i).A(:,2);
        Link_finger6(i).a= Link_finger6(i).A(:,3);
        Link_finger6(i).R=[Link_finger6(i).n(1:3),Link_finger6(i).o(1:3),Link_finger6(i).a(1:3)];

        Link_finger7(i).A=Link_finger7(i-1).A*Link_finger7(i).A;
        Link_finger7(i).p= Link_finger7(i).A(:,4);
        Link_finger7(i).n= Link_finger7(i).A(:,1);
        Link_finger7(i).o= Link_finger7(i).A(:,2);
        Link_finger7(i).a= Link_finger7(i).A(:,3);
        Link_finger7(i).R=[Link_finger7(i).n(1:3),Link_finger7(i).o(1:3),Link_finger7(i).a(1:3)];

        Connect3D(Link_finger0(i-1).p,Link_finger0(i).p,'b',2);
        Connect3D(Link_finger1(i-1).p,Link_finger1(i).p,'b',2);
        Connect3D(Link_finger2(i-1).p,Link_finger2(i).p,'b',2);
        Connect3D(Link_finger3(i-1).p,Link_finger3(i).p,'b',2);
        Connect3D(Link_finger4(i-1).p,Link_finger4(i).p,'b',2);
        Connect3D(Link_finger5(i-1).p,Link_finger5(i).p,'b',2);
        Connect3D(Link_finger6(i-1).p,Link_finger6(i).p,'b',2);
        Connect3D(Link_finger7(i-1).p,Link_finger7(i).p,'b',2);
        hold on;
        
        
    end
    DrawSphere_End(Link_finger0(6).p(1),Link_finger0(6).p(2),Link_finger0(6).p(3),radius_f);
    DrawSphere_End(Link_finger1(6).p(1),Link_finger1(6).p(2),Link_finger1(6).p(3),radius_f);
    DrawSphere_End(Link_finger2(6).p(1),Link_finger2(6).p(2),Link_finger2(6).p(3),radius_f);
    DrawSphere_End(Link_finger3(6).p(1),Link_finger3(6).p(2),Link_finger3(6).p(3),radius_f);
    DrawSphere_End(Link_finger4(6).p(1),Link_finger4(6).p(2),Link_finger4(6).p(3),radius_f);
    DrawSphere_End(Link_finger5(6).p(1),Link_finger5(6).p(2),Link_finger5(6).p(3),radius_f);
    DrawSphere_End(Link_finger6(6).p(1),Link_finger6(6).p(2),Link_finger6(6).p(3),radius_f);
    DrawSphere_End(Link_finger7(6).p(1),Link_finger7(6).p(2),Link_finger7(6).p(3),radius_f);
    hold on;
    hold on;
    axis([-1000,1000,-1000,1000,-500,1000]);
    xlabel('x');
    ylabel('y'); 
    zlabel('z');
    grid on;
    drawnow;
    if(fcla)
        cla;
    end
    
    
    
    
    
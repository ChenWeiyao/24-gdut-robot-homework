function J=Jacobian6DoF_Ln(th1,th2,th3,th4,th5,th6)
% close all
global Link_IRB120

jsize=6;
J=zeros(6,jsize);
% 若有绕z轴转一定角度的，需要补在前面
Link_IRB120(2).th=0.5*pi+th1*pi/180; 
Link_IRB120(3).th=0.5*pi+th2*pi/180;
Link_IRB120(4).th=th3*pi/180;
Link_IRB120(5).th=th4*pi/180;
Link_IRB120(6).th=th5*pi/180;
Link_IRB120(7).th=th6*pi/180;

for i=1:7
    Matrix_DH_IRB120(i);
end

Link_IRB120(1).p=Link_IRB120(1).p(1:3);
for i=2:7
    Link_IRB120(i).A=Link_IRB120(i-1).A*Link_IRB120(i).A;
    Link_IRB120(i).p= Link_IRB120(i).A(1:3,4);
    Link_IRB120(i).n= Link_IRB120(i).A(:,1);
    Link_IRB120(i).o= Link_IRB120(i).A(:,2);
    Link_IRB120(i).a= Link_IRB120(i).A(:,3);
    Link_IRB120(i).R=[Link_IRB120(i).n(1:3),Link_IRB120(i).o(1:3),Link_IRB120(i).a(1:3)];
end

for n=1:jsize
    a=Link_IRB120(n).R*Link_IRB120(n).az;
    J(:,n)=[cross(a,Link_IRB120(7).p-Link_IRB120(n).p); a];
end

function J=Jacobian5DoF_finger2(th1,th2,th3,th4,th5)
% close all
global Link_finger2

jsize=5;
J=zeros(6,jsize);
% 若有绕z轴转一定角度的，需要补在前面
% finger0的第二轴和第三轴有绕z轴转90度和45度
Link_finger2(2).th=0.5*pi+th1*pi/180;
Link_finger2(3).th=0.25*pi+th2*pi/180;
Link_finger2(4).th=th3*pi/180;
Link_finger2(5).th=th4*pi/180;
Link_finger2(6).th=th5*pi/180;

for i=1:6
    Matrix_DH_finger2(i);
end

Link_finger2(1).p=Link_finger2(1).p(1:3);
for i=2:6
    Link_finger2(i).A=Link_finger2(i-1).A*Link_finger2(i).A;
    Link_finger2(i).p= Link_finger2(i).A(1:3,4);
    Link_finger2(i).n= Link_finger2(i).A(:,1);
    Link_finger2(i).o= Link_finger2(i).A(:,2);
    Link_finger2(i).a= Link_finger2(i).A(:,3);
    Link_finger2(i).R=[Link_finger2(i).n(1:3),Link_finger2(i).o(1:3),Link_finger2(i).a(1:3)];
end

for n=1:jsize
    a=Link_finger2(n).R*Link_finger2(n).az;
    J(:,n)=[cross(a,Link_finger2(6).p-Link_finger2(n).p); a];
end

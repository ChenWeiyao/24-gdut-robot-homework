function Matrix_DH_IRB120(i) 
% Caculate the D-H Matrix
global Link_IRB120

ToDeg = 180/pi;
ToRad = pi/180;


C=cos(Link_IRB120(i).th);
S=sin(Link_IRB120(i).th);
Ca=cos(Link_IRB120(i).alf);
Sa=sin(Link_IRB120(i).alf);
a=Link_IRB120(i).dx;    %distance between zi and zi-1
d=Link_IRB120(i).dz;    %distance between xi and xi-1

Link_IRB120(i).n=[C,S,0,0]';
Link_IRB120(i).o=[-1*S*Ca,C*Ca,Sa,0]';
Link_IRB120(i).a=[S*Sa, -1*C*Sa,Ca,0]';
Link_IRB120(i).p=[a*C,a*S,d,1]';
Link_IRB120(i).R=[Link_IRB120(i).n(1:3),Link_IRB120(i).o(1:3),Link_IRB120(i).a(1:3)];
Link_IRB120(i).A=[Link_IRB120(i).n,Link_IRB120(i).o,Link_IRB120(i).a,Link_IRB120(i).p];



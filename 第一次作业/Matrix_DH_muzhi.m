%计算每个关节之间的齐次变换矩阵
function Matrix_DH_muzhi(i) 
global Link_muzhi

ToDeg = 180/pi;
ToRad = pi/180;


C=cos(Link_muzhi(i).th);
S=sin(Link_muzhi(i).th);
Ca=cos(Link_muzhi(i).alf);
Sa=sin(Link_muzhi(i).alf);
a=Link_muzhi(i).dx;    %distance between zi and zi-1
d=Link_muzhi(i).dz;    %distance between xi and xi-1

Link_muzhi(i).n=[C,S,0,0]';
Link_muzhi(i).o=[-1*S*Ca,C*Ca,Sa,0]';
Link_muzhi(i).a=[S*Sa, -1*C*Sa,Ca,0]';
%位置向量
Link_muzhi(i).p=[a*C,a*S,d,1]';
%旋转矩阵
Link_muzhi(i).R=[Link_muzhi(i).n(1:3),Link_muzhi(i).o(1:3),Link_muzhi(i).a(1:3)];
%齐次变换矩阵
Link_muzhi(i).A=[Link_muzhi(i).n,Link_muzhi(i).o,Link_muzhi(i).a,Link_muzhi(i).p];


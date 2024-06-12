%计算每个关节之间的齐次变换矩阵
function Matrix_DH_finger7(i) 
global Link_finger7

ToDeg = 180/pi;
ToRad = pi/180;


C=cos(Link_finger7(i).th);
S=sin(Link_finger7(i).th);
Ca=cos(Link_finger7(i).alf);
Sa=sin(Link_finger7(i).alf);
a=Link_finger7(i).dx;    %distance between zi and zi-1
d=Link_finger7(i).dz;    %distance between xi and xi-1

Link_finger7(i).n=[C,S,0,0]';
Link_finger7(i).o=[-1*S*Ca,C*Ca,Sa,0]';
Link_finger7(i).a=[S*Sa, -1*C*Sa,Ca,0]';
%位置向量
Link_finger7(i).p=[a*C,a*S,d,1]';
%旋转矩阵
Link_finger7(i).R=[Link_finger7(i).n(1:3),Link_finger7(i).o(1:3),Link_finger7(i).a(1:3)];
%齐次变换矩阵
Link_finger7(i).A=[Link_finger7(i).n,Link_finger7(i).o,Link_finger7(i).a,Link_finger7(i).p];


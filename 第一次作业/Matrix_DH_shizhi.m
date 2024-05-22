%计算每个关节之间的齐次变换矩阵
function Matrix_DH_shizhi(i) 
global Link_shizhi

ToDeg = 180/pi;
ToRad = pi/180;


C=cos(Link_shizhi(i).th);
S=sin(Link_shizhi(i).th);
Ca=cos(Link_shizhi(i).alf);
Sa=sin(Link_shizhi(i).alf);
a=Link_shizhi(i).dx;    %distance between zi and zi-1
d=Link_shizhi(i).dz;    %distance between xi and xi-1

Link_shizhi(i).n=[C,S,0,0]';
Link_shizhi(i).o=[-1*S*Ca,C*Ca,Sa,0]';
Link_shizhi(i).a=[S*Sa, -1*C*Sa,Ca,0]';
%位置向量
Link_shizhi(i).p=[a*C,a*S,d,1]';
%旋转矩阵
Link_shizhi(i).R=[Link_shizhi(i).n(1:3),Link_shizhi(i).o(1:3),Link_shizhi(i).a(1:3)];
%齐次变换矩阵
Link_shizhi(i).A=[Link_shizhi(i).n,Link_shizhi(i).o,Link_shizhi(i).a,Link_shizhi(i).p];


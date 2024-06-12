%输入参数
% oR 3x3 旋转矩阵，op 1x3 位置向量， fcla 绘图选项 1-显示后查出，2-显示后保留
function DrawFrame(oR, op, fcla)

ToDeg=180/pi;   
ToRad=pi/180;

%% 设置坐标轴端点
pn=[5,0,0]';
po=[0,5,0]';
pa=[0,0,5]';

pn=oR*pn+op;     %根据姿态矩阵及原点计算n轴端点在世界坐标系下的向量
po=oR*po+op;     %o轴
pa=oR*pa+op;     %a轴

%% 设置固定参考坐标轴端点
wx=[10,0,0]';
wy=[0,10,0]';
wz=[0,0,10]';
wo=[0,0,0]';

% %绘制固定参考坐标系
Connect3D(wo,wx,'r',0.5); hold on;
Connect3D(wo,wy,'b',0.5); hold on;
Connect3D(wo,wz,'g',0.5);hold on;
plot3(wo(1),wo(2),wo(3),'rX');
plot3(wo(1),wo(2),wo(3),'rO');

% %绘制局部坐标系
Connect3D(op,pn,'r',2); hold on;
Connect3D(op,po,'b',2); hold on;
Connect3D(op,pa,'g',2);hold on;


axis equal;                             %%坐标轴比例对齐
axis([-20,20,-40,40,-20,20]);  %%调整坐标系范围
xlabel('x');                              %%显示坐标轴名称
ylabel('y'); 
zlabel('z');
view(163,19)                          %%设置显示视角
grid on;                                 %%绘制网格
if(fcla)                                   %%为1则显示持续0.01s
  pause(0.1);
end

drawnow;                            %%绘制



% view(145,33);
% axis equal
% axis([-10,10,-10,10,-10,10]);
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% grid on;


if(fcla)                                  %为1则擦除整幅图
    cla;
end

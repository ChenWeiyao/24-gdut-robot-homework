function SimTribody


ToDeg=180/pi;
ToRad=pi/180;

%% 设置坐标轴端点
px=[2,0,0]';
py=[0,2,0]';
pz=[0,0,2]';

px=oR*px+op;
py=oR*py+op;
pz=oR*pz+op;

%% 设置固定参考坐标轴端点
wx=[4,0,0]';
wy=[0,4,0]';
wz=[0,0,4]';
wo=[0,0,0]';

% %绘制固定参考坐标系
Connect3D(wo,wx,'r',0.5); hold on;
Connect3D(wo,wy,'b',0.5); hold on;
Connect3D(wo,wz,'g',0.5);hold on;
plot3(wo(1),wo(2),wo(3),'rX');
plot3(wo(1),wo(2),wo(3),'rO');

% %绘制局部坐标系
Connect3D(op,px,'r',2); hold on;
Connect3D(op,py,'b',2); hold on;
Connect3D(op,pz,'g',2);hold on;


view(145,33);
axis equal
axis([-10,10,-10,10,-10,10]);
xlabel('X');
ylabel('Y');
zlabel('Z');
grid on;
pause(0.1);
drawnow;

if(fcla)
    cla;
end

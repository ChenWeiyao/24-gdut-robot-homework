%�������
% oR 3x3 ��ת����op 1x3 λ�������� fcla ��ͼѡ�� 1-��ʾ������2-��ʾ����
function DrawFrame(oR, op, fcla)

ToDeg=180/pi;   
ToRad=pi/180;

%% ����������˵�
pn=[5,0,0]';
po=[0,5,0]';
pa=[0,0,5]';

pn=oR*pn+op;     %������̬����ԭ�����n��˵�����������ϵ�µ�����
po=oR*po+op;     %o��
pa=oR*pa+op;     %a��

%% ���ù̶��ο�������˵�
wx=[10,0,0]';
wy=[0,10,0]';
wz=[0,0,10]';
wo=[0,0,0]';

% %���ƹ̶��ο�����ϵ
Connect3D(wo,wx,'r',0.5); hold on;
Connect3D(wo,wy,'b',0.5); hold on;
Connect3D(wo,wz,'g',0.5);hold on;
plot3(wo(1),wo(2),wo(3),'rX');
plot3(wo(1),wo(2),wo(3),'rO');

% %���ƾֲ�����ϵ
Connect3D(op,pn,'r',2); hold on;
Connect3D(op,po,'b',2); hold on;
Connect3D(op,pa,'g',2);hold on;


axis equal;                             %%�������������
axis([-20,20,-40,40,-20,20]);  %%��������ϵ��Χ
xlabel('x');                              %%��ʾ����������
ylabel('y'); 
zlabel('z');
view(163,19)                          %%������ʾ�ӽ�
grid on;                                 %%��������
if(fcla)                                   %%Ϊ1����ʾ����0.01s
  pause(0.1);
end

drawnow;                            %%����



% view(145,33);
% axis equal
% axis([-10,10,-10,10,-10,10]);
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% grid on;


if(fcla)                                  %Ϊ1���������ͼ
    cla;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%        DD1�˲���ͼ�λ��ƺ���
%                           ������ƣ��ܽ�  ���ڣ�2007/6/4
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function []=ukf_fig(UKFTraceData,UKFIMUData,UKFSinsData,UKFKALData,UKFErrData,UKFGPSData);
g=9.7803698;         %�������ٶ�    ����λ����/��/�룩
%%%%%%%%%%%%%��������%%%%%%%%%
fig_num=0;

% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(UKFTraceData(:,1),UKFTraceData(:,8));
% subplot(3,1,2);plot(UKFTraceData(:,1),UKFTraceData(:,9));
% subplot(3,1,3);plot(UKFTraceData(:,1),UKFTraceData(:,10));
% xlabel('���к������棨���ȣ��ȣ���γ�ȣ��ȣ����߶ȣ��ף���');

% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(UKFTraceData(:,1),UKFTraceData(:,2));
% subplot(3,1,2);plot(UKFTraceData(:,1),UKFTraceData(:,3));
% subplot(3,1,3);plot(UKFTraceData(:,1),UKFTraceData(:,4));
% xlabel('���к������棨����Ƕȣ��ȣ��������Ƕȣ��ȣ�������Ƕȣ��ȣ���');
%    % ��������

% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(UKFIMUData(:,1),UKFIMUData(:,2));
% subplot(3,1,2);plot(UKFIMUData(:,1),UKFIMUData(:,3));
% subplot(3,1,3);plot(UKFIMUData(:,1),UKFIMUData(:,4));
% xlabel('IMU���棨����X�ᣨ��/Сʱ��������Y�ᣨ��/Сʱ��������Z�ᣨ��/Сʱ����');
% 
% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(UKFIMUData(:,1),UKFIMUData(:,5));
% subplot(3,1,2);plot(UKFIMUData(:,1),UKFIMUData(:,6));
% subplot(3,1,3);plot(UKFIMUData(:,1),UKFIMUData(:,7));
% xlabel('IMU���棨�ӱ�X��g�����ӱ�Y�ᣨg�����ӱ�Z�ᣨg����');
   %IMU����

% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(UKFGPSData(:,1),UKFGPSData(:,2));
% subplot(3,1,2);plot(UKFGPSData(:,1),UKFGPSData(:,3));
% subplot(3,1,3);plot(UKFGPSData(:,1),UKFGPSData(:,4));
% xlabel('GPS���棨���ȣ��ȣ���γ�ȣ��ȣ����߶ȣ��ף���');

% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(UKFGPSData(:,1),UKFGPSData(:,5));
% subplot(3,1,2);plot(UKFGPSData(:,1),UKFGPSData(:,6));
% subplot(3,1,3);plot(UKFGPSData(:,1),UKFGPSData(:,7));
% xlabel('GPS���棨�����ٶȣ���/�룩�������ٶȣ���/�룩�������ٶȣ���/�룩��');
   %GPS����

% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(UKFSinsData(:,1),UKFSinsData(:,2));
% subplot(3,1,2);plot(UKFSinsData(:,1),UKFSinsData(:,3));
% subplot(3,1,3);plot(UKFSinsData(:,1),UKFSinsData(:,4));
% xlabel('������̬�����������Ƕȣ��ȣ��������Ƕȣ��ȣ�������Ƕȣ��ȣ���');
%    %��̬�Ƕ�����
%    
% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(UKFSinsData(:,1),UKFSinsData(:,5));
% subplot(3,1,2);plot(UKFSinsData(:,1),UKFSinsData(:,6));
% subplot(3,1,3);plot(UKFSinsData(:,1),UKFSinsData(:,7));
% xlabel('�����ٶ������������/�룩��������/�룩��������/�룩��');
%    %�ٶ�����
% 
% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(UKFSinsData(:,1),UKFSinsData(:,8));
% subplot(3,1,2);plot(UKFSinsData(:,1),UKFSinsData(:,9));
% subplot(3,1,3);plot(UKFSinsData(:,1),UKFSinsData(:,10));
% xlabel('�������������ȣ��ȣ���γ�ȣ��ȣ����߶ȣ��ף���');
%    %λ������

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(UKFTraceData(:,1),UKFSinsData(:,2)-UKFTraceData(:,2));
subplot(3,1,2);plot(UKFTraceData(:,1),UKFSinsData(:,3)-UKFTraceData(:,3));
subplot(3,1,3);plot(UKFTraceData(:,1),UKFSinsData(:,4)-UKFTraceData(:,4));
xlabel('��̬�Ƕ�������Ƕ����ȣ��������Ƕ����ȣ�������Ƕ����ȣ���');
   %������̬�Ƕ��������    
   
fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(UKFTraceData(:,8),UKFTraceData(:,9),'r',UKFSinsData(:,8),UKFSinsData(:,9));
subplot(3,1,2);plot(UKFTraceData(:,1),UKFSinsData(:,8)-UKFTraceData(:,8));
subplot(3,1,3);plot(UKFTraceData(:,1),UKFSinsData(:,9)-UKFTraceData(:,9));
xlabel('��γ��λ�öԱȼ���������ߣ���γ��λ�ã��ȣ����������ȣ���γ�����ȣ���');
   %����λ�ù켣���������

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(UKFGPSData(:,1),UKFGPSData(:,2),'r',UKFSinsData(:,1),UKFSinsData(:,8));
subplot(3,1,2);plot(UKFGPSData(:,1),UKFGPSData(:,3),'r',UKFSinsData(:,1),UKFSinsData(:,9));
subplot(3,1,3);plot(UKFGPSData(:,1),UKFGPSData(:,4),'r',UKFSinsData(:,1),UKFSinsData(:,10));
xlabel('��ϵ�����GPS�Աȣ����ȣ��ȣ���γ�ȣ��ȣ����߶ȣ��ף���');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(UKFGPSData(:,1),UKFGPSData(:,5),'r',UKFSinsData(:,1),UKFSinsData(:,5));
subplot(3,1,2);plot(UKFGPSData(:,1),UKFGPSData(:,6),'r',UKFSinsData(:,1),UKFSinsData(:,6));
subplot(3,1,3);plot(UKFGPSData(:,1),UKFGPSData(:,7),'r',UKFSinsData(:,1),UKFSinsData(:,7));
xlabel('��ϵ�����GPS�Աȣ������ٶȣ���/�룩�������ٶȣ���/�룩�������ٶȣ���/�룩��');
    %��ϵ�����GPS�Ա����

fig_num = fig_num+1;
figure(fig_num);
subplot(4,1,1);plot(UKFKALData(:,1),UKFKALData(:,5));
subplot(4,1,2);plot(UKFKALData(:,1),UKFKALData(:,6));
subplot(4,1,3);plot(UKFKALData(:,1),UKFKALData(:,7));
subplot(4,1,4);plot(UKFKALData(:,1),UKFKALData(:,8));
xlabel('�������˲������ƽ̨����(��Ԫ��)���룩��');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(UKFKALData(:,1),UKFKALData(:,2));
subplot(3,1,2);plot(UKFKALData(:,1),UKFKALData(:,3));
subplot(3,1,3);plot(UKFKALData(:,1),UKFKALData(:,4));
xlabel('�������˲�������ٶ�����/�룩��');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(UKFKALData(:,1),UKFKALData(:,10));
subplot(3,1,2);plot(UKFKALData(:,1),UKFKALData(:,9));
subplot(3,1,3);plot(UKFKALData(:,1),UKFKALData(:,11));
xlabel('�������˲������λ�����ף���');

% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(UKFKALData(:,1),UKFKALData(:,12));
% subplot(3,1,2);plot(UKFKALData(:,1),UKFKALData(:,13));
% subplot(3,1,3);plot(UKFKALData(:,1),UKFKALData(:,14));
% xlabel('���������������/Сʱ��');
% 
% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(UKFKALData(:,1),UKFKALData(:,15));
% subplot(3,1,2);plot(UKFKALData(:,1),UKFKALData(:,16));
% subplot(3,1,3);plot(UKFKALData(:,1),UKFKALData(:,17));
% xlabel('����һ������ɷ�����/Сʱ��');

% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(UKFKALData(:,1),UKFKALData(:,18));
% subplot(3,1,2);plot(UKFKALData(:,1),UKFKALData(:,19));
% subplot(3,1,3);plot(UKFKALData(:,1),UKFKALData(:,20));
% xlabel('���ٶȼ�һ������ɷ���g��');
   %%%%%%%%%%�������������%%%%%%%%%%%
   
%return;

% fig_num = fig_num+1;
% figure(fig_num);
% subplot(4,1,1);plot(UKFErrData(:,1),UKFErrData(:,5)); %sec
% subplot(4,1,2);plot(UKFErrData(:,1),UKFErrData(:,6)); %sec
% subplot(4,1,3);plot(UKFErrData(:,1),UKFErrData(:,7)); %sec
% subplot(4,1,4);plot(UKFErrData(:,1),UKFErrData(:,8)); %sec
% xlabel('ƽ̨������(��Ԫ��)�����룩');

% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(UKFErrData(:,1),UKFErrData(:,2)); %��/��
% subplot(3,1,2);plot(UKFErrData(:,1),UKFErrData(:,3)); %��/��
% subplot(3,1,3);plot(UKFErrData(:,1),UKFErrData(:,4)); %��/��
% xlabel('�ٶ�����������/�룩');

% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(UKFErrData(:,1),UKFErrData(:,10)*180.0/pi); % deg
% subplot(3,1,2);plot(UKFErrData(:,1),UKFErrData(:,9)*180.0/pi); % deg
% subplot(3,1,3);plot(UKFErrData(:,1),UKFErrData(:,11));         % m
% xlabel('λ��������(���ȣ��ȣ���γ�ȣ��ȣ����߶ȣ��ף�)');

% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(UKFErrData(:,1),UKFErrData(:,12)*180.0/pi*3600); %deg/h
% subplot(3,1,2);plot(UKFErrData(:,1),UKFErrData(:,13)*180.0/pi*3600); %deg/h
% subplot(3,1,3);plot(UKFErrData(:,1),UKFErrData(:,14)*180.0/pi*3600); %deg/h
% xlabel('���������������������/Сʱ��');

% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(UKFErrData(:,1),UKFErrData(:,15)*180.0/pi*3600); %deg/h
% subplot(3,1,2);plot(UKFErrData(:,1),UKFErrData(:,16)*180.0/pi*3600); %deg/h
% subplot(3,1,3);plot(UKFErrData(:,1),UKFErrData(:,17)*180.0/pi*3600); %deg/h
% xlabel('����һ������ɷ�����������/Сʱ��');

% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(UKFErrData(:,1),UKFErrData(:,18)/g); %g
% subplot(3,1,2);plot(UKFErrData(:,1),UKFErrData(:,18)/g); %g
% subplot(3,1,3);plot(UKFErrData(:,1),UKFErrData(:,20)/g); %g
% xlabel('���ٶȼ�һ������ɷ���������g��');
% 


% fig_num = fig_num+1;
% figure(fig_num);
% plot3(UKFTraceData(:,8),UKFTraceData(:,9),UKFTraceData(:,10),'r');
% grid on
% xlabel('���к������棨���ȣ��ȣ���γ�ȣ��ȣ����߶ȣ��ף���');
% 
% fig_num = fig_num+1;
% figure(fig_num);
% plot3(UKFSinsData(:,8),UKFSinsData(:,9),UKFSinsData(:,10),'g');
% grid on
% xlabel('�������������ȣ��ȣ���γ�ȣ��ȣ����߶ȣ��ף���');
%   %�������˲��������� 
  
  
fig_num = fig_num+1;
figure(fig_num);
figure(fig_num);
subplot(3,1,1);plot(UKFSinsData(:,1),UKFSinsData(:,5)-UKFTraceData(:,4));
subplot(3,1,2);plot(UKFSinsData(:,1),UKFSinsData(:,6)-UKFTraceData(:,5));
subplot(3,1,3);plot(UKFSinsData(:,1),UKFSinsData(:,7)-UKFTraceData(:,6));
xlabel('��ϵ����ٶ��������ٶȣ���/�룩�������ٶȣ���/�룩�������ٶȣ���/�룩��');

fig_num = fig_num+1;
figure(fig_num);
plot(UKFTraceData(:,8),UKFTraceData(:,9),'r',UKFSinsData(:,8),UKFSinsData(:,9),'g');
grid on
xlabel('���к������棨���ȣ��ȣ���γ�ȣ��ȣ�');
ylabel('�������������ȣ��ȣ���γ�ȣ��ȣ�');

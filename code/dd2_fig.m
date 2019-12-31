%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%        DD1�˲���ͼ�λ��ƺ���
%                           ������ƣ��ܽ�  ���ڣ�2007/6/4
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function []=dd2_fig(DD2TraceData,DD2IMUData,DD2SinsData,DD2KALData,DD2ErrData,DD2GPSData)
g=9.7803698;         %�������ٶ�    ����λ����/��/�룩
%%%%%%%%%%%%%��������%%%%%%%%%
fig_num=0;

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(DD2TraceData(:,1),DD2TraceData(:,8));
subplot(3,1,2);plot(DD2TraceData(:,1),DD2TraceData(:,9));
subplot(3,1,3);plot(DD2TraceData(:,1),DD2TraceData(:,10));
xlabel('���к������棨���ȣ��ȣ���γ�ȣ��ȣ����߶ȣ��ף���');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(DD2TraceData(:,1),DD2TraceData(:,2));
subplot(3,1,2);plot(DD2TraceData(:,1),DD2TraceData(:,3));
subplot(3,1,3);plot(DD2TraceData(:,1),DD2TraceData(:,4));
xlabel('���к������棨����Ƕȣ��ȣ��������Ƕȣ��ȣ�������Ƕȣ��ȣ���');
   % ��������

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(DD2IMUData(:,1),DD2IMUData(:,2));
subplot(3,1,2);plot(DD2IMUData(:,1),DD2IMUData(:,3));
subplot(3,1,3);plot(DD2IMUData(:,1),DD2IMUData(:,4));
xlabel('IMU���棨����X�ᣨ��/Сʱ��������Y�ᣨ��/Сʱ��������Z�ᣨ��/Сʱ����');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(DD2IMUData(:,1),DD2IMUData(:,5));
subplot(3,1,2);plot(DD2IMUData(:,1),DD2IMUData(:,6));
subplot(3,1,3);plot(DD2IMUData(:,1),DD2IMUData(:,7));
xlabel('IMU���棨�ӱ�X��g�����ӱ�Y�ᣨg�����ӱ�Z�ᣨg����');
   %IMU����

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(DD2GPSData(:,1),DD2GPSData(:,2));
subplot(3,1,2);plot(DD2GPSData(:,1),DD2GPSData(:,3));
subplot(3,1,3);plot(DD2GPSData(:,1),DD2GPSData(:,4));
xlabel('GPS���棨���ȣ��ȣ���γ�ȣ��ȣ����߶ȣ��ף���');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(DD2GPSData(:,1),DD2GPSData(:,5));
subplot(3,1,2);plot(DD2GPSData(:,1),DD2GPSData(:,6));
subplot(3,1,3);plot(DD2GPSData(:,1),DD2GPSData(:,7));
xlabel('GPS���棨�����ٶȣ���/�룩�������ٶȣ���/�룩�������ٶȣ���/�룩��');
   %GPS����

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(DD2SinsData(:,1),DD2SinsData(:,2));
subplot(3,1,2);plot(DD2SinsData(:,1),DD2SinsData(:,3));
subplot(3,1,3);plot(DD2SinsData(:,1),DD2SinsData(:,4));
xlabel('������̬�����������Ƕȣ��ȣ��������Ƕȣ��ȣ�������Ƕȣ��ȣ���');
   %��̬�Ƕ�����
   
fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(DD2SinsData(:,1),DD2SinsData(:,5));
subplot(3,1,2);plot(DD2SinsData(:,1),DD2SinsData(:,6));
subplot(3,1,3);plot(DD2SinsData(:,1),DD2SinsData(:,7));
xlabel('�����ٶ������������/�룩��������/�룩��������/�룩��');
   %�ٶ�����

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(DD2SinsData(:,1),DD2SinsData(:,8));
subplot(3,1,2);plot(DD2SinsData(:,1),DD2SinsData(:,9));
subplot(3,1,3);plot(DD2SinsData(:,1),DD2SinsData(:,10));
xlabel('�������������ȣ��ȣ���γ�ȣ��ȣ����߶ȣ��ף���');
   %λ������

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(DD2TraceData(:,1),DD2SinsData(:,2)-DD2TraceData(:,2));
subplot(3,1,2);plot(DD2TraceData(:,1),DD2SinsData(:,3)-DD2TraceData(:,3));
subplot(3,1,3);plot(DD2TraceData(:,1),DD2SinsData(:,4)-DD2TraceData(:,4));
xlabel('��̬�Ƕ�������Ƕ����ȣ��������Ƕ����ȣ�������Ƕ����ȣ���');
   %������̬�Ƕ��������    
   
fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(DD2TraceData(:,8),DD2TraceData(:,9),'r',DD2SinsData(:,8),DD2SinsData(:,9));
subplot(3,1,2);plot(DD2TraceData(:,1),DD2SinsData(:,8)-DD2TraceData(:,8));
subplot(3,1,3);plot(DD2TraceData(:,1),DD2SinsData(:,9)-DD2TraceData(:,9));
xlabel('��γ��λ�öԱȼ���������ߣ���γ��λ�ã��ȣ����������ȣ���γ�����ȣ���');
   %����λ�ù켣���������

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(DD2GPSData(:,1),DD2GPSData(:,2),'r',DD2SinsData(:,1),DD2SinsData(:,8));
subplot(3,1,2);plot(DD2GPSData(:,1),DD2GPSData(:,3),'r',DD2SinsData(:,1),DD2SinsData(:,9));
subplot(3,1,3);plot(DD2GPSData(:,1),DD2GPSData(:,4),'r',DD2SinsData(:,1),DD2SinsData(:,10));
xlabel('��ϵ�����GPS�Աȣ����ȣ��ȣ���γ�ȣ��ȣ����߶ȣ��ף���');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(DD2GPSData(:,1),DD2GPSData(:,5),'r',DD2SinsData(:,1),DD2SinsData(:,5));
subplot(3,1,2);plot(DD2GPSData(:,1),DD2GPSData(:,6),'r',DD2SinsData(:,1),DD2SinsData(:,6));
subplot(3,1,3);plot(DD2GPSData(:,1),DD2GPSData(:,7),'r',DD2SinsData(:,1),DD2SinsData(:,7));
xlabel('��ϵ�����GPS�Աȣ������ٶȣ���/�룩�������ٶȣ���/�룩�������ٶȣ���/�룩��');
    %��ϵ�����GPS�Ա����

fig_num = fig_num+1;
figure(fig_num);
subplot(4,1,1);plot(DD2KALData(:,1),DD2KALData(:,5));
subplot(4,1,2);plot(DD2KALData(:,1),DD2KALData(:,6));
subplot(4,1,3);plot(DD2KALData(:,1),DD2KALData(:,7));
subplot(4,1,4);plot(DD2KALData(:,1),DD2KALData(:,8));
xlabel('�������˲������ƽ̨����(��Ԫ��)���룩��');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(DD2KALData(:,1),DD2KALData(:,2));
subplot(3,1,2);plot(DD2KALData(:,1),DD2KALData(:,3));
subplot(3,1,3);plot(DD2KALData(:,1),DD2KALData(:,4));
xlabel('�������˲�������ٶ�����/�룩��');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(DD2KALData(:,1),DD2KALData(:,10));
subplot(3,1,2);plot(DD2KALData(:,1),DD2KALData(:,9));
subplot(3,1,3);plot(DD2KALData(:,1),DD2KALData(:,11));
xlabel('�������˲������λ�����ף���');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(DD2KALData(:,1),DD2KALData(:,12));
subplot(3,1,2);plot(DD2KALData(:,1),DD2KALData(:,13));
subplot(3,1,3);plot(DD2KALData(:,1),DD2KALData(:,14));
xlabel('���������������/Сʱ��');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(DD2KALData(:,1),DD2KALData(:,15));
subplot(3,1,2);plot(DD2KALData(:,1),DD2KALData(:,16));
subplot(3,1,3);plot(DD2KALData(:,1),DD2KALData(:,17));
xlabel('����һ������ɷ�����/Сʱ��');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(DD2KALData(:,1),DD2KALData(:,18));
subplot(3,1,2);plot(DD2KALData(:,1),DD2KALData(:,19));
subplot(3,1,3);plot(DD2KALData(:,1),DD2KALData(:,20));
xlabel('���ٶȼ�һ������ɷ���g��');
   %%%%%%%%%%�������������%%%%%%%%%%%
   
%return;

fig_num = fig_num+1;
figure(fig_num);
subplot(4,1,1);plot(DD2ErrData(:,1),DD2ErrData(:,5)); %sec
subplot(4,1,2);plot(DD2ErrData(:,1),DD2ErrData(:,6)); %sec
subplot(4,1,3);plot(DD2ErrData(:,1),DD2ErrData(:,7)); %sec
subplot(4,1,4);plot(DD2ErrData(:,1),DD2ErrData(:,8)); %sec
xlabel('ƽ̨������(��Ԫ��)�����룩');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(DD2ErrData(:,1),DD2ErrData(:,2)); %��/��
subplot(3,1,2);plot(DD2ErrData(:,1),DD2ErrData(:,3)); %��/��
subplot(3,1,3);plot(DD2ErrData(:,1),DD2ErrData(:,4)); %��/��
xlabel('�ٶ�����������/�룩');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(DD2ErrData(:,1),DD2ErrData(:,10)*180.0/pi); % deg
subplot(3,1,2);plot(DD2ErrData(:,1),DD2ErrData(:,9)*180.0/pi); % deg
subplot(3,1,3);plot(DD2ErrData(:,1),DD2ErrData(:,11));         % m
xlabel('λ��������(���ȣ��ȣ���γ�ȣ��ȣ����߶ȣ��ף�)');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(DD2ErrData(:,1),DD2ErrData(:,12)*180.0/pi*3600); %deg/h
subplot(3,1,2);plot(DD2ErrData(:,1),DD2ErrData(:,13)*180.0/pi*3600); %deg/h
subplot(3,1,3);plot(DD2ErrData(:,1),DD2ErrData(:,14)*180.0/pi*3600); %deg/h
xlabel('���������������������/Сʱ��');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(DD2ErrData(:,1),DD2ErrData(:,15)*180.0/pi*3600); %deg/h
subplot(3,1,2);plot(DD2ErrData(:,1),DD2ErrData(:,16)*180.0/pi*3600); %deg/h
subplot(3,1,3);plot(DD2ErrData(:,1),DD2ErrData(:,17)*180.0/pi*3600); %deg/h
xlabel('����һ������ɷ�����������/Сʱ��');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(DD2ErrData(:,1),DD2ErrData(:,18)/g); %g
subplot(3,1,2);plot(DD2ErrData(:,1),DD2ErrData(:,18)/g); %g
subplot(3,1,3);plot(DD2ErrData(:,1),DD2ErrData(:,20)/g); %g
xlabel('���ٶȼ�һ������ɷ���������g��');
  %�������˲���������
    
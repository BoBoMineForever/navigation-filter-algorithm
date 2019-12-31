function figure_ukf
t_stop=1200;
[UKFTraceData,UKFIMUData,UKFSinsData,UKFKALData,UKFErrData,UKFGPSData]=ukfm_sins_gps(t_stop);
[AUKFTraceData,AUKFIMUData,AUKFSinsData,AUKFKALData,AUKFErrData,AUKFGPSData]=shuaijianukfm_sins_gps(t_stop);
[SUKFTraceData,SUKFIMUData,SUKFSinsData,SUKFKALData,SUKFErrData,SUKFGPSData]=sf_ukfm_sins_gps(t_stop);

% fig_num=0.0;
% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(UKFTraceData(:,1),UKFSinsData(:,2)-UKFTraceData(:,2),'r',AUKFTraceData(:,1),AUKFSinsData(:,2)-AUKFTraceData(:,2),'b');legend('UKF','AUKF');
% subplot(3,1,2);plot(UKFTraceData(:,1),UKFSinsData(:,3)-UKFTraceData(:,3),'r',AUKFTraceData(:,1),AUKFSinsData(:,3)-AUKFTraceData(:,3),'b');legend('UKF','AUKF');
% subplot(3,1,3);plot(UKFTraceData(:,1),UKFSinsData(:,4)-UKFTraceData(:,4),'r',AUKFTraceData(:,1),AUKFSinsData(:,4)-AUKFTraceData(:,4),'b');legend('UKF','AUKF');
% xlabel('��̬�Ƕ�������Ƕ����ȣ��������Ƕ����ȣ�������Ƕ����ȣ���');
%    %������̬�Ƕ��������    
fig_num=0.0;
fig_num = fig_num+1;
figure(fig_num);plot(UKFTraceData(:,1),(UKFSinsData(:,2)-UKFTraceData(:,2))*3600,'r',AUKFTraceData(:,1),(AUKFSinsData(:,2)-AUKFTraceData(:,2))*3600,'b',SUKFTraceData(:,1),(SUKFSinsData(:,2)-SUKFTraceData(:,2))*3600,'g');legend('UKF','FUKF','SUKF');
xlabel('t/s');
ylabel('����Ƕ���"��');
fig_num = fig_num+1;
figure(fig_num);plot(UKFTraceData(:,1),(UKFSinsData(:,3)-UKFTraceData(:,3))*3600,'r',AUKFTraceData(:,1),(AUKFSinsData(:,3)-AUKFTraceData(:,3))*3600,'b',SUKFTraceData(:,1),(SUKFSinsData(:,3)-SUKFTraceData(:,3))*3600,'g');legend('UKF','FUKF','SUKF');
xlabel('t/s');
ylabel('�����Ƕ���"��');
fig_num = fig_num+1;
figure(fig_num);plot(UKFTraceData(:,1),(UKFSinsData(:,4)-UKFTraceData(:,4))*60,'r',AUKFTraceData(:,1),(AUKFSinsData(:,4)-AUKFTraceData(:,4))*60,'b',SUKFTraceData(:,1),(SUKFSinsData(:,3)-SUKFTraceData(:,3))*60,'g');legend('UKF','FUKF','SUKF');
xlabel('t/s');
ylabel('����Ƕ���''��');  
%������̬�Ƕ��������    
  
fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(UKFSinsData(:,1),UKFSinsData(:,5)-UKFTraceData(:,5),'r',AUKFSinsData(:,1),AUKFSinsData(:,5)-AUKFTraceData(:,5),'b',SUKFSinsData(:,1),SUKFSinsData(:,5)-SUKFTraceData(:,5),'g');legend('UKF','FUKF','SUKF');
ylabel('�����ٶ�����/�룩');xlabel('t/s');
subplot(3,1,2);plot(UKFSinsData(:,1),UKFSinsData(:,6)-UKFTraceData(:,6),'r',AUKFSinsData(:,1),AUKFSinsData(:,6)-AUKFTraceData(:,6),'b',SUKFSinsData(:,1),SUKFSinsData(:,6)-SUKFTraceData(:,6),'g');legend('UKF','FUKF','SUKF');
ylabel('�����ٶ�����/�룩');xlabel('t/s');
subplot(3,1,3);plot(UKFSinsData(:,1),UKFSinsData(:,7)-UKFTraceData(:,7),'r',AUKFSinsData(:,1),AUKFSinsData(:,7)-AUKFTraceData(:,7),'b',SUKFSinsData(:,1),SUKFSinsData(:,7)-SUKFTraceData(:,7),'g');legend('UKF','FUKF','SUKF');
ylabel('�����ٶ�����/�룩');xlabel('t/s');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(UKFSinsData(:,1),UKFSinsData(:,8)-UKFTraceData(:,8),'r',AUKFSinsData(:,1),AUKFSinsData(:,8)-AUKFTraceData(:,8),'b',SUKFSinsData(:,1),SUKFSinsData(:,8)-SUKFTraceData(:,8),'g');legend('UKF','FUKF','SUKF');
ylabel('�������ף�');
xlabel('t/s');
subplot(3,1,2);plot(UKFSinsData(:,1),UKFSinsData(:,9)-UKFTraceData(:,9),'r',AUKFSinsData(:,1),AUKFSinsData(:,9)-AUKFTraceData(:,9),'b',SUKFSinsData(:,1),SUKFSinsData(:,9)-SUKFTraceData(:,9),'g');legend('UKF','FUKF','SUKF');
ylabel('γ�����ף�');
xlabel('t/s');
subplot(3,1,3);plot(UKFSinsData(:,1),UKFSinsData(:,10)-UKFTraceData(:,10),'r',AUKFSinsData(:,1),AUKFSinsData(:,10)-AUKFTraceData(:,10),'b',SUKFSinsData(:,1),SUKFSinsData(:,10)-SUKFTraceData(:,10),'g');legend('UKF','FUKF','SUKF');
ylabel('�߶����ף�');
xlabel('t/s');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(UKFKALData(:,1),UKFKALData(:,2),'r',AUKFKALData(:,1),AUKFKALData(:,2),'b',SUKFKALData(:,1),SUKFKALData(:,2),'g');legend('UKF','FUKF','SUKF');
subplot(3,1,2);plot(UKFKALData(:,1),UKFKALData(:,3),'r',AUKFKALData(:,1),AUKFKALData(:,3),'b',SUKFKALData(:,1),SUKFKALData(:,3),'g');legend('UKF','FUKF','SUKF');
subplot(3,1,3);plot(UKFKALData(:,1),UKFKALData(:,4),'r',AUKFKALData(:,1),AUKFKALData(:,4),'b',SUKFKALData(:,1),SUKFKALData(:,4),'g');legend('UKF','FUKF','SUKF');
xlabel('�������˲�������ٶ�����/�룩��');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(UKFKALData(:,1),UKFKALData(:,10),'r',AUKFKALData(:,1),AUKFKALData(:,10),'b',SUKFKALData(:,1),SUKFKALData(:,10),'g');legend('UKF','FUKF','SUKF');
subplot(3,1,2);plot(UKFKALData(:,1),UKFKALData(:,9),'r',AUKFKALData(:,1),AUKFKALData(:,9),'b',SUKFKALData(:,1),SUKFKALData(:,9),'g');legend('UKF','FUKF','SUKF');
subplot(3,1,3);plot(UKFKALData(:,1),UKFKALData(:,11),'r',AUKFKALData(:,1),AUKFKALData(:,11),'b',SUKFKALData(:,1),SUKFKALData(:,11),'g');legend('UKF','FUKF','SUKF');
xlabel('�������˲������λ�����ף���');

fig_num = fig_num+1;
figure(fig_num);
plot(UKFTraceData(:,8),UKFTraceData(:,9),'r',UKFSinsData(:,8),UKFSinsData(:,9),'g');
grid on
xlabel('���к������棨���ȣ��ȣ���γ�ȣ��ȣ�');
ylabel('�������������ȣ��ȣ���γ�ȣ��ȣ�');

fig_num = fig_num+1;
figure(fig_num);
plot3(AUKFTraceData(:,8),AUKFTraceData(:,9),AUKFTraceData(:,10),'r');
grid on
xlabel('���ȣ��ȣ�');
ylabel('γ�ȣ��ȣ�');
zlabel('�߶ȣ��ף�');

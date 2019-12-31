function []=integrated_err_com
Ts=10;
% for i=1:Ts
%     DDD1(:,:,i)=dd1m_sins_gps;
%     disp('���д���');
%     disp(i);
% end
tic;
for i=1:Ts
    DDD2(:,:,i)=dd2m_sins_gps;
    disp('���д���');
    disp(i);
end
TDD2PF=toc;
tic;
for i=1:Ts
    DUKF(:,:,i)=ukfm_sins_gps;
    disp('���д���');
    disp(i+Ts);
end
TUPF=toc;
% %%%%%%%%%%%%%%%%%���뺽������%%%%%%%%%%%%%%%%%%
load dd2trace.dat;    %���뺽������
% % load dd1kal.dat;  %����Э�������� 
% % load dd2kal.dat;  %����Э�������� 
% % load ukfkal.dat;  %����Э�������� 
% load MDDD2.dat;    %�洢DD2PF�������
% load MDUKF.dat;        %�洢UPF�������
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MDDD1=0;
MDDD2=0;
MDUKF=0;
for i=1:Ts
%     MDDD1=MDDD1+(dd1trace-DDD1(:,:,i));
    MDDD2=MDDD2+(dd2trace-DDD2(:,:,i));
    MDUKF=MDUKF+(dd2trace-DUKF(:,:,i));
end 
% MDDD1=(MDDD1/Ts);
MDDD2=abs(MDDD2/Ts);
MDUKF=abs(MDUKF/Ts);
% %%%%%%%%%%%%%�����������%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% save MDDD1.dat MDDD1 -ASCII -double;        %�洢EPF�������
save MDDD2.dat MDDD2 -ASCII -double;    %�洢DD2PF�������
save MDUKF.dat MDUKF -ASCII -double;        %�洢UPF�������
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RMDD1=0;
RMDD2=0;
RMUKF=0;
for i=1:Ts
% RMDD1=(dd2trace-DDD1(:,:,i)).^2;
RMDD2=(dd2trace-DDD2(:,:,i)).^2;
RMUKF=(dd2trace-DUKF(:,:,i)).^2;
% RMSDD1(i,:)=sqrt(mean(RMDD1));
RMSDD2(i,:)=sqrt(mean(RMDD2));
RMSUKF(i,:)=sqrt(mean(RMUKF));
end
if Ts>1
%     RMSEDD1=(mean(RMSDD1));
    RMSEDD2=(mean(RMSDD2));
    RMSEUKF=(mean(RMSUKF));
    RMSE=[RMSEDD2;RMSEUKF];
else 
%     RMSEDD1=RMSDD1;
    RMSEDD2=RMSDD2;
    RMSEUKF=RMSUKF;
    RMSE=[RMSEDD2;RMSEUKF];
end  
disp('˳��Ϊ  DD1  DD2  UKF');
disp('RMSE ���      ����      ����      �����ٶ�     �����ٶ�    �����ٶ�     ����      γ��    �߶�');
disp(RMSE(:,2:10));
if Ts>1
DRMSDD2=0;
DRMSUKF=0;        
for i=1:Ts
DRMSDD2=DRMSDD2+(RMSDD2(i,:)-RMSEDD2).^2;
DRMSUKF=DRMSUKF+(RMSUKF(i,:)-RMSEUKF).^2;
end
DRMSDD2=DRMSDD2/(Ts-1);
DRMSUKF=DRMSUKF/(Ts-1);
DRMSE=[DRMSDD2;DRMSUKF];
disp(DRMSE(:,2:10));
save DRMSE.dat DRMSE -ASCII -double;        %�洢RMSE
end
save RMSE.dat RMSE -ASCII -double;        %�洢RMSE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fig_num=0;

fig_num = fig_num+1;
figure(fig_num);
%plot(dd2trace(:,1),MDDD1(:,2),':r',dd2trace(:,1),MDUKF(:,2),'-b',dd2trace(:,1),MDDD2(:,2),'--g');
plot(dd2trace(:,1),MDUKF(:,2),'-r',dd2trace(:,1),MDDD2(:,2),'--b');
% plot(dd2trace(:,1),MDDD2(:,2),'-b');
% legend('DD2');
legend('UKF','DD2');
grid on
ylabel('����Ƕ����(deg)');
xlabel('ʱ��(sec)');
fig_num = fig_num+1;
figure(fig_num);
%plot(dd2trace(:,1),MDDD1(:,3),':r',dd2trace(:,1),MDUKF(:,3),'-b',dd2trace(:,1),MDDD2(:,3),'--g');
plot(dd2trace(:,1),MDUKF(:,3),'-r',dd2trace(:,1),MDDD2(:,3),'-b');
% plot(dd2trace(:,1),MDDD2(:,3),'-b');
% legend('DD2');
legend('UKF','DD2');
grid on
ylabel('�����Ƕ����(deg)');
xlabel('ʱ��(sec)');
fig_num = fig_num+1;
figure(fig_num);
%plot(dd2trace(:,1),MDDD1(:,4),':r',dd2trace(:,1),MDUKF(:,4),'-b',dd2trace(:,1),MDDD2(:,4),'--g');
plot(dd2trace(:,1),MDUKF(:,4),'-r',dd2trace(:,1),MDDD2(:,4),'-b');
% plot(dd2trace(:,1),MDDD2(:,4),'-b');
% legend('DD2');
legend('UKF','DD2');
grid on
ylabel('����Ƕ����(deg)');
xlabel('ʱ��(sec)');


fig_num = fig_num+1;
figure(fig_num);
%plot(dd2trace(:,1),MDDD1(:,5),':r',dd2trace(:,1),MDUKF(:,5),'-b',dd2trace(:,1),MDDD2(:,5),'--g');
plot(dd2trace(:,1),MDUKF(:,5),'-r',dd2trace(:,1),MDDD2(:,5),'-b');
% plot(dd2trace(:,1),MDDD2(:,5),'-b');
% legend('DD2');
legend('UKF','DD2');
grid on
ylabel('�����ٶ����(m/sec)');
xlabel('ʱ��(sec)');
fig_num = fig_num+1;
figure(fig_num);
%plot(dd2trace(:,1),MDDD1(:,6),':r',dd2trace(:,1),MDUKF(:,6),'-b',dd2trace(:,1),MDDD2(:,6),'--g');
plot(dd2trace(:,1),MDUKF(:,6),'-r',dd2trace(:,1),MDDD2(:,6),'-b');
% plot(dd2trace(:,1),MDDD2(:,6),'-b');
% legend('DD2');
legend('UKF','DD2');
grid on
ylabel('�����ٶ����(m/sec)');
xlabel('ʱ��(sec)');
fig_num = fig_num+1;
figure(fig_num);
%plot(dd2trace(:,1),MDDD1(:,7),':r',dd2trace(:,1),MDUKF(:,7),'-b',dd2trace(:,1),MDDD2(:,7),'--g');
plot(dd2trace(:,1),MDUKF(:,7),'-r',dd2trace(:,1),MDDD2(:,7),'-b');
% plot(dd2trace(:,1),MDDD2(:,7),'-b');
% legend('DD2');
legend('UKF','DD2');
grid on
ylabel('�����ٶ����(m/sec)');
xlabel('ʱ��(sec)');

 

fig_num = fig_num+1;
figure(fig_num);
%plot(dd2trace(:,1),MDDD1(:,8),':r',dd2trace(:,1),MDUKF(:,8),'-b',dd2trace(:,1),MDDD2(:,8),'--g');
plot(dd2trace(:,1),MDUKF(:,8),'-r',dd2trace(:,1),MDDD2(:,8),'-b');
% plot(dd2trace(:,1),MDDD2(:,8),'-b');
% legend('DD2');
legend('UKF','DD2');
grid on
ylabel('�������(m)');
xlabel('ʱ��(sec)');
fig_num = fig_num+1;
figure(fig_num);
%plot(dd2trace(:,1),MDDD1(:,9),':r',dd2trace(:,1),MDUKF(:,9),'-b',dd2trace(:,1),MDDD2(:,9),'--g');
plot(dd2trace(:,1),MDUKF(:,9),'-r',dd2trace(:,1),MDDD2(:,9),'-b');
% plot(dd2trace(:,1),MDDD2(:,9),'-b');
% legend('DD2');
legend('UKF','DD2');
grid on
ylabel('γ�����(m)');
xlabel('ʱ��(sec)');
fig_num = fig_num+1;
figure(fig_num);
%plot(dd2trace(:,1),MDDD1(:,10),':r',dd2trace(:,1),MDUKF(:,10),'-b',dd2trace(:,1),MDDD2(:,10),'--g');
plot(dd2trace(:,1),MDUKF(:,10),'-r',dd2trace(:,1),MDDD2(:,10),'-b');
% plot(dd2trace(:,1),MDDD2(:,10),'-b');
% legend('DD2');
legend('UKF','DD2');
grid on    
ylabel('�߶����(m)');
xlabel('ʱ��(sec)');



% fig_num = fig_num+1;
% figure(fig_num);
% subplot(4,1,1);plot(dd1kal(:,1),dd1kal(:,5),':r',dd2kal(:,1),dd2kal(:,5),'--g',ukfkal(:,1),ukfkal(:,5),'-b');
% subplot(4,1,2);plot(dd1kal(:,1),dd1kal(:,6),':r',dd2kal(:,1),dd2kal(:,6),'--g',ukfkal(:,1),ukfkal(:,6),'-b');
% subplot(4,1,3);plot(dd1kal(:,1),dd1kal(:,7),':r',dd2kal(:,1),dd2kal(:,7),'--g',ukfkal(:,1),ukfkal(:,7),'-b');
% subplot(4,1,4);plot(dd1kal(:,1),dd1kal(:,8),':r',dd2kal(:,1),dd2kal(:,8),'--g',ukfkal(:,1),ukfkal(:,8),'-b');
% xlabel('�������˲������ƽ̨����(��Ԫ��)���룩��');
% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(dd1kal(:,1),dd1kal(:,2),':r',dd2kal(:,1),dd2kal(:,2),'--g',ukfkal(:,1),ukfkal(:,2),'-b');
% subplot(3,1,2);plot(dd1kal(:,1),dd1kal(:,3),':r',dd2kal(:,1),dd2kal(:,3),'--g',ukfkal(:,1),ukfkal(:,3),'-b');
% subplot(3,1,3);plot(dd1kal(:,1),dd1kal(:,4),':r',dd2kal(:,1),dd2kal(:,4),'--g',ukfkal(:,1),ukfkal(:,4),'-b');
% xlabel('�������˲�������ٶ�����/�룩��');
% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(dd1kal(:,1),dd1kal(:,10),':r',dd2kal(:,1),dd2kal(:,10),'--g',ukfkal(:,1),ukfkal(:,10),'-b');
% subplot(3,1,2);plot(dd1kal(:,1),dd1kal(:,9),':r',dd2kal(:,1),dd2kal(:,9),'--g',ukfkal(:,1),ukfkal(:,9),'-b');
% subplot(3,1,3);plot(dd1kal(:,1),dd1kal(:,11),':r',dd2kal(:,1),dd2kal(:,11),'--g',ukfkal(:,1),ukfkal(:,11),'-b');
% xlabel('�������˲������λ�����ף���');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%          SINSЭ�������
%                           ������ƣ�����  ���ڣ�2002/8/9
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%��������%%%%%%%%%%%
deg_rad=0.01745329252e0;% Transfer from angle degree to rad
g=9.7803698;         %�������ٶ�    ����λ����/��/�룩

%%%%%%%%����ʱ������%%%%%%
t=0;
T=1;
t_stop=5000.0;

%%%%%%%%%%%%%%%%����������%%%%%%%%%%%%%%%%%
atti=zeros(3,1);     %��������������򣨵�λ���ȣ�
atti_rate=zeros(3,1);%������ʡ��������ʡ��������ʣ���λ����/�룩
veloB=zeros(3,1);    %�ɻ��˶��ٶȡ���X����Y��ͷ��Z���򣨵�λ����/�룩
acceB=zeros(3,1);    %�ɻ��˶����ٶȡ���X����Y��ͷ��Z���򣨵�λ����/��/�룩
posi=zeros(3,1);     %������������ʼλ�þ��ȡ�γ�ȡ��߶ȣ���λ���ȡ��ȡ��ף�
posi=[106.491;29.528;300.0];  

atti(1,1)=0.0;
atti(2,1)=0.0;
atti(3,1)=90.0;  %��ʼ����Ƕȣ���λ���ȣ�

veloB(2,1)=0.0; %�ɻ���ʼ�˶��ٶȡ�����ͷ����λ����/�룩

%%%%%%%%%%%%%%%%%%%%IMU���%%%%%%%%%%
Wibb=zeros(3,1);    %����ϵ���������   ����λ����/�룩
Fb=zeros(3,1);      %����ϵ���ٶȼ���� ����λ����/��/�룩

Gyro_fix=zeros(3,1);%����ϵ�����ǹ̶�������   ����λ������/�룩
Acc_fix=zeros(3,1); %����ϵ���ٶȼƹ̶������� ����λ����/��/�룩
Gyro_b=zeros(3,1);  % �����������������/�룩
Gyro_r=zeros(3,1);  % ����һ������ɷ���̣�����/�룩
Gyro_wg=zeros(3,1); %���ݰ�����������/�룩
Acc_r =zeros(3,1);  % ���ٶ�һ������ɷ���̣���/��/�룩

%%%%%%%%%%%%%%%%%%GPS�������%%%%%%%%%%%%%%
posiG = zeros(3,1); %GPS����ķ�����λ�ã����ȣ��ȣ���γ�ȣ��ȣ����߶ȣ��ף����� 
veloG = zeros(3,1); %GPS����ķ������ٶȣ�������/�룩��������/�룩��������/�룩��


%%%%%%%%%%%%%%%%%%%%%�����ߵ�����%%%%%%%%%%%%%
attiN=zeros(3,1);        %��������ʼ��̬
veloN=zeros(3,1);        %��������ʼ�ٶȣ�����ڵ���ϵ��
posiN=zeros(3,1);        %��������ʼλ��
WnbbA_old=zeros(3,1);    %���ٶȻ����������λ�����ȣ�

posiN=posi;              %��ʼλ���뺽��λ��һ��
attiN=atti;              %��ʼ��̬�뺽����̬һ�£������ó�ʼ��׼�����滻��

%%%%%%%%%%%%%%%%%%%%%%%KALMAN�˲����%%%%%%%%%%%%%%%%%
T_D = 1;    %��ɢ���ڣ�
T_M = 0;    %�˲��������ʱ�䣨�룩��
Xc = zeros(18,1);  %�ۺ�ģ��״̬����(�������������)ȫ��Ϊ���ʵ�λ��
PK = zeros(18,18); %Э������
Xerr =zeros(1,18); %״̬�����������ֵ����¼ĳ��ʱ�̵ģ�
kflag=0;           %GPS��Ϣ��Ч��־λ��1����Ч��

Acc_modi = zeros(3,1); %���ٶȼ��������ֵ����/��/�룩��X,Y,Z��
Gyro_modi= zeros(3,1); %�����������ֵ(����/��)(X,Y,Z)

%%%%%%%%%%%%%%%%%%%��ʼ��׼%%%%%%%%%%%%%%%%%%%
kc=0;
tmp_Fb=zeros(3,1);
tmp_Wibb=zeros(3,1);
t_alig  = 0;
while t<=120
  [t_alig,atti,atti_rate,veloB,acceB]=trace(0,T,atti,atti_rate,veloB,acceB);
      %�����������������й켣����
  [Wibb,Fb,posi]=IMUout(T,posi,atti,atti_rate,veloB,acceB);
      %���ݺͼ��ٶȷ��� 

  [Gyro_fix,Acc_fix]=imu_err_fix(Wibb,Fb);
      %��������������
  %Fb=Fb+Acc_fix;  
  %Wibb=Wibb+Gyro_fix/deg_rad;  % deg/s
      %�������������

  [Gyro_b,Gyro_r,Gyro_wg,Acc_r]=imu_err_random(t,T,Gyro_b,Gyro_r,Gyro_wg,Acc_r);
      %�������������ʼֵ����ʱIMU������������״̬�������������Ϊ��ֵ��
  Fb=Fb+Acc_r;
  Wibb=Wibb+Gyro_b/deg_rad+Gyro_r/deg_rad+Gyro_wg/deg_rad;  % deg/s
      %������������

  tmp_Fb=tmp_Fb+Fb;
  tmp_Wibb=tmp_Wibb+Wibb;
  kc=kc+1;
  
  t=t+T;

  disp(t);
  
end

disp('*******��ʼ�����������λ����\Сʱ,��/��/�룩*********');
disp('���ݿ̶�ϵ���Ͱ�װ��� | ���ٶȼƿ̶�ϵ���Ͱ�װ���');
disp([Gyro_fix/deg_rad*3600.0,Acc_fix]);

disp('*******��ʼ���������λ����\Сʱ,��/��/�룩*********');
disp('�����������������һ������ɷ����ݰ�����  | ���ٶ�һ������ɷ�');
disp([Gyro_b/deg_rad*3600.0,Gyro_r/deg_rad*3600.0,Gyro_wg/deg_rad*3600.0,Acc_r]);
disp('*******');

Fb=tmp_Fb/kc;
Wibb=tmp_Wibb/kc;

[attiN]=align_cal(Wibb,Fb,posiN); %��ʼ��׼����
[veloN]=veloN0(attiN,veloB);%�����ʼ�ٶ�

%attiN(3,1)=atti(3,1);
   %�����ݾ��Ƚϲ�ʱ����ֵΪ������������ʼֵ
   
disp([Fb,Wibb,attiN]);
input('press anykey continue...');


%%%%%%%%%%%%%%%%%���ݼ�¼%%%%%%%%%%%
TraceData=[];
IMUData=[];
GPSData=[];
SinsData=[];
KALData=[];
ErrData=[];

kc=1; 
t=0;   %������ʼ

[posiG,veloG]=simu_gps(t,posi,atti,veloB);
   %GPS���

[Xc,PK,Xerr]=kalm_gps_init(posiN,Xc,PK,Xerr);
       %�������˲���ʼ��
GPSData=[GPSData;t,posiG',veloG'];
KALData = [KALData;t,Xerr];
ErrData = [ErrData;t,Xc'];
    %�����ʼ����       
       
while t<=t_stop
    
  if(t>=kc*50-T & t<kc*50)
     kc=kc+1;
     disp(t);
  end
     %������ʾ
     
  [t,atti,atti_rate,veloB,acceB]=trace_s(t,T,atti,atti_rate,veloB,acceB);
      %�����������������й켣����
 
  [Wibb,Fb,posi]=IMUout(T,posi,atti,atti_rate,veloB,acceB);
      %���ݺͼ��ٶȷ��� 

  [Gyro_fix,Acc_fix]=imu_err_fix(Wibb,Fb);
    %��������������
  %Fb=Fb+Acc_fix;  
  %Wibb=Wibb+Gyro_fix/deg_rad;  % deg/s
    %�������������

  [Gyro_b,Gyro_r,Gyro_wg,Acc_r]=imu_err_random(t,T,Gyro_b,Gyro_r,Gyro_wg,Acc_r);
    %�������������ʼֵ����ʱIMU������������״̬�������������Ϊ��ֵ��
  Fb=Fb+Acc_r;
  Wibb=Wibb+Gyro_b/deg_rad+Gyro_r/deg_rad+Gyro_wg/deg_rad;  % deg/s
    %������������

  %disp('*******��ʼ���������λ����\Сʱ,��/��/�룩*********');
  %disp('�����������������һ������ɷ����ݰ�����  | ���ٶ�һ������ɷ�');
  %disp([Gyro_b/deg_rad*3600.0,Gyro_r/deg_rad*3600.0,Gyro_wg/deg_rad*3600.0,Acc_r]);
  %disp('*******');
   
  %[attiN]=atti_cal(T,Wibb,attiN,veloN,posiN);                % ���������㷨
  %[attiN]=atti_cal_cq(T,Wibb,attiN,veloN,posiN); % ��Ԫ���㷨
  [attiN,WnbbA_old]=atti_cal_cq_modi(T,Wibb-Gyro_modi/deg_rad,attiN,veloN,posiN,WnbbA_old);% ��Ԫ���㷨(����Чת��ʸ��������
      %��̬�Ƕ����
      
  [veloN]=velo_cal(T,Fb-Acc_modi,attiN,veloN,posiN);
      %�����任

  [posiN]=posi_cal(T,veloN,posiN);
      %��λ����
  
  t=t+T;
 
  T_M = T_M + T; kflag = 0;  
        
  if( T_M >= 1.0 )     
      [posiG,veloG]=simu_gps(t,posi,atti,veloB);
        %GPS���
        
      T_M = 0.0; kflag = 1; 
      
      %if( t>=100 & t<=200 ) kflag = 0; end %GPS��Чʱ�Ĳ���
      
      Xc = zeros(18,1); %�ջ�����У����������������ֵΪ0
      
      [Xc,PK,Xerr]=kalm_gps(t,T_D,Fb,attiN,veloN,posiN,posiG,veloG,Xc,PK,Xerr,kflag);
         %�������˲�   
     
      [attiN,veloN,posiN]=kalm_modi(attiN,veloN,posiN,Xc);
       %�����˲�����
      
      Gyro_modi(1,1) = Xc(10,1) + Xc(13,1);
      Gyro_modi(2,1) = Xc(11,1) + Xc(14,1);
      Gyro_modi(3,1) = Xc(12,1) + Xc(15,1);
        %����������
        
      Acc_modi(1,1) = Xc(16,1);
      Acc_modi(2,1) = Xc(17,1);
      Acc_modi(3,1) = Xc(18,1);      
        %���ٶȼ�������
        
      GPSData=[GPSData;t,posiG',veloG'];
      KALData = [KALData;t,Xerr];
      ErrData = [ErrData;t,Xc'];
         %�����������       
  end

       
  TraceData=[TraceData;t,posi',veloB(2,1),atti'];
  IMUData=[IMUData;t,3600.0*Wibb',1/g*Fb'];
  SinsData=[SinsData;t,attiN',veloN',posiN'];
      %�����������

end

%%%%%%%%%%%%%%��������%%%%%%%%%
fig_num=0;

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(TraceData(:,1),TraceData(:,2));
subplot(3,1,2);plot(TraceData(:,1),TraceData(:,3));
subplot(3,1,3);plot(TraceData(:,1),TraceData(:,4));
xlabel('���к������棨���ȣ��ȣ���γ�ȣ��ȣ����߶ȣ��ף���');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(TraceData(:,1),TraceData(:,6));
subplot(3,1,2);plot(TraceData(:,1),TraceData(:,7));
subplot(3,1,3);plot(TraceData(:,1),TraceData(:,8));
xlabel('���к������棨����Ƕȣ��ȣ��������Ƕȣ��ȣ�������Ƕȣ��ȣ���');
   % ��������

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(IMUData(:,1),IMUData(:,2));
subplot(3,1,2);plot(IMUData(:,1),IMUData(:,3));
subplot(3,1,3);plot(IMUData(:,1),IMUData(:,4));
xlabel('IMU���棨����X�ᣨ��/Сʱ��������Y�ᣨ��/Сʱ��������Z�ᣨ��/Сʱ����');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(IMUData(:,1),IMUData(:,5));
subplot(3,1,2);plot(IMUData(:,1),IMUData(:,6));
subplot(3,1,3);plot(IMUData(:,1),IMUData(:,7));
xlabel('IMU���棨�ӱ�X��g�����ӱ�Y�ᣨg�����ӱ�Z�ᣨg����');
   %IMU����

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(GPSData(:,1),GPSData(:,2));
subplot(3,1,2);plot(GPSData(:,1),GPSData(:,3));
subplot(3,1,3);plot(GPSData(:,1),GPSData(:,4));
xlabel('GPS���棨���ȣ��ȣ���γ�ȣ��ȣ����߶ȣ��ף���');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(GPSData(:,1),GPSData(:,5));
subplot(3,1,2);plot(GPSData(:,1),GPSData(:,6));
subplot(3,1,3);plot(GPSData(:,1),GPSData(:,7));
xlabel('GPS���棨�����ٶȣ���/�룩�������ٶȣ���/�룩�������ٶȣ���/�룩��');
   %GPS����

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(SinsData(:,1),SinsData(:,2));
subplot(3,1,2);plot(SinsData(:,1),SinsData(:,3));
subplot(3,1,3);plot(SinsData(:,1),SinsData(:,4));
xlabel('������̬�����������Ƕȣ��ȣ��������Ƕȣ��ȣ�������Ƕȣ��ȣ���');
   %��̬�Ƕ�����
   
fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(SinsData(:,1),SinsData(:,5));
subplot(3,1,2);plot(SinsData(:,1),SinsData(:,6));
subplot(3,1,3);plot(SinsData(:,1),SinsData(:,7));
xlabel('�����ٶ������������/�룩��������/�룩��������/�룩��');
   %�ٶ�����

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(SinsData(:,1),SinsData(:,8));
subplot(3,1,2);plot(SinsData(:,1),SinsData(:,9));
subplot(3,1,3);plot(SinsData(:,1),SinsData(:,10));
xlabel('�������������ȣ��ȣ���γ�ȣ��ȣ����߶ȣ��ף���');
   %λ������

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(TraceData(:,1),SinsData(:,2)-TraceData(:,6));
subplot(3,1,2);plot(TraceData(:,1),SinsData(:,3)-TraceData(:,7));
subplot(3,1,3);plot(TraceData(:,1),SinsData(:,4)-TraceData(:,8));
xlabel('��̬�Ƕ�������Ƕ����ȣ��������Ƕ����ȣ�������Ƕ����ȣ���');
   %������̬�Ƕ��������    
   
fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(TraceData(:,2),TraceData(:,3),'r',SinsData(:,8),SinsData(:,9));
subplot(3,1,2);plot(TraceData(:,1),SinsData(:,8)-TraceData(:,2));
subplot(3,1,3);plot(TraceData(:,1),SinsData(:,9)-TraceData(:,3));
xlabel('��γ��λ�öԱȼ���������ߣ���γ��λ�ã��ȣ����������ȣ���γ�����ȣ���');
   %����λ�ù켣���������

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(GPSData(:,1),GPSData(:,2),'r',SinsData(:,1),SinsData(:,8));
subplot(3,1,2);plot(GPSData(:,1),GPSData(:,3),'r',SinsData(:,1),SinsData(:,9));
subplot(3,1,3);plot(GPSData(:,1),GPSData(:,4),'r',SinsData(:,1),SinsData(:,10));
xlabel('��ϵ�����GPS�Աȣ����ȣ��ȣ���γ�ȣ��ȣ����߶ȣ��ף���');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(GPSData(:,1),GPSData(:,5),'r',SinsData(:,1),SinsData(:,5));
subplot(3,1,2);plot(GPSData(:,1),GPSData(:,6),'r',SinsData(:,1),SinsData(:,6));
subplot(3,1,3);plot(GPSData(:,1),GPSData(:,7),'r',SinsData(:,1),SinsData(:,7));
xlabel('��ϵ�����GPS�Աȣ������ٶȣ���/�룩�������ٶȣ���/�룩�������ٶȣ���/�룩��');
    %��ϵ�����GPS�Ա����

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(KALData(:,1),KALData(:,2));
subplot(3,1,2);plot(KALData(:,1),KALData(:,3));
subplot(3,1,3);plot(KALData(:,1),KALData(:,4));
xlabel('�������˲������ƽ̨���ǣ��룩��');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(KALData(:,1),KALData(:,5));
subplot(3,1,2);plot(KALData(:,1),KALData(:,6));
subplot(3,1,3);plot(KALData(:,1),KALData(:,7));
xlabel('�������˲�������ٶ�����/�룩��');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(KALData(:,1),KALData(:,9));
subplot(3,1,2);plot(KALData(:,1),KALData(:,8));
subplot(3,1,3);plot(KALData(:,1),KALData(:,10));
xlabel('�������˲������λ�����ף���');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(KALData(:,1),KALData(:,11));
subplot(3,1,2);plot(KALData(:,1),KALData(:,12));
subplot(3,1,3);plot(KALData(:,1),KALData(:,13));
xlabel('���������������/Сʱ��');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(KALData(:,1),KALData(:,14));
subplot(3,1,2);plot(KALData(:,1),KALData(:,15));
subplot(3,1,3);plot(KALData(:,1),KALData(:,16));
xlabel('����һ������ɷ�����/Сʱ��');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(KALData(:,1),KALData(:,17));
subplot(3,1,2);plot(KALData(:,1),KALData(:,18));
subplot(3,1,3);plot(KALData(:,1),KALData(:,19));
xlabel('���ٶȼ�һ������ɷ���g��');
   %%%%%%%%%%�������������%%%%%%%%%%%
   
%return;

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(ErrData(:,1),ErrData(:,2)*180.0/pi*3600); %sec
subplot(3,1,2);plot(ErrData(:,1),ErrData(:,3)*180.0/pi*3600); %sec
subplot(3,1,3);plot(ErrData(:,1),ErrData(:,4)*180.0/pi*3600); %sec
xlabel('ƽ̨�����������룩');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(ErrData(:,1),ErrData(:,5)); %��/��
subplot(3,1,2);plot(ErrData(:,1),ErrData(:,6)); %��/��
subplot(3,1,3);plot(ErrData(:,1),ErrData(:,7)); %��/��
xlabel('�ٶ�����������/�룩');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(ErrData(:,1),ErrData(:,9)*180.0/pi); % deg
subplot(3,1,2);plot(ErrData(:,1),ErrData(:,8)*180.0/pi); % deg
subplot(3,1,3);plot(ErrData(:,1),ErrData(:,10));         % m
xlabel('λ��������(���ȣ��ȣ���γ�ȣ��ȣ����߶ȣ��ף�)');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(ErrData(:,1),ErrData(:,11)*180.0/pi*3600); %deg/h
subplot(3,1,2);plot(ErrData(:,1),ErrData(:,12)*180.0/pi*3600); %deg/h
subplot(3,1,3);plot(ErrData(:,1),ErrData(:,13)*180.0/pi*3600); %deg/h
xlabel('���������������������/Сʱ��');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(ErrData(:,1),ErrData(:,14)*180.0/pi*3600); %deg/h
subplot(3,1,2);plot(ErrData(:,1),ErrData(:,15)*180.0/pi*3600); %deg/h
subplot(3,1,3);plot(ErrData(:,1),ErrData(:,16)*180.0/pi*3600); %deg/h
xlabel('����һ������ɷ�����������/Сʱ��');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(ErrData(:,1),ErrData(:,17)/g); %g
subplot(3,1,2);plot(ErrData(:,1),ErrData(:,18)/g); %g
subplot(3,1,3);plot(ErrData(:,1),ErrData(:,19)/g); %g
xlabel('���ٶȼ�һ������ɷ���������g��');
  %�������˲���������
    
   
%%%%%%%%%%%%%�洢��������%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
save trace.dat TraceData -ASCII;    %�洢��������
save imu.dat   IMUData   -ASCII;    %�洢���������IMU����
save sins.dat  SinsData  -ASCII;    %�洢�����������
save kal.dat   KALData   -ASCII;    %�洢�������˲�Щ������
save err.dat   ErrData   -ASCII;    %�洢�������˲����Ƶ��������ֵ



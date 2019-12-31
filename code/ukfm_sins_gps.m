%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%          SINSЭ�������
%                           ������ƣ�����  ���ڣ�2002/8/9
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [UKFTraceData,UKFIMUData,UKFSinsData,UKFKALData,UKFErrData,UKFGPSData]=ukfm_sins_gps(t_stop);
%%%%%%%%��������%%%%%%%%%%%
deg_rad=0.01745329252e0;% Transfer from angle degree to rad
g=9.7803698;         %�������ٶ�    ����λ����/��/�룩
randn('state',sum(100*clock)); % random number generator seed
%%%%%%%%����ʱ������%%%%%%
t=1;
T=0.02;
% t_stop=3500.0;
fil_time=0;
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
Gyro_z=zeros(3,1);
Acc_z =zeros(3,1);
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
% Xc = zeros(18,1);  %�ۺ�ģ��״̬����(�������������)ȫ��Ϊ���ʵ�λ��
% PK = zeros(18,18); %Э������
Xu = zeros(19,1);  %UKF�˲���ģ��״̬
PKU = zeros(19,19);%UKF�˲���Э������
UKFXerr =zeros(1,19); %״̬�����������ֵ����¼ĳ��ʱ�̵ģ�
kflag=0;           %GPS��Ϣ��Ч��־λ��1����Ч��

Acc_modi = zeros(3,1); %���ٶȼ��������ֵ����/��/�룩��X,Y,Z��
Gyro_modi= zeros(3,1); %�����������ֵ(����/��)(X,Y,Z)

%%%%%%%%%%%%%%%%%%%��ʼ��׼%%%%%%%%%%%%%%%%%%%
% kc=0;
% tmp_Fb=zeros(3,1);
% tmp_Wibb=zeros(3,1);
% t_alig  = 0;
% while t<=120
%   [t_alig,atti,atti_rate,veloB,acceB]=trace(0,T,atti,atti_rate,veloB,acceB);
%       %�����������������й켣����
%   [Wibb,Fb,posi]=IMUout(T,posi,atti,atti_rate,veloB,acceB);
%       %���ݺͼ��ٶȷ��� 
% 
%   [Gyro_fix,Acc_fix]=imu_err_fix(Wibb,Fb);
%       %��������������
%   %Fb=Fb+Acc_fix;  
%   %Wibb=Wibb+Gyro_fix/deg_rad;  % deg/s
%       %�������������
% 
%   [Gyro_b,Gyro_r,Gyro_wg,Acc_r]=imu_err_random(t,T,Gyro_b,Gyro_r,Gyro_wg,Acc_r);
%       %�������������ʼֵ����ʱIMU������������״̬�������������Ϊ��ֵ��
%   Fb=Fb+Acc_r;
%   Wibb=Wibb+Gyro_b/deg_rad+Gyro_r/deg_rad+Gyro_wg/deg_rad;  % deg/s
%       %������������
% 
%   tmp_Fb=tmp_Fb+Fb;
%   tmp_Wibb=tmp_Wibb+Wibb;
%   kc=kc+1;
%   
%   t=t+T;
% 
%   disp(t);
%   
% end
% 
% disp('*******��ʼ�����������λ����\Сʱ,��/��/�룩*********');
% disp('���ݿ̶�ϵ���Ͱ�װ��� | ���ٶȼƿ̶�ϵ���Ͱ�װ���');
% disp([Gyro_fix/deg_rad*3600.0,Acc_fix]);
% 
% disp('*******��ʼ���������λ����\Сʱ,��/��/�룩*********');
% disp('�����������������һ������ɷ����ݰ�����  | ���ٶ�һ������ɷ�');
% disp([Gyro_b/deg_rad*3600.0,Gyro_r/deg_rad*3600.0,Gyro_wg/deg_rad*3600.0,Acc_r]);
% disp('*******');
% 
% Fb=tmp_Fb/kc;
% Wibb=tmp_Wibb/kc;
% 
% [attiN]=align_cal(Wibb,Fb,posiN); %��ʼ��׼����
% [veloN]=veloN0(attiN,veloB);%�����ʼ�ٶ�
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%20m
attiN=[0.05;0.05;90.1];
veloN=[0.5;0.5;0.5];
Re=6378137.0;                                      %����뾶���ף� 
f=1/298.257;                                        %�������Բ��
long=posi(1,1)*pi/180.0;lati=posi(2,1)*pi/180.0;heig=posi(3,1);
Rm=Re*(1-2*f+3*f*sin(lati)*sin(lati));
Rn=Re*(1+f*sin(lati)*sin(lati));
posiN=[106.491-50/(pi/180.0*(Rn+heig)*cos(lati));29.528-50/(pi/180.0*(Rm+heig));350.0];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% load DD1_imudata.dat;                %����DD1�˲���IMU����,Ϊ�Ա�ʹ��
% data_loop=1;                         %�趨��������
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%attiN(3,1)=atti(3,1);
   %�����ݾ��Ƚϲ�ʱ����ֵΪ������������ʼֵ
   
disp([Fb,Wibb,attiN]);
%input('press anykey continue...');


%%%%%%%%%%%%%%%%%���ݼ�¼%%%%%%%%%%%
UKFTraceData=zeros(t_stop/T+1,10);
UKFIMUData=zeros(t_stop/T+1,7);
UKFGPSData=zeros(t_stop+1,7);
UKFSinsData=zeros(t_stop/T+1,10);
UKFKALData=zeros(t_stop+1,20);
UKFErrData=zeros(t_stop+1,20);
data_i=1;                %��������
data_j=1;                %��������

kc=1; 
t=0;   %������ʼ

[posiG,veloG]=simu_gps(t,posi,atti,veloB);
   %GPS���

[Xu,PKU,UKFXerr]=ukf_gps_init(posiN,atti,Xu,PKU,UKFXerr);
       %�������˲���ʼ��
 %%%%%%%%%%%%%%%%%%%%
  Re=6378137.0;                                      %����뾶���ף� 
  f=1/298.257;                                        %�������Բ��
  long=posi(1,1)*pi/180.0;lati=posi(2,1)*pi/180.0;heig=posi(3,1);
  Rm=Re*(1-2*f+3*f*sin(lati)*sin(lati));
  Rn=Re*(1+f*sin(lati)*sin(lati));
  posiG(1)=posiG(1)*pi/180.0*(Rn+heig)*cos(lati);  
  posiG(2)=posiG(2)*pi/180.0*(Rm+heig);  
  UKFGPSData(data_i,:)=[t,posiG',veloG'];
  posiG(1)=posiG(1)/(pi/180.0*(Rn+heig)*cos(lati));
  posiG(2)=posiG(2)/(pi/180.0*(Rm+heig)); 
  %%%%%%%%%%%%%%%%%%%      
  UKFKALData(data_i,:) = [t,UKFXerr];
  UKFErrData(data_i,:) = [t,Xu'];
  data_i=data_i+1;
  [vel]=veloN0(atti,veloB);    
  %UKFTraceData(data_j,:)=[t,posi',veloB(2,1),atti',vel'];
  %%%%%%%%%%%%%%%%%%
  Re=6378137.0;                                      %����뾶���ף� 
  f=1/298.257;                                        %�������Բ��
  long=posi(1,1)*pi/180.0;lati=posi(2,1)*pi/180.0;heig=posi(3,1);
  Rm=Re*(1-2*f+3*f*sin(lati)*sin(lati));
  Rn=Re*(1+f*sin(lati)*sin(lati));
  posi(1)=posi(1)*pi/180.0*(Rn+heig)*cos(lati);
  posi(2)=posi(2)*pi/180.0*(Rm+heig);
  UKFTraceData(data_j,:)=[t,atti',vel',posi'];
  posi(1)=posi(1)/(pi/180.0*(Rn+heig)*cos(lati));
  posi(2)=posi(2)/(pi/180.0*(Rm+heig)); 
  %%%%%%%%%%%%%%%%%%%%%
  UKFIMUData(data_j,:)=[t,3600.0*Wibb',1/g*Fb'];
  %%%%%%%%%%%%%%%%%%%%
  Re=6378137.0;                                      %����뾶���ף� 
  f=1/298.257;                                        %�������Բ��
  long=posi(1,1)*pi/180.0;lati=posi(2,1)*pi/180.0;heig=posi(3,1);
  Rm=Re*(1-2*f+3*f*sin(lati)*sin(lati));
  Rn=Re*(1+f*sin(lati)*sin(lati));
  posiN(1)=posiN(1)*pi/180.0*(Rn+heig)*cos(lati);
  posiN(2)=posiN(2)*pi/180.0*(Rm+heig);  
  UKFSinsData(data_j,:)=[t,attiN',veloN',posiN'];
  posiN(1)=posiN(1)/(pi/180.0*(Rn+heig)*cos(lati));
  posiN(2)=posiN(2)/(pi/180.0*(Rm+heig)); 
  %%%%%%%%%%%%%%%%%%%%

data_j=data_j+1;
    %�����ʼ����             
while t<=t_stop
    
  if(t>=kc*50-T & t<kc*50)
     kc=kc+1;
     disp(t);
  end
     %������ʾ
     
  [t,atti,atti_rate,veloB,acceB]=trace(t,T,atti,atti_rate,veloB,acceB); 
%   [t,atti,atti_rate,veloB,acceB]=trace_dd(t,T,atti,atti_rate,veloB,acceB);
  %[t,atti,atti_rate,veloB,acceB]=trace_LT(t,T,atti,atti_rate,veloB,acceB);
  %[t,atti,atti_rate,veloB,acceB]=trace_d_d(t,T,atti,atti_rate,veloB,acceB);
  %[t,atti,atti_rate,veloB,acceB]=trace_squ(t,T,atti,atti_rate,veloB,acceB);  
  %[t,atti,atti_rate,veloB,acceB]=trace_pf(t,T,atti,atti_rate,veloB,acceB); 
%   [t,atti,atti_rate,veloB,acceB]=trace(t,T,atti,atti_rate,veloB,acceB);        
  %[t,atti,atti_rate,veloB,acceB]=trace_a(t,T,atti,atti_rate,veloB,acceB);
  %[t,atti,atti_rate,veloB,acceB]=trace_dyn(t,T,atti,atti_rate,veloB,acceB);
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
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%   Fb(1,1)=DD1_imudata(data_loop,1);
%   Fb(2,1)=DD1_imudata(data_loop,2);
%   Fb(3,1)=DD1_imudata(data_loop,3);
%   Wibb(1,1)=DD1_imudata(data_loop,4);
%   Wibb(2,1)=DD1_imudata(data_loop,5);
%   Wibb(3,1)=DD1_imudata(data_loop,6);
%   data_loop=data_loop+1;
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
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
        
  if( (0.99<T_M)&(T_M<1.01)  )  
      [posiG,veloG]=simu_gps(t,posi,atti,veloB);
        %GPS���
        
      T_M = 0.0; kflag = 1; 
      
      %if( t>=100 & t<=200 ) kflag = 0; end %GPS��Чʱ�Ĳ���
      
      Xu = zeros(19,1); %�ջ�����У����������������ֵΪ0 
      tic;
      [Xu,PKU,UKFXerr]=ukf_gps(t,T_D,Fb,Wibb,attiN,veloN,posiN,posiG,veloG,Xu,PKU,UKFXerr,kflag);
      fil_time=fil_time+toc;
         %�������˲�   
      [attiN,veloN,posiN]=ukf_modi(attiN,veloN,posiN,Xu);
       %�����˲�����
      
      Gyro_modi(1,1) = Xu(11,1) + Xu(14,1);
      Gyro_modi(2,1) = Xu(12,1) + Xu(15,1);
      Gyro_modi(3,1) = Xu(13,1) + Xu(16,1);
        %����������
        
      Acc_modi(1,1) = Xu(17,1);
      Acc_modi(2,1) = Xu(18,1);
      Acc_modi(3,1) = Xu(19,1);      
        %���ٶȼ�������
        
      %%%%%%%%%%%%%%%%%%%%
  Re=6378137.0;                                      %����뾶���ף� 
  f=1/298.257;                                        %�������Բ��
  long=posi(1,1)*pi/180.0;lati=posi(2,1)*pi/180.0;heig=posi(3,1);
  Rm=Re*(1-2*f+3*f*sin(lati)*sin(lati));
  Rn=Re*(1+f*sin(lati)*sin(lati));
  posiG(1)=posiG(1)*pi/180.0*(Rn+heig)*cos(lati);  
  posiG(2)=posiG(2)*pi/180.0*(Rm+heig);
  UKFGPSData(data_i,:)=[t,posiG',veloG'];
  posiG(1)=posiG(1)/(pi/180.0*(Rn+heig)*cos(lati));
  posiG(2)=posiG(2)/(pi/180.0*(Rm+heig));  
  %%%%%%%%%%%%%%%%%%%
      UKFKALData (data_i,:)= [t,UKFXerr];
      UKFErrData (data_i,:)= [t,Xu'];
      data_i=data_i+1;
         %�����������       
  end

  [vel]=veloN0(atti,veloB);    
  %UKFTraceData(data_j,:)=[t,posi',veloB(2,1),atti',vel'];
  %%%%%%%%%%%%%%%%%%
  Re=6378137.0;                                      %����뾶���ף� 
  f=1/298.257;                                        %�������Բ��
  long=posi(1,1)*pi/180.0;lati=posi(2,1)*pi/180.0;heig=posi(3,1);
  Rm=Re*(1-2*f+3*f*sin(lati)*sin(lati));
  Rn=Re*(1+f*sin(lati)*sin(lati));
  posi(1)=posi(1)*pi/180.0*(Rn+heig)*cos(lati);
  posi(2)=posi(2)*pi/180.0*(Rm+heig); 
  UKFTraceData(data_j,:)=[t,atti',vel',posi'];
  posi(1)=posi(1)/(pi/180.0*(Rn+heig)*cos(lati));
  posi(2)=posi(2)/(pi/180.0*(Rm+heig));
  %%%%%%%%%%%%%%%%%%%%%
  UKFIMUData(data_j,:)=[t,3600.0*Wibb',1/g*Fb'];
  %%%%%%%%%%%%%%%%%%%%
  Re=6378137.0;                                      %����뾶���ף� 
  f=1/298.257;                                        %�������Բ��
  long=posi(1,1)*pi/180.0;lati=posi(2,1)*pi/180.0;heig=posi(3,1);
  Rm=Re*(1-2*f+3*f*sin(lati)*sin(lati));
  Rn=Re*(1+f*sin(lati)*sin(lati));
  posiN(1)=posiN(1)*pi/180.0*(Rn+heig)*cos(lati);
  posiN(2)=posiN(2)*pi/180.0*(Rm+heig);  
  UKFSinsData(data_j,:)=[t,attiN',veloN',posiN'];
  posiN(1)=posiN(1)/(pi/180.0*(Rn+heig)*cos(lati));
  posiN(2)=posiN(2)/(pi/180.0*(Rm+heig));
  data_j=data_j+1;
      %�����������

end
disp('%%%%%%%%�˲�������ʱ��%%%%%%%%');
disp(fil_time);
% ukf_fig(UKFTraceData,UKFIMUData,UKFSinsData,UKFKALData,UKFErrData,UKFGPSData);
% %%%%%%%%%%%%%�洢��������%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
save ukftrace.dat UKFTraceData -ASCII;    %�洢��������
save ukfimu.dat   UKFIMUData   -ASCII;    %�洢���������IMU����
save ukfsins.dat  UKFSinsData  -ASCII;    %�洢�����������
save ukfkal.dat   UKFKALData   -ASCII;    %�洢�������˲�Щ������
save ukferr.dat   UKFErrData   -ASCII;    %�洢�������˲����Ƶ��������ֵ




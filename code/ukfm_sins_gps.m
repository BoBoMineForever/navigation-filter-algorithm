%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%          SINS协方差仿真
%                           程序设计：熊智  日期：2002/8/9
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [UKFTraceData,UKFIMUData,UKFSinsData,UKFKALData,UKFErrData,UKFGPSData]=ukfm_sins_gps(t_stop);
%%%%%%%%常数设置%%%%%%%%%%%
deg_rad=0.01745329252e0;% Transfer from angle degree to rad
g=9.7803698;         %重力加速度    （单位：米/秒/秒）
randn('state',sum(100*clock)); % random number generator seed
%%%%%%%%仿真时间设置%%%%%%
t=1;
T=0.02;
% t_stop=3500.0;
fil_time=0;
%%%%%%%%%%%%%%%%航迹发生器%%%%%%%%%%%%%%%%%
atti=zeros(3,1);     %横滚、俯仰、航向（单位：度）
atti_rate=zeros(3,1);%横滚速率、俯仰速率、航向速率（单位：度/秒）
veloB=zeros(3,1);    %飞机运动速度——X右翼、Y机头、Z天向（单位：米/秒）
acceB=zeros(3,1);    %飞机运动加速度——X右翼、Y机头、Z天向（单位：米/秒/秒）
posi=zeros(3,1);     %航迹发生器初始位置经度、纬度、高度（单位：度、度、米）
posi=[106.491;29.528;300.0];  

atti(1,1)=0.0;
atti(2,1)=0.0;
atti(3,1)=90.0;  %初始航向角度（单位：度）

veloB(2,1)=0.0; %飞机初始运动速度——机头（单位：米/秒）

%%%%%%%%%%%%%%%%%%%%IMU输出%%%%%%%%%%
Wibb=zeros(3,1);    %机体系陀螺仪输出   （单位：度/秒）
Fb=zeros(3,1);      %机体系加速度计输出 （单位：米/秒/秒）

Gyro_fix=zeros(3,1);%机体系陀螺仪固定误差输出   （单位：弧度/秒）
Acc_fix=zeros(3,1); %机体系加速度计固定误差输出 （单位：米/秒/秒）
Gyro_b=zeros(3,1);  % 陀螺随机常数（弧度/秒）
Gyro_r=zeros(3,1);  % 陀螺一阶马尔可夫过程（弧度/秒）
Gyro_wg=zeros(3,1); %陀螺白噪声（弧度/秒）
Acc_r =zeros(3,1);  % 加速度一阶马尔可夫过程（米/秒/秒）
Gyro_z=zeros(3,1);
Acc_z =zeros(3,1);
%%%%%%%%%%%%%%%%%%GPS仿真输出%%%%%%%%%%%%%%
posiG = zeros(3,1); %GPS输出的飞行器位置（经度（度）、纬度（度）、高度（米））； 
veloG = zeros(3,1); %GPS输出的飞行器速度（东向（米/秒），北向（米/秒），天向（米/秒））


%%%%%%%%%%%%%%%%%%%%%捷联惯导仿真%%%%%%%%%%%%%
attiN=zeros(3,1);        %飞行器初始姿态
veloN=zeros(3,1);        %飞行器初始速度（相对于导航系）
posiN=zeros(3,1);        %飞行器初始位置
WnbbA_old=zeros(3,1);    %角速度积分输出（单位：弧度）

posiN=posi;              %初始位置与航迹位置一致
attiN=atti;              %初始姿态与航迹姿态一致（可以用初始对准函数替换）

%%%%%%%%%%%%%%%%%%%%%%%KALMAN滤波输出%%%%%%%%%%%%%%%%%
T_D = 1;    %离散周期；
T_M = 0;    %滤波量测产生时间（秒）；
% Xc = zeros(18,1);  %综合模型状态量；(可用于误差修正)全部为国际单位制
% PK = zeros(18,18); %协方差阵；
Xu = zeros(19,1);  %UKF滤波器模型状态
PKU = zeros(19,19);%UKF滤波器协方差阵
UKFXerr =zeros(1,19); %状态估计量的误差值（记录某个时刻的）
kflag=0;           %GPS信息有效标志位（1－有效）

Acc_modi = zeros(3,1); %加速度计误差修正值（米/秒/秒）（X,Y,Z）
Gyro_modi= zeros(3,1); %陀螺误差修正值(弧度/秒)(X,Y,Z)

%%%%%%%%%%%%%%%%%%%初始对准%%%%%%%%%%%%%%%%%%%
% kc=0;
% tmp_Fb=zeros(3,1);
% tmp_Wibb=zeros(3,1);
% t_alig  = 0;
% while t<=120
%   [t_alig,atti,atti_rate,veloB,acceB]=trace(0,T,atti,atti_rate,veloB,acceB);
%       %航迹发生器产生飞行轨迹参数
%   [Wibb,Fb,posi]=IMUout(T,posi,atti,atti_rate,veloB,acceB);
%       %陀螺和加速度仿真 
% 
%   [Gyro_fix,Acc_fix]=imu_err_fix(Wibb,Fb);
%       %产生非随机性误差
%   %Fb=Fb+Acc_fix;  
%   %Wibb=Wibb+Gyro_fix/deg_rad;  % deg/s
%       %加入非随机性误差
% 
%   [Gyro_b,Gyro_r,Gyro_wg,Acc_r]=imu_err_random(t,T,Gyro_b,Gyro_r,Gyro_wg,Acc_r);
%       %产生随机性误差初始值（此时IMU进入正常工作状态，则随机常数已为常值）
%   Fb=Fb+Acc_r;
%   Wibb=Wibb+Gyro_b/deg_rad+Gyro_r/deg_rad+Gyro_wg/deg_rad;  % deg/s
%       %加入随机性误差
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
% disp('*******初始非随机性误差（单位：度\小时,米/秒/秒）*********');
% disp('陀螺刻度系数和安装误差 | 加速度计刻度系数和安装误差');
% disp([Gyro_fix/deg_rad*3600.0,Acc_fix]);
% 
% disp('*******初始随机性误差（单位：度\小时,米/秒/秒）*********');
% disp('陀螺随机常数，陀螺一阶马儿可夫，陀螺白噪声  | 加速度一阶马儿可夫');
% disp([Gyro_b/deg_rad*3600.0,Gyro_r/deg_rad*3600.0,Gyro_wg/deg_rad*3600.0,Acc_r]);
% disp('*******');
% 
% Fb=tmp_Fb/kc;
% Wibb=tmp_Wibb/kc;
% 
% [attiN]=align_cal(Wibb,Fb,posiN); %初始对准计算
% [veloN]=veloN0(attiN,veloB);%计算初始速度
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%20m
attiN=[0.05;0.05;90.1];
veloN=[0.5;0.5;0.5];
Re=6378137.0;                                      %地球半径（米） 
f=1/298.257;                                        %地球的椭圆率
long=posi(1,1)*pi/180.0;lati=posi(2,1)*pi/180.0;heig=posi(3,1);
Rm=Re*(1-2*f+3*f*sin(lati)*sin(lati));
Rn=Re*(1+f*sin(lati)*sin(lati));
posiN=[106.491-50/(pi/180.0*(Rn+heig)*cos(lati));29.528-50/(pi/180.0*(Rm+heig));350.0];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% load DD1_imudata.dat;                %载入DD1滤波器IMU数据,为对比使用
% data_loop=1;                         %设定数据索引
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%attiN(3,1)=atti(3,1);
   %当陀螺精度较差时，则赋值为航迹发生器初始值
   
disp([Fb,Wibb,attiN]);
%input('press anykey continue...');


%%%%%%%%%%%%%%%%%数据记录%%%%%%%%%%%
UKFTraceData=zeros(t_stop/T+1,10);
UKFIMUData=zeros(t_stop/T+1,7);
UKFGPSData=zeros(t_stop+1,7);
UKFSinsData=zeros(t_stop/T+1,10);
UKFKALData=zeros(t_stop+1,20);
UKFErrData=zeros(t_stop+1,20);
data_i=1;                %数据索引
data_j=1;                %数据索引

kc=1; 
t=0;   %导航开始

[posiG,veloG]=simu_gps(t,posi,atti,veloB);
   %GPS输出

[Xu,PKU,UKFXerr]=ukf_gps_init(posiN,atti,Xu,PKU,UKFXerr);
       %卡尔曼滤波初始化
 %%%%%%%%%%%%%%%%%%%%
  Re=6378137.0;                                      %地球半径（米） 
  f=1/298.257;                                        %地球的椭圆率
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
  Re=6378137.0;                                      %地球半径（米） 
  f=1/298.257;                                        %地球的椭圆率
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
  Re=6378137.0;                                      %地球半径（米） 
  f=1/298.257;                                        %地球的椭圆率
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
    %保存初始数据             
while t<=t_stop
    
  if(t>=kc*50-T & t<kc*50)
     kc=kc+1;
     disp(t);
  end
     %控制显示
     
  [t,atti,atti_rate,veloB,acceB]=trace(t,T,atti,atti_rate,veloB,acceB); 
%   [t,atti,atti_rate,veloB,acceB]=trace_dd(t,T,atti,atti_rate,veloB,acceB);
  %[t,atti,atti_rate,veloB,acceB]=trace_LT(t,T,atti,atti_rate,veloB,acceB);
  %[t,atti,atti_rate,veloB,acceB]=trace_d_d(t,T,atti,atti_rate,veloB,acceB);
  %[t,atti,atti_rate,veloB,acceB]=trace_squ(t,T,atti,atti_rate,veloB,acceB);  
  %[t,atti,atti_rate,veloB,acceB]=trace_pf(t,T,atti,atti_rate,veloB,acceB); 
%   [t,atti,atti_rate,veloB,acceB]=trace(t,T,atti,atti_rate,veloB,acceB);        
  %[t,atti,atti_rate,veloB,acceB]=trace_a(t,T,atti,atti_rate,veloB,acceB);
  %[t,atti,atti_rate,veloB,acceB]=trace_dyn(t,T,atti,atti_rate,veloB,acceB);
      %航迹发生器产生飞行轨迹参数

  [Wibb,Fb,posi]=IMUout(T,posi,atti,atti_rate,veloB,acceB);
      %陀螺和加速度仿真 

  [Gyro_fix,Acc_fix]=imu_err_fix(Wibb,Fb);
    %产生非随机性误差
  %Fb=Fb+Acc_fix;  
  %Wibb=Wibb+Gyro_fix/deg_rad;  % deg/s
    %加入非随机性误差

  [Gyro_b,Gyro_r,Gyro_wg,Acc_r]=imu_err_random(t,T,Gyro_b,Gyro_r,Gyro_wg,Acc_r);
    %产生随机性误差初始值（此时IMU进入正常工作状态，则随机常数已为常值）
  Fb=Fb+Acc_r;
  Wibb=Wibb+Gyro_b/deg_rad+Gyro_r/deg_rad+Gyro_wg/deg_rad;  % deg/s
    %加入随机性误差
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%   Fb(1,1)=DD1_imudata(data_loop,1);
%   Fb(2,1)=DD1_imudata(data_loop,2);
%   Fb(3,1)=DD1_imudata(data_loop,3);
%   Wibb(1,1)=DD1_imudata(data_loop,4);
%   Wibb(2,1)=DD1_imudata(data_loop,5);
%   Wibb(3,1)=DD1_imudata(data_loop,6);
%   data_loop=data_loop+1;
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
  %disp('*******初始随机性误差（单位：度\小时,米/秒/秒）*********');
  %disp('陀螺随机常数，陀螺一阶马儿可夫，陀螺白噪声  | 加速度一阶马儿可夫');
  %disp([Gyro_b/deg_rad*3600.0,Gyro_r/deg_rad*3600.0,Gyro_wg/deg_rad*3600.0,Acc_r]);
  %disp('*******');
   
  %[attiN]=atti_cal(T,Wibb,attiN,veloN,posiN);                % 方向余弦算法
  %[attiN]=atti_cal_cq(T,Wibb,attiN,veloN,posiN); % 四元数算法
  [attiN,WnbbA_old]=atti_cal_cq_modi(T,Wibb-Gyro_modi/deg_rad,attiN,veloN,posiN,WnbbA_old);% 四元数算法(含等效转动矢量修正）
      %姿态角度求解
      
  [veloN]=velo_cal(T,Fb-Acc_modi,attiN,veloN,posiN);
      %比力变换

  [posiN]=posi_cal(T,veloN,posiN);
      %定位计算
  
  t=t+T;
 
  T_M = T_M + T; kflag = 0;  
        
  if( (0.99<T_M)&(T_M<1.01)  )  
      [posiG,veloG]=simu_gps(t,posi,atti,veloB);
        %GPS输出
        
      T_M = 0.0; kflag = 1; 
      
      %if( t>=100 & t<=200 ) kflag = 0; end %GPS无效时的测试
      
      Xu = zeros(19,1); %闭环反馈校正，控制修正量初值为0 
      tic;
      [Xu,PKU,UKFXerr]=ukf_gps(t,T_D,Fb,Wibb,attiN,veloN,posiN,posiG,veloG,Xu,PKU,UKFXerr,kflag);
      fil_time=fil_time+toc;
         %卡尔曼滤波   
      [attiN,veloN,posiN]=ukf_modi(attiN,veloN,posiN,Xu);
       %进行滤波修正
      
      Gyro_modi(1,1) = Xu(11,1) + Xu(14,1);
      Gyro_modi(2,1) = Xu(12,1) + Xu(15,1);
      Gyro_modi(3,1) = Xu(13,1) + Xu(16,1);
        %陀螺修正量
        
      Acc_modi(1,1) = Xu(17,1);
      Acc_modi(2,1) = Xu(18,1);
      Acc_modi(3,1) = Xu(19,1);      
        %加速度计修正量
        
      %%%%%%%%%%%%%%%%%%%%
  Re=6378137.0;                                      %地球半径（米） 
  f=1/298.257;                                        %地球的椭圆率
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
         %保存仿真数据       
  end

  [vel]=veloN0(atti,veloB);    
  %UKFTraceData(data_j,:)=[t,posi',veloB(2,1),atti',vel'];
  %%%%%%%%%%%%%%%%%%
  Re=6378137.0;                                      %地球半径（米） 
  f=1/298.257;                                        %地球的椭圆率
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
  Re=6378137.0;                                      %地球半径（米） 
  f=1/298.257;                                        %地球的椭圆率
  long=posi(1,1)*pi/180.0;lati=posi(2,1)*pi/180.0;heig=posi(3,1);
  Rm=Re*(1-2*f+3*f*sin(lati)*sin(lati));
  Rn=Re*(1+f*sin(lati)*sin(lati));
  posiN(1)=posiN(1)*pi/180.0*(Rn+heig)*cos(lati);
  posiN(2)=posiN(2)*pi/180.0*(Rm+heig);  
  UKFSinsData(data_j,:)=[t,attiN',veloN',posiN'];
  posiN(1)=posiN(1)/(pi/180.0*(Rn+heig)*cos(lati));
  posiN(2)=posiN(2)/(pi/180.0*(Rm+heig));
  data_j=data_j+1;
      %保存仿真数据

end
disp('%%%%%%%%滤波器工作时间%%%%%%%%');
disp(fil_time);
% ukf_fig(UKFTraceData,UKFIMUData,UKFSinsData,UKFKALData,UKFErrData,UKFGPSData);
% %%%%%%%%%%%%%存储仿真数据%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
save ukftrace.dat UKFTraceData -ASCII;    %存储航迹数据
save ukfimu.dat   UKFIMUData   -ASCII;    %存储仿真产生的IMU数据
save ukfsins.dat  UKFSinsData  -ASCII;    %存储导航输出数据
save ukfkal.dat   UKFKALData   -ASCII;    %存储卡尔曼滤波些方差阵
save ukferr.dat   UKFErrData   -ASCII;    %存储卡尔曼滤波估计的误差修正值




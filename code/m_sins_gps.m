
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%          SINS协方差仿真
%                           程序设计：熊智  日期：2002/8/9
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%常数设置%%%%%%%%%%%
deg_rad=0.01745329252e0;% Transfer from angle degree to rad
g=9.7803698;         %重力加速度    （单位：米/秒/秒）

%%%%%%%%仿真时间设置%%%%%%
t=0;
T=1;
t_stop=5000.0;

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
Xc = zeros(18,1);  %综合模型状态量；(可用于误差修正)全部为国际单位制
PK = zeros(18,18); %协方差阵；
Xerr =zeros(1,18); %状态估计量的误差值（记录某个时刻的）
kflag=0;           %GPS信息有效标志位（1－有效）

Acc_modi = zeros(3,1); %加速度计误差修正值（米/秒/秒）（X,Y,Z）
Gyro_modi= zeros(3,1); %陀螺误差修正值(弧度/秒)(X,Y,Z)

%%%%%%%%%%%%%%%%%%%初始对准%%%%%%%%%%%%%%%%%%%
kc=0;
tmp_Fb=zeros(3,1);
tmp_Wibb=zeros(3,1);
t_alig  = 0;
while t<=120
  [t_alig,atti,atti_rate,veloB,acceB]=trace(0,T,atti,atti_rate,veloB,acceB);
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

  tmp_Fb=tmp_Fb+Fb;
  tmp_Wibb=tmp_Wibb+Wibb;
  kc=kc+1;
  
  t=t+T;

  disp(t);
  
end

disp('*******初始非随机性误差（单位：度\小时,米/秒/秒）*********');
disp('陀螺刻度系数和安装误差 | 加速度计刻度系数和安装误差');
disp([Gyro_fix/deg_rad*3600.0,Acc_fix]);

disp('*******初始随机性误差（单位：度\小时,米/秒/秒）*********');
disp('陀螺随机常数，陀螺一阶马儿可夫，陀螺白噪声  | 加速度一阶马儿可夫');
disp([Gyro_b/deg_rad*3600.0,Gyro_r/deg_rad*3600.0,Gyro_wg/deg_rad*3600.0,Acc_r]);
disp('*******');

Fb=tmp_Fb/kc;
Wibb=tmp_Wibb/kc;

[attiN]=align_cal(Wibb,Fb,posiN); %初始对准计算
[veloN]=veloN0(attiN,veloB);%计算初始速度

%attiN(3,1)=atti(3,1);
   %当陀螺精度较差时，则赋值为航迹发生器初始值
   
disp([Fb,Wibb,attiN]);
input('press anykey continue...');


%%%%%%%%%%%%%%%%%数据记录%%%%%%%%%%%
TraceData=[];
IMUData=[];
GPSData=[];
SinsData=[];
KALData=[];
ErrData=[];

kc=1; 
t=0;   %导航开始

[posiG,veloG]=simu_gps(t,posi,atti,veloB);
   %GPS输出

[Xc,PK,Xerr]=kalm_gps_init(posiN,Xc,PK,Xerr);
       %卡尔曼滤波初始化
GPSData=[GPSData;t,posiG',veloG'];
KALData = [KALData;t,Xerr];
ErrData = [ErrData;t,Xc'];
    %保存初始数据       
       
while t<=t_stop
    
  if(t>=kc*50-T & t<kc*50)
     kc=kc+1;
     disp(t);
  end
     %控制显示
     
  [t,atti,atti_rate,veloB,acceB]=trace_s(t,T,atti,atti_rate,veloB,acceB);
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
        
  if( T_M >= 1.0 )     
      [posiG,veloG]=simu_gps(t,posi,atti,veloB);
        %GPS输出
        
      T_M = 0.0; kflag = 1; 
      
      %if( t>=100 & t<=200 ) kflag = 0; end %GPS无效时的测试
      
      Xc = zeros(18,1); %闭环反馈校正，控制修正量初值为0
      
      [Xc,PK,Xerr]=kalm_gps(t,T_D,Fb,attiN,veloN,posiN,posiG,veloG,Xc,PK,Xerr,kflag);
         %卡尔曼滤波   
     
      [attiN,veloN,posiN]=kalm_modi(attiN,veloN,posiN,Xc);
       %进行滤波修正
      
      Gyro_modi(1,1) = Xc(10,1) + Xc(13,1);
      Gyro_modi(2,1) = Xc(11,1) + Xc(14,1);
      Gyro_modi(3,1) = Xc(12,1) + Xc(15,1);
        %陀螺修正量
        
      Acc_modi(1,1) = Xc(16,1);
      Acc_modi(2,1) = Xc(17,1);
      Acc_modi(3,1) = Xc(18,1);      
        %加速度计修正量
        
      GPSData=[GPSData;t,posiG',veloG'];
      KALData = [KALData;t,Xerr];
      ErrData = [ErrData;t,Xc'];
         %保存仿真数据       
  end

       
  TraceData=[TraceData;t,posi',veloB(2,1),atti'];
  IMUData=[IMUData;t,3600.0*Wibb',1/g*Fb'];
  SinsData=[SinsData;t,attiN',veloN',posiN'];
      %保存仿真数据

end

%%%%%%%%%%%%%%绘制曲线%%%%%%%%%
fig_num=0;

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(TraceData(:,1),TraceData(:,2));
subplot(3,1,2);plot(TraceData(:,1),TraceData(:,3));
subplot(3,1,3);plot(TraceData(:,1),TraceData(:,4));
xlabel('飞行航迹仿真（经度（度），纬度（度），高度（米））');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(TraceData(:,1),TraceData(:,6));
subplot(3,1,2);plot(TraceData(:,1),TraceData(:,7));
subplot(3,1,3);plot(TraceData(:,1),TraceData(:,8));
xlabel('飞行航迹仿真（横滚角度（度），俯仰角度（度），航向角度（度））');
   % 航迹数据

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(IMUData(:,1),IMUData(:,2));
subplot(3,1,2);plot(IMUData(:,1),IMUData(:,3));
subplot(3,1,3);plot(IMUData(:,1),IMUData(:,4));
xlabel('IMU仿真（陀螺X轴（度/小时），陀螺Y轴（度/小时），陀螺Z轴（度/小时））');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(IMUData(:,1),IMUData(:,5));
subplot(3,1,2);plot(IMUData(:,1),IMUData(:,6));
subplot(3,1,3);plot(IMUData(:,1),IMUData(:,7));
xlabel('IMU仿真（加表X轴g），加表Y轴（g），加表Z轴（g））');
   %IMU数据

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(GPSData(:,1),GPSData(:,2));
subplot(3,1,2);plot(GPSData(:,1),GPSData(:,3));
subplot(3,1,3);plot(GPSData(:,1),GPSData(:,4));
xlabel('GPS仿真（经度（度），纬度（度），高度（米））');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(GPSData(:,1),GPSData(:,5));
subplot(3,1,2);plot(GPSData(:,1),GPSData(:,6));
subplot(3,1,3);plot(GPSData(:,1),GPSData(:,7));
xlabel('GPS仿真（东向速度（米/秒），北向速度（米/秒），天向速度（米/秒））');
   %GPS仿真

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(SinsData(:,1),SinsData(:,2));
subplot(3,1,2);plot(SinsData(:,1),SinsData(:,3));
subplot(3,1,3);plot(SinsData(:,1),SinsData(:,4));
xlabel('导航姿态角输出（横滚角度（度），俯仰角度（度），航向角度（度））');
   %姿态角度数据
   
fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(SinsData(:,1),SinsData(:,5));
subplot(3,1,2);plot(SinsData(:,1),SinsData(:,6));
subplot(3,1,3);plot(SinsData(:,1),SinsData(:,7));
xlabel('导航速度输出（东向（米/秒），北向（米/秒），天向（米/秒））');
   %速度数据

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(SinsData(:,1),SinsData(:,8));
subplot(3,1,2);plot(SinsData(:,1),SinsData(:,9));
subplot(3,1,3);plot(SinsData(:,1),SinsData(:,10));
xlabel('导航航迹（经度（度），纬度（度），高度（米））');
   %位置数据

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(TraceData(:,1),SinsData(:,2)-TraceData(:,6));
subplot(3,1,2);plot(TraceData(:,1),SinsData(:,3)-TraceData(:,7));
subplot(3,1,3);plot(TraceData(:,1),SinsData(:,4)-TraceData(:,8));
xlabel('姿态角度误差（横滚角度误差（度），俯仰角度误差（度），航向角度误差（度））');
   %飞行姿态角度误差曲线    
   
fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(TraceData(:,2),TraceData(:,3),'r',SinsData(:,8),SinsData(:,9));
subplot(3,1,2);plot(TraceData(:,1),SinsData(:,8)-TraceData(:,2));
subplot(3,1,3);plot(TraceData(:,1),SinsData(:,9)-TraceData(:,3));
xlabel('经纬度位置对比及其误差曲线（经纬度位置（度），经度误差（度），纬度误差（度））');
   %飞行位置轨迹及误差曲线

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(GPSData(:,1),GPSData(:,2),'r',SinsData(:,1),SinsData(:,8));
subplot(3,1,2);plot(GPSData(:,1),GPSData(:,3),'r',SinsData(:,1),SinsData(:,9));
subplot(3,1,3);plot(GPSData(:,1),GPSData(:,4),'r',SinsData(:,1),SinsData(:,10));
xlabel('组合导航与GPS对比（经度（度），纬度（度），高度（米））');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(GPSData(:,1),GPSData(:,5),'r',SinsData(:,1),SinsData(:,5));
subplot(3,1,2);plot(GPSData(:,1),GPSData(:,6),'r',SinsData(:,1),SinsData(:,6));
subplot(3,1,3);plot(GPSData(:,1),GPSData(:,7),'r',SinsData(:,1),SinsData(:,7));
xlabel('组合导航与GPS对比（东向速度（米/秒），北向速度（米/秒），天向速度（米/秒））');
    %组合导航与GPS对比输出

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(KALData(:,1),KALData(:,2));
subplot(3,1,2);plot(KALData(:,1),KALData(:,3));
subplot(3,1,3);plot(KALData(:,1),KALData(:,4));
xlabel('卡尔曼滤波输出（平台误差角（秒））');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(KALData(:,1),KALData(:,5));
subplot(3,1,2);plot(KALData(:,1),KALData(:,6));
subplot(3,1,3);plot(KALData(:,1),KALData(:,7));
xlabel('卡尔曼滤波输出（速度误差（米/秒））');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(KALData(:,1),KALData(:,9));
subplot(3,1,2);plot(KALData(:,1),KALData(:,8));
subplot(3,1,3);plot(KALData(:,1),KALData(:,10));
xlabel('卡尔曼滤波输出（位置误差（米））');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(KALData(:,1),KALData(:,11));
subplot(3,1,2);plot(KALData(:,1),KALData(:,12));
subplot(3,1,3);plot(KALData(:,1),KALData(:,13));
xlabel('陀螺随机常数误差（度/小时）');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(KALData(:,1),KALData(:,14));
subplot(3,1,2);plot(KALData(:,1),KALData(:,15));
subplot(3,1,3);plot(KALData(:,1),KALData(:,16));
xlabel('陀螺一阶马尔可夫误差（度/小时）');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(KALData(:,1),KALData(:,17));
subplot(3,1,2);plot(KALData(:,1),KALData(:,18));
subplot(3,1,3);plot(KALData(:,1),KALData(:,19));
xlabel('加速度计一阶马尔可夫误差（g）');
   %%%%%%%%%%卡尔曼估计误差%%%%%%%%%%%
   
%return;

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(ErrData(:,1),ErrData(:,2)*180.0/pi*3600); %sec
subplot(3,1,2);plot(ErrData(:,1),ErrData(:,3)*180.0/pi*3600); %sec
subplot(3,1,3);plot(ErrData(:,1),ErrData(:,4)*180.0/pi*3600); %sec
xlabel('平台误差补偿量（角秒）');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(ErrData(:,1),ErrData(:,5)); %米/秒
subplot(3,1,2);plot(ErrData(:,1),ErrData(:,6)); %米/秒
subplot(3,1,3);plot(ErrData(:,1),ErrData(:,7)); %米/秒
xlabel('速度误差补偿量（米/秒）');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(ErrData(:,1),ErrData(:,9)*180.0/pi); % deg
subplot(3,1,2);plot(ErrData(:,1),ErrData(:,8)*180.0/pi); % deg
subplot(3,1,3);plot(ErrData(:,1),ErrData(:,10));         % m
xlabel('位置误差补偿量(经度（度），纬度（度），高度（米）)');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(ErrData(:,1),ErrData(:,11)*180.0/pi*3600); %deg/h
subplot(3,1,2);plot(ErrData(:,1),ErrData(:,12)*180.0/pi*3600); %deg/h
subplot(3,1,3);plot(ErrData(:,1),ErrData(:,13)*180.0/pi*3600); %deg/h
xlabel('陀螺随机常数误差补偿量（度/小时）');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(ErrData(:,1),ErrData(:,14)*180.0/pi*3600); %deg/h
subplot(3,1,2);plot(ErrData(:,1),ErrData(:,15)*180.0/pi*3600); %deg/h
subplot(3,1,3);plot(ErrData(:,1),ErrData(:,16)*180.0/pi*3600); %deg/h
xlabel('陀螺一阶马尔可夫误差补偿量（度/小时）');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(ErrData(:,1),ErrData(:,17)/g); %g
subplot(3,1,2);plot(ErrData(:,1),ErrData(:,18)/g); %g
subplot(3,1,3);plot(ErrData(:,1),ErrData(:,19)/g); %g
xlabel('加速度计一阶马尔可夫误差补偿量（g）');
  %卡尔曼滤波修正数据
    
   
%%%%%%%%%%%%%存储仿真数据%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
save trace.dat TraceData -ASCII;    %存储航迹数据
save imu.dat   IMUData   -ASCII;    %存储仿真产生的IMU数据
save sins.dat  SinsData  -ASCII;    %存储导航输出数据
save kal.dat   KALData   -ASCII;    %存储卡尔曼滤波些方差阵
save err.dat   ErrData   -ASCII;    %存储卡尔曼滤波估计的误差修正值



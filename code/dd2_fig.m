%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%        DD1滤波器图形绘制函数
%                           程序设计：熊剑  日期：2007/6/4
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function []=dd2_fig(DD2TraceData,DD2IMUData,DD2SinsData,DD2KALData,DD2ErrData,DD2GPSData)
g=9.7803698;         %重力加速度    （单位：米/秒/秒）
%%%%%%%%%%%%%绘制曲线%%%%%%%%%
fig_num=0;

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(DD2TraceData(:,1),DD2TraceData(:,8));
subplot(3,1,2);plot(DD2TraceData(:,1),DD2TraceData(:,9));
subplot(3,1,3);plot(DD2TraceData(:,1),DD2TraceData(:,10));
xlabel('飞行航迹仿真（经度（度），纬度（度），高度（米））');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(DD2TraceData(:,1),DD2TraceData(:,2));
subplot(3,1,2);plot(DD2TraceData(:,1),DD2TraceData(:,3));
subplot(3,1,3);plot(DD2TraceData(:,1),DD2TraceData(:,4));
xlabel('飞行航迹仿真（横滚角度（度），俯仰角度（度），航向角度（度））');
   % 航迹数据

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(DD2IMUData(:,1),DD2IMUData(:,2));
subplot(3,1,2);plot(DD2IMUData(:,1),DD2IMUData(:,3));
subplot(3,1,3);plot(DD2IMUData(:,1),DD2IMUData(:,4));
xlabel('IMU仿真（陀螺X轴（度/小时），陀螺Y轴（度/小时），陀螺Z轴（度/小时））');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(DD2IMUData(:,1),DD2IMUData(:,5));
subplot(3,1,2);plot(DD2IMUData(:,1),DD2IMUData(:,6));
subplot(3,1,3);plot(DD2IMUData(:,1),DD2IMUData(:,7));
xlabel('IMU仿真（加表X轴g），加表Y轴（g），加表Z轴（g））');
   %IMU数据

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(DD2GPSData(:,1),DD2GPSData(:,2));
subplot(3,1,2);plot(DD2GPSData(:,1),DD2GPSData(:,3));
subplot(3,1,3);plot(DD2GPSData(:,1),DD2GPSData(:,4));
xlabel('GPS仿真（经度（度），纬度（度），高度（米））');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(DD2GPSData(:,1),DD2GPSData(:,5));
subplot(3,1,2);plot(DD2GPSData(:,1),DD2GPSData(:,6));
subplot(3,1,3);plot(DD2GPSData(:,1),DD2GPSData(:,7));
xlabel('GPS仿真（东向速度（米/秒），北向速度（米/秒），天向速度（米/秒））');
   %GPS仿真

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(DD2SinsData(:,1),DD2SinsData(:,2));
subplot(3,1,2);plot(DD2SinsData(:,1),DD2SinsData(:,3));
subplot(3,1,3);plot(DD2SinsData(:,1),DD2SinsData(:,4));
xlabel('导航姿态角输出（横滚角度（度），俯仰角度（度），航向角度（度））');
   %姿态角度数据
   
fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(DD2SinsData(:,1),DD2SinsData(:,5));
subplot(3,1,2);plot(DD2SinsData(:,1),DD2SinsData(:,6));
subplot(3,1,3);plot(DD2SinsData(:,1),DD2SinsData(:,7));
xlabel('导航速度输出（东向（米/秒），北向（米/秒），天向（米/秒））');
   %速度数据

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(DD2SinsData(:,1),DD2SinsData(:,8));
subplot(3,1,2);plot(DD2SinsData(:,1),DD2SinsData(:,9));
subplot(3,1,3);plot(DD2SinsData(:,1),DD2SinsData(:,10));
xlabel('导航航迹（经度（度），纬度（度），高度（米））');
   %位置数据

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(DD2TraceData(:,1),DD2SinsData(:,2)-DD2TraceData(:,2));
subplot(3,1,2);plot(DD2TraceData(:,1),DD2SinsData(:,3)-DD2TraceData(:,3));
subplot(3,1,3);plot(DD2TraceData(:,1),DD2SinsData(:,4)-DD2TraceData(:,4));
xlabel('姿态角度误差（横滚角度误差（度），俯仰角度误差（度），航向角度误差（度））');
   %飞行姿态角度误差曲线    
   
fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(DD2TraceData(:,8),DD2TraceData(:,9),'r',DD2SinsData(:,8),DD2SinsData(:,9));
subplot(3,1,2);plot(DD2TraceData(:,1),DD2SinsData(:,8)-DD2TraceData(:,8));
subplot(3,1,3);plot(DD2TraceData(:,1),DD2SinsData(:,9)-DD2TraceData(:,9));
xlabel('经纬度位置对比及其误差曲线（经纬度位置（度），经度误差（度），纬度误差（度））');
   %飞行位置轨迹及误差曲线

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(DD2GPSData(:,1),DD2GPSData(:,2),'r',DD2SinsData(:,1),DD2SinsData(:,8));
subplot(3,1,2);plot(DD2GPSData(:,1),DD2GPSData(:,3),'r',DD2SinsData(:,1),DD2SinsData(:,9));
subplot(3,1,3);plot(DD2GPSData(:,1),DD2GPSData(:,4),'r',DD2SinsData(:,1),DD2SinsData(:,10));
xlabel('组合导航与GPS对比（经度（度），纬度（度），高度（米））');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(DD2GPSData(:,1),DD2GPSData(:,5),'r',DD2SinsData(:,1),DD2SinsData(:,5));
subplot(3,1,2);plot(DD2GPSData(:,1),DD2GPSData(:,6),'r',DD2SinsData(:,1),DD2SinsData(:,6));
subplot(3,1,3);plot(DD2GPSData(:,1),DD2GPSData(:,7),'r',DD2SinsData(:,1),DD2SinsData(:,7));
xlabel('组合导航与GPS对比（东向速度（米/秒），北向速度（米/秒），天向速度（米/秒））');
    %组合导航与GPS对比输出

fig_num = fig_num+1;
figure(fig_num);
subplot(4,1,1);plot(DD2KALData(:,1),DD2KALData(:,5));
subplot(4,1,2);plot(DD2KALData(:,1),DD2KALData(:,6));
subplot(4,1,3);plot(DD2KALData(:,1),DD2KALData(:,7));
subplot(4,1,4);plot(DD2KALData(:,1),DD2KALData(:,8));
xlabel('卡尔曼滤波输出（平台误差角(四元数)（秒））');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(DD2KALData(:,1),DD2KALData(:,2));
subplot(3,1,2);plot(DD2KALData(:,1),DD2KALData(:,3));
subplot(3,1,3);plot(DD2KALData(:,1),DD2KALData(:,4));
xlabel('卡尔曼滤波输出（速度误差（米/秒））');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(DD2KALData(:,1),DD2KALData(:,10));
subplot(3,1,2);plot(DD2KALData(:,1),DD2KALData(:,9));
subplot(3,1,3);plot(DD2KALData(:,1),DD2KALData(:,11));
xlabel('卡尔曼滤波输出（位置误差（米））');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(DD2KALData(:,1),DD2KALData(:,12));
subplot(3,1,2);plot(DD2KALData(:,1),DD2KALData(:,13));
subplot(3,1,3);plot(DD2KALData(:,1),DD2KALData(:,14));
xlabel('陀螺随机常数误差（度/小时）');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(DD2KALData(:,1),DD2KALData(:,15));
subplot(3,1,2);plot(DD2KALData(:,1),DD2KALData(:,16));
subplot(3,1,3);plot(DD2KALData(:,1),DD2KALData(:,17));
xlabel('陀螺一阶马尔可夫误差（度/小时）');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(DD2KALData(:,1),DD2KALData(:,18));
subplot(3,1,2);plot(DD2KALData(:,1),DD2KALData(:,19));
subplot(3,1,3);plot(DD2KALData(:,1),DD2KALData(:,20));
xlabel('加速度计一阶马尔可夫误差（g）');
   %%%%%%%%%%卡尔曼估计误差%%%%%%%%%%%
   
%return;

fig_num = fig_num+1;
figure(fig_num);
subplot(4,1,1);plot(DD2ErrData(:,1),DD2ErrData(:,5)); %sec
subplot(4,1,2);plot(DD2ErrData(:,1),DD2ErrData(:,6)); %sec
subplot(4,1,3);plot(DD2ErrData(:,1),DD2ErrData(:,7)); %sec
subplot(4,1,4);plot(DD2ErrData(:,1),DD2ErrData(:,8)); %sec
xlabel('平台误差补偿量(四元数)（角秒）');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(DD2ErrData(:,1),DD2ErrData(:,2)); %米/秒
subplot(3,1,2);plot(DD2ErrData(:,1),DD2ErrData(:,3)); %米/秒
subplot(3,1,3);plot(DD2ErrData(:,1),DD2ErrData(:,4)); %米/秒
xlabel('速度误差补偿量（米/秒）');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(DD2ErrData(:,1),DD2ErrData(:,10)*180.0/pi); % deg
subplot(3,1,2);plot(DD2ErrData(:,1),DD2ErrData(:,9)*180.0/pi); % deg
subplot(3,1,3);plot(DD2ErrData(:,1),DD2ErrData(:,11));         % m
xlabel('位置误差补偿量(经度（度），纬度（度），高度（米）)');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(DD2ErrData(:,1),DD2ErrData(:,12)*180.0/pi*3600); %deg/h
subplot(3,1,2);plot(DD2ErrData(:,1),DD2ErrData(:,13)*180.0/pi*3600); %deg/h
subplot(3,1,3);plot(DD2ErrData(:,1),DD2ErrData(:,14)*180.0/pi*3600); %deg/h
xlabel('陀螺随机常数误差补偿量（度/小时）');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(DD2ErrData(:,1),DD2ErrData(:,15)*180.0/pi*3600); %deg/h
subplot(3,1,2);plot(DD2ErrData(:,1),DD2ErrData(:,16)*180.0/pi*3600); %deg/h
subplot(3,1,3);plot(DD2ErrData(:,1),DD2ErrData(:,17)*180.0/pi*3600); %deg/h
xlabel('陀螺一阶马尔可夫误差补偿量（度/小时）');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(DD2ErrData(:,1),DD2ErrData(:,18)/g); %g
subplot(3,1,2);plot(DD2ErrData(:,1),DD2ErrData(:,18)/g); %g
subplot(3,1,3);plot(DD2ErrData(:,1),DD2ErrData(:,20)/g); %g
xlabel('加速度计一阶马尔可夫误差补偿量（g）');
  %卡尔曼滤波修正数据
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%        DD1滤波器图形绘制函数
%                           程序设计：熊剑  日期：2007/6/4
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function []=ukf_fig(UKFTraceData,UKFIMUData,UKFSinsData,UKFKALData,UKFErrData,UKFGPSData);
g=9.7803698;         %重力加速度    （单位：米/秒/秒）
%%%%%%%%%%%%%绘制曲线%%%%%%%%%
fig_num=0;

% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(UKFTraceData(:,1),UKFTraceData(:,8));
% subplot(3,1,2);plot(UKFTraceData(:,1),UKFTraceData(:,9));
% subplot(3,1,3);plot(UKFTraceData(:,1),UKFTraceData(:,10));
% xlabel('飞行航迹仿真（经度（度），纬度（度），高度（米））');

% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(UKFTraceData(:,1),UKFTraceData(:,2));
% subplot(3,1,2);plot(UKFTraceData(:,1),UKFTraceData(:,3));
% subplot(3,1,3);plot(UKFTraceData(:,1),UKFTraceData(:,4));
% xlabel('飞行航迹仿真（横滚角度（度），俯仰角度（度），航向角度（度））');
%    % 航迹数据

% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(UKFIMUData(:,1),UKFIMUData(:,2));
% subplot(3,1,2);plot(UKFIMUData(:,1),UKFIMUData(:,3));
% subplot(3,1,3);plot(UKFIMUData(:,1),UKFIMUData(:,4));
% xlabel('IMU仿真（陀螺X轴（度/小时），陀螺Y轴（度/小时），陀螺Z轴（度/小时））');
% 
% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(UKFIMUData(:,1),UKFIMUData(:,5));
% subplot(3,1,2);plot(UKFIMUData(:,1),UKFIMUData(:,6));
% subplot(3,1,3);plot(UKFIMUData(:,1),UKFIMUData(:,7));
% xlabel('IMU仿真（加表X轴g），加表Y轴（g），加表Z轴（g））');
   %IMU数据

% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(UKFGPSData(:,1),UKFGPSData(:,2));
% subplot(3,1,2);plot(UKFGPSData(:,1),UKFGPSData(:,3));
% subplot(3,1,3);plot(UKFGPSData(:,1),UKFGPSData(:,4));
% xlabel('GPS仿真（经度（度），纬度（度），高度（米））');

% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(UKFGPSData(:,1),UKFGPSData(:,5));
% subplot(3,1,2);plot(UKFGPSData(:,1),UKFGPSData(:,6));
% subplot(3,1,3);plot(UKFGPSData(:,1),UKFGPSData(:,7));
% xlabel('GPS仿真（东向速度（米/秒），北向速度（米/秒），天向速度（米/秒））');
   %GPS仿真

% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(UKFSinsData(:,1),UKFSinsData(:,2));
% subplot(3,1,2);plot(UKFSinsData(:,1),UKFSinsData(:,3));
% subplot(3,1,3);plot(UKFSinsData(:,1),UKFSinsData(:,4));
% xlabel('导航姿态角输出（横滚角度（度），俯仰角度（度），航向角度（度））');
%    %姿态角度数据
%    
% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(UKFSinsData(:,1),UKFSinsData(:,5));
% subplot(3,1,2);plot(UKFSinsData(:,1),UKFSinsData(:,6));
% subplot(3,1,3);plot(UKFSinsData(:,1),UKFSinsData(:,7));
% xlabel('导航速度输出（东向（米/秒），北向（米/秒），天向（米/秒））');
%    %速度数据
% 
% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(UKFSinsData(:,1),UKFSinsData(:,8));
% subplot(3,1,2);plot(UKFSinsData(:,1),UKFSinsData(:,9));
% subplot(3,1,3);plot(UKFSinsData(:,1),UKFSinsData(:,10));
% xlabel('导航航迹（经度（度），纬度（度），高度（米））');
%    %位置数据

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(UKFTraceData(:,1),UKFSinsData(:,2)-UKFTraceData(:,2));
subplot(3,1,2);plot(UKFTraceData(:,1),UKFSinsData(:,3)-UKFTraceData(:,3));
subplot(3,1,3);plot(UKFTraceData(:,1),UKFSinsData(:,4)-UKFTraceData(:,4));
xlabel('姿态角度误差（横滚角度误差（度），俯仰角度误差（度），航向角度误差（度））');
   %飞行姿态角度误差曲线    
   
fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(UKFTraceData(:,8),UKFTraceData(:,9),'r',UKFSinsData(:,8),UKFSinsData(:,9));
subplot(3,1,2);plot(UKFTraceData(:,1),UKFSinsData(:,8)-UKFTraceData(:,8));
subplot(3,1,3);plot(UKFTraceData(:,1),UKFSinsData(:,9)-UKFTraceData(:,9));
xlabel('经纬度位置对比及其误差曲线（经纬度位置（度），经度误差（度），纬度误差（度））');
   %飞行位置轨迹及误差曲线

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(UKFGPSData(:,1),UKFGPSData(:,2),'r',UKFSinsData(:,1),UKFSinsData(:,8));
subplot(3,1,2);plot(UKFGPSData(:,1),UKFGPSData(:,3),'r',UKFSinsData(:,1),UKFSinsData(:,9));
subplot(3,1,3);plot(UKFGPSData(:,1),UKFGPSData(:,4),'r',UKFSinsData(:,1),UKFSinsData(:,10));
xlabel('组合导航与GPS对比（经度（度），纬度（度），高度（米））');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(UKFGPSData(:,1),UKFGPSData(:,5),'r',UKFSinsData(:,1),UKFSinsData(:,5));
subplot(3,1,2);plot(UKFGPSData(:,1),UKFGPSData(:,6),'r',UKFSinsData(:,1),UKFSinsData(:,6));
subplot(3,1,3);plot(UKFGPSData(:,1),UKFGPSData(:,7),'r',UKFSinsData(:,1),UKFSinsData(:,7));
xlabel('组合导航与GPS对比（东向速度（米/秒），北向速度（米/秒），天向速度（米/秒））');
    %组合导航与GPS对比输出

fig_num = fig_num+1;
figure(fig_num);
subplot(4,1,1);plot(UKFKALData(:,1),UKFKALData(:,5));
subplot(4,1,2);plot(UKFKALData(:,1),UKFKALData(:,6));
subplot(4,1,3);plot(UKFKALData(:,1),UKFKALData(:,7));
subplot(4,1,4);plot(UKFKALData(:,1),UKFKALData(:,8));
xlabel('卡尔曼滤波输出（平台误差角(四元数)（秒））');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(UKFKALData(:,1),UKFKALData(:,2));
subplot(3,1,2);plot(UKFKALData(:,1),UKFKALData(:,3));
subplot(3,1,3);plot(UKFKALData(:,1),UKFKALData(:,4));
xlabel('卡尔曼滤波输出（速度误差（米/秒））');

fig_num = fig_num+1;
figure(fig_num);
subplot(3,1,1);plot(UKFKALData(:,1),UKFKALData(:,10));
subplot(3,1,2);plot(UKFKALData(:,1),UKFKALData(:,9));
subplot(3,1,3);plot(UKFKALData(:,1),UKFKALData(:,11));
xlabel('卡尔曼滤波输出（位置误差（米））');

% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(UKFKALData(:,1),UKFKALData(:,12));
% subplot(3,1,2);plot(UKFKALData(:,1),UKFKALData(:,13));
% subplot(3,1,3);plot(UKFKALData(:,1),UKFKALData(:,14));
% xlabel('陀螺随机常数误差（度/小时）');
% 
% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(UKFKALData(:,1),UKFKALData(:,15));
% subplot(3,1,2);plot(UKFKALData(:,1),UKFKALData(:,16));
% subplot(3,1,3);plot(UKFKALData(:,1),UKFKALData(:,17));
% xlabel('陀螺一阶马尔可夫误差（度/小时）');

% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(UKFKALData(:,1),UKFKALData(:,18));
% subplot(3,1,2);plot(UKFKALData(:,1),UKFKALData(:,19));
% subplot(3,1,3);plot(UKFKALData(:,1),UKFKALData(:,20));
% xlabel('加速度计一阶马尔可夫误差（g）');
   %%%%%%%%%%卡尔曼估计误差%%%%%%%%%%%
   
%return;

% fig_num = fig_num+1;
% figure(fig_num);
% subplot(4,1,1);plot(UKFErrData(:,1),UKFErrData(:,5)); %sec
% subplot(4,1,2);plot(UKFErrData(:,1),UKFErrData(:,6)); %sec
% subplot(4,1,3);plot(UKFErrData(:,1),UKFErrData(:,7)); %sec
% subplot(4,1,4);plot(UKFErrData(:,1),UKFErrData(:,8)); %sec
% xlabel('平台误差补偿量(四元数)（角秒）');

% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(UKFErrData(:,1),UKFErrData(:,2)); %米/秒
% subplot(3,1,2);plot(UKFErrData(:,1),UKFErrData(:,3)); %米/秒
% subplot(3,1,3);plot(UKFErrData(:,1),UKFErrData(:,4)); %米/秒
% xlabel('速度误差补偿量（米/秒）');

% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(UKFErrData(:,1),UKFErrData(:,10)*180.0/pi); % deg
% subplot(3,1,2);plot(UKFErrData(:,1),UKFErrData(:,9)*180.0/pi); % deg
% subplot(3,1,3);plot(UKFErrData(:,1),UKFErrData(:,11));         % m
% xlabel('位置误差补偿量(经度（度），纬度（度），高度（米）)');

% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(UKFErrData(:,1),UKFErrData(:,12)*180.0/pi*3600); %deg/h
% subplot(3,1,2);plot(UKFErrData(:,1),UKFErrData(:,13)*180.0/pi*3600); %deg/h
% subplot(3,1,3);plot(UKFErrData(:,1),UKFErrData(:,14)*180.0/pi*3600); %deg/h
% xlabel('陀螺随机常数误差补偿量（度/小时）');

% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(UKFErrData(:,1),UKFErrData(:,15)*180.0/pi*3600); %deg/h
% subplot(3,1,2);plot(UKFErrData(:,1),UKFErrData(:,16)*180.0/pi*3600); %deg/h
% subplot(3,1,3);plot(UKFErrData(:,1),UKFErrData(:,17)*180.0/pi*3600); %deg/h
% xlabel('陀螺一阶马尔可夫误差补偿量（度/小时）');

% fig_num = fig_num+1;
% figure(fig_num);
% subplot(3,1,1);plot(UKFErrData(:,1),UKFErrData(:,18)/g); %g
% subplot(3,1,2);plot(UKFErrData(:,1),UKFErrData(:,18)/g); %g
% subplot(3,1,3);plot(UKFErrData(:,1),UKFErrData(:,20)/g); %g
% xlabel('加速度计一阶马尔可夫误差补偿量（g）');
% 


% fig_num = fig_num+1;
% figure(fig_num);
% plot3(UKFTraceData(:,8),UKFTraceData(:,9),UKFTraceData(:,10),'r');
% grid on
% xlabel('飞行航迹仿真（经度（度），纬度（度），高度（米））');
% 
% fig_num = fig_num+1;
% figure(fig_num);
% plot3(UKFSinsData(:,8),UKFSinsData(:,9),UKFSinsData(:,10),'g');
% grid on
% xlabel('导航航迹（经度（度），纬度（度），高度（米））');
%   %卡尔曼滤波修正数据 
  
  
fig_num = fig_num+1;
figure(fig_num);
figure(fig_num);
subplot(3,1,1);plot(UKFSinsData(:,1),UKFSinsData(:,5)-UKFTraceData(:,4));
subplot(3,1,2);plot(UKFSinsData(:,1),UKFSinsData(:,6)-UKFTraceData(:,5));
subplot(3,1,3);plot(UKFSinsData(:,1),UKFSinsData(:,7)-UKFTraceData(:,6));
xlabel('组合导航速度误差（东向速度（米/秒），北向速度（米/秒），天向速度（米/秒））');

fig_num = fig_num+1;
figure(fig_num);
plot(UKFTraceData(:,8),UKFTraceData(:,9),'r',UKFSinsData(:,8),UKFSinsData(:,9),'g');
grid on
xlabel('飞行航迹仿真（经度（度），纬度（度）');
ylabel('导航航迹（经度（度），纬度（度）');

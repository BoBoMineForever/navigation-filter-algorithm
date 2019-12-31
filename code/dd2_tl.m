%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%                   四阶龙格库塔解f(y(x),v(x))形式微分方程
%
%  输入参数:t-仿真时间，T_D-离散周期；
%           Fb-加速度计输出；
%           Wibb-陀螺仪输出（度）；
%           attiN-导航计算的姿态角度横滚，俯仰，航向（度，度，度）；
%           veloN-飞行器相对机体系的运动速度东向、北向、天向（米/秒）；
%           posiN-导航输出的飞行器位置（经度（度）、纬度（度）、高度（米））； 
%           Xd-综合模型状态量；Vd-白噪声误差项
%  输出参数：Xd-综合模型状态量；         
%
%                           程序设计：熊剑  日期：2007/5/18
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [Xd]=dd2_rk(T_D,Fb,Wibb,attiN,veloN,posiN,Xd,Vd)
  Re=6378137.0;                                      %地球半径（米） 
  f=1/298.257;                                       %地球的椭圆率
  Wie=7.292115147e-5;                                %地球自转角速度
  g=9.7803698;                                       %重力加速度

  long=posiN(1,1)*pi/180.0;lati=posiN(2,1)*pi/180.0;heig=posiN(3,1);
  %飞行器位置

  %地球曲率半径求解
  Rm=Re*(1-2*f+3*f*sin(lati)*sin(lati));
  Rn=Re*(1+f*sin(lati)*sin(lati));
      
  roll=attiN(1,1)*pi/180.0;pitch=attiN(2,1)*pi/180.0;head=attiN(3,1)*pi/180.0;
    
  Cnb=[cos(roll)*cos(head)+sin(roll)*sin(pitch)*sin(head), -cos(roll)*sin(head)+sin(roll)*sin(pitch)*cos(head), -sin(roll)*cos(pitch);
       cos(pitch)*sin(head),                               cos(pitch)*cos(head),                                sin(pitch);
       sin(roll)*cos(head)-cos(roll)*sin(pitch)*sin(head), -sin(roll)*sin(head)-cos(roll)*sin(pitch)*cos(head), cos(roll)*cos(pitch)];
  %坐标系N-->B
  Cbn=Cnb';
  Fn=Cnb'*Fb;
  Ve=veloN(1,1);Vn=veloN(2,1);Vu=veloN(3,1);
  
  Wenn=[-Vn/(Rm+heig); Ve/(Rn+heig);  Ve/(Rn+heig)*tan(lati)];
  Wien=[0;             Wie*cos(lati); Wie*sin(lati)];
  Winn=Wenn+Wien;
  Wibb=Wibb*pi/180.0;
  %单位：弧度/秒
  Q=[cos(head/2)*cos(pitch/2)*cos(roll/2)+sin(head/2)*sin(pitch/2)*sin(roll/2);
     cos(head/2)*sin(pitch/2)*cos(roll/2)+sin(head/2)*cos(pitch/2)*sin(roll/2);
     cos(head/2)*cos(pitch/2)*sin(roll/2)-sin(head/2)*sin(pitch/2)*cos(roll/2);
     -1.0*sin(head/2)*cos(pitch/2)*cos(roll/2)+cos(head/2)*sin(pitch/2)*sin(roll/2)];

  Tgx=3600.0; Tgy=3600.0;  Tgz=3600.0; 
  Tax=1800.0; Tay=1800.0;  Taz=1800.0; 
  %陀螺和加速度计的一阶马尔可夫相关时间（与IMU仿真同）
  Ouw=[0,-Wibb(1,1),-Wibb(2,1),-Wibb(3,1);
       Wibb(1,1),0,Wibb(3,1),-Wibb(2,1);
       Wibb(2,1),-Wibb(3,1),0,Wibb(1,1);
       Wibb(3,1),Wibb(2,1),-Wibb(1,1),0];
  Odw=[0,-Winn(1,1),-Winn(2,1),-Winn(3,1);
       Winn(1,1),0,-Winn(3,1),Winn(2,1);
       Winn(2,1),Winn(3,1),0,-Winn(1,1);
       Winn(3,1),-Winn(2,1),Winn(1,1),0];
  UQ=[-Q(2,1),-Q(3,1),-Q(4,1);
      Q(1,1),-Q(4,1),Q(3,1);
      Q(4,1),Q(1,1),-Q(2,1);
      -Q(3,1),Q(2,1),Q(1,1)];
  YQ=[-Q(2,1),-Q(3,1),-Q(4,1);
      Q(1,1),Q(4,1),-Q(3,1);
      -Q(4,1),Q(1,1),Q(2,1);
      Q(3,1),-Q(2,1),Q(1,1)];
  VV=[Vn*tan(lati)/(Rn+heig)-Vu/(Rn+heig),(2*Wie+Ve*sec(lati)/(Rn+heig))*sin(lati),-(2*Wie+Ve*sec(lati)/(Rn+heig))*cos(lati);
      -(2*Wie+Ve*sec(lati)/(Rn+heig))*sin(lati),-Vu/(Rm+heig),-Vn/(Rm+heig);
     (2*Wie+Ve*sec(lati)/(Rn+heig))*cos(lati),2*Vn/(Rm+heig),0];
  VP=[2*Wie*sin(lati)*Vu+(Ve*sec(lati)*sec(lati)/(Rn+heig)+2*Wie*cos(lati))*Vn,0,Vn*Ve/(Rn+heig)^2-Vn*Ve*tan(lati)/(Rn+heig)^2;
      -Ve*(Ve*sec(lati)*sec(lati)/(Rn+heig)+2*Wie*cos(lati)),0,Vu*Vn/(Rm+heig)^2-Ve*Ve*tan(lati)/(Rn+heig)^2;
      -Ve*2*Wie*sin(lati),0,-Vn*Vn/(Rm+heig)^2-Ve*Ve/(Rn+heig)^2];
  QV=[0,-1/(Rm+heig),0;1/(Rn+heig),0,0;tan(lati)/(Rn+heig),0,0];
  QP=[0,0,Vn/(Rm+heig)^2;-Wie*sin(lati),0,-Ve/(Rn+heig)^2;Ve*sec(lati)*sec(lati)/(Rn+heig)+Wie*cos(lati),0,-Ve*tan(lati)/(Rn+heig)^2];
  PV=[0,1/(Rm+heig),0;1/(Rn+heig)/cos(lati),0,0;0,0,1];
  PP=[0,0,-Vn/(Rm+heig)^2;Ve/(Rn+heig)*sec(lati)*tan(lati),0,-Ve/(Rn+heig)^2/cos(lati);0,0,0];
  FI=[-0.5*YQ*QV,0.5*Ouw-0.5*Odw,-0.5*YQ*QP,0.5*UQ,0.5*UQ,zeros(4,3);
      PV,zeros(3,4),PP,zeros(3,9);
      zeros(9,10),diag([0,0,0,-1.0/Tgx,-1.0/Tgy,-1.0/Tgz,-1.0/Tax,-1.0/Tay,-1.0/Taz])];
  GI=[zeros(3,9);0.5*UQ,zeros(4,6);zeros(6,9);zeros(3,3),eye(3),zeros(3,3);zeros(3,6),eye(3)];
  Xd_tmp=Xd;
  Vd_tmp=Vd;
  %%%%%%%泰勒展开解微分方程%%%%%%%%
  XQ=Xd_tmp(4:7,1);
  Vnon=zeros(3,3);
  Vnon(1,1)=2*(Q(1,1)*XQ(1,1)+Q(2,1)*XQ(2,1)-Q(3,1)*XQ(3,1)-Q(4,1)*XQ(4,1))-XQ(1,1)*XQ(1,1)-XQ(2,1)*XQ(2,1)+XQ(3,1)*XQ(3,1)+XQ(4,1)*XQ(4,1);
  Vnon(2,2)=2*(Q(1,1)*XQ(1,1)-Q(2,1)*XQ(2,1)+Q(3,1)*XQ(3,1)-Q(4,1)*XQ(4,1))-XQ(1,1)*XQ(1,1)+XQ(2,1)*XQ(2,1)-XQ(3,1)*XQ(3,1)+XQ(4,1)*XQ(4,1);
  Vnon(3,3)=2*(Q(1,1)*XQ(1,1)-Q(2,1)*XQ(2,1)-Q(3,1)*XQ(3,1)+Q(4,1)*XQ(4,1))-XQ(1,1)*XQ(1,1)+XQ(2,1)*XQ(2,1)+XQ(3,1)*XQ(3,1)-XQ(4,1)*XQ(4,1);
  Vnon(1,2)=2*(Q(2,1)*XQ(3,1)+Q(3,1)*XQ(2,1)-Q(1,1)*XQ(4,1)-Q(4,1)*XQ(1,1)-XQ(2,1)*XQ(3,1)+XQ(1,1)*XQ(4,1));
  Vnon(1,3)=2*(Q(2,1)*XQ(4,1)+Q(4,1)*XQ(2,1)+Q(1,1)*XQ(3,1)+Q(3,1)*XQ(1,1)-XQ(2,1)*XQ(4,1)-XQ(1,1)*XQ(3,1));
  Vnon(2,1)=2*(Q(2,1)*XQ(3,1)+Q(3,1)*XQ(2,1)+Q(1,1)*XQ(4,1)+Q(4,1)*XQ(1,1)-XQ(2,1)*XQ(3,1)-XQ(1,1)*XQ(4,1));
  Vnon(2,3)=2*(Q(3,1)*XQ(4,1)+Q(4,1)*XQ(3,1)-Q(1,1)*XQ(2,1)-Q(2,1)*XQ(1,1)-XQ(3,1)*XQ(4,1)+XQ(1,1)*XQ(2,1));
  Vnon(3,1)=2*(Q(2,1)*XQ(4,1)+Q(4,1)*XQ(2,1)-Q(1,1)*XQ(3,1)-Q(3,1)*XQ(1,1)-XQ(2,1)*XQ(4,1)+XQ(1,1)*XQ(3,1));
  Vnon(3,2)=2*(Q(1,1)*XQ(2,1)+Q(2,1)*XQ(1,1)+Q(3,1)*XQ(4,1)+Q(4,1)*XQ(3,1)-XQ(3,1)*XQ(4,1)-XQ(1,1)*XQ(2,1));
  FX(1:3,1)=VV*Xd_tmp(1:3,1)+Vnon*Fb+VP*Xd_tmp(8:10,1)+Cbn*Xd_tmp(17:19,1);
  FX(4:19,1)=FI*Xd_tmp;    
%   DQ=zeros(3,4);
%   DQ(1,1)=2*(Q(1,1)-XQ(1,1))*Fb(1,1)+2*(-Q(4,1)+XQ(4,1))*Fb(2,1)+2*(Q(3,1)-XQ(3,1))*Fb(3,1);
%   DQ(1,2)=2*(Q(2,1)-XQ(2,1))*Fb(1,1)+2*(Q(3,1)-XQ(3,1))*Fb(2,1)+2*(Q(4,1)-XQ(4,1))*Fb(3,1);
%   DQ(1,3)=2*(-Q(3,1)+XQ(3,1))*Fb(1,1)+2*(Q(2,1)-XQ(2,1))*Fb(2,1)+2*(Q(1,1)-XQ(1,1))*Fb(3,1);
%   DQ(1,4)=2*(-Q(4,1)+XQ(4,1))*Fb(1,1)+2*(-Q(1,1)+XQ(1,1))*Fb(2,1)+2*(Q(2,1)-XQ(2,1))*Fb(3,1);  
%   DQ(2,1)=2*(Q(4,1)-XQ(4,1))*Fb(1,1)+2*(Q(1,1)-XQ(1,1))*Fb(2,1)+2*(-Q(2,1)+XQ(2,1))*Fb(3,1);
%   DQ(2,2)=2*(Q(3,1)-XQ(3,1))*Fb(1,1)+2*(-Q(2,1)+XQ(2,1))*Fb(2,1)+2*(-Q(1,1)+XQ(1,1))*Fb(3,1);
%   DQ(2,3)=2*(Q(2,1)-XQ(2,1))*Fb(1,1)+2*(Q(3,1)-XQ(3,1))*Fb(2,1)+2*(Q(4,1)-XQ(4,1))*Fb(3,1);
%   DQ(2,4)=2*(Q(1,1)-XQ(1,1))*Fb(1,1)+2*(-Q(4,1)+XQ(4,1))*Fb(2,1)+2*(Q(3,1)-XQ(3,1))*Fb(3,1); 
%   DQ(3,1)=2*(-Q(3,1)+XQ(3,1))*Fb(1,1)+2*(Q(2,1)-XQ(2,1))*Fb(2,1)+2*(Q(1,1)-XQ(1,1))*Fb(3,1);
%   DQ(3,2)=2*(Q(4,1)-XQ(4,1))*Fb(1,1)+2*(Q(1,1)-XQ(1,1))*Fb(2,1)+2*(-Q(2,1)+XQ(2,1))*Fb(3,1);
%   DQ(3,3)=2*(-Q(1,1)+XQ(1,1))*Fb(1,1)+2*(Q(4,1)-XQ(4,1))*Fb(2,1)+2*(-Q(3,1)+XQ(3,1))*Fb(3,1);
%   DQ(3,4)=2*(Q(2,1)-XQ(2,1))*Fb(1,1)+2*(Q(3,1)-XQ(3,1))*Fb(2,1)+2*(Q(4,1)-XQ(4,1))*Fb(3,1);
%   DX=[VV,DQ,VP,zeros(3,6),Cbn;
%       -0.5*YQ*QV,0.5*Ouw-0.5*Odw,-0.5*YQ*QP,0.5*UQ,0.5*UQ,zeros(4,3);
%       PV,zeros(3,4),PP,zeros(3,9);
%       zeros(9,10),diag([0,0,0,-1.0/Tgx,-1.0/Tgy,-1.0/Tgz,-1.0/Tax,-1.0/Tay,-1.0/Taz])];
%   Xd=Xd+FX*T_D+DX*FX*T_D*T_D/2;
    Xd=Xd+FX*T_D+GI*T_D*Vd_tmp;
  
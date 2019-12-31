%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%                   卡尔曼滤波初始化程序(利用GPS进行位置速度组合)
%
%  输入参数:
%           posiN-初始的飞行器位置（经度（度）、纬度（度）、高度（米））； 
%           Xc-综合模型状态量；PK-协方差阵；
%           Xerr-状态估计量的误差值（记录所有时刻的）
%  输出参数：Xc-综合模型状态量；PK-协方差阵；Xerr-状态估计量的误差值（记录某一时刻的）         
%
%                           程序设计：熊智  日期：2003/10/04
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [Xd,PKD,DD2Xerr]=dd2_gps_init(posiN,atti,Xd,PKD,DD2Xerr);
 
  Re=6378137.0;                                      %地球半径（米） 
  f=1/298.257;                                        %地球的椭圆率
  Wie=7.292115147e-5;                          %地球自转角速度
  g=9.7803698;                                      %重力加速度

  long=posiN(1,1)*pi/180.0;lati=posiN(2,1)*pi/180.0;heig=posiN(3,1);
    %飞行器位置
  roll=atti(1,1)*pi/180.0;pitch=atti(2,1)*pi/180.0;head=atti(3,1)*pi/180.0;
  %地球曲率半径求解
  Rm=Re*(1-2*f+3*f*sin(lati)*sin(lati));
  Rn=Re*(1+f*sin(lati)*sin(lati));
  Qexp=[cos(head/2)*cos(pitch/2)*cos(roll/2)+sin(head/2)*sin(pitch/2)*sin(roll/2);
        cos(head/2)*sin(pitch/2)*cos(roll/2)+sin(head/2)*cos(pitch/2)*sin(roll/2);
        cos(head/2)*cos(pitch/2)*sin(roll/2)-sin(head/2)*sin(pitch/2)*cos(roll/2);
        -1.0*sin(head/2)*cos(pitch/2)*cos(roll/2)+cos(head/2)*sin(pitch/2)*sin(roll/2)];
   roll=roll+180.0*pi/(3600.0*180.0);
   pitch=pitch+180.0*pi/(3600.0*180.0);
   head=head+360.0*pi/(3600.0*180.0);
   Qact=[cos(head/2)*cos(pitch/2)*cos(roll/2)+sin(head/2)*sin(pitch/2)*sin(roll/2);
         cos(head/2)*sin(pitch/2)*cos(roll/2)+sin(head/2)*cos(pitch/2)*sin(roll/2);
         cos(head/2)*cos(pitch/2)*sin(roll/2)-sin(head/2)*sin(pitch/2)*cos(roll/2);
         -1.0*sin(head/2)*cos(pitch/2)*cos(roll/2)+cos(head/2)*sin(pitch/2)*sin(roll/2)];
    Qini=Qexp-Qact;
    DD2Xerr0=[0.5,0.5,0.5,Qini(1,1),Qini(2,1),Qini(3,1),Qini(4,1),50.0/(Rm+heig),50.0/(Rn+heig)/cos(lati),50.0, ...
         0.1*pi/(3600.0*180.0),0.1*pi/(3600.0*180.0),0.1*pi/(3600.0*180.0),0.1*pi/(3600.0*180.0),0.1*pi/(3600.0*180.0),0.1*pi/(3600.0*180.0),(1e-4)*g,(1e-4)*g,(1e-4)*g];
      %估计误差初值（与IMU仿真同）单位：rad,rad,rad,m/s,m/s,m/s,rad,rad,m,rad/s,rad/s,rad/s,rad/s,rad/s,rad/s,m/s/s,m/s/s,m/s/s
          
    PKD=diag((DD2Xerr0.^2));
    Xd=zeros(19,1);
    
    DD2Xerr(1,1)=sqrt(PKD(1,1));                    %m/s
    DD2Xerr(1,2)=sqrt(PKD(2,2));                    %m/s
    DD2Xerr(1,3)=sqrt(PKD(3,3));                    %m/s
    DD2Xerr(1,4)=sqrt(PKD(4,4));    %四元数
    DD2Xerr(1,5)=sqrt(PKD(5,5));    %四元数
    DD2Xerr(1,6)=sqrt(PKD(6,6));    %四元数
    DD2Xerr(1,7)=sqrt(PKD(7,7));    %四元数
    DD2Xerr(1,8)=sqrt(PKD(8,8))*(Rm+heig);          %m
    DD2Xerr(1,9)=sqrt(PKD(9,9))*(Rn+heig)*cos(lati);%m
    DD2Xerr(1,10)=sqrt(PKD(10,10));                 %m
         %INS的10个导航量误差
         
    DD2Xerr(1,11)=sqrt(PKD(11,11))*180.0*3600.0/pi;   %deg/h
    DD2Xerr(1,12)=sqrt(PKD(12,12))*180.0*3600.0/pi;   %deg/h
    DD2Xerr(1,13)=sqrt(PKD(13,13))*180.0*3600.0/pi;   %deg/h
    DD2Xerr(1,14)=sqrt(PKD(14,14))*180.0*3600.0/pi;   %deg/h
    DD2Xerr(1,15)=sqrt(PKD(15,15))*180.0*3600.0/pi;   %deg/h
    DD2Xerr(1,16)=sqrt(PKD(16,16))*180.0*3600.0/pi;   %deg/h
    DD2Xerr(1,17)=sqrt(PKD(17,17))/g;                 %g
    DD2Xerr(1,18)=sqrt(PKD(18,18))/g;                 %g
    DD2Xerr(1,19)=sqrt(PKD(19,19))/g;                 %g
        %IMU的9个误差量
        


      

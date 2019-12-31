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


function [Xu,PKU,UKFXerr]=ukf_gps_init(posiN,atti,Xu,PKU,UKFXerr);
 
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
    UKFXerr0=[0.5,0.5,0.5,Qini(1,1),Qini(2,1),Qini(3,1),Qini(4,1),50.0/(Rm+heig),50.0/(Rn+heig)/cos(lati),50.0, ...
         0.1*pi/(3600.0*180.0),0.1*pi/(3600.0*180.0),0.1*pi/(3600.0*180.0),0.1*pi/(3600.0*180.0),0.1*pi/(3600.0*180.0),0.1*pi/(3600.0*180.0),(1e-4)*g,(1e-4)*g,(1e-4)*g];
      %估计误差初值（与IMU仿真同）单位：rad,rad,rad,m/s,m/s,m/s,rad,rad,m,rad/s,rad/s,rad/s,rad/s,rad/s,rad/s,m/s/s,m/s/s,m/s/s
          
    PKU=diag((UKFXerr0.^2));
    Xu=zeros(19,1);
    
    UKFXerr(1,1)=sqrt(PKU(1,1));                    %m/s
    UKFXerr(1,2)=sqrt(PKU(2,2));                    %m/s
    UKFXerr(1,3)=sqrt(PKU(3,3));                    %m/s
    UKFXerr(1,4)=sqrt(PKU(4,4));    %四元数
    UKFXerr(1,5)=sqrt(PKU(5,5));    %四元数
    UKFXerr(1,6)=sqrt(PKU(6,6));    %四元数
    UKFXerr(1,7)=sqrt(PKU(7,7));    %四元数
    UKFXerr(1,8)=sqrt(PKU(8,8))*(Rm+heig);          %m
    UKFXerr(1,9)=sqrt(PKU(9,9))*(Rn+heig)*cos(lati);%m
    UKFXerr(1,10)=sqrt(PKU(10,10));                 %m
         %INS的10个导航量误差
         
    UKFXerr(1,11)=sqrt(PKU(11,11))*180.0*3600.0/pi;   %deg/h
    UKFXerr(1,12)=sqrt(PKU(12,12))*180.0*3600.0/pi;   %deg/h
    UKFXerr(1,13)=sqrt(PKU(13,13))*180.0*3600.0/pi;   %deg/h
    UKFXerr(1,14)=sqrt(PKU(14,14))*180.0*3600.0/pi;   %deg/h
    UKFXerr(1,15)=sqrt(PKU(15,15))*180.0*3600.0/pi;   %deg/h
    UKFXerr(1,16)=sqrt(PKU(16,16))*180.0*3600.0/pi;   %deg/h
    UKFXerr(1,17)=sqrt(PKU(17,17))/g;                 %g
    UKFXerr(1,18)=sqrt(PKU(18,18))/g;                 %g
    UKFXerr(1,19)=sqrt(PKU(19,19))/g;                 %g
        %IMU的9个误差量
        


      

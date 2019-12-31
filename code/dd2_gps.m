%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%                   卡尔曼滤波程序(利用GPS进行位置速度组合)
%
%  输入参数:t-仿真时间，T_D-离散周期；
%           Fb-加速度计输出；
%           attiN-导航计算的姿态角度横滚，俯仰，航向（度，度，度）；
%           veloN-飞行器相对机体系的运动速度东向、北向、天向（米/秒）；
%           posiG-GPS输出的飞行器位置（经度（度）、纬度（度）、高度（米））； 
%           veloG-GPS输出的飞行器速度（东向（米/秒），北向（米/秒），天向（米/秒））
%           Xc-综合模型状态量；PK-协方差阵；
%           Xerr-状态估计量的误差值（记录所有时刻的）
%           kflag-GPS信息有效标志位（1－有效）
%  输出参数：Xc-综合模型状态量；PK-协方差阵；Xerr-状态估计量的误差值（记录某一时刻的）         
%
%                           程序设计：熊智  日期：2003/9/29
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [Xd,PKD,DD2Xerr]=dd2_gps(t,T_D,Fb,Wibb,attiN,veloN,posiN,posiG,veloG,Xd,PKD,DD2Xerr,kflag)
 
  Re=6378137.0;                                      %地球半径（米） 
  f=1/298.257;                                        %地球的椭圆率
  Wie=7.292115147e-5;                          %地球自转角速度
  g=9.7803698;                                      %重力加速度

  long=posiN(1,1)*pi/180.0;lati=posiN(2,1)*pi/180.0;heig=posiN(3,1);
    %飞行器位置

  %地球曲率半径求解
  Rm=Re*(1-2*f+3*f*sin(lati)*sin(lati));
  Rn=Re*(1+f*sin(lati)*sin(lati));
      
  roll=attiN(1,1)*pi/180.0;pitch=attiN(2,1)*pi/180.0;head=attiN(3,1)*pi/180.0;
    
  Cbn=[cos(roll)*cos(head)+sin(roll)*sin(pitch)*sin(head), -cos(roll)*sin(head)+sin(roll)*sin(pitch)*cos(head), -sin(roll)*cos(pitch);
       cos(pitch)*sin(head),                               cos(pitch)*cos(head),                                sin(pitch);
       sin(roll)*cos(head)-cos(roll)*sin(pitch)*sin(head), -sin(roll)*sin(head)-cos(roll)*sin(pitch)*cos(head), cos(roll)*cos(pitch)];
  %坐标系N-->B
  
  Fn=Cbn'*Fb;

  Ve=veloN(1,1);Vn=veloN(2,1);Vu=veloN(3,1);
 
  Tgx=3600.0; Tgy=3600.0;  Tgz=3600.0; 
  Tax=1800.0; Tay=1800.0;  Taz=1800.0; 
    %陀螺和加速度计的一阶马尔可夫相关时间（与IMU仿真同）
  a=0.1;  
  W=[a*pi/(3600*180),a*pi/(3600*180),a*pi/(3600*180), ...
     sqrt(2*T_D/Tgx)*a*pi/(3600*180),sqrt(2*T_D/Tgy)*a*pi/(3600*180),sqrt(2*T_D/Tgz)*a*pi/(3600*180), ...
     sqrt(2*T_D/Tax)*(1e-4)*g,sqrt(2*T_D/Tay)*(1e-4)*g,sqrt(2*T_D/Taz)*(1e-4)*g]';  
      %系统噪声阵（与IMU仿真同）单位：rad/s,rad/s,rad/s,rad/s,rad/s,rad/s,m/s/s,m/s/s,m/s/s

  %GPS/INS位置量测矩阵
  HG=[zeros(3,7),diag([Rm,Rn*cos(lati),1]),zeros(3,9)];
  VG=[0.1;0.1;0.2];  % 需要与GPS仿真精度相同

  %GPS/INS位置＋速度量测方程
  HG=[HG;diag([1,1,1]),zeros(3,4),zeros(3,12)];
  VG=[VG;0.2;0.2;0.2]; % 需要与GPS仿真精度相同
  
  Q=diag((W.^2)');
  RG=diag((VG.^2)');
  %修正量计算    
  Yc=[(posiN(2,1)-posiG(2,1))*pi/180.0*(Rm+heig);
      (posiN(1,1)-posiG(1,1))*pi/180.0*(Rn+heig)*cos(lati);
       posiN(3,1)-posiG(3,1)]; %量测次序为纬度、经度、高度
  Yc=[Yc;veloN-veloG]; %单位（米，米/秒）
  
  %系统状态量误差方差计算
    
  if( kflag == 1 )
    %Stirling插值滤波DD2，公式参考《用于微小卫星姿态确定的非线性估计算法》《无陀螺卫星姿态的二阶插值非线性滤波估计》
    Hdd=sqrt(3);                       %论文中的有误，原始论文为sqrt(3)
    Sv=chol(Q);
    Sw=chol(RG);
    Sx=chol(PKD)';

    for i=1:19
        Fxp(:,i)=dd2_tl(T_D,Fb,Wibb,attiN,veloN,posiN,Xd+Hdd*Sx(:,i),zeros(9,1));
        Fxm(:,i)=dd2_tl(T_D,Fb,Wibb,attiN,veloN,posiN,Xd-Hdd*Sx(:,i),zeros(9,1));
    end
    Fx=dd2_tl(T_D,Fb,Wibb,attiN,veloN,posiN,Xd,zeros(9,1));
    for i=1:19
        Sxx1(:,i)=1/(2*Hdd)*(Fxp(:,i)-Fxm(:,i));
        Sxx2(:,i)=sqrt(Hdd^2-1)/(2*Hdd^2)*(Fxp(:,i)+Fxm(:,i)-2*Fx);
    end
    for i=1:9
        Fvp(:,i)=dd2_tl(T_D,Fb,Wibb,attiN,veloN,posiN,Xd,Hdd*Sv(:,i));
        Fvm(:,i)=dd2_tl(T_D,Fb,Wibb,attiN,veloN,posiN,Xd,-Hdd*Sv(:,i));
    end
    for i=1:9
        Sxv1(:,i)=1/(2*Hdd)*(Fvp(:,i)-Fvm(:,i));
        Sxv2(:,i)=sqrt(Hdd^2-1)/(2*Hdd^2)*(Fvp(:,i)+Fvm(:,i)-2*Fx);
    end
    Xd=((Hdd^2-19-9)/Hdd^2)*Fx;
    for i=1:19
        Xd=Xd+1/(2*Hdd^2)*(Fxp(:,i)+Fxm(:,i));
    end
    for i=1:9
        Xd=Xd+1/(2*Hdd^2)*(Fvp(:,i)+Fvm(:,i));
    end    
    Sx=[Sxx1';Sxv1';Sxx2';Sxv2']';
    [QQ,RR]=qr(Sx');
    Sx=(RR(1:19,:))';
    PKD=Sx*Sx';
    for i=1:19
        Fyp(:,i)=HG*(Xd+Hdd*Sx(:,i));
        Fym(:,i)=HG*(Xd-Hdd*Sx(:,i));
    end
    Fy=HG*Xd;
    for i=1:19
        Syx1(:,i)=1/(2*Hdd)*(Fyp(:,i)-Fym(:,i));
        Syx2(:,i)=sqrt(Hdd^2-1)/(2*Hdd^2)*(Fyp(:,i)+Fym(:,i)-2*Fy);
    end
    for i=1:6
        Fwp(:,i)=HG*Xd+Hdd*Sw(:,i);
        Fwm(:,i)=HG*Xd-Hdd*Sw(:,i);     
    end
    for i=1:6
        Syw1(:,i)=1/(2*Hdd)*(Fwp(:,i)-Fwm(:,i));
        Syw2(:,i)=sqrt(Hdd^2-1)/(2*Hdd^2)*(Fwp(:,i)+Fwm(:,i)-2*Fy);
    end
    Ycc=((Hdd^2-19-6)/Hdd^2)*Fy;
    for i=1:19
        Ycc=Ycc+1/(2*Hdd^2)*(Fyp(:,i)+Fym(:,i));
    end
    for i=1:6
        Ycc=Ycc+1/(2*Hdd^2)*(Fwp(:,i)+Fwm(:,i));
    end    
    Sy=[Syx1';Syw1';Syx2';Syw2']';
    [QQ,RR]=qr(Sy');
    Sy=(RR(1:6,:))';
    Py=Sy*Sy';
    Pxy=Sx*Syx1';                                                       %《无陀螺卫星姿态的二阶插值非线性滤波估计》
    KK=Pxy*inv(Py);
    Xd=Xd+KK*(Yc-Ycc);
    Sx=[(Sx-KK*Syx1)';(KK*Syw1)';(KK*Syx2)';(KK*Syw2)']';
    [QQ,RR]=qr(Sx');
    Sx=(RR(1:19,:))';
    PKD=Sx*Sx';
  end

  %%%%%%%%%%%%%%%%%滤波估计精度%%%%%%%%%%%%
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


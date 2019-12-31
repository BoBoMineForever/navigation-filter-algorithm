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


function [Xu,PKU,UKFXerr]=ukf_gps(t,T_D,Fb,Wibb,attiN,veloN,posiN,posiG,veloG,Xu,PKU,UKFXerr,kflag);
 
  Re=6378137.0;                                      %地球半径（米） 
  f=1/298.257;                                        %地球的椭圆率
  Wie=7.292115147e-5;                          %地球自转角速度
  g=9.7803698;                                      %重力加速度
  %%%%%%%%%%%%%%%%%%
  Alpha=0.001;												%定义UKF比例修正参数
  K=0;
  Beta=2;
  L=19+9+6;
  Lambada=(Alpha^2)*(L+K)-L;    
  %%%%%%%%%%%%%%%%%%
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
  VG=[20;20;50];  % 需要与GPS仿真精度相同

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
  Xua=[Xu;zeros(9,1);zeros(6,1)];               %状态初始条件扩维(3-40)
  PKUA=[PKU,zeros(19,9),zeros(19,6);             %状态协方差扩维
      zeros(9,19),Q,zeros(9,6);
      zeros(6,19),zeros(6,9),RG];
  Xuaa=zeros(34,L);
  for i=1:L
  Xuaa(:,i)=Xua;    
  end
  Sigmaa=[Xua,Xuaa+chol((L+Lambada)*PKUA)',Xuaa-chol((L+Lambada)*PKUA)']; %sigma点扩维采样(3-41)
  Wm=Lambada/(L+Lambada);                                                 %计算权值(3-21)
  Wc=Lambada/(L+Lambada)+(1-Alpha^2+Beta);
  for i=1:2*L
  Wm=[Wm;1/(2*(L+Lambada))];
  Wc=[Wc;1/(2*(L+Lambada))];
  end
  %sigma点状态，过程噪声，观测噪声分离
  Sigmax=Sigmaa(1:19,:);
  Sigmav=Sigmaa(20:28,:);
  Sigman=Sigmaa(29:34,:);
  %开始递推计算
  for i=1:(2*L+1)
     [Xsigma(:,i)]=ukf_tl(T_D,Fb,Wibb,attiN,veloN,posiN,Sigmax(:,i),Sigmav(:,i));      %(3-30)
     %Xsigma=FL*Sigmax+GL*Sigmav;               
  end
  Xu=Wm(1,1)*Xsigma(:,1);                       %(3-32)
  for i=1:2*L
  Xu=Xu+Wm(i+1,1)*Xsigma(:,i+1);
  end                                               
  PKU=Wc(1,1)*[(Xsigma(:,1)-Xu)*(Xsigma(:,1)-Xu)'];       %(3-33)
  for i=1:2*L
  PKU=PKU+Wc(i+1,1)*[(Xsigma(:,i+1)-Xu)*(Xsigma(:,i+1)-Xu)'];
  end      
  Ysigma=HG*Xsigma+Sigman;                             %(3-31)
  Ycc=Wm(1,1)*Ysigma(:,1);                               %(3-34)
  for i=1:2*L
  Ycc=Ycc+Wm(i+1,1)*Ysigma(:,i+1);
  end  
  Pyy=Wc(1,1)*[(Ysigma(:,1)-Ycc)*(Ysigma(:,1)-Ycc)'];     %(3-35)
  for i=1:2*L
  Pyy=Pyy+Wc(i+1,1)*[(Ysigma(:,i+1)-Ycc)*(Ysigma(:,i+1)-Ycc)'];
  end      
  Pxy=Wc(1,1)*[(Xsigma(:,1)-Xu)*(Ysigma(:,1)-Ycc)'];      %(3-36)
  for i=1:2*L
  Pxy=Pxy+Wc(i+1,1)*[(Xsigma(:,i+1)-Xu)*(Ysigma(:,i+1)-Ycc)'];
  end   
  KK=Pxy*inv(Pyy);                                       %(3-37)
  Xu=Xu+KK*(Yc-Ycc);                                     %(3-38)
  PKU=PKU-KK*Pyy*KK';                                      %(3-39)
  end
  %%%%%%%%%%%%%%%%%滤波估计精度%%%%%%%%%%%%
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


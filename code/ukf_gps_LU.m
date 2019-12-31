function [Xc,PK,Xerr]=ukf_gps_LU(t,T_D,Fb,attiN,veloN,posiN,posiG,veloG,Wibb,Xc,PK,Xerr,kflag);
 
  Re=6378137.0;                                      %地球半径（米） 
  f=1/298.257;                                        %地球的椭圆率
  Wie=7.292115147e-5;                          %地球自转角速度
  g=9.7803698;                                      %重力加速度

  long=posiN(1,1)*pi/180.0;lati=posiN(2,1)*pi/180.0;heig=posiN(3,1);
  Ve=veloN(1,1);Vn=veloN(2,1);Vu=veloN(3,1);
    %飞行器位置

  %地球曲率半径求解
  Rm=Re*(1-2*f+3*f*sin(lati)*sin(lati));
  Rn=Re*(1+f*sin(lati)*sin(lati));
      
  roll=attiN(1,1)*pi/180.0;pitch=attiN(2,1)*pi/180.0;head=attiN(3,1)*pi/180.0;
  Cbn=[cos(roll)*cos(head)+sin(roll)*sin(pitch)*sin(head), -cos(roll)*sin(head)+sin(roll)*sin(pitch)*cos(head), -sin(roll)*cos(pitch);
       cos(pitch)*sin(head),                               cos(pitch)*cos(head),                                sin(pitch);
       sin(roll)*cos(head)-cos(roll)*sin(pitch)*sin(head), -sin(roll)*sin(head)-cos(roll)*sin(pitch)*cos(head), cos(roll)*cos(pitch)];

  Q=[cos(head/2)*cos(pitch/2)*cos(roll/2)+sin(head/2)*sin(pitch/2)*sin(roll/2);
     cos(head/2)*sin(pitch/2)*cos(roll/2)+sin(head/2)*cos(pitch/2)*sin(roll/2);
     cos(head/2)*cos(pitch/2)*sin(roll/2)-sin(head/2)*sin(pitch/2)*cos(roll/2);
     -1.0*sin(head/2)*cos(pitch/2)*cos(roll/2)+cos(head/2)*sin(pitch/2)*sin(roll/2)];
%   Cbn=[Q(2,1)^2+Q(1,1)^2-Q(4,1)^2-Q(3,1)^2, 2*(Q(2,1)*Q(3,1)+Q(1,1)*Q(4,1)),      2*(Q(2,1)*Q(4,1)-Q(1,1)*Q(3,1));
%        2*(Q(2,1)*Q(3,1)-Q(1,1)*Q(4,1)),     Q(3,1)^2-Q(4,1)^2+Q(1,1)^2-Q(2,1)^2,  2*(Q(3,1)*Q(4,1)+Q(1,1)*Q(2,1));
%        2*(Q(2,1)*Q(4,1)+Q(1,1)*Q(3,1)),     2*(Q(3,1)*Q(4,1)-Q(1,1)*Q(2,1)),      Q(4,1)^2-Q(3,1)^2-Q(2,1)^2+Q(1,1)^2];
  Cnb=Cbn'  ;    
        Y_Q=[-Q(2,1),-Q(3,1),-Q(4,1);
              Q(1,1), Q(4,1), -Q(3,1);
             -Q(4,1),Q(1,1), Q(2,1);
              Q(3,1), -Q(2,1), Q(1,1)];
        U_Q=[-Q(2,1),-Q(3,1),-Q(4,1);
              Q(1,1), -Q(4,1), Q(3,1);
              Q(4,1),Q(1,1), -Q(2,1);
             -Q(3,1), Q(2,1), Q(1,1)];
       Wenn=[-veloN(2,1)/(Rm+heig);veloN(1,1)/(Rn+heig);veloN(1,1)*tan(lati)/(Rn+heig)];%rad/s
       Wien=[0;Wie*cos(lati);Wie*sin(lati)];%rad/s
       Winn=Wien+Wenn;
       Wibb=Wibb*pi/180.0;
         %%<Wibb>=Wibb_Q
       Wibb_Q=[0,  -Wibb(1,1),   -Wibb(2,1),  -Wibb(3,1);
               Wibb(1,1),  0,     Wibb(3,1),  -Wibb(2,1);
               Wibb(2,1), -Wibb(3,1),    0,    Wibb(1,1);
               Wibb(3,1),  Wibb(2,1),   -Wibb(1,1),   0];
        %%[Winn]=Winn_Q
       Winn_Q=[0,  -Winn(1,1),   -Winn(2,1),  -Winn(3,1);
               Winn(1,1),  0,    -Winn(3,1),  Winn(2,1);
               Winn(2,1),  Winn(3,1),    0,   -Winn(1,1);
               Winn(3,1), -Winn(2,1),   Winn(1,1),  0];
           
       Q_Q=0.5*(Wibb_Q-Winn_Q);
       Y1=[0,-1.0/(Rm+heig),0;
           1.0/(Rn+heig),0,0;
           tan(lati)/(Rn+heig),0,0];
       Q_V=-0.5*Y_Q*Y1;
       Y2=[0,0,Vn/(Rm+heig)^2;
           -Wie*sin(lati),0,-Ve/(Rn+heig)^2;
           Wie*cos(lati)+Ve/(Rn+heig)*sec(lati)*sec(lati),0,-Ve*tan(lati)/(Rn+heig)^2];
       Q_R=-0.5*Y_Q*Y2;
%              
%        Q_Q=0.5*(Wibb_Q-Winn_Q);
%        Y1=[0,-1.0/(Rm+heig),0;
%            1.0/(Rn+heig),0,0;
%            tan(lati)/(Rn+heig),0,0];
%        Q_V=0.5*Y_Q*Y1;
%        Y2=[0,0,Vn/(Rm+heig)^2;
%            -Wie*sin(lati),0,-Ve/(Rn+heig)^2;
%            Wie*cos(lati)*Ve/(Rn+heig)*sec(lati)*sec(lati),0,-Ve*tan(lati)/(Rn+heig)^2];
%        Q_R=0.5*Y_Q*Y2;
%              

        A=Cbn'*Fb;
        A_1=[  0           -A(3,1)    A(2,1);  
               A(3,1)       0        -A(1,1);
               -A(2,1)   A(1,1)        0];
        V_Q=-2*A_1*Y_Q'+2*A*Q';
        V_V=[Vn*tan(lati)/(Rn+heig)-Vu/(Rn+heig),       2*Wie*sin(lati)+Ve*tan(lati)/(Rn+heig),   -(2*Wie*cos(lati)+Ve/(Rn+heig));
            -(2*Wie*sin(lati)+Ve*tan(lati)/(Rn+heig)),     -Vu/(Rm+heig),                      -Vn/(Rm+heig);             
           (2*Wie*cos(lati)+Ve/(Rn+heig)),                  2*Vn/(Rm+heig),                                0   ];

%        VX=[0,-Vu,Vn;
%            Vu,0,-Ve;
%            -Vn,Ve,0];
%        V_R=VX*[0,0,Vn/(Rm+heig)^2;0,0,Ve/(Rn+heig)^2;Ve/(Rn+heig)*sec(lati)*sec(lati),0,-Ve*tan(lati)/(Rn+heig)^2];
         V_R=[2*Wie*sin(lati)*Vu+(Ve*sec(lati)*sec(lati)/(Rn+heig)+2*Wie*cos(lati))*Vn,0,Vn*Ve/(Rn+heig)^2-Vn*Ve*tan(lati)/(Rn+heig)^2;
             -Ve*(Ve*sec(lati)*sec(lati)/(Rn+heig)+2*Wie*cos(lati)),0,Vu*Vn/(Rm+heig)^2-Ve*Ve*tan(lati)/(Rn+heig)^2;
             -Ve*2*Wie*sin(lati),0,-Vn*Vn/(Rm+heig)^2-Ve*Ve/(Rn+heig)^2];
  
      
        R_Q=zeros(3,4);
        R_V=[0,1.0/(Rm+heig),0;
            sec(lati)/(Rn+heig),0,0;
            0,0,1];
        R_R=[0,0,-Vn/(Rm+heig)^2;
            Ve/(Rn+heig)*sec(lati)*tan(lati),0,-Ve*sec(lati)/(Rn+heig)^2;
            0,0,0];
        
%             FN=[Q_Q,Q_V,Q_R;
%                 V_Q,V_V,V_R;
%                 R_Q,R_V,R_R];
FN=[V_V,V_Q,V_R;
    Q_V,Q_Q,Q_R;
    R_V,R_Q,R_R];
           
%    FS=[0.5*U_Q*Cbn',0.5*U_Q*Cbn',zeros(4,3);
%         zeros(3,6),Cbn';
%         zeros(3,9)];
%    FS=[ zeros(3,6),Cbn';
%        0.5*U_Q*Cbn',0.5*U_Q*Cbn',zeros(4,3);
%         zeros(3,9)];
  %去掉了Cbn阵
    FS=[ zeros(3,6),Cbn';
         0.5*U_Q*Cbn',0.5*U_Q*Cbn',zeros(4,3);
         zeros(3,9)];
   Tgx=3600.0; Tgy=3600.0;  Tgz=3600.0; 
   Tax=1800.0; Tay=1800.0;  Taz=1800.0; 
    %陀螺和加速度计的一阶马尔可夫相关时间（与IMU仿真同）

   FM=diag([0,0,0,-1.0/Tgx,-1.0/Tgy,-1.0/Tgz,-1.0/Tax,-1.0/Tay,-1.0/Taz]);

   FI=[FN,        FS;
      zeros(9,10),FM];      
%   GI=[0.5*U_Q*Cbn',      zeros(4,6);
%       zeros(9,9);
%       zeros(3,3),eye(3),zeros(3,3);
%       zeros(3,6),       eye(3)];
%  %P_275 6.24
GI=[zeros(3,9);
      0.5*U_Q*Cbn',      zeros(4,6);
      zeros(6,9);
      zeros(3,3),eye(3),zeros(3,3);
      zeros(3,6),       eye(3)];
    %量测矩阵
%     
  I=eye(size(FI));
%   FL=I+FI*T_D+FI*FI/2.0*T_D*T_D;
%   GL=(I+FI/2.0*T_D+FI*FI/6.0*T_D*T_D)*GI*T_D;
%     离散化综合模型
    
  W=[0.1*pi/(3600*180),0.1*pi/(3600*180),0.1*pi/(3600*180), ...
     sqrt(2*T_D/Tgx)*0.1*pi/(3600*180),sqrt(2*T_D/Tgy)*0.1*pi/(3600*180),sqrt(2*T_D/Tgz)*0.1*pi/(3600*180), ...
     sqrt(2*T_D/Tax)*(1e-4)*g,sqrt(2*T_D/Tay)*(1e-4)*g,sqrt(2*T_D/Taz)*(1e-4)*g]';  
      %系统噪声阵（与IMU仿真同）单位：rad/s,rad/s,rad/s,rad/s,rad/s,rad/s,m/s/s,m/s/s,m/s/s
  V=diag((W.^2)');
  
        %GPS/INS位置量测矩阵
      HG=[zeros(3,7),diag([Rm,Rn*cos(lati),1]),zeros(3,9)];
      VG=[20;20;50];  % 需要与GPS仿真精度相同

      %GPS/INS位置＋速度量测方程
%       HG=[HG;zeros(3,4),diag([1,1,1]),zeros(3,12)];
      HG=[HG;diag([1,1,1]),zeros(3,4),zeros(3,12)];
      VG=[VG;1.5;1.5;1.5 ]; % 需要与GPS仿真精度相同
      RG=diag((VG.^2)'); 
  % UKF filter
        N=19+9+6;%dimention expand
        a=1e-3;
        b=N*a^2-N;
        W0m=b/(b+N);
        W0c=b/(b+N)+1-a^2-b;
        Wi=1/(2*(N+b));
        
        if(kflag==1)
            Xu=[Xc;zeros(9,1);zeros(6,1)];                 %状态扩维
            PKU=[PK,zeros(19,9),zeros(19,6);             %状态协方差扩维
                 zeros(9,19),V,zeros(9,6);
                 zeros(6,19),zeros(6,9),RG];
            P_chol=chol(PKU)';
            for i=1:N
                Xu_1(:,i)=Xu+sqrt(b+N)*P_chol(:,i);
                Xu_2(:,i)=Xu-sqrt(b+N)*P_chol(:,i);
            end
            X_sigma=[Xu,Xu_1,Xu_2];
            %%%速度误差非线性项  
            for i=1:2*N+1
                Q_err=[X_sigma(4,i),X_sigma(5,i),X_sigma(6,i),X_sigma(7,i)]';
                Y_Q_err=[-Q_err(2,1),-Q_err(3,1),-Q_err(4,1);
                          Q_err(1,1), Q_err(4,1),-Q_err(3,1);
                         -Q_err(4,1), Q_err(1,1), Q_err(2,1);
                          Q_err(3,1),-Q_err(2,1), Q_err(1,1)];
                U_Q_err=[-Q_err(2,1), -Q_err(3,1),-Q_err(4,1);
                          Q_err(1,1), -Q_err(4,1), Q_err(3,1);
                          Q_err(4,1),  Q_err(1,1),-Q_err(2,1);
                         -Q_err(3,1),  Q_err(2,1), Q_err(1,1)];
                Q_V_err=-Y_Q_err'*U_Q_err*Fb;
                X(1:19,i)=X_sigma(1:19,i)+FI*X_sigma(1:19,i)*T_D+GI*X_sigma(20:28,i)*T_D;
                %%加上非线性项
                X(1:3,i)= X(1:3,i)+Q_V_err*T_D;
            end
        
            X_pre=W0m*X(:,1);
            for i=2:2*N+1
                X_pre=X_pre+Wi*X(:,i);
            end
        
            PK_pre=W0c*(X(:,1)-X_pre)*(X(:,1)-X_pre)';
            for i=2:2*N+1
                PK_pre=PK_pre+Wi*(X(:,i)-X_pre)*(X(:,i)-X_pre)';
            end
%             PK_pre=PK_pre+GL*V*GL';
        

      YK=HG*X_pre;
      PYY=HG*PK_pre*HG'+RG;
      PXY=PK_pre*HG';
      K=PXY*inv(PYY);
      Yc=[(posiN(2,1)-posiG(2,1))*pi/180.0*(Rm+heig);
           (posiN(1,1)-posiG(1,1))*pi/180.0*(Rn+heig)*cos(lati);
            posiN(3,1)-posiG(3,1)]; %量测次序为纬度、经度、高度
      Yc=[Yc;veloN-veloG]; %单位（米，米/秒） 
      Xc=X_pre+K*(Yc-YK);
      PK=PK_pre-K*PYY*K';
        end
        



  %%%%%%%%%%%%%%%%%滤波估计精度%%%%%%%%%%%%
  %%速度误差
  Xerr(1,1)=sqrt(PK(1,1));  
  Xerr(1,2)=sqrt(PK(2,2));   
  Xerr(1,3)=sqrt(PK(3,3));  
  %%四元数误差
  Xerr(1,4)=sqrt(PK(4,4));  
  Xerr(1,5)=sqrt(PK(5,5));                    
  Xerr(1,6)=sqrt(PK(6,6));                  
  Xerr(1,7)=sqrt(PK(7,7)); 
  %%位置误差
  Xerr(1,8)=sqrt(PK(8,8))*(Rm+heig);          %m
  Xerr(1,9)=sqrt(PK(9,9))*(Rn+heig)*cos(lati);%m
  Xerr(1,10)=sqrt(PK(10,10));                    %m
       %INS的9个导航量误差
         
  Xerr(1,11)=sqrt(PK(11,11))*180.0*3600.0/pi;   %deg/h
  Xerr(1,12)=sqrt(PK(12,12))*180.0*3600.0/pi;   %deg/h
  Xerr(1,13)=sqrt(PK(13,13))*180.0*3600.0/pi;   %deg/h
  Xerr(1,14)=sqrt(PK(14,14))*180.0*3600.0/pi;   %deg/h
  Xerr(1,15)=sqrt(PK(15,15))*180.0*3600.0/pi;   %deg/h
  Xerr(1,16)=sqrt(PK(16,16))*180.0*3600.0/pi;   %deg/h
  Xerr(1,17)=sqrt(PK(17,17))/g;                 %g
  Xerr(1,18)=sqrt(PK(18,18))/g;                 %g
  Xerr(1,19)=sqrt(PK(19,19))/g;                 %g
      %IMU的9个误差量

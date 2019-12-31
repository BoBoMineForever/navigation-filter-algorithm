%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%                   �������˲�����(����GPS����λ���ٶ����)
%
%  �������:t-����ʱ�䣬T_D-��ɢ���ڣ�
%           Fb-���ٶȼ������
%           attiN-�����������̬�ǶȺ�������������򣨶ȣ��ȣ��ȣ���
%           veloN-��������Ի���ϵ���˶��ٶȶ��򡢱���������/�룩��
%           posiG-GPS����ķ�����λ�ã����ȣ��ȣ���γ�ȣ��ȣ����߶ȣ��ף����� 
%           veloG-GPS����ķ������ٶȣ�������/�룩��������/�룩��������/�룩��
%           Xc-�ۺ�ģ��״̬����PK-Э������
%           Xerr-״̬�����������ֵ����¼����ʱ�̵ģ�
%           kflag-GPS��Ϣ��Ч��־λ��1����Ч��
%  ���������Xc-�ۺ�ģ��״̬����PK-Э������Xerr-״̬�����������ֵ����¼ĳһʱ�̵ģ�         
%
%                           ������ƣ�����  ���ڣ�2003/9/29
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [Xd,PKD,DD1Xerr]=dd1_gps(t,T_D,Fb,Wibb,attiN,veloN,posiN,posiG,veloG,Xd,PKD,DD1Xerr,kflag)
 
  Re=6378137.0;                                      %����뾶���ף� 
  f=1/298.257;                                        %�������Բ��
  Wie=7.292115147e-5;                          %������ת���ٶ�
  g=9.7803698;                                      %�������ٶ�

  long=posiN(1,1)*pi/180.0;lati=posiN(2,1)*pi/180.0;heig=posiN(3,1);
    %������λ��

  %�������ʰ뾶���
  Rm=Re*(1-2*f+3*f*sin(lati)*sin(lati));
  Rn=Re*(1+f*sin(lati)*sin(lati));
      
  roll=attiN(1,1)*pi/180.0;pitch=attiN(2,1)*pi/180.0;head=attiN(3,1)*pi/180.0;
    
  Cbn=[cos(roll)*cos(head)+sin(roll)*sin(pitch)*sin(head), -cos(roll)*sin(head)+sin(roll)*sin(pitch)*cos(head), -sin(roll)*cos(pitch);
       cos(pitch)*sin(head),                               cos(pitch)*cos(head),                                sin(pitch);
       sin(roll)*cos(head)-cos(roll)*sin(pitch)*sin(head), -sin(roll)*sin(head)-cos(roll)*sin(pitch)*cos(head), cos(roll)*cos(pitch)];
  %����ϵN-->B
  
  Fn=Cbn'*Fb;

  Ve=veloN(1,1);Vn=veloN(2,1);Vu=veloN(3,1);
 
  Tgx=3600.0; Tgy=3600.0;  Tgz=3600.0; 
  Tax=1800.0; Tay=1800.0;  Taz=1800.0; 
    %���ݺͼ��ٶȼƵ�һ������ɷ����ʱ�䣨��IMU����ͬ��
    
  W=[0.1*pi/(3600*180),0.1*pi/(3600*180),0.1*pi/(3600*180), ...
     sqrt(2*T_D/Tgx)*0.1*pi/(3600*180),sqrt(2*T_D/Tgy)*0.1*pi/(3600*180),sqrt(2*T_D/Tgz)*0.1*pi/(3600*180), ...
     sqrt(2*T_D/Tax)*(1e-4)*g,sqrt(2*T_D/Tay)*(1e-4)*g,sqrt(2*T_D/Taz)*(1e-4)*g]';   
      %ϵͳ��������IMU����ͬ����λ��rad/s,rad/s,rad/s,rad/s,rad/s,rad/s,m/s/s,m/s/s,m/s/s

  %GPS/INSλ���������
  HG=[zeros(3,7),diag([Rm,Rn*cos(lati),1]),zeros(3,9)];
  VG=[20;20;50];  % ��Ҫ��GPS���澫����ͬ

  %GPS/INSλ�ã��ٶ����ⷽ��
  HG=[HG;diag([1,1,1]),zeros(3,4),zeros(3,12)];
  VG=[VG;0.2;0.2;0.2]; % ��Ҫ��GPS���澫����ͬ
  
  Q=diag((W.^2)');
  RG=diag((VG.^2)');
  %����������    
  Yc=[(posiN(2,1)-posiG(2,1))*pi/180.0*(Rm+heig);
      (posiN(1,1)-posiG(1,1))*pi/180.0*(Rn+heig)*cos(lati);
       posiN(3,1)-posiG(3,1)]; %�������Ϊγ�ȡ����ȡ��߶�
  Yc=[Yc;veloN-veloG]; %��λ���ף���/�룩
  
  %ϵͳ״̬���������
    
  if( kflag == 1 )
    %Stirling��ֵ�˲�DD1����ʽ�ο�������΢С������̬ȷ���ķ����Թ����㷨����������������̬�Ķ��ײ�ֵ�������˲����ơ�
    Hdd=sqrt(3);                       %�����е�����ԭʼ����Ϊsqrt(3)
    Sv=chol(Q);
    Sw=chol(RG);
    Sx=chol(PKD)';
  
    for i=1:19    
        Sxx(:,i)=1/(2*Hdd)*(dd1_tl(T_D,Fb,Wibb,attiN,veloN,posiN,Xd+Hdd*Sx(:,i),zeros(9,1))-dd1_tl(T_D,Fb,Wibb,attiN,veloN,posiN,Xd-Hdd*Sx(:,i),zeros(9,1)));
    end
    for i=1:9
        Sxv(:,i)=1/(2*Hdd)*(dd1_tl(T_D,Fb,Wibb,attiN,veloN,posiN,Xd,Hdd*Sv(:,i))-dd1_tl(T_D,Fb,Wibb,attiN,veloN,posiN,Xd,-Hdd*Sv(:,i)));
        %Sxv(:,i)=1/(2*Hdd)*dd1_tl(T_D,Fb,Wibb,attiN,veloN,posiN,zeros(19,1),2*Hdd*Sv(:,i));
    end
    [Xd]=dd1_tl(T_D,Fb,Wibb,attiN,veloN,posiN,Xd,zeros(9,1));
    Sx=[Sxx,Sxv];
    [QQ,RR]=qr(Sx');
    Sx=(RR(1:19,:))';
    PKD=Sx*Sx';
    for i=1:19
        %Syx(:,i)=1/(2*Hdd)*(HG*(Xd+Hdd*Sx(:,i))-HG*(Xd-Hdd*Sx(:,i)));
        Syx(:,i)=1/(2*Hdd)*(2*HG*Hdd*Sx(:,i));
    end
    for i=1:6
        %Syw(:,i)=1/(2*Hdd)*((HG*Xd+Hdd*Sw(:,i))-(HG*Xd-Hdd*Sw(:,i)));
        Syw(:,i)=1/(2*Hdd)*(2*Hdd*Sw(:,i));
    end
    Ycc=HG*Xd;
    Sy=[Syx,Syw];
    [QQ,RR]=qr(Sy');
    Sy=(RR(1:6,:))';   
    Py=Sy*Sy';
    Pxy=Sx*Syx';
    KK=Pxy*inv(Py);
    Xd=Xd+KK*(Yc-Ycc);
    Sx=[Sx-KK*Syx,KK*Syw];
    [QQ,RR]=qr(Sx');
    Sx=(RR(1:19,:))'; 
    PKD=Sx*Sx';
  end

  %%%%%%%%%%%%%%%%%�˲����ƾ���%%%%%%%%%%%%
    DD1Xerr(1,1)=sqrt(PKD(1,1));                    %m/s
    DD1Xerr(1,2)=sqrt(PKD(2,2));                    %m/s
    DD1Xerr(1,3)=sqrt(PKD(3,3));                    %m/s
    DD1Xerr(1,4)=sqrt(PKD(4,4));    %��Ԫ��
    DD1Xerr(1,5)=sqrt(PKD(5,5));    %��Ԫ��
    DD1Xerr(1,6)=sqrt(PKD(6,6));    %��Ԫ��
    DD1Xerr(1,7)=sqrt(PKD(7,7));    %��Ԫ��
    DD1Xerr(1,8)=sqrt(PKD(8,8))*(Rm+heig);          %m
    DD1Xerr(1,9)=sqrt(PKD(9,9))*(Rn+heig)*cos(lati);%m
    DD1Xerr(1,10)=sqrt(PKD(10,10));                 %m
         %INS��10�����������
         
    DD1Xerr(1,11)=sqrt(PKD(11,11))*180.0*3600.0/pi;   %deg/h
    DD1Xerr(1,12)=sqrt(PKD(12,12))*180.0*3600.0/pi;   %deg/h
    DD1Xerr(1,13)=sqrt(PKD(13,13))*180.0*3600.0/pi;   %deg/h
    DD1Xerr(1,14)=sqrt(PKD(14,14))*180.0*3600.0/pi;   %deg/h
    DD1Xerr(1,15)=sqrt(PKD(15,15))*180.0*3600.0/pi;   %deg/h
    DD1Xerr(1,16)=sqrt(PKD(16,16))*180.0*3600.0/pi;   %deg/h
    DD1Xerr(1,17)=sqrt(PKD(17,17))/g;                 %g
    DD1Xerr(1,18)=sqrt(PKD(18,18))/g;                 %g
    DD1Xerr(1,19)=sqrt(PKD(19,19))/g;                 %g
        %IMU��9�������


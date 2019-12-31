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


function [Xu,PKU,UKFXerr]=ukf_gps(t,T_D,Fb,Wibb,attiN,veloN,posiN,posiG,veloG,Xu,PKU,UKFXerr,kflag);
 
  Re=6378137.0;                                      %����뾶���ף� 
  f=1/298.257;                                        %�������Բ��
  Wie=7.292115147e-5;                          %������ת���ٶ�
  g=9.7803698;                                      %�������ٶ�
  %%%%%%%%%%%%%%%%%%
  Alpha=0.001;												%����UKF������������
  K=0;
  Beta=2;
  L=19+9+6;
  Lambada=(Alpha^2)*(L+K)-L;    
  %%%%%%%%%%%%%%%%%%
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
  a=0.1;   
  W=[a*pi/(3600*180),a*pi/(3600*180),a*pi/(3600*180), ...
     sqrt(2*T_D/Tgx)*a*pi/(3600*180),sqrt(2*T_D/Tgy)*a*pi/(3600*180),sqrt(2*T_D/Tgz)*a*pi/(3600*180), ...
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
  Xua=[Xu;zeros(9,1);zeros(6,1)];               %״̬��ʼ������ά(3-40)
  PKUA=[PKU,zeros(19,9),zeros(19,6);             %״̬Э������ά
      zeros(9,19),Q,zeros(9,6);
      zeros(6,19),zeros(6,9),RG];
  Xuaa=zeros(34,L);
  for i=1:L
  Xuaa(:,i)=Xua;    
  end
  Sigmaa=[Xua,Xuaa+chol((L+Lambada)*PKUA)',Xuaa-chol((L+Lambada)*PKUA)']; %sigma����ά����(3-41)
  Wm=Lambada/(L+Lambada);                                                 %����Ȩֵ(3-21)
  Wc=Lambada/(L+Lambada)+(1-Alpha^2+Beta);
  for i=1:2*L
  Wm=[Wm;1/(2*(L+Lambada))];
  Wc=[Wc;1/(2*(L+Lambada))];
  end
  %sigma��״̬�������������۲���������
  Sigmax=Sigmaa(1:19,:);
  Sigmav=Sigmaa(20:28,:);
  Sigman=Sigmaa(29:34,:);
  %��ʼ���Ƽ���
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
  %%%%%%%%%%%%%%%%%�˲����ƾ���%%%%%%%%%%%%
    UKFXerr(1,1)=sqrt(PKU(1,1));                    %m/s
    UKFXerr(1,2)=sqrt(PKU(2,2));                    %m/s
    UKFXerr(1,3)=sqrt(PKU(3,3));                    %m/s
    UKFXerr(1,4)=sqrt(PKU(4,4));    %��Ԫ��
    UKFXerr(1,5)=sqrt(PKU(5,5));    %��Ԫ��
    UKFXerr(1,6)=sqrt(PKU(6,6));    %��Ԫ��
    UKFXerr(1,7)=sqrt(PKU(7,7));    %��Ԫ��
    UKFXerr(1,8)=sqrt(PKU(8,8))*(Rm+heig);          %m
    UKFXerr(1,9)=sqrt(PKU(9,9))*(Rn+heig)*cos(lati);%m
    UKFXerr(1,10)=sqrt(PKU(10,10));                 %m
         %INS��10�����������
         
    UKFXerr(1,11)=sqrt(PKU(11,11))*180.0*3600.0/pi;   %deg/h
    UKFXerr(1,12)=sqrt(PKU(12,12))*180.0*3600.0/pi;   %deg/h
    UKFXerr(1,13)=sqrt(PKU(13,13))*180.0*3600.0/pi;   %deg/h
    UKFXerr(1,14)=sqrt(PKU(14,14))*180.0*3600.0/pi;   %deg/h
    UKFXerr(1,15)=sqrt(PKU(15,15))*180.0*3600.0/pi;   %deg/h
    UKFXerr(1,16)=sqrt(PKU(16,16))*180.0*3600.0/pi;   %deg/h
    UKFXerr(1,17)=sqrt(PKU(17,17))/g;                 %g
    UKFXerr(1,18)=sqrt(PKU(18,18))/g;                 %g
    UKFXerr(1,19)=sqrt(PKU(19,19))/g;                 %g
        %IMU��9�������


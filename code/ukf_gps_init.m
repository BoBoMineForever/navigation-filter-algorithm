%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%                   �������˲���ʼ������(����GPS����λ���ٶ����)
%
%  �������:
%           posiN-��ʼ�ķ�����λ�ã����ȣ��ȣ���γ�ȣ��ȣ����߶ȣ��ף����� 
%           Xc-�ۺ�ģ��״̬����PK-Э������
%           Xerr-״̬�����������ֵ����¼����ʱ�̵ģ�
%  ���������Xc-�ۺ�ģ��״̬����PK-Э������Xerr-״̬�����������ֵ����¼ĳһʱ�̵ģ�         
%
%                           ������ƣ�����  ���ڣ�2003/10/04
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [Xu,PKU,UKFXerr]=ukf_gps_init(posiN,atti,Xu,PKU,UKFXerr);
 
  Re=6378137.0;                                      %����뾶���ף� 
  f=1/298.257;                                        %�������Բ��
  Wie=7.292115147e-5;                          %������ת���ٶ�
  g=9.7803698;                                      %�������ٶ�

  long=posiN(1,1)*pi/180.0;lati=posiN(2,1)*pi/180.0;heig=posiN(3,1);
    %������λ��
  roll=atti(1,1)*pi/180.0;pitch=atti(2,1)*pi/180.0;head=atti(3,1)*pi/180.0;
  %�������ʰ뾶���
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
      %��������ֵ����IMU����ͬ����λ��rad,rad,rad,m/s,m/s,m/s,rad,rad,m,rad/s,rad/s,rad/s,rad/s,rad/s,rad/s,m/s/s,m/s/s,m/s/s
          
    PKU=diag((UKFXerr0.^2));
    Xu=zeros(19,1);
    
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
        


      

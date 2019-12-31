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


function [Xd,PKD,DD2Xerr]=dd2_gps_init(posiN,atti,Xd,PKD,DD2Xerr);
 
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
    DD2Xerr0=[0.5,0.5,0.5,Qini(1,1),Qini(2,1),Qini(3,1),Qini(4,1),50.0/(Rm+heig),50.0/(Rn+heig)/cos(lati),50.0, ...
         0.1*pi/(3600.0*180.0),0.1*pi/(3600.0*180.0),0.1*pi/(3600.0*180.0),0.1*pi/(3600.0*180.0),0.1*pi/(3600.0*180.0),0.1*pi/(3600.0*180.0),(1e-4)*g,(1e-4)*g,(1e-4)*g];
      %��������ֵ����IMU����ͬ����λ��rad,rad,rad,m/s,m/s,m/s,rad,rad,m,rad/s,rad/s,rad/s,rad/s,rad/s,rad/s,m/s/s,m/s/s,m/s/s
          
    PKD=diag((DD2Xerr0.^2));
    Xd=zeros(19,1);
    
    DD2Xerr(1,1)=sqrt(PKD(1,1));                    %m/s
    DD2Xerr(1,2)=sqrt(PKD(2,2));                    %m/s
    DD2Xerr(1,3)=sqrt(PKD(3,3));                    %m/s
    DD2Xerr(1,4)=sqrt(PKD(4,4));    %��Ԫ��
    DD2Xerr(1,5)=sqrt(PKD(5,5));    %��Ԫ��
    DD2Xerr(1,6)=sqrt(PKD(6,6));    %��Ԫ��
    DD2Xerr(1,7)=sqrt(PKD(7,7));    %��Ԫ��
    DD2Xerr(1,8)=sqrt(PKD(8,8))*(Rm+heig);          %m
    DD2Xerr(1,9)=sqrt(PKD(9,9))*(Rn+heig)*cos(lati);%m
    DD2Xerr(1,10)=sqrt(PKD(10,10));                 %m
         %INS��10�����������
         
    DD2Xerr(1,11)=sqrt(PKD(11,11))*180.0*3600.0/pi;   %deg/h
    DD2Xerr(1,12)=sqrt(PKD(12,12))*180.0*3600.0/pi;   %deg/h
    DD2Xerr(1,13)=sqrt(PKD(13,13))*180.0*3600.0/pi;   %deg/h
    DD2Xerr(1,14)=sqrt(PKD(14,14))*180.0*3600.0/pi;   %deg/h
    DD2Xerr(1,15)=sqrt(PKD(15,15))*180.0*3600.0/pi;   %deg/h
    DD2Xerr(1,16)=sqrt(PKD(16,16))*180.0*3600.0/pi;   %deg/h
    DD2Xerr(1,17)=sqrt(PKD(17,17))/g;                 %g
    DD2Xerr(1,18)=sqrt(PKD(18,18))/g;                 %g
    DD2Xerr(1,19)=sqrt(PKD(19,19))/g;                 %g
        %IMU��9�������
        


      

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%                    IMU�������
%
%  �������:
%  T          ���沽��
%  atti       ��������������򣨵�λ���ȣ�
%  atti_rate  ������ʡ��������ʡ��������ʣ���λ����/�룩
%  veloB      �ɻ��˶��ٶȡ���X����Y��ͷ��Z���򣨵�λ����/�룩
%  acceB      �ɻ��˶����ٶȡ���X����Y��ͷ��Z���򣨵�λ����/��/�룩
%  posi       ������������ʼλ�þ��ȡ�γ�ȡ��߶ȣ���λ���ȡ��ȡ��ף�
%
%  ���������
%  Wibb       ����ϵ���������   ����λ����/�룩
%  Fb         ����ϵ���ٶȼ���� ����λ����/��/�룩
%      
%                           ������ƣ�����  ���ڣ�2002/8/9
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [Wibb,Fb,posi]=IMUout(T,posi,atti,atti_rate,veloB,acceB)

  %%%%%%%%%%%%%%%������йصĲ���%%%%%%%%%%%%%%%%%%%%%%
  Re=6378137.0;        %����뾶     ����λ���ף� 
  f=1/298.257;         %�������Բ��
  Wie=7.292115147e-5;  %������ת���ٶȣ���λ������/�룩
  g=9.7803698;         %�������ٶ�    ����λ����/��/�룩

  long=posi(1,1)*pi/180.0;lati=posi(2,1)*pi/180.0;heig=posi(3,1);
     %������λ��

  Rm=Re*(1-2*f+3*f*sin(lati)*sin(lati));
  Rn=Re*(1+f*sin(lati)*sin(lati));
     %�������ʰ뾶���

  roll=atti(1,1)*pi/180.0;pitch=atti(2,1)*pi/180.0;head=atti(3,1)*pi/180.0;
  droll=atti_rate(1,1)*pi/180.0;dpitch=atti_rate(2,1)*pi/180.0; dhead=atti_rate(3,1)*pi/180.0;
     %��̬�Ǻ���̬������

  Cbn=[cos(roll)*cos(head)+sin(roll)*sin(pitch)*sin(head), -cos(roll)*sin(head)+sin(roll)*sin(pitch)*cos(head), -sin(roll)*cos(pitch);
       cos(pitch)*sin(head),                               cos(pitch)*cos(head),                                sin(pitch);
       sin(roll)*cos(head)-cos(roll)*sin(pitch)*sin(head), -sin(roll)*sin(head)-cos(roll)*sin(pitch)*cos(head), cos(roll)*cos(pitch)];
    %����ϵN-->B

  Eluer_M=[cos(roll), 0, sin(roll)*cos(pitch);
           0,         1, -sin(pitch);
           sin(roll), 0, -cos(pitch)*cos(roll)];   
    %ŷ���Ǳ任����

  %%%%%%%%%%%���������%%%%%%%%%%%%
  Wnbb=Eluer_M*[dpitch;droll;dhead];

  veloN=Cbn'*veloB;
  Ve=veloN(1,1);Vn=veloN(2,1);Vu=veloN(3,1);

  Wenn=[-Vn/(Rm+heig); Ve/(Rn+heig);  Ve/(Rn+heig)*tan(lati)];
  Wien=[0;             Wie*cos(lati); Wie*sin(lati)];
  
  Wibb=Cbn*(Wien+Wenn)+Wnbb; %��λ������/��

  Wibb=Wibb*180.0/pi;        %��λ����/��

  %%%%%%%%%%%%%%���ٶȼ����%%%%%%%%%%%%%%
  acceN=Cbn'*(acceB+cross(Wnbb,veloB));
  Fn=acceN+cross(2*Wien+Wenn,veloN)-[0.0;0.0;-1.0*g];  %??������
  Fb=Cbn*Fn;                 %��λ����/��/��

  %%%%%%%%%%%%%%%λ�ü���%%%%%%%%%%%%%
  heig=heig+T*Vu;
  lati=lati+T*(Vn/(Rm+heig));
  long=long+T*(Ve/((Rn+heig)*cos(lati)));

  posi(1,1)=long*180.0/pi;
  posi(2,1)=lati*180.0/pi;
  posi(3,1)=heig;


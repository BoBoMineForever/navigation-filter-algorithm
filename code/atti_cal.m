%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%                    ��̬�������򣨷������ҷ���
%
%  T          ���沽��
%  Wibb       ����ϵ���������   ����λ����/�룩
%  attiN      ��������������򣨵�λ���ȣ�
%  veloN      �ɻ��˶��ٶȡ���X����Y����Z���򣨵�λ����/�룩
%  posiN      ���ȡ�γ�ȡ��߶ȣ���λ���ȡ��ȡ��ף�
%
%  ���������
%  attiN      ��������������򣨵�λ���ȣ�   
%      
%                           ������ƣ�����  ���ڣ�2002/8/9
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [attiN]=atti_cal(T,Wibb,attiN,veloN,posiN)

  Re=6378137.0;       %����뾶���ף� 
  f=1/298.257;        %�������Բ��
  Wie=7.292115147e-5; %������ת���ٶ�

  long=posiN(1,1)*pi/180.0;lati=posiN(2,1)*pi/180.0;heig=posiN(3,1);
    %������λ��

  Rm=Re*(1-2*f+3*f*sin(lati)*sin(lati));
  Rn=Re*(1+f*sin(lati)*sin(lati));
    %�������ʰ뾶���

  roll=attiN(1,1)*pi/180.0;pitch=attiN(2,1)*pi/180.0;head=attiN(3,1)*pi/180.0;

  Cbn=[cos(roll)*cos(head)+sin(roll)*sin(pitch)*sin(head), -cos(roll)*sin(head)+sin(roll)*sin(pitch)*cos(head), -sin(roll)*cos(pitch);
       cos(pitch)*sin(head),                               cos(pitch)*cos(head),                                sin(pitch);
       sin(roll)*cos(head)-cos(roll)*sin(pitch)*sin(head), -sin(roll)*sin(head)-cos(roll)*sin(pitch)*cos(head), cos(roll)*cos(pitch)];
    %����ϵN-->B

  Ve=veloN(1,1);Vn=veloN(2,1);Vu=veloN(3,1);

  Wenn=[-Vn/(Rm+heig); Ve/(Rn+heig);  Ve/(Rn+heig)*tan(lati)];
  Wien=[0;             Wie*cos(lati); Wie*sin(lati)];
    %��λ������/��

  Wnbb=Wibb*pi/180.0-Cbn*(Wien+Wenn);  %��λ������/��

  %%%%%%%%%%%%%%�������ҷ�����̬����%%%%%%%%%%%%%%%
  Cnb=Cbn';
    %����ϵB-->N

  WnbbX=T*[0,         -Wnbb(3,1), Wnbb(2,1);
         Wnbb(3,1),  0,         -Wnbb(1,1);
         -Wnbb(2,1), Wnbb(1,1), 0         ];

%  Cnb=Cnb*(eye(3)+WnbbX); %һ���㷨
  Cnb=Cnb*(eye(3)+WnbbX+0.5*WnbbX*WnbbX); %2���㷨

  %%%%%%%%%%��̬����������%%%%%%%%%
%  Cnb=1.5*Cnb-0.5*Cnb*Cnb'*Cnb;
%  Cnb=1.5*Cnb-0.5*Cnb*Cnb'*Cnb;

  Cbn=Cnb';

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %����̬(���������������
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  attiN(1,1)=atan(-Cbn(1,3)/Cbn(3,3));
  attiN(2,1)=atan(Cbn(2,3)/sqrt(Cbn(2,1)*Cbn(2,1)+Cbn(2,2)*Cbn(2,2)));
  attiN(3,1)=atan(Cbn(2,1)/Cbn(2,2));
    %��λ������

  %�����ж�
  attiN(1,1)=attiN(1,1)*180.0/pi;
  attiN(2,1)=attiN(2,1)*180.0/pi;
  attiN(3,1)=attiN(3,1)*180.0/pi;
    % ��λ����

  if(Cbn(2,2)<0 ) 
   attiN(3,1)=180.0+attiN(3,1);
  else 
   if(Cbn(2,1)<0) attiN(3,1)=360.0+attiN(3,1); end
  end
    %����Ƕȣ���λ���ȣ�

  if(Cbn(3,3)<0)
   if(Cbn(1,3)>0) attiN(1,1)=180.0-attiN(1,1); end
   if(Cbn(1,3)<0) attiN(1,1)=-(180.0+attiN(1,1)); end
  end
    %����Ƕȣ���λ���ȣ�



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%          ����IMU���ģ�ͣ����������
%  ���룺
%  Wibb       ����ϵ���������   ����λ������/�룩
%  Fb         ����ϵ���ٶȼ���� ����λ����/��/�룩
%  �����
%  Wibb       ����ϵ���������   ����λ������/�룩
%  Fb         ����ϵ���ٶȼ���� ����λ����/��/�룩
%  Wibb       ����ϵ���������   ����λ������/�룩
%  Fb         ����ϵ���ٶȼ���� ����λ����/��/�룩
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [Wibb,Fb,Wibb_err,Fb_err]=imu_err_modi(Wibb,Fb)

g=9.7803698;         %�������ٶ�    ����λ����/��/�룩
Wie=7.292115147e-5;  %������ת���ٶȣ���λ������/�룩
deg_rad=0.01745329252e0;% Transfer from angle degree to rad

%%%%%%%%%%%%%%%��ȡ���ģ�Ͳ���%%%%%%%%%%%%%%%%%
%%%%%%%%%%%�����������(�̶�ϵ���Ͱ�װ���)%%%%%%%%%%%
Da_bias=zeros(3,1);  %��ƫ
Ka_bias=zeros(3,3);  %�̶�ϵ�������
Aa_bias=zeros(3,3);  %��װ�����
  %���ٶȼ�

Dg_bias=zeros(3,1);  %�����������
Kg_bias=zeros(3,3); %�̶�ϵ�������
Ag_bias=zeros(3,3); %��װ�����
  %������

fid=fopen('sins_cal.ini','r');

tmp_imu=fgets(fid);
if( tmp_imu == -1 ) 
    disp('error!!!!!!!!!!!!!!!');
    return;
end
[Da_bias]=sscanf(tmp_imu,'%f%f%f');

tmp_imu=fgets(fid);
if( tmp_imu == -1 ) 
    disp('error!!!!!!!!!!!!!!!');
    return;
end
[Dg_bias]=sscanf(tmp_imu,'%f%f%f');

tmp_imu=fgets(fid);
if( tmp_imu == -1 ) 
    disp('error!!!!!!!!!!!!!!!');
    return;
end
[tmp_data]=sscanf(tmp_imu,'%f%f%f');
Ka_bias(1,1)=tmp_data(1,1);
Ka_bias(2,2)=tmp_data(2,1);
Ka_bias(3,3)=tmp_data(3,1);

tmp_imu=fgets(fid);
if( tmp_imu == -1 ) 
    disp('error!!!!!!!!!!!!!!!');
    return;
end
[tmp_data]=sscanf(tmp_imu,'%f%f%f');
Kg_bias(1,1)=tmp_data(1,1);
Kg_bias(2,2)=tmp_data(2,1);
Kg_bias(3,3)=tmp_data(3,1);

for i=1:3
  tmp_imu=fgets(fid);
  if( tmp_imu == -1 ) 
      disp('error!!!!!!!!!!!!!!!');
      return;
  end
  [tmp_data]=sscanf(tmp_imu,'%f%f%f');
  Aa_bias(i,1)=tmp_data(1,1);  
  Aa_bias(i,2)=tmp_data(2,1);  
  Aa_bias(i,3)=tmp_data(3,1);  
end

for i=1:3
  tmp_imu=fgets(fid);
  if( tmp_imu == -1 ) 
      disp('error!!!!!!!!!!!!!!!');
      return;
  end
  [tmp_data]=sscanf(tmp_imu,'%f%f%f');
  Ag_bias(i,1)=tmp_data(1,1);  
  Ag_bias(i,2)=tmp_data(2,1);  
  Ag_bias(i,3)=tmp_data(3,1);  
end

fclose(fid);
%%%%%%%%%%%%%%%%%%%%��ʾ���ģ��ϵ��%%%%%%%%%%%%%%%
%disp('Da(m/s/s)   Dg(deg/h)');
%disp([Da_bias,Dg_bias]);
%disp('Ka');
%disp(Ka_bias);
%disp('Kg');
%disp(Kg_bias);

%disp('Aa(��)');
%disp([Aa_bias]);
%disp('Ag(��)');
%disp([Ag_bias]);

%%%%%%%%%%%ת��Ϊ���ʵ�λ��%%%%%%%%%%%
Dg_bias=Dg_bias*deg_rad/3600.0;  % rad/s
Aa_bias=Aa_bias/60.0*deg_rad;    % rad
Ag_bias=Ag_bias/60.0*deg_rad;    % rad

Aa_bias(1,3)=-1.0*Aa_bias(1,3);
Aa_bias(2,1)=-1.0*Aa_bias(2,1);
Aa_bias(3,2)=-1.0*Aa_bias(3,2);
    %��ð�װ������
Ag_bias(1,3)=-1.0*Ag_bias(1,3);
Ag_bias(2,1)=-1.0*Ag_bias(2,1);
Ag_bias(3,2)=-1.0*Ag_bias(3,2);
    %��ð�װ������
    
%%%%%%%%%%%%%%%%%%%%��ȡ�ܵ�������%%%%%%%%%%%%%%%
Alla=eye(3)-Ka_bias-Aa_bias;
Allg=eye(3)-Kg_bias-Ag_bias;

%disp('Alla');
%disp(Alla);
%disp('Allg');
%disp(Allg);

%%%%%%%%%%%%%%����%%%%%%%%%%%%%
%tmp_Fb=(eye(3)-Ka_bias-Aa_bias)*(Fb-Da_bias);  %m/s/s
%tmp_Wibb=(eye(3)-Kg_bias-Ag_bias)*(Wibb-Dg_bias);  %rad/s
  % way(1)
tmp_Fb=inv(eye(3)+Ka_bias+Aa_bias)*(Fb-Da_bias);  %m/s/s
tmp_Wibb=inv(eye(3)+Kg_bias+Ag_bias)*(Wibb-Dg_bias);  %rad/s
  % way(2)
  
Wibb_err=Wibb-tmp_Wibb;
Fb_err=Fb-tmp_Fb;

Wibb=tmp_Wibb;
Fb=tmp_Fb;





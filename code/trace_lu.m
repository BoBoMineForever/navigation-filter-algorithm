function   [t,atti,atti_rate,veloB,acceB]=trace_lu(t,T,atti,atti_rate,veloB,acceB);; 

  if( t==0 )
      acceB(2,1) = 0;               %��ʼ��׼ʱ��  
      veloB(2,1)=100;
%    elseif(t<=20)                       %����
%       acceB(2,1)=10.0;
%     elseif(t<=24)                   %��������
%       atti_rate(2,1)=7.5;
%       acceB(2,1)=15.0;            
%    elseif (t<=64)                   %����
%       atti_rate(2,1)=0.0;
%    elseif (t<=68)                   %��ƽ
%       atti_rate(2,1)=-7.5;
%       acceB(2,1)=0.0;
   elseif (t<=668)                  %ƽֱ����
      atti_rate(2,1)=0.0;
   elseif (t<=671)                  %��бԤת��
      atti_rate(1,1)=10.0;
   elseif (t<=731)                  %ת��
      atti_rate(3,1)=1.5;
      atti_rate(1,1)=0.0;
   elseif (t<=734)                  %��ƽ
      atti_rate(3,1)=0.0;
      atti_rate(1,1)=-10.0;
   elseif (t<=1334)                 %ƽֱ����
      atti_rate(1,1)=0.0;
  end

 veloB(2,1)=veloB(2,1)+acceB(2,1)*T;    

   atti(1,1)=atti(1,1)+atti_rate(1,1)*T;
   atti(2,1)=atti(2,1)+atti_rate(2,1)*T;
   atti(3,1)=atti(3,1)+atti_rate(3,1)*T;
  
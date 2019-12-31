function   [t,atti,atti_rate,veloB,acceB]=trace_lu(t,T,atti,atti_rate,veloB,acceB);; 

  if( t==0 )
      acceB(2,1) = 0;               %初始对准时间  
      veloB(2,1)=100;
%    elseif(t<=20)                       %滑跑
%       acceB(2,1)=10.0;
%     elseif(t<=24)                   %加速拉起
%       atti_rate(2,1)=7.5;
%       acceB(2,1)=15.0;            
%    elseif (t<=64)                   %爬高
%       atti_rate(2,1)=0.0;
%    elseif (t<=68)                   %改平
%       atti_rate(2,1)=-7.5;
%       acceB(2,1)=0.0;
   elseif (t<=668)                  %平直飞行
      atti_rate(2,1)=0.0;
   elseif (t<=671)                  %倾斜预转弯
      atti_rate(1,1)=10.0;
   elseif (t<=731)                  %转弯
      atti_rate(3,1)=1.5;
      atti_rate(1,1)=0.0;
   elseif (t<=734)                  %改平
      atti_rate(3,1)=0.0;
      atti_rate(1,1)=-10.0;
   elseif (t<=1334)                 %平直飞行
      atti_rate(1,1)=0.0;
  end

 veloB(2,1)=veloB(2,1)+acceB(2,1)*T;    

   atti(1,1)=atti(1,1)+atti_rate(1,1)*T;
   atti(2,1)=atti(2,1)+atti_rate(2,1)*T;
   atti(3,1)=atti(3,1)+atti_rate(3,1)*T;
  
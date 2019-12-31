%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%                    航迹发生器
%
%      参数:t-仿真时间,T-仿真步长；
%           飞行器姿态角（度），姿态角角速度（度/秒）；
%           飞行器相对机体系的运动速度（米/秒）；
%           飞行器相对机体系的运动加速度（米/秒秒）；
%        
%                           程序设计：熊智  日期：2002/4/15
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [t,atti,atti_rate,veloB,acceB]=tracesar(t,T,atti,atti_rate,veloB,acceB)

    if(t==0 )
      acceB(2,1)=0.0;               %初始对准时间
    elseif(t<=20)                   %滑跑
      acceB(2,1)=4.0;
    elseif(t<=24)                   %加速拉起
      atti_rate(2,1)=7.5;
      acceB(2,1)=5.0;            
   elseif (t<=64)                   %爬高
      atti_rate(2,1)=0.0;
   elseif (t<=68)                   %改平
      atti_rate(2,1)=-7.5;
      acceB(2,1)=0.0;
   elseif (t<=668)                  %平直飞行
      atti_rate(2,1)=0.0;
   elseif (t<=671)                  %倾斜预转弯
      atti_rate(3,1)=10.0;
   elseif (t<=731)                  %转弯
      atti_rate(1,1)=1.5;
      atti_rate(3,1)=0.0;
   elseif (t<=734)                  %改平
      atti_rate(1,1)=0.0;
      atti_rate(3,1)=-10.0;
   elseif (t<=1334)                 %平直飞行
      atti_rate(3,1)=0.0;
   elseif (t<=1342)                 %加速拉起
      atti_rate(2,1)=7.5;
      acceB(2,1)=2.0;
   elseif (t<=1374)                 %加速爬高
      atti_rate(2,1)=0.0;
   elseif (t<=1382)                 %改平
      atti_rate(2,1)=-7.5;
      acceB(2,1)=0.0;
   elseif (t<=1862)                 %平直飞行
       atti_rate(2,1)=0.0;
   elseif (t<=1902)                 %减速飞行
      acceB(2,1)=-2.5;
   elseif (t<=1905)                 %倾斜预转弯
      atti_rate(3,1)=10.0;
      acceB(2,1)=0.0;
   elseif (t<=1965)                 %转弯
      atti_rate(1,1)=1.5;
      atti_rate(3,1)=0.0;
   elseif (t<=1968)                 %改平
      atti_rate(1,1)=0.0;
      atti_rate(3,1)=-10.0;
   elseif (t<=2568)                 %平直飞行
      atti_rate(3,1)=0.0;
   elseif (t<=2574)                 %低头
      atti_rate(2,1)=-7.5;
   elseif (t<=2594)                 %俯冲
      atti_rate(2,1)=0.0;
   elseif (t<=2600)                 %改平
      atti_rate(2,1)=7.5;
   elseif (t<=2603)                 %倾斜预转弯
      atti_rate(2,1)=0.0;
      atti_rate(3,1)=10.0;
   elseif (t<=2663)                 %转弯
      atti_rate(1,1)=1.5;
      atti_rate(3,1)=0.0;
   elseif (t<=2666)                 %改平
      atti_rate(1,1)=0.0;
      atti_rate(3,1)=-10.0;
   elseif (t<=3600)                 %平直飞行
      atti_rate(3,1)=0.0;            
   end

   veloB(2,1)=veloB(2,1)+acceB(2,1)*T;    

   atti(1,1)=atti(1,1)+atti_rate(1,1)*T;
   atti(2,1)=atti(2,1)+atti_rate(2,1)*T;
   atti(3,1)=atti(3,1)+atti_rate(3,1)*T;
  
%   if(atti(1,1)>360)
%	atti(1,1)=atti(1,1)-fix(atti(1,1)/360)*360;
%   end



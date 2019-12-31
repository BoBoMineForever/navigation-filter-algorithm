%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%                    航迹发生器
%
%  输入参数:
%  t          仿真时间
%  T          仿真步长
%  atti       横滚、俯仰、航向（单位：度）
%  atti_rate  横滚速率、俯仰速率、航向速率（单位：度/秒）
%  veloB      飞机运动速度——X右翼、Y机头、Z天向（单位：米/秒）
%  acceB      飞机运动加速度——X右翼、Y机头、Z天向（单位：米/秒/秒）
%  posi       航迹发生器初始位置经度、纬度、高度（单位：度、度、米）
%
%        
%                           程序设计：熊智  日期：2002/8/9
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [t,atti,atti_rate,veloB,acceB]=trace_squ(t,T,atti,atti_rate,veloB,acceB)

   %%%%%%%%%%%%%%%%模拟姿态变化%%%%%%%%%%%%%%%    
   %  静止状态
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   if(t<=5)
      return;
   elseif(t<=10)
      acceB(2,1)=2.0;
   elseif(t<=50)
      acceB(2,1)=0.0;
   elseif(t<=55)
      acceB(2,1)=-2.0;
   elseif(t<=60)
      acceB(2,1)=0.0;
      atti_rate(3,1)=-18.0;
   elseif(t<=65)
      acceB(2,1)=2.0;
      atti_rate(3,1)=0.0;
   elseif(t<=105)
      acceB(2,1)=0.0;
  elseif(t<=110)
      acceB(2,1)=-2.0;
  elseif(t<=115)
      acceB(2,1)=0.0;
      atti_rate(3,1)=-18.0;
  elseif(t<=120)
      acceB(2,1)=2.0;
      atti_rate(3,1)=0.0;      
  elseif(t<=160)
      acceB(2,1)=0.0;
  elseif(t<=165)
      acceB(2,1)=-2.0;
  elseif(t<=170)
      acceB(2,1)=0.0;
      atti_rate(3,1)=-18.0;
  elseif(t<=175)
      acceB(2,1)=2.0;
      atti_rate(3,1)=0.0;
  elseif(t<=215)
      acceB(2,1)=0.0;
  elseif(t<=220)
      acceB(2,1)=-2.0;
  elseif(t<=225)
      acceB(2,1)=0.0;
      atti_rate(3,1)=-18.0;
  elseif(t<=250)
      atti_rate(3,1)=0.0;
   end

   veloB(2,1)=veloB(2,1)+acceB(2,1)*T;   %飞机初始运动速度（米/秒）——机头

   atti(1,1)=atti(1,1)+atti_rate(1,1)*T;
   atti(2,1)=atti(2,1)+atti_rate(2,1)*T;
   atti(3,1)=atti(3,1)+atti_rate(3,1)*T;
  


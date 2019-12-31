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


function [t,atti,atti_rate,veloB,acceB]=trace_dyn(t,T,atti,atti_rate,veloB,acceB)

   %%%%%%%%%%%%%%%%模拟姿态变化%%%%%%%%%%%%%%%    
   %  静止状态
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
   if(t<=120)                  
      return;
   elseif (t<=125)                  
      atti_rate(1,1)=9.0;
   elseif (t<=240)                  
      atti_rate(1,1)=0.0;
   elseif (t<=245)                  
      atti_rate(1,1)=-9.0;
   elseif (t<=360)                  
      atti_rate(1,1)=0.0;
   elseif(t<=365)
      atti_rate(1,1)=-9.0;
   elseif(t<=480)
      atti_rate(1,1)=0.0;
   elseif(t<=485)
      atti_rate(1,1)=9.0;
   else
      atti_rate(1,1)=0.0;
   end
     %横滚变化

   if(t<=600)                  
      atti_rate(2,1)=0.0;
   elseif (t<=605)                  
      atti_rate(2,1)=9.0;
   elseif (t<=720)                  
      atti_rate(2,1)=0.0;
   elseif (t<=725)                  
      atti_rate(2,1)=-9.0;
   elseif (t<=840)                  
      atti_rate(2,1)=0.0;
   elseif(t<=845)
      atti_rate(2,1)=-9.0;
   elseif(t<=960)
      atti_rate(2,1)=0.0;
   elseif(t<=965)
      atti_rate(2,1)=9.0;
   else
      atti_rate(2,1)=0.0;
   end
     %俯仰变化
 
   if(t<=1080)                  
      atti_rate(3,1)=0.0;
   elseif (t<=1085)                  
      atti_rate(3,1)=9.0;
   elseif (t<=1200)                  
      atti_rate(3,1)=0.0;
   elseif (t<=1205)                  
      atti_rate(3,1)=-9.0;
   elseif (t<=1320)                  
      atti_rate(3,1)=0.0;
   elseif(t<=1325)
      atti_rate(3,1)=-9.0;
   elseif(t<=1440)
      atti_rate(3,1)=0.0;
   elseif(t<=1445)
      atti_rate(3,1)=9.0;
   else
      atti_rate(3,1)=0.0;
   end
     %航向变化

   veloB(2,1)=veloB(2,1)+acceB(2,1)*T;   %飞机初始运动速度（米/秒）——机头

   atti(1,1)=atti(1,1)+atti_rate(1,1)*T;
   atti(2,1)=atti(2,1)+atti_rate(2,1)*T;
   atti(3,1)=atti(3,1)+atti_rate(3,1)*T;
  


A=zeros(length(dp)); A(1,1:5)=1; b=zeros(length(dp),1); b(1)=1;
Aeq=zeros(length(dp)); beq=zeros(length(dp),1);
dpmin=fmincon(@(x) DroneDesign(x),dp,A,b,Aeq,beq,dprng(1,:),dprng(2,:))

function M0 = DroneDesign(dp)

global payloadWeight specCapSlineID maxTakeoffThrotle minBatteryCapacity hoveringTime specCapSpl

mubat=dp(1);
muel=dp(2);
mufr=dp(3);
mueng=dp(4);
nCell=dp(5);
Crate=5*dp(6);
nprop=2*dp(7);


M0=payloadWeight/(1-(mubat+muel+mufr+nprop*mueng));
if M0<=0
    M0=100*payloadWeight;
    return
end

disp(['Drone mass ',num2str(M0,4),' kg'])

Mbat=mubat*M0;
Mel=muel*M0;
Mfr=mufr*M0;
Mmot=mueng*M0/1.18;
Mprop=mueng*M0*(1-1/1.18);

Imotmax=3.5*sqrt(1000*Mmot);

Imax=nprop*Imotmax;

disp(['Max. motor current ',num2str(Imax,4),' A'])

disp(['Masses: Bat. ',num2str(Mbat,2),' kg; El. ',num2str(Mel,2),' kg; Frame ',num2str(Mfr,2),...
    ' kg; Motor ',num2str(Mmot,2),' kg; Prop ',num2str(Mprop,2)])

maxVoltage=nCell*3.7;
disp(['Max. Voltage ',num2str(maxVoltage,4),' V'])

specCap=getYByX(specCapSpl,Crate)*1000;
disp(['Specific cell capacity ',num2str(specCap,4),' mAh/kg'])
Cap=specCap*Mbat/nCell;
Charge=Cap*3600/1000;
disp(['Capacity ',num2str(Cap,4),' mAh'])
disp(['Charge ',num2str(Charge,4),' C'])

maxCurrent=Cap*Crate/1000;
disp(['Max. battery current ',num2str(maxCurrent,4),' A'])

if Imax > maxCurrent
    M0=100*payloadWeight;
    return
end

Imotdsch=Imotmax*maxTakeoffThrotle^2.5;

Thr=Mmot*Imotdsch^0.7;
disp(['Motor thrust ',num2str(Thr,4),' kg'])

if 4*Thr < M0
    M0=100*payloadWeight;
    return
end

Idsch=4*Imotdsch;

%tspan = [1 100];
%opts = odeset('RelTol',1e-6,'AbsTol',1e-6,'Events',@EventsFcn);
%[t,c] = ode45(@(t,c) odefun(t,c), tspan, [Charge], opts);

%    function dcdt = odefun(t,c)
%        dcdt=-maxTakeoffThrotle*Imotmax;
%    end

%    function [value,isterminal,direction] = EventsFcn(t,c)
%        value=[minBatteryCapacity*c];
%        isterminal=[1];
%        direction=[-1];
%    end

simTime=(1-minBatteryCapacity)*Charge/Idsch;

disp(['Simulation time ', num2str(simTime,3),' s'])
if simTime < hoveringTime || isnan(simTime)
    M0=100*payloadWeight;
    return
end

end

function t = ThrustByCurrent(A, b, I)
t = A*I^b
return
end

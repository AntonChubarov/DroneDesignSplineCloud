function M0 = DroneDesign(dp)

global payloadWeight specCapSlineID maxTakeoffThrotle minBatteryCapacity hoveringTime

mubat=dp(1);
muel=dp(2);
mufr=dp(3);
mumot=dp(4);
muprop=dp(5);
nCell=dp(6);
Crate=5*dp(7);
nprop=dp(8);


M0=payloadWeight/(1-(mubat+muel+mufr+nprop*mumot+nprop*muprop));
if M0<=0
    M0=100*payloadWeight;
    return
end

Mbat=mubat*M0;
Mel=muel*M0;
Mfr=mufr*M0;
Mmot=mumot*M0;
Mprop=muprop*M0;

disp(['Masses: Bat. ',num2str(Mbat,2),' kg; El. ',num2str(Mel,2),' kg; Frame ',num2str(Mfr,2),...
    ' kg; Motor ',num2str(Mmot,2),' kg; Prop ',num2str(Mprop,2)])

maxVoltage=nCell*3.7;
disp(['Max. Voltage ',num2str(maxVoltage,4),' V'])

specCapSpl=GetNURBSfromSC(specCapSlineID);
specCap=getYByX(specCapSpl,Crate)*1000;
disp(['Specific cell capacity ',num2str(specCap,4),' mAh/kg'])
Cap=specCap*Mbat/nCell;
Charge=Cap*3600/1000;
disp(['Capacity ',num2str(Cap,4),' mAh'])
disp(['Charge ',num2str(Charge,4),' C'])

maxCurrent=Cap*Crate/1000;
disp(['Max. current ',num2str(maxCurrent,4),' A'])

tspan = [1 100];
opts = odeset('RelTol',1e-6,'AbsTol',1e-6,'Events',@EventsFcn);
[t,c] = ode45(@(t,c) odefun(t,c), tspan, [Charge], opts);

    function dcdt = odefun(t,c)
    dcdt=-maxTakeoffThrotle*maxCurrent;
    end

    function [value,isterminal,direction] = EventsFcn(t,c)
    value=[minBatteryCapacity*c];
    isterminal=[1];
    direction=[-1];
    end

disp(['Simulation time ', num2str(t(end),2),' s'])
if t(end) < hoveringTime
    M0=100*payloadWeight;
    return
end

end

function nurbs = GetNURBSfromSC(curveId)
spline = fetchCurve(curveId);
[ctrlPoints, knots] = extractNurbsParams(spline);
nurbs = nrbmak(ctrlPoints, knots);
end

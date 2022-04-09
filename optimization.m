global specCapSlineID specCapSpl

specCapSpl=GetNURBSfromSC(specCapSlineID);

options = optimoptions('ga',...
    'ConstraintTolerance',1e-3,...
    'CreationFcn','gacreationuniform',...
    'PopulationSize',500,...
    'MaxGenerations',100,...
    'PlotFcn', @gaplotbestf);
dpmin=ga(@(x) DroneDesign(x),length(dp),[],[],[],[],dprng(1,:),dprng(2,:),[],[5;6;7],options)

function nurbs = GetNURBSfromSC(curveId)
spline = fetchCurve(curveId);
[ctrlPoints, knots] = extractNurbsParams(spline);
nurbs = nrbmak(ctrlPoints, knots);
end
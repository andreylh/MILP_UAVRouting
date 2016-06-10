param unidadeDeTempo;
param dimSquare;
param dL;
param BIGM;

set P;
param pointDX{P}, >=0, <= dimSquare, integer;
param pointDY{P}, >=0, <= dimSquare, integer;
param pointPackageCAP{P}; /*Vetor real com a quantidade a ser coletada*/
param pointToDeliverDX{P}; /* ponto cordinate Dx*/
param pointToDeliverDY{P}; /* ponto cordinate Dy*/
param pointEnergy{P}, binary;

set T;
param pointEnergyPrice{P,T}; /*O preco da energy no instante t*/

set DRONE;
param droneInitialDX{DRONE}, >=0, <= dimSquare;
param droneInitialDY{DRONE}, >=0, <= dimSquare;
param droneMS{DRONE};
param droneMaxCap{DRONE};
param droneDoD{DRONE}; /*Maximo que se pode descarregar*/
param droneBatteryPower{DRONE}; /* 60Kwh*/
param isDroneDeliveringAtBegin{DRONE,P}; /* 60Kwh*/

/*VARIAVEIS*/
var droneDX{DRONE, T}, >=0, <= dimSquare;
var droneDY{DRONE, T}, >=0, <= dimSquare;
var droneCurrentCap{DRONE, T}, >=0;
var droneCollected{DRONE, P, T}, binary;
var droneDelivered{DRONE, P, T}, binary;
var droneIsDelivering{DRONE, P, T}, binary;
var droneIsUsed{DRONE}, binary;
var droneSpeed{DRONE,T}, >=0;

var droneBatteryRate{DRONE, T};

/*AUXILIAR VARIABLES*/
var diffPosX{DRONE, T}, >=0;
var diffNegX{DRONE, T}, >=0;
var diffPosY{DRONE, T}, >=0;
var diffNegY{DRONE, T}, >=0;
var timePerProductDelivered{P}, >=0;

/*OBJ VARIABLES*/
var totalDist, >=0;
var timePerDeliver, >=0;
var nUsedDrones, >=0;
var dronesMaxSpeed, >=0;


s.t. 

calcTotalDist: totalDist = sum{t in T:t>=2} sum{d in DRONE} (diffNegX[d,t] + diffPosX[d,t] + diffPosY[d,t] + diffNegY[d,t]) * dL;
calcTotalTimePerDeliver: timePerDeliver = sum{p in P} timePerProductDelivered[p];
calcNDrones: nUsedDrones = sum{d in DRONE} droneIsUsed[d];
calcDronesAVGSpeed{d in DRONE, t in T:t>=2}: dronesMaxSpeed >= droneSpeed[d,t];

minimize obj: totalDist + timePerDeliver + nUsedDrones + dronesMaxSpeed;




respectDroneCap{d in DRONE, t in T}: droneCurrentCap[d,t] <= droneMaxCap[d];

updatedCAP{d in DRONE, t in T:t>=2}:  droneCurrentCap[d,t] = droneCurrentCap[d,t-1] + sum{p in P} ( droneCollected[d,p,t]*pointPackageCAP[p] - droneDelivered[d,p,t]*pointPackageCAP[p]);
updatedCAP1{d in DRONE, t in T:t<2}:  droneCurrentCap[d,t] = sum{p in P} ( droneCollected[d,p,t]*pointPackageCAP[p] - droneDelivered[d,p,t]*pointPackageCAP[p]);

droneShouldCollect{p in P}: sum{t in T} sum{d in DRONE}  (droneCollected[d,p,t]*BIGM) >= pointPackageCAP[p];
droneShouldCollectAlone{p in P}: sum{t in T} sum{d in DRONE}  droneCollected[d,p,t] <= 1;
droneShouldCollectAndDeliver{p in P, d in DRONE}: sum{t in T} droneDelivered[d,p,t] = sum{t in T} droneCollected[d,p,t];
droneDeliverAfterCollecting{p in P, d in DRONE,t in T:t>=2}: droneDelivered[d,p,t] >= droneIsDelivering[d,p,t-1];
droneIsBeingUsed{d in DRONE}: droneIsUsed[d]*BIGM >= sum{p in P} sum{t in T} droneCollected[d,p,t];

updatedDelivering{p in P, d in DRONE,t in T:t>=2}: droneIsDelivering[d,p,t] = droneIsDelivering[d,p,t-1] + droneCollected[d,p,t] - droneDelivered[d,p,t];
updatedDelivering2{p in P, d in DRONE,t in T:t<2}: droneIsDelivering[d,p,t] = isDroneDeliveringAtBegin[d,p] + droneCollected[d,p,t];

productDeliveringTime{p in P}: timePerProductDelivered[p] = sum{d in DRONE} sum{t in T} droneIsDelivering[d,p,t];

initializePositionX{d in DRONE,t in T:t<2}: droneDX[d,t] = droneInitialDX[d];
initializePositionY{d in DRONE,t in T:t<2}: droneDY[d,t] = droneInitialDY[d];
                                                                    
packageWasCollectedAtRightPositionX1{p in P, d in DRONE,t in T}:  droneDX[d,t]  - pointDX[p] <=  dimSquare*(1 - droneCollected[d,p,t]);
packageWasCollectedAtRightPositionX2{p in P, d in DRONE,t in T}:  -droneDX[d,t]  + pointDX[p]  <=  dimSquare*(1 - droneCollected[d,p,t]);
packageWasCollectedAtRightPositionY1{p in P, d in DRONE,t in T}:  droneDY[d,t]  - pointDY[p] <=  dimSquare*(1 - droneCollected[d,p,t]);
packageWasCollectedAtRightPositionY2{p in P, d in DRONE,t in T}:  -droneDY[d,t]  + pointDY[p]   <=  dimSquare*(1 - droneCollected[d,p,t]);


packageWasDeliveredAtRightPositionX1{p in P, d in DRONE,t in T}:  droneDX[d,t]  - pointToDeliverDX[p] <=  dimSquare*(1 - droneDelivered[d,p,t]);
packageWasDeliveredAtRightPositionX2{p in P, d in DRONE,t in T}:  -droneDX[d,t]  + pointToDeliverDX[p] <=  dimSquare*(1 - droneDelivered[d,p,t]);
packageWasDeliveredAtRightPositionY1{p in P, d in DRONE,t in T}:  droneDY[d,t]  - pointToDeliverDY[p] <=  dimSquare*(1 - droneDelivered[d,p,t]);
packageWasDeliveredAtRightPositionY2{p in P, d in DRONE,t in T}:  -droneDY[d,t]  + pointToDeliverDY[p] <=  dimSquare*(1 - droneDelivered[d,p,t]);


calcABSMoveX1{d in DRONE,t in T:t>=2}: (droneDX[d,t] - droneDX[d,t-1]) + diffPosX[d,t] >=  0;
calcABSMoveX2{d in DRONE,t in T:t>=2}: (droneDX[d,t] - droneDX[d,t-1]) - diffNegX[d,t] <=  0;
calcABSMoveY1{d in DRONE,t in T:t>=2}: (droneDY[d,t] - droneDY[d,t-1]) + diffPosY[d,t] >=  0;
calcABSMoveY2{d in DRONE,t in T:t>=2}: (droneDY[d,t] - droneDY[d,t-1]) - diffNegY[d,t] <=  0;


calcDronesSpeed{d in DRONE, t in T:t>=2}:  droneSpeed[d,t] = ((diffPosX[d,t] + diffPosX[d,t] + diffPosY[d,t] + diffNegY[d,t])*dL / unidadeDeTempo );

respectDroneMaxSpeed{d in DRONE, t in T:t>=2}:  droneSpeed[d,t] <= droneMS[d];



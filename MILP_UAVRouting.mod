param unidadeDeTempo;
param dimSquare; /* max number of points - size will be meters * dimSquare */
param dL; /* in meters */
param BIGM;
param lastT;

set P;
param pointDX{P}, >=0, <= dimSquare;
param pointDY{P}, >=0, <= dimSquare;
param pointPackageCAP{P}; /*Vetor real com a quantidade a ser coletada in KG*/
param pointToDeliverDX{P}; /* ponto cordinate Dx*/
param pointToDeliverDY{P}; /* ponto cordinate Dy*/

set T;

set EP;
param ePDX{EP}, >=0, <= dimSquare;
param ePDY{EP}, >=0, <= dimSquare;

/* param pointEnergyPrice{P,T};*/ /*O preco da energy no instante t*/ 



set DRONE;
param droneInitialDX{DRONE}, >=0, <= dimSquare;
param droneInitialDY{DRONE}, >=0, <= dimSquare;
param droneMS{DRONE}; /*Drone speed in KM/h */
param droneVEC{DRONE}; /*Drone Variable Energy Consumption related to its MaxSpeed (per Minutes) */
param droneFEC{DRONE}; /*Drone Fixed Energy Consumption (per Minutes)  */
param droneMaxCap{DRONE}; /* Drone max cap in KG */
param droneDoD{DRONE}; /*Maximo que se pode descarregar*/
param droneRateOfCharge{DRONE}; /* Drone Rate of Charge (per Minute)*/
param droneBROnArr{DRONE}; /* Drone battery rate on Arrival (%)*/
param droneBatteryPower{DRONE}; /* 60Kwh*/
param droneIsDeliveringAtBegin{DRONE,P}, binary; /* 60Kwh*/

/*VARIAVEIS*/
var droneDX{DRONE, T}, >=0, <= dimSquare;
var droneDY{DRONE, T}, >=0, <= dimSquare;
var droneCurrentCap{DRONE, T}, >=0;
var droneCollected{DRONE, P, T}, binary;
var droneDelivered{DRONE, P, T}, binary;
var droneIsDelivering{DRONE, P, T}, binary;


/*AUXILIAR VARIABLES*/
var diffPosX{DRONE, T}, >=0;
var diffNegX{DRONE, T}, >=0;
var diffPosY{DRONE, T}, >=0;
var diffNegY{DRONE, T}, >=0;
var timePerProductDelivered{P}, >=0;
var droneIsUsed{DRONE}, binary;
var droneSpeed{DRONE,T},>=0;
var droneBatteryRate{DRONE,T}, >=0, <=100;
var excessOfEnergy{DRONE,T}, >=0, <=20;
var droneIsChargingAtEP{DRONE,T, EP}, binary;
var droneIsOn{DRONE,T}, binary;

/*OBJ VARIABLES*/
var totalDist, >=0;
var timeToDeliver, >=0;
var nUsedDrones, >=0;
var dronesMaxSpeed, >=0;
var totalExcessOfEnergy, >=0;


s.t. 

calcTotalDist: totalDist = sum{t in T:t>=2} sum{d in DRONE} (diffNegX[d,t] + diffPosX[d,t] + diffPosY[d,t] + diffNegY[d,t]) * dL / 1000;
calcTotalTimeToDeliver: timeToDeliver = sum{p in P} timePerProductDelivered[p];
calcNDrones: nUsedDrones = sum{d in DRONE} droneIsUsed[d];
calcDronesAVGSpeed{d in DRONE, t in T:t>=2}: dronesMaxSpeed >= droneSpeed[d,t];
calcBatteriesOverLoad: totalExcessOfEnergy = sum{d in DRONE} sum{t in T} excessOfEnergy[d,t];

minimize obj: totalDist + timeToDeliver + nUsedDrones + dronesMaxSpeed + totalExcessOfEnergy ;

respectDroneDoD{d in DRONE, t in T:t>=2}: droneBatteryRate[d,t] >= droneDoD[d];
respectDroneCap{d in DRONE, t in T}: droneCurrentCap[d,t] <= droneMaxCap[d];

updatedCAP{d in DRONE, t in T:t>=2}:  droneCurrentCap[d,t] = droneCurrentCap[d,t-1] + sum{p in P} ( droneCollected[d,p,t]*pointPackageCAP[p] - droneDelivered[d,p,t]*pointPackageCAP[p]);
updatedCAPAtBegin{d in DRONE, t in T:t==1}:  droneCurrentCap[d,t] = sum{p in P} droneIsDeliveringAtBegin[d,p]*pointPackageCAP[p];

droneShouldCollect{p in P}: (sum{d in DRONE} droneIsDeliveringAtBegin[d,p] + (sum{t in T} sum{d in DRONE} droneCollected[d,p,t]) )*pointPackageCAP[p] >= pointPackageCAP[p];
/* maybe remove second term */
droneShouldCollectAlone{p in P}: (sum{d in DRONE} droneIsDeliveringAtBegin[d,p]) + (sum{t in T} sum{d in DRONE} droneCollected[d,p,t]) = 1;
droneShouldCollectAndDeliver{p in P, d in DRONE}: sum{t in T} droneDelivered[d,p,t] = droneIsDeliveringAtBegin[d,p] + (sum{t in T} droneCollected[d,p,t]); 


updatedDelivering{p in P, d in DRONE,t in T:t>=2}: droneIsDelivering[d,p,t] = droneIsDelivering[d,p,t-1] + droneCollected[d,p,t] - droneDelivered[d,p,t];
updatedDeliveringAtBegin{p in P, d in DRONE,t in T:t==1}: droneIsDelivering[d,p,t] = droneIsDeliveringAtBegin[d,p];
droneShouldBeDoingNothingAtTheEnd{p in P, d in DRONE, t in T:t==lastT}: droneIsDelivering[d,p,t] = 0;

droneIsBeingUsed{d in DRONE}: droneIsUsed[d]*BIGM >= sum{p in P} sum{t in T} droneIsDelivering[d,p,t];
updateDroneIsOn{d in DRONE, t in T}: droneIsOn[d,t]*BIGM >= sum{p in P} droneIsDelivering[d,p,t];

calcBaterryRateFirstInterval{d in DRONE}: droneBatteryRate[d,1] = droneBROnArr[d];
calcBaterryRate{d in DRONE, t in T:t>=2}: droneBatteryRate[d,t] = droneBatteryRate[d,t-1] - (droneVEC[d]*droneSpeed[d,t])/droneMS[d]*unidadeDeTempo - droneIsOn[d,t]*droneFEC[d] + sum{ep in EP} (droneIsChargingAtEP[d,t,ep]*droneRateOfCharge[d]) - excessOfEnergy[d,t];


productDeliveringTime{p in P}: timePerProductDelivered[p] = sum{d in DRONE} sum{t in T} droneIsDelivering[d,p,t];

initializePositionX{d in DRONE,t in T:t==1}: droneDX[d,t] = droneInitialDX[d];
initializePositionY{d in DRONE,t in T:t==1}: droneDY[d,t] = droneInitialDY[d];
                                                                    
packageWasCollectedAtRightPositionX1{p in P, d in DRONE,t in T}:  droneDX[d,t]  - pointDX[p] <=  dimSquare*(1 - droneCollected[d,p,t]);
packageWasCollectedAtRightPositionX2{p in P, d in DRONE,t in T}:  -droneDX[d,t]  + pointDX[p]  <=  dimSquare*(1 - droneCollected[d,p,t]);
packageWasCollectedAtRightPositionY1{p in P, d in DRONE,t in T}:  droneDY[d,t]  - pointDY[p] <=  dimSquare*(1 - droneCollected[d,p,t]);
packageWasCollectedAtRightPositionY2{p in P, d in DRONE,t in T}:  -droneDY[d,t]  + pointDY[p]   <=  dimSquare*(1 - droneCollected[d,p,t]);


packageWasDeliveredAtRightPositionX1{p in P, d in DRONE,t in T}:  droneDX[d,t]  - pointToDeliverDX[p] <=  dimSquare*(1 - droneDelivered[d,p,t]);
packageWasDeliveredAtRightPositionX2{p in P, d in DRONE,t in T}:  -droneDX[d,t]  + pointToDeliverDX[p] <=  dimSquare*(1 - droneDelivered[d,p,t]);
packageWasDeliveredAtRightPositionY1{p in P, d in DRONE,t in T}:  droneDY[d,t]  - pointToDeliverDY[p] <=  dimSquare*(1 - droneDelivered[d,p,t]);
packageWasDeliveredAtRightPositionY2{p in P, d in DRONE,t in T}:  -droneDY[d,t]  + pointToDeliverDY[p] <=  dimSquare*(1 - droneDelivered[d,p,t]);

droneChargedAtRightPositionX1{ep in EP, d in DRONE,t in T}:  droneDX[d,t]  - ePDX[ep] <=  dimSquare*(1 - droneIsChargingAtEP[d,t,ep]);
droneChargedRightPositionX2{ep in EP, d in DRONE,t in T}:  -droneDX[d,t]  + ePDX[ep]  <=  dimSquare*(1 - droneIsChargingAtEP[d,t,ep]);
droneChargedAtRightPositionY1{ep in EP, d in DRONE,t in T}:  droneDY[d,t]  - ePDY[ep] <=  dimSquare*(1 - droneIsChargingAtEP[d,t,ep]);
droneChargedAtRightPositionY2{ep in EP, d in DRONE,t in T}:  -droneDY[d,t]  + ePDY[ep]   <=  dimSquare*(1 - droneIsChargingAtEP[d,t,ep]);



calcABSMoveX1{d in DRONE,t in T:t>=2}: (droneDX[d,t] - droneDX[d,t-1]) + diffPosX[d,t] >=  0;
calcABSMoveX2{d in DRONE,t in T:t>=2}: (droneDX[d,t] - droneDX[d,t-1]) - diffNegX[d,t] <=  0;
calcABSMoveY1{d in DRONE,t in T:t>=2}: (droneDY[d,t] - droneDY[d,t-1]) + diffPosY[d,t] >=  0;
calcABSMoveY2{d in DRONE,t in T:t>=2}: (droneDY[d,t] - droneDY[d,t-1]) - diffNegY[d,t] <=  0;


calcDronesSpeed{d in DRONE, t in T:t>=2}:  droneSpeed[d,t] = ( (diffPosX[d,t] + diffPosX[d,t] + diffPosY[d,t] + diffNegY[d,t])*dL / unidadeDeTempo * 60 / 1000);

respectDroneMaxSpeed{d in DRONE, t in T:t>=2}:  droneSpeed[d,t] <= droneMS[d];



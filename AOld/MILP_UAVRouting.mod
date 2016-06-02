param unidadeDeTempo;
param dimSquare;
param dL;
param BIGM = 10000;

set P;
param pointDX{P}, >=0, <= dimSquare, integer;
param pointDY{P}, >=0, <= dimSquare, integer;
param pointAmmountToCollet{P}; /*Vetor real com a quantidade a ser coletada*/
param pointToDeliverDX{P}; /* ponto cordinate Dx*/
param pointToDeliverDY{P}; /* ponto cordinate Dy*/
param pointEnergy{P}, binary;

set T;
param pointEnergyPrice{P,T}; /*O preco da energy no instante t*/

set DRONE;
param droneMS{DRONE};
param droneMaxCap{DRONE};
param droneDoD{DRONE}; /*Maximo que se pode descarregar*/
param droneBatteryPower{DRONE}; /* 60Kwh*/

/*VARIAVEIS*/
var droneDX{DRONE, T}, >=0, <= dimSquare;
var droneDY{DRONE, T}, >=0, <= dimSquare;
var droneCurrentCap{DRONE, T}, >=0;
var droneBatteryRate{DRONE, T};
var droneCollectPoint{DRONE, P}, binary;
var pointCollected{P,T}, binary;
var pointDeliver{P,T}, binary;

s.t. 

respectDroneMaxSpeed{d in DRONE, t in T:t>=2}:  ( (droneDX[d,t] - droneDX[d,t-1])*dL +  (droneDY[d,t] - droneDY[d,t-1])*dL )  / unidadeDeTempo <= droneMS[d];

respectDroneCap{d in DRONE, t in T}: droneCurrentCap[d,t] <= droneMaxCap[d];

updatedCAP{d in DRONE, t in T:t>=2}:  droneCurrentCap[d,t] = droneCurrentCap[d,t-1] + sum{p in P} ( droneCollectPoint[d,p]*pointAmmountToCollet[p]);

updatedCAP1{d in DRONE, t in T:t<2}:  droneCurrentCap[d,t] = sum{p in P} ( droneCollectPoint[d,p]*pointAmmountToCollet[p]);

onlyOneDronePerPoint{p in P}: sum{d in DRONE} droneCollectPoint[d,p] = 1;

onlyCollectPointInSingleT{p in P, d in DRONE}: sum{t in T} pointCollected[p,t] = droneCollectPoint[d,p];

pointWasCollected{p in P}:  sum{d in DRONE} 1000000*droneCollectPoint[d,p] >= pointAmmountToCollet[p];

pointWasCollectedAtRightPositionX{p in P, d in DRONE}:  sum{t in T} ( 1 + droneDX[d,t] - pointDX[p])  = droneCollectPoint[d,p];
pointWasCollectedAtRightPositionY{p in P, d in DRONE}:  sum{t in T} ( 1 + droneDY[d,t] - pointDY[p])  = droneCollectPoint[d,p];

/*onlyOneDrone{p in P, t in T}: sum{d in DRONE} droneIsDelivering[d,p,t] = 1;*/
/*
onlyOneDronePerPoint{p in P}: sum{d in DRONE} droneCollectPoint[d,p] = 1;
onlyCollectPointInSingleT{p in P, d in DRONE}: sum{t in T} pointCollected[p,t] = droneCollectPoint[d,p];
pointWasCollected{p in P}:  sum{d in DRONE} 1000000*droneCollectPoint[d,p] >= pointAmmountToCollet[p];
*/

/*
pointWasCollectedAtRightPositionX{p in P, d in DRONE,t in T}:  droneDX[d,t]   =  (pointDX[p] * droneCollected[d,p,t]);
pointWasCollectedAtRightPositionY{p in P, d in DRONE,t in T}:  droneDY[d,t]   =  (pointDY[p] * droneCollected[d,p,t]);
pointWasDeliveredAtRightPositionX{p in P, d in DRONE,t in T}:  droneDX[d,t]   =  (pointToDeliverDX[p] * droneDelivered[d,p,t]) ;
pointWasDeliveredAtRightPositionY{p in P, d in DRONE,t in T}:  droneDY[d,t]   =  (pointToDeliverDY[p] * droneDelivered[d,p,t]) ;
updateTotalDistNeg: totalDistNeg = sum{t in T:t>=2} sum{d in DRONE} ( (-droneDX[d,t] + droneDX[d,t-1])*dL + (-droneDY[d,t] + droneDY[d,t-1])*dL );
*/


/*
var droneIsDeliveringDX{DRONE, P, T}, >=0, <= dimSquare;
var droneIsDeliveringDY{DRONE, P, T}, >=0, <= dimSquare;
var droneCollectPoint{DRONE, P}, binary;
var pointCollected{P,T}, binary;
*/





\* Problem: MILP_UAVRouting *\

Minimize
 obj: + totalDist + timeToDeliver + nUsedDrones + dronesMaxSpeed
 + totalExcessOfEnergy

Subject To
 calcTotalDist: - 0.01 diffPosX(D1,2) - 0.01 diffPosX(D2,2)
 - 0.01 diffPosX(D3,2) - 0.01 diffPosX(D4,2) - 0.01 diffPosX(D5,2)
 - 0.01 diffPosX(D1,3) - 0.01 diffPosX(D2,3) - 0.01 diffPosX(D3,3)
 - 0.01 diffPosX(D4,3) - 0.01 diffPosX(D5,3) - 0.01 diffPosX(D1,4)
 - 0.01 diffPosX(D2,4) - 0.01 diffPosX(D3,4) - 0.01 diffPosX(D4,4)
 - 0.01 diffPosX(D5,4) - 0.01 diffPosX(D1,5) - 0.01 diffPosX(D2,5)
 - 0.01 diffPosX(D3,5) - 0.01 diffPosX(D4,5) - 0.01 diffPosX(D5,5)
 - 0.01 diffNegX(D1,2) - 0.01 diffNegX(D2,2) - 0.01 diffNegX(D3,2)
 - 0.01 diffNegX(D4,2) - 0.01 diffNegX(D5,2) - 0.01 diffNegX(D1,3)
 - 0.01 diffNegX(D2,3) - 0.01 diffNegX(D3,3) - 0.01 diffNegX(D4,3)
 - 0.01 diffNegX(D5,3) - 0.01 diffNegX(D1,4) - 0.01 diffNegX(D2,4)
 - 0.01 diffNegX(D3,4) - 0.01 diffNegX(D4,4) - 0.01 diffNegX(D5,4)
 - 0.01 diffNegX(D1,5) - 0.01 diffNegX(D2,5) - 0.01 diffNegX(D3,5)
 - 0.01 diffNegX(D4,5) - 0.01 diffNegX(D5,5) - 0.01 diffPosY(D1,2)
 - 0.01 diffPosY(D2,2) - 0.01 diffPosY(D3,2) - 0.01 diffPosY(D4,2)
 - 0.01 diffPosY(D5,2) - 0.01 diffPosY(D1,3) - 0.01 diffPosY(D2,3)
 - 0.01 diffPosY(D3,3) - 0.01 diffPosY(D4,3) - 0.01 diffPosY(D5,3)
 - 0.01 diffPosY(D1,4) - 0.01 diffPosY(D2,4) - 0.01 diffPosY(D3,4)
 - 0.01 diffPosY(D4,4) - 0.01 diffPosY(D5,4) - 0.01 diffPosY(D1,5)
 - 0.01 diffPosY(D2,5) - 0.01 diffPosY(D3,5) - 0.01 diffPosY(D4,5)
 - 0.01 diffPosY(D5,5) - 0.01 diffNegY(D1,2) - 0.01 diffNegY(D2,2)
 - 0.01 diffNegY(D3,2) - 0.01 diffNegY(D4,2) - 0.01 diffNegY(D5,2)
 - 0.01 diffNegY(D1,3) - 0.01 diffNegY(D2,3) - 0.01 diffNegY(D3,3)
 - 0.01 diffNegY(D4,3) - 0.01 diffNegY(D5,3) - 0.01 diffNegY(D1,4)
 - 0.01 diffNegY(D2,4) - 0.01 diffNegY(D3,4) - 0.01 diffNegY(D4,4)
 - 0.01 diffNegY(D5,4) - 0.01 diffNegY(D1,5) - 0.01 diffNegY(D2,5)
 - 0.01 diffNegY(D3,5) - 0.01 diffNegY(D4,5) - 0.01 diffNegY(D5,5)
 + totalDist = -0
 calcTotalTimeToDeliver: - timePerProductDelivered(P1)
 - timePerProductDelivered(P2) - timePerProductDelivered(P3)
 - timePerProductDelivered(P4) - timePerProductDelivered(P5)
 - timePerProductDelivered(P6) + timeToDeliver = -0
 calcNDrones: - droneIsUsed(D1) - droneIsUsed(D2) - droneIsUsed(D3)
 - droneIsUsed(D4) - droneIsUsed(D5) + nUsedDrones = -0
 calcDronesAVGSpeed(D1,2): - droneSpeed(D1,2) + dronesMaxSpeed >= -0
 calcDronesAVGSpeed(D1,3): - droneSpeed(D1,3) + dronesMaxSpeed >= -0
 calcDronesAVGSpeed(D1,4): - droneSpeed(D1,4) + dronesMaxSpeed >= -0
 calcDronesAVGSpeed(D1,5): - droneSpeed(D1,5) + dronesMaxSpeed >= -0
 calcDronesAVGSpeed(D2,2): - droneSpeed(D2,2) + dronesMaxSpeed >= -0
 calcDronesAVGSpeed(D2,3): - droneSpeed(D2,3) + dronesMaxSpeed >= -0
 calcDronesAVGSpeed(D2,4): - droneSpeed(D2,4) + dronesMaxSpeed >= -0
 calcDronesAVGSpeed(D2,5): - droneSpeed(D2,5) + dronesMaxSpeed >= -0
 calcDronesAVGSpeed(D3,2): - droneSpeed(D3,2) + dronesMaxSpeed >= -0
 calcDronesAVGSpeed(D3,3): - droneSpeed(D3,3) + dronesMaxSpeed >= -0
 calcDronesAVGSpeed(D3,4): - droneSpeed(D3,4) + dronesMaxSpeed >= -0
 calcDronesAVGSpeed(D3,5): - droneSpeed(D3,5) + dronesMaxSpeed >= -0
 calcDronesAVGSpeed(D4,2): - droneSpeed(D4,2) + dronesMaxSpeed >= -0
 calcDronesAVGSpeed(D4,3): - droneSpeed(D4,3) + dronesMaxSpeed >= -0
 calcDronesAVGSpeed(D4,4): - droneSpeed(D4,4) + dronesMaxSpeed >= -0
 calcDronesAVGSpeed(D4,5): - droneSpeed(D4,5) + dronesMaxSpeed >= -0
 calcDronesAVGSpeed(D5,2): - droneSpeed(D5,2) + dronesMaxSpeed >= -0
 calcDronesAVGSpeed(D5,3): - droneSpeed(D5,3) + dronesMaxSpeed >= -0
 calcDronesAVGSpeed(D5,4): - droneSpeed(D5,4) + dronesMaxSpeed >= -0
 calcDronesAVGSpeed(D5,5): - droneSpeed(D5,5) + dronesMaxSpeed >= -0
 calcBatteriesOverLoad: - excessOfEnergy(D1,1) - excessOfEnergy(D1,2)
 - excessOfEnergy(D1,3) - excessOfEnergy(D1,4) - excessOfEnergy(D1,5)
 - excessOfEnergy(D2,1) - excessOfEnergy(D2,2) - excessOfEnergy(D2,3)
 - excessOfEnergy(D2,4) - excessOfEnergy(D2,5) - excessOfEnergy(D3,1)
 - excessOfEnergy(D3,2) - excessOfEnergy(D3,3) - excessOfEnergy(D3,4)
 - excessOfEnergy(D3,5) - excessOfEnergy(D4,1) - excessOfEnergy(D4,2)
 - excessOfEnergy(D4,3) - excessOfEnergy(D4,4) - excessOfEnergy(D4,5)
 - excessOfEnergy(D5,1) - excessOfEnergy(D5,2) - excessOfEnergy(D5,3)
 - excessOfEnergy(D5,4) - excessOfEnergy(D5,5) + totalExcessOfEnergy
 = -0
 respectDroneDoD(D1,2): + droneBatteryRate(D1,2) >= 30
 respectDroneDoD(D1,3): + droneBatteryRate(D1,3) >= 30
 respectDroneDoD(D1,4): + droneBatteryRate(D1,4) >= 30
 respectDroneDoD(D1,5): + droneBatteryRate(D1,5) >= 30
 respectDroneDoD(D2,2): + droneBatteryRate(D2,2) >= 20
 respectDroneDoD(D2,3): + droneBatteryRate(D2,3) >= 20
 respectDroneDoD(D2,4): + droneBatteryRate(D2,4) >= 20
 respectDroneDoD(D2,5): + droneBatteryRate(D2,5) >= 20
 respectDroneDoD(D3,2): + droneBatteryRate(D3,2) >= 20
 respectDroneDoD(D3,3): + droneBatteryRate(D3,3) >= 20
 respectDroneDoD(D3,4): + droneBatteryRate(D3,4) >= 20
 respectDroneDoD(D3,5): + droneBatteryRate(D3,5) >= 20
 respectDroneDoD(D4,2): + droneBatteryRate(D4,2) >= 20
 respectDroneDoD(D4,3): + droneBatteryRate(D4,3) >= 20
 respectDroneDoD(D4,4): + droneBatteryRate(D4,4) >= 20
 respectDroneDoD(D4,5): + droneBatteryRate(D4,5) >= 20
 respectDroneDoD(D5,2): + droneBatteryRate(D5,2) >= 10
 respectDroneDoD(D5,3): + droneBatteryRate(D5,3) >= 10
 respectDroneDoD(D5,4): + droneBatteryRate(D5,4) >= 10
 respectDroneDoD(D5,5): + droneBatteryRate(D5,5) >= 10
 respectDroneCap(D1,1): + droneCurrentCap(D1,1) <= 1.8
 respectDroneCap(D1,2): + droneCurrentCap(D1,2) <= 1.8
 respectDroneCap(D1,3): + droneCurrentCap(D1,3) <= 1.8
 respectDroneCap(D1,4): + droneCurrentCap(D1,4) <= 1.8
 respectDroneCap(D1,5): + droneCurrentCap(D1,5) <= 1.8
 respectDroneCap(D2,1): + droneCurrentCap(D2,1) <= 1
 respectDroneCap(D2,2): + droneCurrentCap(D2,2) <= 1
 respectDroneCap(D2,3): + droneCurrentCap(D2,3) <= 1
 respectDroneCap(D2,4): + droneCurrentCap(D2,4) <= 1
 respectDroneCap(D2,5): + droneCurrentCap(D2,5) <= 1
 respectDroneCap(D3,1): + droneCurrentCap(D3,1) <= 0.3
 respectDroneCap(D3,2): + droneCurrentCap(D3,2) <= 0.3
 respectDroneCap(D3,3): + droneCurrentCap(D3,3) <= 0.3
 respectDroneCap(D3,4): + droneCurrentCap(D3,4) <= 0.3
 respectDroneCap(D3,5): + droneCurrentCap(D3,5) <= 0.3
 respectDroneCap(D4,1): + droneCurrentCap(D4,1) <= 0.2
 respectDroneCap(D4,2): + droneCurrentCap(D4,2) <= 0.2
 respectDroneCap(D4,3): + droneCurrentCap(D4,3) <= 0.2
 respectDroneCap(D4,4): + droneCurrentCap(D4,4) <= 0.2
 respectDroneCap(D4,5): + droneCurrentCap(D4,5) <= 0.2
 respectDroneCap(D5,1): + droneCurrentCap(D5,1) <= 0.6
 respectDroneCap(D5,2): + droneCurrentCap(D5,2) <= 0.6
 respectDroneCap(D5,3): + droneCurrentCap(D5,3) <= 0.6
 respectDroneCap(D5,4): + droneCurrentCap(D5,4) <= 0.6
 respectDroneCap(D5,5): + droneCurrentCap(D5,5) <= 0.6
 updatedCAP(D1,2): - droneCurrentCap(D1,1) + droneCurrentCap(D1,2)
 - droneCollected(D1,P1,2) - droneCollected(D1,P2,2)
 - 0.5 droneCollected(D1,P3,2) - 0.2 droneCollected(D1,P4,2)
 - 0.1 droneCollected(D1,P5,2) - 0.3 droneCollected(D1,P6,2)
 + droneDelivered(D1,P1,2) + droneDelivered(D1,P2,2)
 + 0.5 droneDelivered(D1,P3,2) + 0.2 droneDelivered(D1,P4,2)
 + 0.1 droneDelivered(D1,P5,2) + 0.3 droneDelivered(D1,P6,2) = -0
 updatedCAP(D1,3): - droneCurrentCap(D1,2) + droneCurrentCap(D1,3)
 - droneCollected(D1,P1,3) - droneCollected(D1,P2,3)
 - 0.5 droneCollected(D1,P3,3) - 0.2 droneCollected(D1,P4,3)
 - 0.1 droneCollected(D1,P5,3) - 0.3 droneCollected(D1,P6,3)
 + droneDelivered(D1,P1,3) + droneDelivered(D1,P2,3)
 + 0.5 droneDelivered(D1,P3,3) + 0.2 droneDelivered(D1,P4,3)
 + 0.1 droneDelivered(D1,P5,3) + 0.3 droneDelivered(D1,P6,3) = -0
 updatedCAP(D1,4): - droneCurrentCap(D1,3) + droneCurrentCap(D1,4)
 - droneCollected(D1,P1,4) - droneCollected(D1,P2,4)
 - 0.5 droneCollected(D1,P3,4) - 0.2 droneCollected(D1,P4,4)
 - 0.1 droneCollected(D1,P5,4) - 0.3 droneCollected(D1,P6,4)
 + droneDelivered(D1,P1,4) + droneDelivered(D1,P2,4)
 + 0.5 droneDelivered(D1,P3,4) + 0.2 droneDelivered(D1,P4,4)
 + 0.1 droneDelivered(D1,P5,4) + 0.3 droneDelivered(D1,P6,4) = -0
 updatedCAP(D1,5): - droneCurrentCap(D1,4) + droneCurrentCap(D1,5)
 - droneCollected(D1,P1,5) - droneCollected(D1,P2,5)
 - 0.5 droneCollected(D1,P3,5) - 0.2 droneCollected(D1,P4,5)
 - 0.1 droneCollected(D1,P5,5) - 0.3 droneCollected(D1,P6,5)
 + droneDelivered(D1,P1,5) + droneDelivered(D1,P2,5)
 + 0.5 droneDelivered(D1,P3,5) + 0.2 droneDelivered(D1,P4,5)
 + 0.1 droneDelivered(D1,P5,5) + 0.3 droneDelivered(D1,P6,5) = -0
 updatedCAP(D2,2): - droneCurrentCap(D2,1) + droneCurrentCap(D2,2)
 - droneCollected(D2,P1,2) - droneCollected(D2,P2,2)
 - 0.5 droneCollected(D2,P3,2) - 0.2 droneCollected(D2,P4,2)
 - 0.1 droneCollected(D2,P5,2) - 0.3 droneCollected(D2,P6,2)
 + droneDelivered(D2,P1,2) + droneDelivered(D2,P2,2)
 + 0.5 droneDelivered(D2,P3,2) + 0.2 droneDelivered(D2,P4,2)
 + 0.1 droneDelivered(D2,P5,2) + 0.3 droneDelivered(D2,P6,2) = -0
 updatedCAP(D2,3): - droneCurrentCap(D2,2) + droneCurrentCap(D2,3)
 - droneCollected(D2,P1,3) - droneCollected(D2,P2,3)
 - 0.5 droneCollected(D2,P3,3) - 0.2 droneCollected(D2,P4,3)
 - 0.1 droneCollected(D2,P5,3) - 0.3 droneCollected(D2,P6,3)
 + droneDelivered(D2,P1,3) + droneDelivered(D2,P2,3)
 + 0.5 droneDelivered(D2,P3,3) + 0.2 droneDelivered(D2,P4,3)
 + 0.1 droneDelivered(D2,P5,3) + 0.3 droneDelivered(D2,P6,3) = -0
 updatedCAP(D2,4): - droneCurrentCap(D2,3) + droneCurrentCap(D2,4)
 - droneCollected(D2,P1,4) - droneCollected(D2,P2,4)
 - 0.5 droneCollected(D2,P3,4) - 0.2 droneCollected(D2,P4,4)
 - 0.1 droneCollected(D2,P5,4) - 0.3 droneCollected(D2,P6,4)
 + droneDelivered(D2,P1,4) + droneDelivered(D2,P2,4)
 + 0.5 droneDelivered(D2,P3,4) + 0.2 droneDelivered(D2,P4,4)
 + 0.1 droneDelivered(D2,P5,4) + 0.3 droneDelivered(D2,P6,4) = -0
 updatedCAP(D2,5): - droneCurrentCap(D2,4) + droneCurrentCap(D2,5)
 - droneCollected(D2,P1,5) - droneCollected(D2,P2,5)
 - 0.5 droneCollected(D2,P3,5) - 0.2 droneCollected(D2,P4,5)
 - 0.1 droneCollected(D2,P5,5) - 0.3 droneCollected(D2,P6,5)
 + droneDelivered(D2,P1,5) + droneDelivered(D2,P2,5)
 + 0.5 droneDelivered(D2,P3,5) + 0.2 droneDelivered(D2,P4,5)
 + 0.1 droneDelivered(D2,P5,5) + 0.3 droneDelivered(D2,P6,5) = -0
 updatedCAP(D3,2): - droneCurrentCap(D3,1) + droneCurrentCap(D3,2)
 - droneCollected(D3,P1,2) - droneCollected(D3,P2,2)
 - 0.5 droneCollected(D3,P3,2) - 0.2 droneCollected(D3,P4,2)
 - 0.1 droneCollected(D3,P5,2) - 0.3 droneCollected(D3,P6,2)
 + droneDelivered(D3,P1,2) + droneDelivered(D3,P2,2)
 + 0.5 droneDelivered(D3,P3,2) + 0.2 droneDelivered(D3,P4,2)
 + 0.1 droneDelivered(D3,P5,2) + 0.3 droneDelivered(D3,P6,2) = -0
 updatedCAP(D3,3): - droneCurrentCap(D3,2) + droneCurrentCap(D3,3)
 - droneCollected(D3,P1,3) - droneCollected(D3,P2,3)
 - 0.5 droneCollected(D3,P3,3) - 0.2 droneCollected(D3,P4,3)
 - 0.1 droneCollected(D3,P5,3) - 0.3 droneCollected(D3,P6,3)
 + droneDelivered(D3,P1,3) + droneDelivered(D3,P2,3)
 + 0.5 droneDelivered(D3,P3,3) + 0.2 droneDelivered(D3,P4,3)
 + 0.1 droneDelivered(D3,P5,3) + 0.3 droneDelivered(D3,P6,3) = -0
 updatedCAP(D3,4): - droneCurrentCap(D3,3) + droneCurrentCap(D3,4)
 - droneCollected(D3,P1,4) - droneCollected(D3,P2,4)
 - 0.5 droneCollected(D3,P3,4) - 0.2 droneCollected(D3,P4,4)
 - 0.1 droneCollected(D3,P5,4) - 0.3 droneCollected(D3,P6,4)
 + droneDelivered(D3,P1,4) + droneDelivered(D3,P2,4)
 + 0.5 droneDelivered(D3,P3,4) + 0.2 droneDelivered(D3,P4,4)
 + 0.1 droneDelivered(D3,P5,4) + 0.3 droneDelivered(D3,P6,4) = -0
 updatedCAP(D3,5): - droneCurrentCap(D3,4) + droneCurrentCap(D3,5)
 - droneCollected(D3,P1,5) - droneCollected(D3,P2,5)
 - 0.5 droneCollected(D3,P3,5) - 0.2 droneCollected(D3,P4,5)
 - 0.1 droneCollected(D3,P5,5) - 0.3 droneCollected(D3,P6,5)
 + droneDelivered(D3,P1,5) + droneDelivered(D3,P2,5)
 + 0.5 droneDelivered(D3,P3,5) + 0.2 droneDelivered(D3,P4,5)
 + 0.1 droneDelivered(D3,P5,5) + 0.3 droneDelivered(D3,P6,5) = -0
 updatedCAP(D4,2): - droneCurrentCap(D4,1) + droneCurrentCap(D4,2)
 - droneCollected(D4,P1,2) - droneCollected(D4,P2,2)
 - 0.5 droneCollected(D4,P3,2) - 0.2 droneCollected(D4,P4,2)
 - 0.1 droneCollected(D4,P5,2) - 0.3 droneCollected(D4,P6,2)
 + droneDelivered(D4,P1,2) + droneDelivered(D4,P2,2)
 + 0.5 droneDelivered(D4,P3,2) + 0.2 droneDelivered(D4,P4,2)
 + 0.1 droneDelivered(D4,P5,2) + 0.3 droneDelivered(D4,P6,2) = -0
 updatedCAP(D4,3): - droneCurrentCap(D4,2) + droneCurrentCap(D4,3)
 - droneCollected(D4,P1,3) - droneCollected(D4,P2,3)
 - 0.5 droneCollected(D4,P3,3) - 0.2 droneCollected(D4,P4,3)
 - 0.1 droneCollected(D4,P5,3) - 0.3 droneCollected(D4,P6,3)
 + droneDelivered(D4,P1,3) + droneDelivered(D4,P2,3)
 + 0.5 droneDelivered(D4,P3,3) + 0.2 droneDelivered(D4,P4,3)
 + 0.1 droneDelivered(D4,P5,3) + 0.3 droneDelivered(D4,P6,3) = -0
 updatedCAP(D4,4): - droneCurrentCap(D4,3) + droneCurrentCap(D4,4)
 - droneCollected(D4,P1,4) - droneCollected(D4,P2,4)
 - 0.5 droneCollected(D4,P3,4) - 0.2 droneCollected(D4,P4,4)
 - 0.1 droneCollected(D4,P5,4) - 0.3 droneCollected(D4,P6,4)
 + droneDelivered(D4,P1,4) + droneDelivered(D4,P2,4)
 + 0.5 droneDelivered(D4,P3,4) + 0.2 droneDelivered(D4,P4,4)
 + 0.1 droneDelivered(D4,P5,4) + 0.3 droneDelivered(D4,P6,4) = -0
 updatedCAP(D4,5): - droneCurrentCap(D4,4) + droneCurrentCap(D4,5)
 - droneCollected(D4,P1,5) - droneCollected(D4,P2,5)
 - 0.5 droneCollected(D4,P3,5) - 0.2 droneCollected(D4,P4,5)
 - 0.1 droneCollected(D4,P5,5) - 0.3 droneCollected(D4,P6,5)
 + droneDelivered(D4,P1,5) + droneDelivered(D4,P2,5)
 + 0.5 droneDelivered(D4,P3,5) + 0.2 droneDelivered(D4,P4,5)
 + 0.1 droneDelivered(D4,P5,5) + 0.3 droneDelivered(D4,P6,5) = -0
 updatedCAP(D5,2): - droneCurrentCap(D5,1) + droneCurrentCap(D5,2)
 - droneCollected(D5,P1,2) - droneCollected(D5,P2,2)
 - 0.5 droneCollected(D5,P3,2) - 0.2 droneCollected(D5,P4,2)
 - 0.1 droneCollected(D5,P5,2) - 0.3 droneCollected(D5,P6,2)
 + droneDelivered(D5,P1,2) + droneDelivered(D5,P2,2)
 + 0.5 droneDelivered(D5,P3,2) + 0.2 droneDelivered(D5,P4,2)
 + 0.1 droneDelivered(D5,P5,2) + 0.3 droneDelivered(D5,P6,2) = -0
 updatedCAP(D5,3): - droneCurrentCap(D5,2) + droneCurrentCap(D5,3)
 - droneCollected(D5,P1,3) - droneCollected(D5,P2,3)
 - 0.5 droneCollected(D5,P3,3) - 0.2 droneCollected(D5,P4,3)
 - 0.1 droneCollected(D5,P5,3) - 0.3 droneCollected(D5,P6,3)
 + droneDelivered(D5,P1,3) + droneDelivered(D5,P2,3)
 + 0.5 droneDelivered(D5,P3,3) + 0.2 droneDelivered(D5,P4,3)
 + 0.1 droneDelivered(D5,P5,3) + 0.3 droneDelivered(D5,P6,3) = -0
 updatedCAP(D5,4): - droneCurrentCap(D5,3) + droneCurrentCap(D5,4)
 - droneCollected(D5,P1,4) - droneCollected(D5,P2,4)
 - 0.5 droneCollected(D5,P3,4) - 0.2 droneCollected(D5,P4,4)
 - 0.1 droneCollected(D5,P5,4) - 0.3 droneCollected(D5,P6,4)
 + droneDelivered(D5,P1,4) + droneDelivered(D5,P2,4)
 + 0.5 droneDelivered(D5,P3,4) + 0.2 droneDelivered(D5,P4,4)
 + 0.1 droneDelivered(D5,P5,4) + 0.3 droneDelivered(D5,P6,4) = -0
 updatedCAP(D5,5): - droneCurrentCap(D5,4) + droneCurrentCap(D5,5)
 - droneCollected(D5,P1,5) - droneCollected(D5,P2,5)
 - 0.5 droneCollected(D5,P3,5) - 0.2 droneCollected(D5,P4,5)
 - 0.1 droneCollected(D5,P5,5) - 0.3 droneCollected(D5,P6,5)
 + droneDelivered(D5,P1,5) + droneDelivered(D5,P2,5)
 + 0.5 droneDelivered(D5,P3,5) + 0.2 droneDelivered(D5,P4,5)
 + 0.1 droneDelivered(D5,P5,5) + 0.3 droneDelivered(D5,P6,5) = -0
 updatedCAPAtBegin(D1,1): + droneCurrentCap(D1,1) = 1
 updatedCAPAtBegin(D2,1): + droneCurrentCap(D2,1) = -0
 updatedCAPAtBegin(D3,1): + droneCurrentCap(D3,1) = -0
 updatedCAPAtBegin(D4,1): + droneCurrentCap(D4,1) = -0
 updatedCAPAtBegin(D5,1): + droneCurrentCap(D5,1) = -0
 droneShouldCollect(P1): + droneCollected(D1,P1,2)
 + droneCollected(D1,P1,3) + droneCollected(D1,P1,4)
 + droneCollected(D1,P1,5) + droneCollected(D2,P1,2)
 + droneCollected(D2,P1,3) + droneCollected(D2,P1,4)
 + droneCollected(D2,P1,5) + droneCollected(D3,P1,2)
 + droneCollected(D3,P1,3) + droneCollected(D3,P1,4)
 + droneCollected(D3,P1,5) + droneCollected(D4,P1,2)
 + droneCollected(D4,P1,3) + droneCollected(D4,P1,4)
 + droneCollected(D4,P1,5) + droneCollected(D5,P1,2)
 + droneCollected(D5,P1,3) + droneCollected(D5,P1,4)
 + droneCollected(D5,P1,5) + droneCollected(D1,P1,1)
 + droneCollected(D2,P1,1) + droneCollected(D3,P1,1)
 + droneCollected(D4,P1,1) + droneCollected(D5,P1,1) >= -0
 droneShouldCollect(P2): + droneCollected(D1,P2,2)
 + droneCollected(D1,P2,3) + droneCollected(D1,P2,4)
 + droneCollected(D1,P2,5) + droneCollected(D2,P2,2)
 + droneCollected(D2,P2,3) + droneCollected(D2,P2,4)
 + droneCollected(D2,P2,5) + droneCollected(D3,P2,2)
 + droneCollected(D3,P2,3) + droneCollected(D3,P2,4)
 + droneCollected(D3,P2,5) + droneCollected(D4,P2,2)
 + droneCollected(D4,P2,3) + droneCollected(D4,P2,4)
 + droneCollected(D4,P2,5) + droneCollected(D5,P2,2)
 + droneCollected(D5,P2,3) + droneCollected(D5,P2,4)
 + droneCollected(D5,P2,5) + droneCollected(D1,P2,1)
 + droneCollected(D2,P2,1) + droneCollected(D3,P2,1)
 + droneCollected(D4,P2,1) + droneCollected(D5,P2,1) >= 1
 droneShouldCollect(P3): + 0.5 droneCollected(D1,P3,2)
 + 0.5 droneCollected(D1,P3,3) + 0.5 droneCollected(D1,P3,4)
 + 0.5 droneCollected(D1,P3,5) + 0.5 droneCollected(D2,P3,2)
 + 0.5 droneCollected(D2,P3,3) + 0.5 droneCollected(D2,P3,4)
 + 0.5 droneCollected(D2,P3,5) + 0.5 droneCollected(D3,P3,2)
 + 0.5 droneCollected(D3,P3,3) + 0.5 droneCollected(D3,P3,4)
 + 0.5 droneCollected(D3,P3,5) + 0.5 droneCollected(D4,P3,2)
 + 0.5 droneCollected(D4,P3,3) + 0.5 droneCollected(D4,P3,4)
 + 0.5 droneCollected(D4,P3,5) + 0.5 droneCollected(D5,P3,2)
 + 0.5 droneCollected(D5,P3,3) + 0.5 droneCollected(D5,P3,4)
 + 0.5 droneCollected(D5,P3,5) + 0.5 droneCollected(D1,P3,1)
 + 0.5 droneCollected(D2,P3,1) + 0.5 droneCollected(D3,P3,1)
 + 0.5 droneCollected(D4,P3,1) + 0.5 droneCollected(D5,P3,1) >= 0.5
 droneShouldCollect(P4): + 0.2 droneCollected(D1,P4,2)
 + 0.2 droneCollected(D1,P4,3) + 0.2 droneCollected(D1,P4,4)
 + 0.2 droneCollected(D1,P4,5) + 0.2 droneCollected(D2,P4,2)
 + 0.2 droneCollected(D2,P4,3) + 0.2 droneCollected(D2,P4,4)
 + 0.2 droneCollected(D2,P4,5) + 0.2 droneCollected(D3,P4,2)
 + 0.2 droneCollected(D3,P4,3) + 0.2 droneCollected(D3,P4,4)
 + 0.2 droneCollected(D3,P4,5) + 0.2 droneCollected(D4,P4,2)
 + 0.2 droneCollected(D4,P4,3) + 0.2 droneCollected(D4,P4,4)
 + 0.2 droneCollected(D4,P4,5) + 0.2 droneCollected(D5,P4,2)
 + 0.2 droneCollected(D5,P4,3) + 0.2 droneCollected(D5,P4,4)
 + 0.2 droneCollected(D5,P4,5) + 0.2 droneCollected(D1,P4,1)
 + 0.2 droneCollected(D2,P4,1) + 0.2 droneCollected(D3,P4,1)
 + 0.2 droneCollected(D4,P4,1) + 0.2 droneCollected(D5,P4,1) >= 0.2
 droneShouldCollect(P5): + 0.1 droneCollected(D1,P5,2)
 + 0.1 droneCollected(D1,P5,3) + 0.1 droneCollected(D1,P5,4)
 + 0.1 droneCollected(D1,P5,5) + 0.1 droneCollected(D2,P5,2)
 + 0.1 droneCollected(D2,P5,3) + 0.1 droneCollected(D2,P5,4)
 + 0.1 droneCollected(D2,P5,5) + 0.1 droneCollected(D3,P5,2)
 + 0.1 droneCollected(D3,P5,3) + 0.1 droneCollected(D3,P5,4)
 + 0.1 droneCollected(D3,P5,5) + 0.1 droneCollected(D4,P5,2)
 + 0.1 droneCollected(D4,P5,3) + 0.1 droneCollected(D4,P5,4)
 + 0.1 droneCollected(D4,P5,5) + 0.1 droneCollected(D5,P5,2)
 + 0.1 droneCollected(D5,P5,3) + 0.1 droneCollected(D5,P5,4)
 + 0.1 droneCollected(D5,P5,5) + 0.1 droneCollected(D1,P5,1)
 + 0.1 droneCollected(D2,P5,1) + 0.1 droneCollected(D3,P5,1)
 + 0.1 droneCollected(D4,P5,1) + 0.1 droneCollected(D5,P5,1) >= 0.1
 droneShouldCollect(P6): + 0.3 droneCollected(D1,P6,2)
 + 0.3 droneCollected(D1,P6,3) + 0.3 droneCollected(D1,P6,4)
 + 0.3 droneCollected(D1,P6,5) + 0.3 droneCollected(D2,P6,2)
 + 0.3 droneCollected(D2,P6,3) + 0.3 droneCollected(D2,P6,4)
 + 0.3 droneCollected(D2,P6,5) + 0.3 droneCollected(D3,P6,2)
 + 0.3 droneCollected(D3,P6,3) + 0.3 droneCollected(D3,P6,4)
 + 0.3 droneCollected(D3,P6,5) + 0.3 droneCollected(D4,P6,2)
 + 0.3 droneCollected(D4,P6,3) + 0.3 droneCollected(D4,P6,4)
 + 0.3 droneCollected(D4,P6,5) + 0.3 droneCollected(D5,P6,2)
 + 0.3 droneCollected(D5,P6,3) + 0.3 droneCollected(D5,P6,4)
 + 0.3 droneCollected(D5,P6,5) + 0.3 droneCollected(D1,P6,1)
 + 0.3 droneCollected(D2,P6,1) + 0.3 droneCollected(D3,P6,1)
 + 0.3 droneCollected(D4,P6,1) + 0.3 droneCollected(D5,P6,1) >= 0.3
 droneShouldCollectAlone(P1): + droneCollected(D1,P1,2)
 + droneCollected(D1,P1,3) + droneCollected(D1,P1,4)
 + droneCollected(D1,P1,5) + droneCollected(D2,P1,2)
 + droneCollected(D2,P1,3) + droneCollected(D2,P1,4)
 + droneCollected(D2,P1,5) + droneCollected(D3,P1,2)
 + droneCollected(D3,P1,3) + droneCollected(D3,P1,4)
 + droneCollected(D3,P1,5) + droneCollected(D4,P1,2)
 + droneCollected(D4,P1,3) + droneCollected(D4,P1,4)
 + droneCollected(D4,P1,5) + droneCollected(D5,P1,2)
 + droneCollected(D5,P1,3) + droneCollected(D5,P1,4)
 + droneCollected(D5,P1,5) + droneCollected(D1,P1,1)
 + droneCollected(D2,P1,1) + droneCollected(D3,P1,1)
 + droneCollected(D4,P1,1) + droneCollected(D5,P1,1) = -0
 droneShouldCollectAlone(P2): + droneCollected(D1,P2,2)
 + droneCollected(D1,P2,3) + droneCollected(D1,P2,4)
 + droneCollected(D1,P2,5) + droneCollected(D2,P2,2)
 + droneCollected(D2,P2,3) + droneCollected(D2,P2,4)
 + droneCollected(D2,P2,5) + droneCollected(D3,P2,2)
 + droneCollected(D3,P2,3) + droneCollected(D3,P2,4)
 + droneCollected(D3,P2,5) + droneCollected(D4,P2,2)
 + droneCollected(D4,P2,3) + droneCollected(D4,P2,4)
 + droneCollected(D4,P2,5) + droneCollected(D5,P2,2)
 + droneCollected(D5,P2,3) + droneCollected(D5,P2,4)
 + droneCollected(D5,P2,5) + droneCollected(D1,P2,1)
 + droneCollected(D2,P2,1) + droneCollected(D3,P2,1)
 + droneCollected(D4,P2,1) + droneCollected(D5,P2,1) = 1
 droneShouldCollectAlone(P3): + droneCollected(D1,P3,2)
 + droneCollected(D1,P3,3) + droneCollected(D1,P3,4)
 + droneCollected(D1,P3,5) + droneCollected(D2,P3,2)
 + droneCollected(D2,P3,3) + droneCollected(D2,P3,4)
 + droneCollected(D2,P3,5) + droneCollected(D3,P3,2)
 + droneCollected(D3,P3,3) + droneCollected(D3,P3,4)
 + droneCollected(D3,P3,5) + droneCollected(D4,P3,2)
 + droneCollected(D4,P3,3) + droneCollected(D4,P3,4)
 + droneCollected(D4,P3,5) + droneCollected(D5,P3,2)
 + droneCollected(D5,P3,3) + droneCollected(D5,P3,4)
 + droneCollected(D5,P3,5) + droneCollected(D1,P3,1)
 + droneCollected(D2,P3,1) + droneCollected(D3,P3,1)
 + droneCollected(D4,P3,1) + droneCollected(D5,P3,1) = 1
 droneShouldCollectAlone(P4): + droneCollected(D1,P4,2)
 + droneCollected(D1,P4,3) + droneCollected(D1,P4,4)
 + droneCollected(D1,P4,5) + droneCollected(D2,P4,2)
 + droneCollected(D2,P4,3) + droneCollected(D2,P4,4)
 + droneCollected(D2,P4,5) + droneCollected(D3,P4,2)
 + droneCollected(D3,P4,3) + droneCollected(D3,P4,4)
 + droneCollected(D3,P4,5) + droneCollected(D4,P4,2)
 + droneCollected(D4,P4,3) + droneCollected(D4,P4,4)
 + droneCollected(D4,P4,5) + droneCollected(D5,P4,2)
 + droneCollected(D5,P4,3) + droneCollected(D5,P4,4)
 + droneCollected(D5,P4,5) + droneCollected(D1,P4,1)
 + droneCollected(D2,P4,1) + droneCollected(D3,P4,1)
 + droneCollected(D4,P4,1) + droneCollected(D5,P4,1) = 1
 droneShouldCollectAlone(P5): + droneCollected(D1,P5,2)
 + droneCollected(D1,P5,3) + droneCollected(D1,P5,4)
 + droneCollected(D1,P5,5) + droneCollected(D2,P5,2)
 + droneCollected(D2,P5,3) + droneCollected(D2,P5,4)
 + droneCollected(D2,P5,5) + droneCollected(D3,P5,2)
 + droneCollected(D3,P5,3) + droneCollected(D3,P5,4)
 + droneCollected(D3,P5,5) + droneCollected(D4,P5,2)
 + droneCollected(D4,P5,3) + droneCollected(D4,P5,4)
 + droneCollected(D4,P5,5) + droneCollected(D5,P5,2)
 + droneCollected(D5,P5,3) + droneCollected(D5,P5,4)
 + droneCollected(D5,P5,5) + droneCollected(D1,P5,1)
 + droneCollected(D2,P5,1) + droneCollected(D3,P5,1)
 + droneCollected(D4,P5,1) + droneCollected(D5,P5,1) = 1
 droneShouldCollectAlone(P6): + droneCollected(D1,P6,2)
 + droneCollected(D1,P6,3) + droneCollected(D1,P6,4)
 + droneCollected(D1,P6,5) + droneCollected(D2,P6,2)
 + droneCollected(D2,P6,3) + droneCollected(D2,P6,4)
 + droneCollected(D2,P6,5) + droneCollected(D3,P6,2)
 + droneCollected(D3,P6,3) + droneCollected(D3,P6,4)
 + droneCollected(D3,P6,5) + droneCollected(D4,P6,2)
 + droneCollected(D4,P6,3) + droneCollected(D4,P6,4)
 + droneCollected(D4,P6,5) + droneCollected(D5,P6,2)
 + droneCollected(D5,P6,3) + droneCollected(D5,P6,4)
 + droneCollected(D5,P6,5) + droneCollected(D1,P6,1)
 + droneCollected(D2,P6,1) + droneCollected(D3,P6,1)
 + droneCollected(D4,P6,1) + droneCollected(D5,P6,1) = 1
 droneShouldCollectAndDeliver(P1,D1): - droneCollected(D1,P1,2)
 - droneCollected(D1,P1,3) - droneCollected(D1,P1,4)
 - droneCollected(D1,P1,5) - droneCollected(D1,P1,1)
 + droneDelivered(D1,P1,2) + droneDelivered(D1,P1,3)
 + droneDelivered(D1,P1,4) + droneDelivered(D1,P1,5)
 + droneDelivered(D1,P1,1) = 1
 droneShouldCollectAndDeliver(P1,D2): - droneCollected(D2,P1,2)
 - droneCollected(D2,P1,3) - droneCollected(D2,P1,4)
 - droneCollected(D2,P1,5) - droneCollected(D2,P1,1)
 + droneDelivered(D2,P1,2) + droneDelivered(D2,P1,3)
 + droneDelivered(D2,P1,4) + droneDelivered(D2,P1,5)
 + droneDelivered(D2,P1,1) = -0
 droneShouldCollectAndDeliver(P1,D3): - droneCollected(D3,P1,2)
 - droneCollected(D3,P1,3) - droneCollected(D3,P1,4)
 - droneCollected(D3,P1,5) - droneCollected(D3,P1,1)
 + droneDelivered(D3,P1,2) + droneDelivered(D3,P1,3)
 + droneDelivered(D3,P1,4) + droneDelivered(D3,P1,5)
 + droneDelivered(D3,P1,1) = -0
 droneShouldCollectAndDeliver(P1,D4): - droneCollected(D4,P1,2)
 - droneCollected(D4,P1,3) - droneCollected(D4,P1,4)
 - droneCollected(D4,P1,5) - droneCollected(D4,P1,1)
 + droneDelivered(D4,P1,2) + droneDelivered(D4,P1,3)
 + droneDelivered(D4,P1,4) + droneDelivered(D4,P1,5)
 + droneDelivered(D4,P1,1) = -0
 droneShouldCollectAndDeliver(P1,D5): - droneCollected(D5,P1,2)
 - droneCollected(D5,P1,3) - droneCollected(D5,P1,4)
 - droneCollected(D5,P1,5) - droneCollected(D5,P1,1)
 + droneDelivered(D5,P1,2) + droneDelivered(D5,P1,3)
 + droneDelivered(D5,P1,4) + droneDelivered(D5,P1,5)
 + droneDelivered(D5,P1,1) = -0
 droneShouldCollectAndDeliver(P2,D1): - droneCollected(D1,P2,2)
 - droneCollected(D1,P2,3) - droneCollected(D1,P2,4)
 - droneCollected(D1,P2,5) - droneCollected(D1,P2,1)
 + droneDelivered(D1,P2,2) + droneDelivered(D1,P2,3)
 + droneDelivered(D1,P2,4) + droneDelivered(D1,P2,5)
 + droneDelivered(D1,P2,1) = -0
 droneShouldCollectAndDeliver(P2,D2): - droneCollected(D2,P2,2)
 - droneCollected(D2,P2,3) - droneCollected(D2,P2,4)
 - droneCollected(D2,P2,5) - droneCollected(D2,P2,1)
 + droneDelivered(D2,P2,2) + droneDelivered(D2,P2,3)
 + droneDelivered(D2,P2,4) + droneDelivered(D2,P2,5)
 + droneDelivered(D2,P2,1) = -0
 droneShouldCollectAndDeliver(P2,D3): - droneCollected(D3,P2,2)
 - droneCollected(D3,P2,3) - droneCollected(D3,P2,4)
 - droneCollected(D3,P2,5) - droneCollected(D3,P2,1)
 + droneDelivered(D3,P2,2) + droneDelivered(D3,P2,3)
 + droneDelivered(D3,P2,4) + droneDelivered(D3,P2,5)
 + droneDelivered(D3,P2,1) = -0
 droneShouldCollectAndDeliver(P2,D4): - droneCollected(D4,P2,2)
 - droneCollected(D4,P2,3) - droneCollected(D4,P2,4)
 - droneCollected(D4,P2,5) - droneCollected(D4,P2,1)
 + droneDelivered(D4,P2,2) + droneDelivered(D4,P2,3)
 + droneDelivered(D4,P2,4) + droneDelivered(D4,P2,5)
 + droneDelivered(D4,P2,1) = -0
 droneShouldCollectAndDeliver(P2,D5): - droneCollected(D5,P2,2)
 - droneCollected(D5,P2,3) - droneCollected(D5,P2,4)
 - droneCollected(D5,P2,5) - droneCollected(D5,P2,1)
 + droneDelivered(D5,P2,2) + droneDelivered(D5,P2,3)
 + droneDelivered(D5,P2,4) + droneDelivered(D5,P2,5)
 + droneDelivered(D5,P2,1) = -0
 droneShouldCollectAndDeliver(P3,D1): - droneCollected(D1,P3,2)
 - droneCollected(D1,P3,3) - droneCollected(D1,P3,4)
 - droneCollected(D1,P3,5) - droneCollected(D1,P3,1)
 + droneDelivered(D1,P3,2) + droneDelivered(D1,P3,3)
 + droneDelivered(D1,P3,4) + droneDelivered(D1,P3,5)
 + droneDelivered(D1,P3,1) = -0
 droneShouldCollectAndDeliver(P3,D2): - droneCollected(D2,P3,2)
 - droneCollected(D2,P3,3) - droneCollected(D2,P3,4)
 - droneCollected(D2,P3,5) - droneCollected(D2,P3,1)
 + droneDelivered(D2,P3,2) + droneDelivered(D2,P3,3)
 + droneDelivered(D2,P3,4) + droneDelivered(D2,P3,5)
 + droneDelivered(D2,P3,1) = -0
 droneShouldCollectAndDeliver(P3,D3): - droneCollected(D3,P3,2)
 - droneCollected(D3,P3,3) - droneCollected(D3,P3,4)
 - droneCollected(D3,P3,5) - droneCollected(D3,P3,1)
 + droneDelivered(D3,P3,2) + droneDelivered(D3,P3,3)
 + droneDelivered(D3,P3,4) + droneDelivered(D3,P3,5)
 + droneDelivered(D3,P3,1) = -0
 droneShouldCollectAndDeliver(P3,D4): - droneCollected(D4,P3,2)
 - droneCollected(D4,P3,3) - droneCollected(D4,P3,4)
 - droneCollected(D4,P3,5) - droneCollected(D4,P3,1)
 + droneDelivered(D4,P3,2) + droneDelivered(D4,P3,3)
 + droneDelivered(D4,P3,4) + droneDelivered(D4,P3,5)
 + droneDelivered(D4,P3,1) = -0
 droneShouldCollectAndDeliver(P3,D5): - droneCollected(D5,P3,2)
 - droneCollected(D5,P3,3) - droneCollected(D5,P3,4)
 - droneCollected(D5,P3,5) - droneCollected(D5,P3,1)
 + droneDelivered(D5,P3,2) + droneDelivered(D5,P3,3)
 + droneDelivered(D5,P3,4) + droneDelivered(D5,P3,5)
 + droneDelivered(D5,P3,1) = -0
 droneShouldCollectAndDeliver(P4,D1): - droneCollected(D1,P4,2)
 - droneCollected(D1,P4,3) - droneCollected(D1,P4,4)
 - droneCollected(D1,P4,5) - droneCollected(D1,P4,1)
 + droneDelivered(D1,P4,2) + droneDelivered(D1,P4,3)
 + droneDelivered(D1,P4,4) + droneDelivered(D1,P4,5)
 + droneDelivered(D1,P4,1) = -0
 droneShouldCollectAndDeliver(P4,D2): - droneCollected(D2,P4,2)
 - droneCollected(D2,P4,3) - droneCollected(D2,P4,4)
 - droneCollected(D2,P4,5) - droneCollected(D2,P4,1)
 + droneDelivered(D2,P4,2) + droneDelivered(D2,P4,3)
 + droneDelivered(D2,P4,4) + droneDelivered(D2,P4,5)
 + droneDelivered(D2,P4,1) = -0
 droneShouldCollectAndDeliver(P4,D3): - droneCollected(D3,P4,2)
 - droneCollected(D3,P4,3) - droneCollected(D3,P4,4)
 - droneCollected(D3,P4,5) - droneCollected(D3,P4,1)
 + droneDelivered(D3,P4,2) + droneDelivered(D3,P4,3)
 + droneDelivered(D3,P4,4) + droneDelivered(D3,P4,5)
 + droneDelivered(D3,P4,1) = -0
 droneShouldCollectAndDeliver(P4,D4): - droneCollected(D4,P4,2)
 - droneCollected(D4,P4,3) - droneCollected(D4,P4,4)
 - droneCollected(D4,P4,5) - droneCollected(D4,P4,1)
 + droneDelivered(D4,P4,2) + droneDelivered(D4,P4,3)
 + droneDelivered(D4,P4,4) + droneDelivered(D4,P4,5)
 + droneDelivered(D4,P4,1) = -0
 droneShouldCollectAndDeliver(P4,D5): - droneCollected(D5,P4,2)
 - droneCollected(D5,P4,3) - droneCollected(D5,P4,4)
 - droneCollected(D5,P4,5) - droneCollected(D5,P4,1)
 + droneDelivered(D5,P4,2) + droneDelivered(D5,P4,3)
 + droneDelivered(D5,P4,4) + droneDelivered(D5,P4,5)
 + droneDelivered(D5,P4,1) = -0
 droneShouldCollectAndDeliver(P5,D1): - droneCollected(D1,P5,2)
 - droneCollected(D1,P5,3) - droneCollected(D1,P5,4)
 - droneCollected(D1,P5,5) - droneCollected(D1,P5,1)
 + droneDelivered(D1,P5,2) + droneDelivered(D1,P5,3)
 + droneDelivered(D1,P5,4) + droneDelivered(D1,P5,5)
 + droneDelivered(D1,P5,1) = -0
 droneShouldCollectAndDeliver(P5,D2): - droneCollected(D2,P5,2)
 - droneCollected(D2,P5,3) - droneCollected(D2,P5,4)
 - droneCollected(D2,P5,5) - droneCollected(D2,P5,1)
 + droneDelivered(D2,P5,2) + droneDelivered(D2,P5,3)
 + droneDelivered(D2,P5,4) + droneDelivered(D2,P5,5)
 + droneDelivered(D2,P5,1) = -0
 droneShouldCollectAndDeliver(P5,D3): - droneCollected(D3,P5,2)
 - droneCollected(D3,P5,3) - droneCollected(D3,P5,4)
 - droneCollected(D3,P5,5) - droneCollected(D3,P5,1)
 + droneDelivered(D3,P5,2) + droneDelivered(D3,P5,3)
 + droneDelivered(D3,P5,4) + droneDelivered(D3,P5,5)
 + droneDelivered(D3,P5,1) = -0
 droneShouldCollectAndDeliver(P5,D4): - droneCollected(D4,P5,2)
 - droneCollected(D4,P5,3) - droneCollected(D4,P5,4)
 - droneCollected(D4,P5,5) - droneCollected(D4,P5,1)
 + droneDelivered(D4,P5,2) + droneDelivered(D4,P5,3)
 + droneDelivered(D4,P5,4) + droneDelivered(D4,P5,5)
 + droneDelivered(D4,P5,1) = -0
 droneShouldCollectAndDeliver(P5,D5): - droneCollected(D5,P5,2)
 - droneCollected(D5,P5,3) - droneCollected(D5,P5,4)
 - droneCollected(D5,P5,5) - droneCollected(D5,P5,1)
 + droneDelivered(D5,P5,2) + droneDelivered(D5,P5,3)
 + droneDelivered(D5,P5,4) + droneDelivered(D5,P5,5)
 + droneDelivered(D5,P5,1) = -0
 droneShouldCollectAndDeliver(P6,D1): - droneCollected(D1,P6,2)
 - droneCollected(D1,P6,3) - droneCollected(D1,P6,4)
 - droneCollected(D1,P6,5) - droneCollected(D1,P6,1)
 + droneDelivered(D1,P6,2) + droneDelivered(D1,P6,3)
 + droneDelivered(D1,P6,4) + droneDelivered(D1,P6,5)
 + droneDelivered(D1,P6,1) = -0
 droneShouldCollectAndDeliver(P6,D2): - droneCollected(D2,P6,2)
 - droneCollected(D2,P6,3) - droneCollected(D2,P6,4)
 - droneCollected(D2,P6,5) - droneCollected(D2,P6,1)
 + droneDelivered(D2,P6,2) + droneDelivered(D2,P6,3)
 + droneDelivered(D2,P6,4) + droneDelivered(D2,P6,5)
 + droneDelivered(D2,P6,1) = -0
 droneShouldCollectAndDeliver(P6,D3): - droneCollected(D3,P6,2)
 - droneCollected(D3,P6,3) - droneCollected(D3,P6,4)
 - droneCollected(D3,P6,5) - droneCollected(D3,P6,1)
 + droneDelivered(D3,P6,2) + droneDelivered(D3,P6,3)
 + droneDelivered(D3,P6,4) + droneDelivered(D3,P6,5)
 + droneDelivered(D3,P6,1) = -0
 droneShouldCollectAndDeliver(P6,D4): - droneCollected(D4,P6,2)
 - droneCollected(D4,P6,3) - droneCollected(D4,P6,4)
 - droneCollected(D4,P6,5) - droneCollected(D4,P6,1)
 + droneDelivered(D4,P6,2) + droneDelivered(D4,P6,3)
 + droneDelivered(D4,P6,4) + droneDelivered(D4,P6,5)
 + droneDelivered(D4,P6,1) = -0
 droneShouldCollectAndDeliver(P6,D5): - droneCollected(D5,P6,2)
 - droneCollected(D5,P6,3) - droneCollected(D5,P6,4)
 - droneCollected(D5,P6,5) - droneCollected(D5,P6,1)
 + droneDelivered(D5,P6,2) + droneDelivered(D5,P6,3)
 + droneDelivered(D5,P6,4) + droneDelivered(D5,P6,5)
 + droneDelivered(D5,P6,1) = -0
 updatedDelivering(P1,D1,2): - droneCollected(D1,P1,2)
 + droneDelivered(D1,P1,2) + droneIsDelivering(D1,P1,2)
 - droneIsDelivering(D1,P1,1) = -0
 updatedDelivering(P1,D1,3): - droneCollected(D1,P1,3)
 + droneDelivered(D1,P1,3) - droneIsDelivering(D1,P1,2)
 + droneIsDelivering(D1,P1,3) = -0
 updatedDelivering(P1,D1,4): - droneCollected(D1,P1,4)
 + droneDelivered(D1,P1,4) - droneIsDelivering(D1,P1,3)
 + droneIsDelivering(D1,P1,4) = -0
 updatedDelivering(P1,D1,5): - droneCollected(D1,P1,5)
 + droneDelivered(D1,P1,5) - droneIsDelivering(D1,P1,4)
 + droneIsDelivering(D1,P1,5) = -0
 updatedDelivering(P1,D2,2): - droneCollected(D2,P1,2)
 + droneDelivered(D2,P1,2) + droneIsDelivering(D2,P1,2)
 - droneIsDelivering(D2,P1,1) = -0
 updatedDelivering(P1,D2,3): - droneCollected(D2,P1,3)
 + droneDelivered(D2,P1,3) - droneIsDelivering(D2,P1,2)
 + droneIsDelivering(D2,P1,3) = -0
 updatedDelivering(P1,D2,4): - droneCollected(D2,P1,4)
 + droneDelivered(D2,P1,4) - droneIsDelivering(D2,P1,3)
 + droneIsDelivering(D2,P1,4) = -0
 updatedDelivering(P1,D2,5): - droneCollected(D2,P1,5)
 + droneDelivered(D2,P1,5) - droneIsDelivering(D2,P1,4)
 + droneIsDelivering(D2,P1,5) = -0
 updatedDelivering(P1,D3,2): - droneCollected(D3,P1,2)
 + droneDelivered(D3,P1,2) + droneIsDelivering(D3,P1,2)
 - droneIsDelivering(D3,P1,1) = -0
 updatedDelivering(P1,D3,3): - droneCollected(D3,P1,3)
 + droneDelivered(D3,P1,3) - droneIsDelivering(D3,P1,2)
 + droneIsDelivering(D3,P1,3) = -0
 updatedDelivering(P1,D3,4): - droneCollected(D3,P1,4)
 + droneDelivered(D3,P1,4) - droneIsDelivering(D3,P1,3)
 + droneIsDelivering(D3,P1,4) = -0
 updatedDelivering(P1,D3,5): - droneCollected(D3,P1,5)
 + droneDelivered(D3,P1,5) - droneIsDelivering(D3,P1,4)
 + droneIsDelivering(D3,P1,5) = -0
 updatedDelivering(P1,D4,2): - droneCollected(D4,P1,2)
 + droneDelivered(D4,P1,2) + droneIsDelivering(D4,P1,2)
 - droneIsDelivering(D4,P1,1) = -0
 updatedDelivering(P1,D4,3): - droneCollected(D4,P1,3)
 + droneDelivered(D4,P1,3) - droneIsDelivering(D4,P1,2)
 + droneIsDelivering(D4,P1,3) = -0
 updatedDelivering(P1,D4,4): - droneCollected(D4,P1,4)
 + droneDelivered(D4,P1,4) - droneIsDelivering(D4,P1,3)
 + droneIsDelivering(D4,P1,4) = -0
 updatedDelivering(P1,D4,5): - droneCollected(D4,P1,5)
 + droneDelivered(D4,P1,5) - droneIsDelivering(D4,P1,4)
 + droneIsDelivering(D4,P1,5) = -0
 updatedDelivering(P1,D5,2): - droneCollected(D5,P1,2)
 + droneDelivered(D5,P1,2) + droneIsDelivering(D5,P1,2)
 - droneIsDelivering(D5,P1,1) = -0
 updatedDelivering(P1,D5,3): - droneCollected(D5,P1,3)
 + droneDelivered(D5,P1,3) - droneIsDelivering(D5,P1,2)
 + droneIsDelivering(D5,P1,3) = -0
 updatedDelivering(P1,D5,4): - droneCollected(D5,P1,4)
 + droneDelivered(D5,P1,4) - droneIsDelivering(D5,P1,3)
 + droneIsDelivering(D5,P1,4) = -0
 updatedDelivering(P1,D5,5): - droneCollected(D5,P1,5)
 + droneDelivered(D5,P1,5) - droneIsDelivering(D5,P1,4)
 + droneIsDelivering(D5,P1,5) = -0
 updatedDelivering(P2,D1,2): - droneCollected(D1,P2,2)
 + droneDelivered(D1,P2,2) + droneIsDelivering(D1,P2,2)
 - droneIsDelivering(D1,P2,1) = -0
 updatedDelivering(P2,D1,3): - droneCollected(D1,P2,3)
 + droneDelivered(D1,P2,3) - droneIsDelivering(D1,P2,2)
 + droneIsDelivering(D1,P2,3) = -0
 updatedDelivering(P2,D1,4): - droneCollected(D1,P2,4)
 + droneDelivered(D1,P2,4) - droneIsDelivering(D1,P2,3)
 + droneIsDelivering(D1,P2,4) = -0
 updatedDelivering(P2,D1,5): - droneCollected(D1,P2,5)
 + droneDelivered(D1,P2,5) - droneIsDelivering(D1,P2,4)
 + droneIsDelivering(D1,P2,5) = -0
 updatedDelivering(P2,D2,2): - droneCollected(D2,P2,2)
 + droneDelivered(D2,P2,2) + droneIsDelivering(D2,P2,2)
 - droneIsDelivering(D2,P2,1) = -0
 updatedDelivering(P2,D2,3): - droneCollected(D2,P2,3)
 + droneDelivered(D2,P2,3) - droneIsDelivering(D2,P2,2)
 + droneIsDelivering(D2,P2,3) = -0
 updatedDelivering(P2,D2,4): - droneCollected(D2,P2,4)
 + droneDelivered(D2,P2,4) - droneIsDelivering(D2,P2,3)
 + droneIsDelivering(D2,P2,4) = -0
 updatedDelivering(P2,D2,5): - droneCollected(D2,P2,5)
 + droneDelivered(D2,P2,5) - droneIsDelivering(D2,P2,4)
 + droneIsDelivering(D2,P2,5) = -0
 updatedDelivering(P2,D3,2): - droneCollected(D3,P2,2)
 + droneDelivered(D3,P2,2) + droneIsDelivering(D3,P2,2)
 - droneIsDelivering(D3,P2,1) = -0
 updatedDelivering(P2,D3,3): - droneCollected(D3,P2,3)
 + droneDelivered(D3,P2,3) - droneIsDelivering(D3,P2,2)
 + droneIsDelivering(D3,P2,3) = -0
 updatedDelivering(P2,D3,4): - droneCollected(D3,P2,4)
 + droneDelivered(D3,P2,4) - droneIsDelivering(D3,P2,3)
 + droneIsDelivering(D3,P2,4) = -0
 updatedDelivering(P2,D3,5): - droneCollected(D3,P2,5)
 + droneDelivered(D3,P2,5) - droneIsDelivering(D3,P2,4)
 + droneIsDelivering(D3,P2,5) = -0
 updatedDelivering(P2,D4,2): - droneCollected(D4,P2,2)
 + droneDelivered(D4,P2,2) + droneIsDelivering(D4,P2,2)
 - droneIsDelivering(D4,P2,1) = -0
 updatedDelivering(P2,D4,3): - droneCollected(D4,P2,3)
 + droneDelivered(D4,P2,3) - droneIsDelivering(D4,P2,2)
 + droneIsDelivering(D4,P2,3) = -0
 updatedDelivering(P2,D4,4): - droneCollected(D4,P2,4)
 + droneDelivered(D4,P2,4) - droneIsDelivering(D4,P2,3)
 + droneIsDelivering(D4,P2,4) = -0
 updatedDelivering(P2,D4,5): - droneCollected(D4,P2,5)
 + droneDelivered(D4,P2,5) - droneIsDelivering(D4,P2,4)
 + droneIsDelivering(D4,P2,5) = -0
 updatedDelivering(P2,D5,2): - droneCollected(D5,P2,2)
 + droneDelivered(D5,P2,2) + droneIsDelivering(D5,P2,2)
 - droneIsDelivering(D5,P2,1) = -0
 updatedDelivering(P2,D5,3): - droneCollected(D5,P2,3)
 + droneDelivered(D5,P2,3) - droneIsDelivering(D5,P2,2)
 + droneIsDelivering(D5,P2,3) = -0
 updatedDelivering(P2,D5,4): - droneCollected(D5,P2,4)
 + droneDelivered(D5,P2,4) - droneIsDelivering(D5,P2,3)
 + droneIsDelivering(D5,P2,4) = -0
 updatedDelivering(P2,D5,5): - droneCollected(D5,P2,5)
 + droneDelivered(D5,P2,5) - droneIsDelivering(D5,P2,4)
 + droneIsDelivering(D5,P2,5) = -0
 updatedDelivering(P3,D1,2): - droneCollected(D1,P3,2)
 + droneDelivered(D1,P3,2) + droneIsDelivering(D1,P3,2)
 - droneIsDelivering(D1,P3,1) = -0
 updatedDelivering(P3,D1,3): - droneCollected(D1,P3,3)
 + droneDelivered(D1,P3,3) - droneIsDelivering(D1,P3,2)
 + droneIsDelivering(D1,P3,3) = -0
 updatedDelivering(P3,D1,4): - droneCollected(D1,P3,4)
 + droneDelivered(D1,P3,4) - droneIsDelivering(D1,P3,3)
 + droneIsDelivering(D1,P3,4) = -0
 updatedDelivering(P3,D1,5): - droneCollected(D1,P3,5)
 + droneDelivered(D1,P3,5) - droneIsDelivering(D1,P3,4)
 + droneIsDelivering(D1,P3,5) = -0
 updatedDelivering(P3,D2,2): - droneCollected(D2,P3,2)
 + droneDelivered(D2,P3,2) + droneIsDelivering(D2,P3,2)
 - droneIsDelivering(D2,P3,1) = -0
 updatedDelivering(P3,D2,3): - droneCollected(D2,P3,3)
 + droneDelivered(D2,P3,3) - droneIsDelivering(D2,P3,2)
 + droneIsDelivering(D2,P3,3) = -0
 updatedDelivering(P3,D2,4): - droneCollected(D2,P3,4)
 + droneDelivered(D2,P3,4) - droneIsDelivering(D2,P3,3)
 + droneIsDelivering(D2,P3,4) = -0
 updatedDelivering(P3,D2,5): - droneCollected(D2,P3,5)
 + droneDelivered(D2,P3,5) - droneIsDelivering(D2,P3,4)
 + droneIsDelivering(D2,P3,5) = -0
 updatedDelivering(P3,D3,2): - droneCollected(D3,P3,2)
 + droneDelivered(D3,P3,2) + droneIsDelivering(D3,P3,2)
 - droneIsDelivering(D3,P3,1) = -0
 updatedDelivering(P3,D3,3): - droneCollected(D3,P3,3)
 + droneDelivered(D3,P3,3) - droneIsDelivering(D3,P3,2)
 + droneIsDelivering(D3,P3,3) = -0
 updatedDelivering(P3,D3,4): - droneCollected(D3,P3,4)
 + droneDelivered(D3,P3,4) - droneIsDelivering(D3,P3,3)
 + droneIsDelivering(D3,P3,4) = -0
 updatedDelivering(P3,D3,5): - droneCollected(D3,P3,5)
 + droneDelivered(D3,P3,5) - droneIsDelivering(D3,P3,4)
 + droneIsDelivering(D3,P3,5) = -0
 updatedDelivering(P3,D4,2): - droneCollected(D4,P3,2)
 + droneDelivered(D4,P3,2) + droneIsDelivering(D4,P3,2)
 - droneIsDelivering(D4,P3,1) = -0
 updatedDelivering(P3,D4,3): - droneCollected(D4,P3,3)
 + droneDelivered(D4,P3,3) - droneIsDelivering(D4,P3,2)
 + droneIsDelivering(D4,P3,3) = -0
 updatedDelivering(P3,D4,4): - droneCollected(D4,P3,4)
 + droneDelivered(D4,P3,4) - droneIsDelivering(D4,P3,3)
 + droneIsDelivering(D4,P3,4) = -0
 updatedDelivering(P3,D4,5): - droneCollected(D4,P3,5)
 + droneDelivered(D4,P3,5) - droneIsDelivering(D4,P3,4)
 + droneIsDelivering(D4,P3,5) = -0
 updatedDelivering(P3,D5,2): - droneCollected(D5,P3,2)
 + droneDelivered(D5,P3,2) + droneIsDelivering(D5,P3,2)
 - droneIsDelivering(D5,P3,1) = -0
 updatedDelivering(P3,D5,3): - droneCollected(D5,P3,3)
 + droneDelivered(D5,P3,3) - droneIsDelivering(D5,P3,2)
 + droneIsDelivering(D5,P3,3) = -0
 updatedDelivering(P3,D5,4): - droneCollected(D5,P3,4)
 + droneDelivered(D5,P3,4) - droneIsDelivering(D5,P3,3)
 + droneIsDelivering(D5,P3,4) = -0
 updatedDelivering(P3,D5,5): - droneCollected(D5,P3,5)
 + droneDelivered(D5,P3,5) - droneIsDelivering(D5,P3,4)
 + droneIsDelivering(D5,P3,5) = -0
 updatedDelivering(P4,D1,2): - droneCollected(D1,P4,2)
 + droneDelivered(D1,P4,2) + droneIsDelivering(D1,P4,2)
 - droneIsDelivering(D1,P4,1) = -0
 updatedDelivering(P4,D1,3): - droneCollected(D1,P4,3)
 + droneDelivered(D1,P4,3) - droneIsDelivering(D1,P4,2)
 + droneIsDelivering(D1,P4,3) = -0
 updatedDelivering(P4,D1,4): - droneCollected(D1,P4,4)
 + droneDelivered(D1,P4,4) - droneIsDelivering(D1,P4,3)
 + droneIsDelivering(D1,P4,4) = -0
 updatedDelivering(P4,D1,5): - droneCollected(D1,P4,5)
 + droneDelivered(D1,P4,5) - droneIsDelivering(D1,P4,4)
 + droneIsDelivering(D1,P4,5) = -0
 updatedDelivering(P4,D2,2): - droneCollected(D2,P4,2)
 + droneDelivered(D2,P4,2) + droneIsDelivering(D2,P4,2)
 - droneIsDelivering(D2,P4,1) = -0
 updatedDelivering(P4,D2,3): - droneCollected(D2,P4,3)
 + droneDelivered(D2,P4,3) - droneIsDelivering(D2,P4,2)
 + droneIsDelivering(D2,P4,3) = -0
 updatedDelivering(P4,D2,4): - droneCollected(D2,P4,4)
 + droneDelivered(D2,P4,4) - droneIsDelivering(D2,P4,3)
 + droneIsDelivering(D2,P4,4) = -0
 updatedDelivering(P4,D2,5): - droneCollected(D2,P4,5)
 + droneDelivered(D2,P4,5) - droneIsDelivering(D2,P4,4)
 + droneIsDelivering(D2,P4,5) = -0
 updatedDelivering(P4,D3,2): - droneCollected(D3,P4,2)
 + droneDelivered(D3,P4,2) + droneIsDelivering(D3,P4,2)
 - droneIsDelivering(D3,P4,1) = -0
 updatedDelivering(P4,D3,3): - droneCollected(D3,P4,3)
 + droneDelivered(D3,P4,3) - droneIsDelivering(D3,P4,2)
 + droneIsDelivering(D3,P4,3) = -0
 updatedDelivering(P4,D3,4): - droneCollected(D3,P4,4)
 + droneDelivered(D3,P4,4) - droneIsDelivering(D3,P4,3)
 + droneIsDelivering(D3,P4,4) = -0
 updatedDelivering(P4,D3,5): - droneCollected(D3,P4,5)
 + droneDelivered(D3,P4,5) - droneIsDelivering(D3,P4,4)
 + droneIsDelivering(D3,P4,5) = -0
 updatedDelivering(P4,D4,2): - droneCollected(D4,P4,2)
 + droneDelivered(D4,P4,2) + droneIsDelivering(D4,P4,2)
 - droneIsDelivering(D4,P4,1) = -0
 updatedDelivering(P4,D4,3): - droneCollected(D4,P4,3)
 + droneDelivered(D4,P4,3) - droneIsDelivering(D4,P4,2)
 + droneIsDelivering(D4,P4,3) = -0
 updatedDelivering(P4,D4,4): - droneCollected(D4,P4,4)
 + droneDelivered(D4,P4,4) - droneIsDelivering(D4,P4,3)
 + droneIsDelivering(D4,P4,4) = -0
 updatedDelivering(P4,D4,5): - droneCollected(D4,P4,5)
 + droneDelivered(D4,P4,5) - droneIsDelivering(D4,P4,4)
 + droneIsDelivering(D4,P4,5) = -0
 updatedDelivering(P4,D5,2): - droneCollected(D5,P4,2)
 + droneDelivered(D5,P4,2) + droneIsDelivering(D5,P4,2)
 - droneIsDelivering(D5,P4,1) = -0
 updatedDelivering(P4,D5,3): - droneCollected(D5,P4,3)
 + droneDelivered(D5,P4,3) - droneIsDelivering(D5,P4,2)
 + droneIsDelivering(D5,P4,3) = -0
 updatedDelivering(P4,D5,4): - droneCollected(D5,P4,4)
 + droneDelivered(D5,P4,4) - droneIsDelivering(D5,P4,3)
 + droneIsDelivering(D5,P4,4) = -0
 updatedDelivering(P4,D5,5): - droneCollected(D5,P4,5)
 + droneDelivered(D5,P4,5) - droneIsDelivering(D5,P4,4)
 + droneIsDelivering(D5,P4,5) = -0
 updatedDelivering(P5,D1,2): - droneCollected(D1,P5,2)
 + droneDelivered(D1,P5,2) + droneIsDelivering(D1,P5,2)
 - droneIsDelivering(D1,P5,1) = -0
 updatedDelivering(P5,D1,3): - droneCollected(D1,P5,3)
 + droneDelivered(D1,P5,3) - droneIsDelivering(D1,P5,2)
 + droneIsDelivering(D1,P5,3) = -0
 updatedDelivering(P5,D1,4): - droneCollected(D1,P5,4)
 + droneDelivered(D1,P5,4) - droneIsDelivering(D1,P5,3)
 + droneIsDelivering(D1,P5,4) = -0
 updatedDelivering(P5,D1,5): - droneCollected(D1,P5,5)
 + droneDelivered(D1,P5,5) - droneIsDelivering(D1,P5,4)
 + droneIsDelivering(D1,P5,5) = -0
 updatedDelivering(P5,D2,2): - droneCollected(D2,P5,2)
 + droneDelivered(D2,P5,2) + droneIsDelivering(D2,P5,2)
 - droneIsDelivering(D2,P5,1) = -0
 updatedDelivering(P5,D2,3): - droneCollected(D2,P5,3)
 + droneDelivered(D2,P5,3) - droneIsDelivering(D2,P5,2)
 + droneIsDelivering(D2,P5,3) = -0
 updatedDelivering(P5,D2,4): - droneCollected(D2,P5,4)
 + droneDelivered(D2,P5,4) - droneIsDelivering(D2,P5,3)
 + droneIsDelivering(D2,P5,4) = -0
 updatedDelivering(P5,D2,5): - droneCollected(D2,P5,5)
 + droneDelivered(D2,P5,5) - droneIsDelivering(D2,P5,4)
 + droneIsDelivering(D2,P5,5) = -0
 updatedDelivering(P5,D3,2): - droneCollected(D3,P5,2)
 + droneDelivered(D3,P5,2) + droneIsDelivering(D3,P5,2)
 - droneIsDelivering(D3,P5,1) = -0
 updatedDelivering(P5,D3,3): - droneCollected(D3,P5,3)
 + droneDelivered(D3,P5,3) - droneIsDelivering(D3,P5,2)
 + droneIsDelivering(D3,P5,3) = -0
 updatedDelivering(P5,D3,4): - droneCollected(D3,P5,4)
 + droneDelivered(D3,P5,4) - droneIsDelivering(D3,P5,3)
 + droneIsDelivering(D3,P5,4) = -0
 updatedDelivering(P5,D3,5): - droneCollected(D3,P5,5)
 + droneDelivered(D3,P5,5) - droneIsDelivering(D3,P5,4)
 + droneIsDelivering(D3,P5,5) = -0
 updatedDelivering(P5,D4,2): - droneCollected(D4,P5,2)
 + droneDelivered(D4,P5,2) + droneIsDelivering(D4,P5,2)
 - droneIsDelivering(D4,P5,1) = -0
 updatedDelivering(P5,D4,3): - droneCollected(D4,P5,3)
 + droneDelivered(D4,P5,3) - droneIsDelivering(D4,P5,2)
 + droneIsDelivering(D4,P5,3) = -0
 updatedDelivering(P5,D4,4): - droneCollected(D4,P5,4)
 + droneDelivered(D4,P5,4) - droneIsDelivering(D4,P5,3)
 + droneIsDelivering(D4,P5,4) = -0
 updatedDelivering(P5,D4,5): - droneCollected(D4,P5,5)
 + droneDelivered(D4,P5,5) - droneIsDelivering(D4,P5,4)
 + droneIsDelivering(D4,P5,5) = -0
 updatedDelivering(P5,D5,2): - droneCollected(D5,P5,2)
 + droneDelivered(D5,P5,2) + droneIsDelivering(D5,P5,2)
 - droneIsDelivering(D5,P5,1) = -0
 updatedDelivering(P5,D5,3): - droneCollected(D5,P5,3)
 + droneDelivered(D5,P5,3) - droneIsDelivering(D5,P5,2)
 + droneIsDelivering(D5,P5,3) = -0
 updatedDelivering(P5,D5,4): - droneCollected(D5,P5,4)
 + droneDelivered(D5,P5,4) - droneIsDelivering(D5,P5,3)
 + droneIsDelivering(D5,P5,4) = -0
 updatedDelivering(P5,D5,5): - droneCollected(D5,P5,5)
 + droneDelivered(D5,P5,5) - droneIsDelivering(D5,P5,4)
 + droneIsDelivering(D5,P5,5) = -0
 updatedDelivering(P6,D1,2): - droneCollected(D1,P6,2)
 + droneDelivered(D1,P6,2) + droneIsDelivering(D1,P6,2)
 - droneIsDelivering(D1,P6,1) = -0
 updatedDelivering(P6,D1,3): - droneCollected(D1,P6,3)
 + droneDelivered(D1,P6,3) - droneIsDelivering(D1,P6,2)
 + droneIsDelivering(D1,P6,3) = -0
 updatedDelivering(P6,D1,4): - droneCollected(D1,P6,4)
 + droneDelivered(D1,P6,4) - droneIsDelivering(D1,P6,3)
 + droneIsDelivering(D1,P6,4) = -0
 updatedDelivering(P6,D1,5): - droneCollected(D1,P6,5)
 + droneDelivered(D1,P6,5) - droneIsDelivering(D1,P6,4)
 + droneIsDelivering(D1,P6,5) = -0
 updatedDelivering(P6,D2,2): - droneCollected(D2,P6,2)
 + droneDelivered(D2,P6,2) + droneIsDelivering(D2,P6,2)
 - droneIsDelivering(D2,P6,1) = -0
 updatedDelivering(P6,D2,3): - droneCollected(D2,P6,3)
 + droneDelivered(D2,P6,3) - droneIsDelivering(D2,P6,2)
 + droneIsDelivering(D2,P6,3) = -0
 updatedDelivering(P6,D2,4): - droneCollected(D2,P6,4)
 + droneDelivered(D2,P6,4) - droneIsDelivering(D2,P6,3)
 + droneIsDelivering(D2,P6,4) = -0
 updatedDelivering(P6,D2,5): - droneCollected(D2,P6,5)
 + droneDelivered(D2,P6,5) - droneIsDelivering(D2,P6,4)
 + droneIsDelivering(D2,P6,5) = -0
 updatedDelivering(P6,D3,2): - droneCollected(D3,P6,2)
 + droneDelivered(D3,P6,2) + droneIsDelivering(D3,P6,2)
 - droneIsDelivering(D3,P6,1) = -0
 updatedDelivering(P6,D3,3): - droneCollected(D3,P6,3)
 + droneDelivered(D3,P6,3) - droneIsDelivering(D3,P6,2)
 + droneIsDelivering(D3,P6,3) = -0
 updatedDelivering(P6,D3,4): - droneCollected(D3,P6,4)
 + droneDelivered(D3,P6,4) - droneIsDelivering(D3,P6,3)
 + droneIsDelivering(D3,P6,4) = -0
 updatedDelivering(P6,D3,5): - droneCollected(D3,P6,5)
 + droneDelivered(D3,P6,5) - droneIsDelivering(D3,P6,4)
 + droneIsDelivering(D3,P6,5) = -0
 updatedDelivering(P6,D4,2): - droneCollected(D4,P6,2)
 + droneDelivered(D4,P6,2) + droneIsDelivering(D4,P6,2)
 - droneIsDelivering(D4,P6,1) = -0
 updatedDelivering(P6,D4,3): - droneCollected(D4,P6,3)
 + droneDelivered(D4,P6,3) - droneIsDelivering(D4,P6,2)
 + droneIsDelivering(D4,P6,3) = -0
 updatedDelivering(P6,D4,4): - droneCollected(D4,P6,4)
 + droneDelivered(D4,P6,4) - droneIsDelivering(D4,P6,3)
 + droneIsDelivering(D4,P6,4) = -0
 updatedDelivering(P6,D4,5): - droneCollected(D4,P6,5)
 + droneDelivered(D4,P6,5) - droneIsDelivering(D4,P6,4)
 + droneIsDelivering(D4,P6,5) = -0
 updatedDelivering(P6,D5,2): - droneCollected(D5,P6,2)
 + droneDelivered(D5,P6,2) + droneIsDelivering(D5,P6,2)
 - droneIsDelivering(D5,P6,1) = -0
 updatedDelivering(P6,D5,3): - droneCollected(D5,P6,3)
 + droneDelivered(D5,P6,3) - droneIsDelivering(D5,P6,2)
 + droneIsDelivering(D5,P6,3) = -0
 updatedDelivering(P6,D5,4): - droneCollected(D5,P6,4)
 + droneDelivered(D5,P6,4) - droneIsDelivering(D5,P6,3)
 + droneIsDelivering(D5,P6,4) = -0
 updatedDelivering(P6,D5,5): - droneCollected(D5,P6,5)
 + droneDelivered(D5,P6,5) - droneIsDelivering(D5,P6,4)
 + droneIsDelivering(D5,P6,5) = -0
 updatedDeliveringAtBegin(P1,D1,1): + droneIsDelivering(D1,P1,1) = 1
 updatedDeliveringAtBegin(P1,D2,1): + droneIsDelivering(D2,P1,1) = -0
 updatedDeliveringAtBegin(P1,D3,1): + droneIsDelivering(D3,P1,1) = -0
 updatedDeliveringAtBegin(P1,D4,1): + droneIsDelivering(D4,P1,1) = -0
 updatedDeliveringAtBegin(P1,D5,1): + droneIsDelivering(D5,P1,1) = -0
 updatedDeliveringAtBegin(P2,D1,1): + droneIsDelivering(D1,P2,1) = -0
 updatedDeliveringAtBegin(P2,D2,1): + droneIsDelivering(D2,P2,1) = -0
 updatedDeliveringAtBegin(P2,D3,1): + droneIsDelivering(D3,P2,1) = -0
 updatedDeliveringAtBegin(P2,D4,1): + droneIsDelivering(D4,P2,1) = -0
 updatedDeliveringAtBegin(P2,D5,1): + droneIsDelivering(D5,P2,1) = -0
 updatedDeliveringAtBegin(P3,D1,1): + droneIsDelivering(D1,P3,1) = -0
 updatedDeliveringAtBegin(P3,D2,1): + droneIsDelivering(D2,P3,1) = -0
 updatedDeliveringAtBegin(P3,D3,1): + droneIsDelivering(D3,P3,1) = -0
 updatedDeliveringAtBegin(P3,D4,1): + droneIsDelivering(D4,P3,1) = -0
 updatedDeliveringAtBegin(P3,D5,1): + droneIsDelivering(D5,P3,1) = -0
 updatedDeliveringAtBegin(P4,D1,1): + droneIsDelivering(D1,P4,1) = -0
 updatedDeliveringAtBegin(P4,D2,1): + droneIsDelivering(D2,P4,1) = -0
 updatedDeliveringAtBegin(P4,D3,1): + droneIsDelivering(D3,P4,1) = -0
 updatedDeliveringAtBegin(P4,D4,1): + droneIsDelivering(D4,P4,1) = -0
 updatedDeliveringAtBegin(P4,D5,1): + droneIsDelivering(D5,P4,1) = -0
 updatedDeliveringAtBegin(P5,D1,1): + droneIsDelivering(D1,P5,1) = -0
 updatedDeliveringAtBegin(P5,D2,1): + droneIsDelivering(D2,P5,1) = -0
 updatedDeliveringAtBegin(P5,D3,1): + droneIsDelivering(D3,P5,1) = -0
 updatedDeliveringAtBegin(P5,D4,1): + droneIsDelivering(D4,P5,1) = -0
 updatedDeliveringAtBegin(P5,D5,1): + droneIsDelivering(D5,P5,1) = -0
 updatedDeliveringAtBegin(P6,D1,1): + droneIsDelivering(D1,P6,1) = -0
 updatedDeliveringAtBegin(P6,D2,1): + droneIsDelivering(D2,P6,1) = -0
 updatedDeliveringAtBegin(P6,D3,1): + droneIsDelivering(D3,P6,1) = -0
 updatedDeliveringAtBegin(P6,D4,1): + droneIsDelivering(D4,P6,1) = -0
 updatedDeliveringAtBegin(P6,D5,1): + droneIsDelivering(D5,P6,1) = -0
 droneIsBeingUsed(D1): - droneIsDelivering(D1,P1,2)
 - droneIsDelivering(D1,P1,1) - droneIsDelivering(D1,P1,3)
 - droneIsDelivering(D1,P1,4) - droneIsDelivering(D1,P1,5)
 - droneIsDelivering(D1,P2,2) - droneIsDelivering(D1,P2,1)
 - droneIsDelivering(D1,P2,3) - droneIsDelivering(D1,P2,4)
 - droneIsDelivering(D1,P2,5) - droneIsDelivering(D1,P3,2)
 - droneIsDelivering(D1,P3,1) - droneIsDelivering(D1,P3,3)
 - droneIsDelivering(D1,P3,4) - droneIsDelivering(D1,P3,5)
 - droneIsDelivering(D1,P4,2) - droneIsDelivering(D1,P4,1)
 - droneIsDelivering(D1,P4,3) - droneIsDelivering(D1,P4,4)
 - droneIsDelivering(D1,P4,5) - droneIsDelivering(D1,P5,2)
 - droneIsDelivering(D1,P5,1) - droneIsDelivering(D1,P5,3)
 - droneIsDelivering(D1,P5,4) - droneIsDelivering(D1,P5,5)
 - droneIsDelivering(D1,P6,2) - droneIsDelivering(D1,P6,1)
 - droneIsDelivering(D1,P6,3) - droneIsDelivering(D1,P6,4)
 - droneIsDelivering(D1,P6,5) + 1000000 droneIsUsed(D1) >= -0
 droneIsBeingUsed(D2): - droneIsDelivering(D2,P1,2)
 - droneIsDelivering(D2,P1,1) - droneIsDelivering(D2,P1,3)
 - droneIsDelivering(D2,P1,4) - droneIsDelivering(D2,P1,5)
 - droneIsDelivering(D2,P2,2) - droneIsDelivering(D2,P2,1)
 - droneIsDelivering(D2,P2,3) - droneIsDelivering(D2,P2,4)
 - droneIsDelivering(D2,P2,5) - droneIsDelivering(D2,P3,2)
 - droneIsDelivering(D2,P3,1) - droneIsDelivering(D2,P3,3)
 - droneIsDelivering(D2,P3,4) - droneIsDelivering(D2,P3,5)
 - droneIsDelivering(D2,P4,2) - droneIsDelivering(D2,P4,1)
 - droneIsDelivering(D2,P4,3) - droneIsDelivering(D2,P4,4)
 - droneIsDelivering(D2,P4,5) - droneIsDelivering(D2,P5,2)
 - droneIsDelivering(D2,P5,1) - droneIsDelivering(D2,P5,3)
 - droneIsDelivering(D2,P5,4) - droneIsDelivering(D2,P5,5)
 - droneIsDelivering(D2,P6,2) - droneIsDelivering(D2,P6,1)
 - droneIsDelivering(D2,P6,3) - droneIsDelivering(D2,P6,4)
 - droneIsDelivering(D2,P6,5) + 1000000 droneIsUsed(D2) >= -0
 droneIsBeingUsed(D3): - droneIsDelivering(D3,P1,2)
 - droneIsDelivering(D3,P1,1) - droneIsDelivering(D3,P1,3)
 - droneIsDelivering(D3,P1,4) - droneIsDelivering(D3,P1,5)
 - droneIsDelivering(D3,P2,2) - droneIsDelivering(D3,P2,1)
 - droneIsDelivering(D3,P2,3) - droneIsDelivering(D3,P2,4)
 - droneIsDelivering(D3,P2,5) - droneIsDelivering(D3,P3,2)
 - droneIsDelivering(D3,P3,1) - droneIsDelivering(D3,P3,3)
 - droneIsDelivering(D3,P3,4) - droneIsDelivering(D3,P3,5)
 - droneIsDelivering(D3,P4,2) - droneIsDelivering(D3,P4,1)
 - droneIsDelivering(D3,P4,3) - droneIsDelivering(D3,P4,4)
 - droneIsDelivering(D3,P4,5) - droneIsDelivering(D3,P5,2)
 - droneIsDelivering(D3,P5,1) - droneIsDelivering(D3,P5,3)
 - droneIsDelivering(D3,P5,4) - droneIsDelivering(D3,P5,5)
 - droneIsDelivering(D3,P6,2) - droneIsDelivering(D3,P6,1)
 - droneIsDelivering(D3,P6,3) - droneIsDelivering(D3,P6,4)
 - droneIsDelivering(D3,P6,5) + 1000000 droneIsUsed(D3) >= -0
 droneIsBeingUsed(D4): - droneIsDelivering(D4,P1,2)
 - droneIsDelivering(D4,P1,1) - droneIsDelivering(D4,P1,3)
 - droneIsDelivering(D4,P1,4) - droneIsDelivering(D4,P1,5)
 - droneIsDelivering(D4,P2,2) - droneIsDelivering(D4,P2,1)
 - droneIsDelivering(D4,P2,3) - droneIsDelivering(D4,P2,4)
 - droneIsDelivering(D4,P2,5) - droneIsDelivering(D4,P3,2)
 - droneIsDelivering(D4,P3,1) - droneIsDelivering(D4,P3,3)
 - droneIsDelivering(D4,P3,4) - droneIsDelivering(D4,P3,5)
 - droneIsDelivering(D4,P4,2) - droneIsDelivering(D4,P4,1)
 - droneIsDelivering(D4,P4,3) - droneIsDelivering(D4,P4,4)
 - droneIsDelivering(D4,P4,5) - droneIsDelivering(D4,P5,2)
 - droneIsDelivering(D4,P5,1) - droneIsDelivering(D4,P5,3)
 - droneIsDelivering(D4,P5,4) - droneIsDelivering(D4,P5,5)
 - droneIsDelivering(D4,P6,2) - droneIsDelivering(D4,P6,1)
 - droneIsDelivering(D4,P6,3) - droneIsDelivering(D4,P6,4)
 - droneIsDelivering(D4,P6,5) + 1000000 droneIsUsed(D4) >= -0
 droneIsBeingUsed(D5): - droneIsDelivering(D5,P1,2)
 - droneIsDelivering(D5,P1,1) - droneIsDelivering(D5,P1,3)
 - droneIsDelivering(D5,P1,4) - droneIsDelivering(D5,P1,5)
 - droneIsDelivering(D5,P2,2) - droneIsDelivering(D5,P2,1)
 - droneIsDelivering(D5,P2,3) - droneIsDelivering(D5,P2,4)
 - droneIsDelivering(D5,P2,5) - droneIsDelivering(D5,P3,2)
 - droneIsDelivering(D5,P3,1) - droneIsDelivering(D5,P3,3)
 - droneIsDelivering(D5,P3,4) - droneIsDelivering(D5,P3,5)
 - droneIsDelivering(D5,P4,2) - droneIsDelivering(D5,P4,1)
 - droneIsDelivering(D5,P4,3) - droneIsDelivering(D5,P4,4)
 - droneIsDelivering(D5,P4,5) - droneIsDelivering(D5,P5,2)
 - droneIsDelivering(D5,P5,1) - droneIsDelivering(D5,P5,3)
 - droneIsDelivering(D5,P5,4) - droneIsDelivering(D5,P5,5)
 - droneIsDelivering(D5,P6,2) - droneIsDelivering(D5,P6,1)
 - droneIsDelivering(D5,P6,3) - droneIsDelivering(D5,P6,4)
 - droneIsDelivering(D5,P6,5) + 1000000 droneIsUsed(D5) >= -0
 updateDroneIsOn(D1,1): - droneIsDelivering(D1,P1,1)
 - droneIsDelivering(D1,P2,1) - droneIsDelivering(D1,P3,1)
 - droneIsDelivering(D1,P4,1) - droneIsDelivering(D1,P5,1)
 - droneIsDelivering(D1,P6,1) + 1000000 droneIsOn(D1,1) >= -0
 updateDroneIsOn(D1,2): - droneIsDelivering(D1,P1,2)
 - droneIsDelivering(D1,P2,2) - droneIsDelivering(D1,P3,2)
 - droneIsDelivering(D1,P4,2) - droneIsDelivering(D1,P5,2)
 - droneIsDelivering(D1,P6,2) + 1000000 droneIsOn(D1,2) >= -0
 updateDroneIsOn(D1,3): - droneIsDelivering(D1,P1,3)
 - droneIsDelivering(D1,P2,3) - droneIsDelivering(D1,P3,3)
 - droneIsDelivering(D1,P4,3) - droneIsDelivering(D1,P5,3)
 - droneIsDelivering(D1,P6,3) + 1000000 droneIsOn(D1,3) >= -0
 updateDroneIsOn(D1,4): - droneIsDelivering(D1,P1,4)
 - droneIsDelivering(D1,P2,4) - droneIsDelivering(D1,P3,4)
 - droneIsDelivering(D1,P4,4) - droneIsDelivering(D1,P5,4)
 - droneIsDelivering(D1,P6,4) + 1000000 droneIsOn(D1,4) >= -0
 updateDroneIsOn(D1,5): - droneIsDelivering(D1,P1,5)
 - droneIsDelivering(D1,P2,5) - droneIsDelivering(D1,P3,5)
 - droneIsDelivering(D1,P4,5) - droneIsDelivering(D1,P5,5)
 - droneIsDelivering(D1,P6,5) + 1000000 droneIsOn(D1,5) >= -0
 updateDroneIsOn(D2,1): - droneIsDelivering(D2,P1,1)
 - droneIsDelivering(D2,P2,1) - droneIsDelivering(D2,P3,1)
 - droneIsDelivering(D2,P4,1) - droneIsDelivering(D2,P5,1)
 - droneIsDelivering(D2,P6,1) + 1000000 droneIsOn(D2,1) >= -0
 updateDroneIsOn(D2,2): - droneIsDelivering(D2,P1,2)
 - droneIsDelivering(D2,P2,2) - droneIsDelivering(D2,P3,2)
 - droneIsDelivering(D2,P4,2) - droneIsDelivering(D2,P5,2)
 - droneIsDelivering(D2,P6,2) + 1000000 droneIsOn(D2,2) >= -0
 updateDroneIsOn(D2,3): - droneIsDelivering(D2,P1,3)
 - droneIsDelivering(D2,P2,3) - droneIsDelivering(D2,P3,3)
 - droneIsDelivering(D2,P4,3) - droneIsDelivering(D2,P5,3)
 - droneIsDelivering(D2,P6,3) + 1000000 droneIsOn(D2,3) >= -0
 updateDroneIsOn(D2,4): - droneIsDelivering(D2,P1,4)
 - droneIsDelivering(D2,P2,4) - droneIsDelivering(D2,P3,4)
 - droneIsDelivering(D2,P4,4) - droneIsDelivering(D2,P5,4)
 - droneIsDelivering(D2,P6,4) + 1000000 droneIsOn(D2,4) >= -0
 updateDroneIsOn(D2,5): - droneIsDelivering(D2,P1,5)
 - droneIsDelivering(D2,P2,5) - droneIsDelivering(D2,P3,5)
 - droneIsDelivering(D2,P4,5) - droneIsDelivering(D2,P5,5)
 - droneIsDelivering(D2,P6,5) + 1000000 droneIsOn(D2,5) >= -0
 updateDroneIsOn(D3,1): - droneIsDelivering(D3,P1,1)
 - droneIsDelivering(D3,P2,1) - droneIsDelivering(D3,P3,1)
 - droneIsDelivering(D3,P4,1) - droneIsDelivering(D3,P5,1)
 - droneIsDelivering(D3,P6,1) + 1000000 droneIsOn(D3,1) >= -0
 updateDroneIsOn(D3,2): - droneIsDelivering(D3,P1,2)
 - droneIsDelivering(D3,P2,2) - droneIsDelivering(D3,P3,2)
 - droneIsDelivering(D3,P4,2) - droneIsDelivering(D3,P5,2)
 - droneIsDelivering(D3,P6,2) + 1000000 droneIsOn(D3,2) >= -0
 updateDroneIsOn(D3,3): - droneIsDelivering(D3,P1,3)
 - droneIsDelivering(D3,P2,3) - droneIsDelivering(D3,P3,3)
 - droneIsDelivering(D3,P4,3) - droneIsDelivering(D3,P5,3)
 - droneIsDelivering(D3,P6,3) + 1000000 droneIsOn(D3,3) >= -0
 updateDroneIsOn(D3,4): - droneIsDelivering(D3,P1,4)
 - droneIsDelivering(D3,P2,4) - droneIsDelivering(D3,P3,4)
 - droneIsDelivering(D3,P4,4) - droneIsDelivering(D3,P5,4)
 - droneIsDelivering(D3,P6,4) + 1000000 droneIsOn(D3,4) >= -0
 updateDroneIsOn(D3,5): - droneIsDelivering(D3,P1,5)
 - droneIsDelivering(D3,P2,5) - droneIsDelivering(D3,P3,5)
 - droneIsDelivering(D3,P4,5) - droneIsDelivering(D3,P5,5)
 - droneIsDelivering(D3,P6,5) + 1000000 droneIsOn(D3,5) >= -0
 updateDroneIsOn(D4,1): - droneIsDelivering(D4,P1,1)
 - droneIsDelivering(D4,P2,1) - droneIsDelivering(D4,P3,1)
 - droneIsDelivering(D4,P4,1) - droneIsDelivering(D4,P5,1)
 - droneIsDelivering(D4,P6,1) + 1000000 droneIsOn(D4,1) >= -0
 updateDroneIsOn(D4,2): - droneIsDelivering(D4,P1,2)
 - droneIsDelivering(D4,P2,2) - droneIsDelivering(D4,P3,2)
 - droneIsDelivering(D4,P4,2) - droneIsDelivering(D4,P5,2)
 - droneIsDelivering(D4,P6,2) + 1000000 droneIsOn(D4,2) >= -0
 updateDroneIsOn(D4,3): - droneIsDelivering(D4,P1,3)
 - droneIsDelivering(D4,P2,3) - droneIsDelivering(D4,P3,3)
 - droneIsDelivering(D4,P4,3) - droneIsDelivering(D4,P5,3)
 - droneIsDelivering(D4,P6,3) + 1000000 droneIsOn(D4,3) >= -0
 updateDroneIsOn(D4,4): - droneIsDelivering(D4,P1,4)
 - droneIsDelivering(D4,P2,4) - droneIsDelivering(D4,P3,4)
 - droneIsDelivering(D4,P4,4) - droneIsDelivering(D4,P5,4)
 - droneIsDelivering(D4,P6,4) + 1000000 droneIsOn(D4,4) >= -0
 updateDroneIsOn(D4,5): - droneIsDelivering(D4,P1,5)
 - droneIsDelivering(D4,P2,5) - droneIsDelivering(D4,P3,5)
 - droneIsDelivering(D4,P4,5) - droneIsDelivering(D4,P5,5)
 - droneIsDelivering(D4,P6,5) + 1000000 droneIsOn(D4,5) >= -0
 updateDroneIsOn(D5,1): - droneIsDelivering(D5,P1,1)
 - droneIsDelivering(D5,P2,1) - droneIsDelivering(D5,P3,1)
 - droneIsDelivering(D5,P4,1) - droneIsDelivering(D5,P5,1)
 - droneIsDelivering(D5,P6,1) + 1000000 droneIsOn(D5,1) >= -0
 updateDroneIsOn(D5,2): - droneIsDelivering(D5,P1,2)
 - droneIsDelivering(D5,P2,2) - droneIsDelivering(D5,P3,2)
 - droneIsDelivering(D5,P4,2) - droneIsDelivering(D5,P5,2)
 - droneIsDelivering(D5,P6,2) + 1000000 droneIsOn(D5,2) >= -0
 updateDroneIsOn(D5,3): - droneIsDelivering(D5,P1,3)
 - droneIsDelivering(D5,P2,3) - droneIsDelivering(D5,P3,3)
 - droneIsDelivering(D5,P4,3) - droneIsDelivering(D5,P5,3)
 - droneIsDelivering(D5,P6,3) + 1000000 droneIsOn(D5,3) >= -0
 updateDroneIsOn(D5,4): - droneIsDelivering(D5,P1,4)
 - droneIsDelivering(D5,P2,4) - droneIsDelivering(D5,P3,4)
 - droneIsDelivering(D5,P4,4) - droneIsDelivering(D5,P5,4)
 - droneIsDelivering(D5,P6,4) + 1000000 droneIsOn(D5,4) >= -0
 updateDroneIsOn(D5,5): - droneIsDelivering(D5,P1,5)
 - droneIsDelivering(D5,P2,5) - droneIsDelivering(D5,P3,5)
 - droneIsDelivering(D5,P4,5) - droneIsDelivering(D5,P5,5)
 - droneIsDelivering(D5,P6,5) + 1000000 droneIsOn(D5,5) >= -0
 calcBaterryRateFirstInterval(D1): + droneBatteryRate(D1,1) = 100
 calcBaterryRateFirstInterval(D2): + droneBatteryRate(D2,1) = 100
 calcBaterryRateFirstInterval(D3): + droneBatteryRate(D3,1) = 100
 calcBaterryRateFirstInterval(D4): + droneBatteryRate(D4,1) = 100
 calcBaterryRateFirstInterval(D5): + droneBatteryRate(D5,1) = 100
 calcBaterryRate(D1,2): + 0.0714285714285714 droneSpeed(D1,2)
 + droneBatteryRate(D1,2) - droneBatteryRate(D1,1)
 + excessOfEnergy(D1,2) - 4 droneIsChargingAtEP(D1,2,1)
 - 4 droneIsChargingAtEP(D1,2,2) - 4 droneIsChargingAtEP(D1,2,3)
 - 4 droneIsChargingAtEP(D1,2,4) + 3 droneIsOn(D1,2) = -0
 calcBaterryRate(D1,3): + 0.0714285714285714 droneSpeed(D1,3)
 - droneBatteryRate(D1,2) + droneBatteryRate(D1,3)
 + excessOfEnergy(D1,3) - 4 droneIsChargingAtEP(D1,3,1)
 - 4 droneIsChargingAtEP(D1,3,2) - 4 droneIsChargingAtEP(D1,3,3)
 - 4 droneIsChargingAtEP(D1,3,4) + 3 droneIsOn(D1,3) = -0
 calcBaterryRate(D1,4): + 0.0714285714285714 droneSpeed(D1,4)
 - droneBatteryRate(D1,3) + droneBatteryRate(D1,4)
 + excessOfEnergy(D1,4) - 4 droneIsChargingAtEP(D1,4,1)
 - 4 droneIsChargingAtEP(D1,4,2) - 4 droneIsChargingAtEP(D1,4,3)
 - 4 droneIsChargingAtEP(D1,4,4) + 3 droneIsOn(D1,4) = -0
 calcBaterryRate(D1,5): + 0.0714285714285714 droneSpeed(D1,5)
 - droneBatteryRate(D1,4) + droneBatteryRate(D1,5)
 + excessOfEnergy(D1,5) - 4 droneIsChargingAtEP(D1,5,1)
 - 4 droneIsChargingAtEP(D1,5,2) - 4 droneIsChargingAtEP(D1,5,3)
 - 4 droneIsChargingAtEP(D1,5,4) + 3 droneIsOn(D1,5) = -0
 calcBaterryRate(D2,2): + 0.05 droneSpeed(D2,2) + droneBatteryRate(D2,2)
 - droneBatteryRate(D2,1) + excessOfEnergy(D2,2)
 - 2 droneIsChargingAtEP(D2,2,1) - 2 droneIsChargingAtEP(D2,2,2)
 - 2 droneIsChargingAtEP(D2,2,3) - 2 droneIsChargingAtEP(D2,2,4)
 + 3 droneIsOn(D2,2) = -0
 calcBaterryRate(D2,3): + 0.05 droneSpeed(D2,3) - droneBatteryRate(D2,2)
 + droneBatteryRate(D2,3) + excessOfEnergy(D2,3)
 - 2 droneIsChargingAtEP(D2,3,1) - 2 droneIsChargingAtEP(D2,3,2)
 - 2 droneIsChargingAtEP(D2,3,3) - 2 droneIsChargingAtEP(D2,3,4)
 + 3 droneIsOn(D2,3) = -0
 calcBaterryRate(D2,4): + 0.05 droneSpeed(D2,4) - droneBatteryRate(D2,3)
 + droneBatteryRate(D2,4) + excessOfEnergy(D2,4)
 - 2 droneIsChargingAtEP(D2,4,1) - 2 droneIsChargingAtEP(D2,4,2)
 - 2 droneIsChargingAtEP(D2,4,3) - 2 droneIsChargingAtEP(D2,4,4)
 + 3 droneIsOn(D2,4) = -0
 calcBaterryRate(D2,5): + 0.05 droneSpeed(D2,5) - droneBatteryRate(D2,4)
 + droneBatteryRate(D2,5) + excessOfEnergy(D2,5)
 - 2 droneIsChargingAtEP(D2,5,1) - 2 droneIsChargingAtEP(D2,5,2)
 - 2 droneIsChargingAtEP(D2,5,3) - 2 droneIsChargingAtEP(D2,5,4)
 + 3 droneIsOn(D2,5) = -0
 calcBaterryRate(D3,2): + 0.0875 droneSpeed(D3,2)
 + droneBatteryRate(D3,2) - droneBatteryRate(D3,1)
 + excessOfEnergy(D3,2) - 2 droneIsChargingAtEP(D3,2,1)
 - 2 droneIsChargingAtEP(D3,2,2) - 2 droneIsChargingAtEP(D3,2,3)
 - 2 droneIsChargingAtEP(D3,2,4) + 3 droneIsOn(D3,2) = -0
 calcBaterryRate(D3,3): + 0.0875 droneSpeed(D3,3)
 - droneBatteryRate(D3,2) + droneBatteryRate(D3,3)
 + excessOfEnergy(D3,3) - 2 droneIsChargingAtEP(D3,3,1)
 - 2 droneIsChargingAtEP(D3,3,2) - 2 droneIsChargingAtEP(D3,3,3)
 - 2 droneIsChargingAtEP(D3,3,4) + 3 droneIsOn(D3,3) = -0
 calcBaterryRate(D3,4): + 0.0875 droneSpeed(D3,4)
 - droneBatteryRate(D3,3) + droneBatteryRate(D3,4)
 + excessOfEnergy(D3,4) - 2 droneIsChargingAtEP(D3,4,1)
 - 2 droneIsChargingAtEP(D3,4,2) - 2 droneIsChargingAtEP(D3,4,3)
 - 2 droneIsChargingAtEP(D3,4,4) + 3 droneIsOn(D3,4) = -0
 calcBaterryRate(D3,5): + 0.0875 droneSpeed(D3,5)
 - droneBatteryRate(D3,4) + droneBatteryRate(D3,5)
 + excessOfEnergy(D3,5) - 2 droneIsChargingAtEP(D3,5,1)
 - 2 droneIsChargingAtEP(D3,5,2) - 2 droneIsChargingAtEP(D3,5,3)
 - 2 droneIsChargingAtEP(D3,5,4) + 3 droneIsOn(D3,5) = -0
 calcBaterryRate(D4,2): + 0.0833333333333333 droneSpeed(D4,2)
 + droneBatteryRate(D4,2) - droneBatteryRate(D4,1)
 + excessOfEnergy(D4,2) - 2 droneIsChargingAtEP(D4,2,1)
 - 2 droneIsChargingAtEP(D4,2,2) - 2 droneIsChargingAtEP(D4,2,3)
 - 2 droneIsChargingAtEP(D4,2,4) + 3 droneIsOn(D4,2) = -0
 calcBaterryRate(D4,3): + 0.0833333333333333 droneSpeed(D4,3)
 - droneBatteryRate(D4,2) + droneBatteryRate(D4,3)
 + excessOfEnergy(D4,3) - 2 droneIsChargingAtEP(D4,3,1)
 - 2 droneIsChargingAtEP(D4,3,2) - 2 droneIsChargingAtEP(D4,3,3)
 - 2 droneIsChargingAtEP(D4,3,4) + 3 droneIsOn(D4,3) = -0
 calcBaterryRate(D4,4): + 0.0833333333333333 droneSpeed(D4,4)
 - droneBatteryRate(D4,3) + droneBatteryRate(D4,4)
 + excessOfEnergy(D4,4) - 2 droneIsChargingAtEP(D4,4,1)
 - 2 droneIsChargingAtEP(D4,4,2) - 2 droneIsChargingAtEP(D4,4,3)
 - 2 droneIsChargingAtEP(D4,4,4) + 3 droneIsOn(D4,4) = -0
 calcBaterryRate(D4,5): + 0.0833333333333333 droneSpeed(D4,5)
 - droneBatteryRate(D4,4) + droneBatteryRate(D4,5)
 + excessOfEnergy(D4,5) - 2 droneIsChargingAtEP(D4,5,1)
 - 2 droneIsChargingAtEP(D4,5,2) - 2 droneIsChargingAtEP(D4,5,3)
 - 2 droneIsChargingAtEP(D4,5,4) + 3 droneIsOn(D4,5) = -0
 calcBaterryRate(D5,2): + 0.125 droneSpeed(D5,2)
 + droneBatteryRate(D5,2) - droneBatteryRate(D5,1)
 + excessOfEnergy(D5,2) - 4 droneIsChargingAtEP(D5,2,1)
 - 4 droneIsChargingAtEP(D5,2,2) - 4 droneIsChargingAtEP(D5,2,3)
 - 4 droneIsChargingAtEP(D5,2,4) + 3 droneIsOn(D5,2) = -0
 calcBaterryRate(D5,3): + 0.125 droneSpeed(D5,3)
 - droneBatteryRate(D5,2) + droneBatteryRate(D5,3)
 + excessOfEnergy(D5,3) - 4 droneIsChargingAtEP(D5,3,1)
 - 4 droneIsChargingAtEP(D5,3,2) - 4 droneIsChargingAtEP(D5,3,3)
 - 4 droneIsChargingAtEP(D5,3,4) + 3 droneIsOn(D5,3) = -0
 calcBaterryRate(D5,4): + 0.125 droneSpeed(D5,4)
 - droneBatteryRate(D5,3) + droneBatteryRate(D5,4)
 + excessOfEnergy(D5,4) - 4 droneIsChargingAtEP(D5,4,1)
 - 4 droneIsChargingAtEP(D5,4,2) - 4 droneIsChargingAtEP(D5,4,3)
 - 4 droneIsChargingAtEP(D5,4,4) + 3 droneIsOn(D5,4) = -0
 calcBaterryRate(D5,5): + 0.125 droneSpeed(D5,5)
 - droneBatteryRate(D5,4) + droneBatteryRate(D5,5)
 + excessOfEnergy(D5,5) - 4 droneIsChargingAtEP(D5,5,1)
 - 4 droneIsChargingAtEP(D5,5,2) - 4 droneIsChargingAtEP(D5,5,3)
 - 4 droneIsChargingAtEP(D5,5,4) + 3 droneIsOn(D5,5) = -0
 productDeliveringTime(P1): - droneIsDelivering(D1,P1,2)
 - droneIsDelivering(D1,P1,1) - droneIsDelivering(D1,P1,3)
 - droneIsDelivering(D1,P1,4) - droneIsDelivering(D1,P1,5)
 - droneIsDelivering(D2,P1,2) - droneIsDelivering(D2,P1,1)
 - droneIsDelivering(D2,P1,3) - droneIsDelivering(D2,P1,4)
 - droneIsDelivering(D2,P1,5) - droneIsDelivering(D3,P1,2)
 - droneIsDelivering(D3,P1,1) - droneIsDelivering(D3,P1,3)
 - droneIsDelivering(D3,P1,4) - droneIsDelivering(D3,P1,5)
 - droneIsDelivering(D4,P1,2) - droneIsDelivering(D4,P1,1)
 - droneIsDelivering(D4,P1,3) - droneIsDelivering(D4,P1,4)
 - droneIsDelivering(D4,P1,5) - droneIsDelivering(D5,P1,2)
 - droneIsDelivering(D5,P1,1) - droneIsDelivering(D5,P1,3)
 - droneIsDelivering(D5,P1,4) - droneIsDelivering(D5,P1,5)
 + timePerProductDelivered(P1) = -0
 productDeliveringTime(P2): - droneIsDelivering(D1,P2,2)
 - droneIsDelivering(D1,P2,1) - droneIsDelivering(D1,P2,3)
 - droneIsDelivering(D1,P2,4) - droneIsDelivering(D1,P2,5)
 - droneIsDelivering(D2,P2,2) - droneIsDelivering(D2,P2,1)
 - droneIsDelivering(D2,P2,3) - droneIsDelivering(D2,P2,4)
 - droneIsDelivering(D2,P2,5) - droneIsDelivering(D3,P2,2)
 - droneIsDelivering(D3,P2,1) - droneIsDelivering(D3,P2,3)
 - droneIsDelivering(D3,P2,4) - droneIsDelivering(D3,P2,5)
 - droneIsDelivering(D4,P2,2) - droneIsDelivering(D4,P2,1)
 - droneIsDelivering(D4,P2,3) - droneIsDelivering(D4,P2,4)
 - droneIsDelivering(D4,P2,5) - droneIsDelivering(D5,P2,2)
 - droneIsDelivering(D5,P2,1) - droneIsDelivering(D5,P2,3)
 - droneIsDelivering(D5,P2,4) - droneIsDelivering(D5,P2,5)
 + timePerProductDelivered(P2) = -0
 productDeliveringTime(P3): - droneIsDelivering(D1,P3,2)
 - droneIsDelivering(D1,P3,1) - droneIsDelivering(D1,P3,3)
 - droneIsDelivering(D1,P3,4) - droneIsDelivering(D1,P3,5)
 - droneIsDelivering(D2,P3,2) - droneIsDelivering(D2,P3,1)
 - droneIsDelivering(D2,P3,3) - droneIsDelivering(D2,P3,4)
 - droneIsDelivering(D2,P3,5) - droneIsDelivering(D3,P3,2)
 - droneIsDelivering(D3,P3,1) - droneIsDelivering(D3,P3,3)
 - droneIsDelivering(D3,P3,4) - droneIsDelivering(D3,P3,5)
 - droneIsDelivering(D4,P3,2) - droneIsDelivering(D4,P3,1)
 - droneIsDelivering(D4,P3,3) - droneIsDelivering(D4,P3,4)
 - droneIsDelivering(D4,P3,5) - droneIsDelivering(D5,P3,2)
 - droneIsDelivering(D5,P3,1) - droneIsDelivering(D5,P3,3)
 - droneIsDelivering(D5,P3,4) - droneIsDelivering(D5,P3,5)
 + timePerProductDelivered(P3) = -0
 productDeliveringTime(P4): - droneIsDelivering(D1,P4,2)
 - droneIsDelivering(D1,P4,1) - droneIsDelivering(D1,P4,3)
 - droneIsDelivering(D1,P4,4) - droneIsDelivering(D1,P4,5)
 - droneIsDelivering(D2,P4,2) - droneIsDelivering(D2,P4,1)
 - droneIsDelivering(D2,P4,3) - droneIsDelivering(D2,P4,4)
 - droneIsDelivering(D2,P4,5) - droneIsDelivering(D3,P4,2)
 - droneIsDelivering(D3,P4,1) - droneIsDelivering(D3,P4,3)
 - droneIsDelivering(D3,P4,4) - droneIsDelivering(D3,P4,5)
 - droneIsDelivering(D4,P4,2) - droneIsDelivering(D4,P4,1)
 - droneIsDelivering(D4,P4,3) - droneIsDelivering(D4,P4,4)
 - droneIsDelivering(D4,P4,5) - droneIsDelivering(D5,P4,2)
 - droneIsDelivering(D5,P4,1) - droneIsDelivering(D5,P4,3)
 - droneIsDelivering(D5,P4,4) - droneIsDelivering(D5,P4,5)
 + timePerProductDelivered(P4) = -0
 productDeliveringTime(P5): - droneIsDelivering(D1,P5,2)
 - droneIsDelivering(D1,P5,1) - droneIsDelivering(D1,P5,3)
 - droneIsDelivering(D1,P5,4) - droneIsDelivering(D1,P5,5)
 - droneIsDelivering(D2,P5,2) - droneIsDelivering(D2,P5,1)
 - droneIsDelivering(D2,P5,3) - droneIsDelivering(D2,P5,4)
 - droneIsDelivering(D2,P5,5) - droneIsDelivering(D3,P5,2)
 - droneIsDelivering(D3,P5,1) - droneIsDelivering(D3,P5,3)
 - droneIsDelivering(D3,P5,4) - droneIsDelivering(D3,P5,5)
 - droneIsDelivering(D4,P5,2) - droneIsDelivering(D4,P5,1)
 - droneIsDelivering(D4,P5,3) - droneIsDelivering(D4,P5,4)
 - droneIsDelivering(D4,P5,5) - droneIsDelivering(D5,P5,2)
 - droneIsDelivering(D5,P5,1) - droneIsDelivering(D5,P5,3)
 - droneIsDelivering(D5,P5,4) - droneIsDelivering(D5,P5,5)
 + timePerProductDelivered(P5) = -0
 productDeliveringTime(P6): - droneIsDelivering(D1,P6,2)
 - droneIsDelivering(D1,P6,1) - droneIsDelivering(D1,P6,3)
 - droneIsDelivering(D1,P6,4) - droneIsDelivering(D1,P6,5)
 - droneIsDelivering(D2,P6,2) - droneIsDelivering(D2,P6,1)
 - droneIsDelivering(D2,P6,3) - droneIsDelivering(D2,P6,4)
 - droneIsDelivering(D2,P6,5) - droneIsDelivering(D3,P6,2)
 - droneIsDelivering(D3,P6,1) - droneIsDelivering(D3,P6,3)
 - droneIsDelivering(D3,P6,4) - droneIsDelivering(D3,P6,5)
 - droneIsDelivering(D4,P6,2) - droneIsDelivering(D4,P6,1)
 - droneIsDelivering(D4,P6,3) - droneIsDelivering(D4,P6,4)
 - droneIsDelivering(D4,P6,5) - droneIsDelivering(D5,P6,2)
 - droneIsDelivering(D5,P6,1) - droneIsDelivering(D5,P6,3)
 - droneIsDelivering(D5,P6,4) - droneIsDelivering(D5,P6,5)
 + timePerProductDelivered(P6) = -0
 initializePositionX(D1,1): + droneDX(D1,1) = 10
 initializePositionX(D2,1): + droneDX(D2,1) = 5
 initializePositionX(D3,1): + droneDX(D3,1) = 5
 initializePositionX(D4,1): + droneDX(D4,1) = 5
 initializePositionX(D5,1): + droneDX(D5,1) = -0
 initializePositionY(D1,1): + droneDY(D1,1) = 20
 initializePositionY(D2,1): + droneDY(D2,1) = 5
 initializePositionY(D3,1): + droneDY(D3,1) = 5
 initializePositionY(D4,1): + droneDY(D4,1) = 5
 initializePositionY(D5,1): + droneDY(D5,1) = -0
 packageWasCollectedAtRightPositionX1(P1,D1,1): + droneDX(D1,1)
 + 100 droneCollected(D1,P1,1) <= 102
 packageWasCollectedAtRightPositionX1(P1,D1,2): + droneDX(D1,2)
 + 100 droneCollected(D1,P1,2) <= 102
 packageWasCollectedAtRightPositionX1(P1,D1,3): + droneDX(D1,3)
 + 100 droneCollected(D1,P1,3) <= 102
 packageWasCollectedAtRightPositionX1(P1,D1,4): + droneDX(D1,4)
 + 100 droneCollected(D1,P1,4) <= 102
 packageWasCollectedAtRightPositionX1(P1,D1,5): + droneDX(D1,5)
 + 100 droneCollected(D1,P1,5) <= 102
 packageWasCollectedAtRightPositionX1(P1,D2,1): + droneDX(D2,1)
 + 100 droneCollected(D2,P1,1) <= 102
 packageWasCollectedAtRightPositionX1(P1,D2,2): + droneDX(D2,2)
 + 100 droneCollected(D2,P1,2) <= 102
 packageWasCollectedAtRightPositionX1(P1,D2,3): + droneDX(D2,3)
 + 100 droneCollected(D2,P1,3) <= 102
 packageWasCollectedAtRightPositionX1(P1,D2,4): + droneDX(D2,4)
 + 100 droneCollected(D2,P1,4) <= 102
 packageWasCollectedAtRightPositionX1(P1,D2,5): + droneDX(D2,5)
 + 100 droneCollected(D2,P1,5) <= 102
 packageWasCollectedAtRightPositionX1(P1,D3,1): + droneDX(D3,1)
 + 100 droneCollected(D3,P1,1) <= 102
 packageWasCollectedAtRightPositionX1(P1,D3,2): + droneDX(D3,2)
 + 100 droneCollected(D3,P1,2) <= 102
 packageWasCollectedAtRightPositionX1(P1,D3,3): + droneDX(D3,3)
 + 100 droneCollected(D3,P1,3) <= 102
 packageWasCollectedAtRightPositionX1(P1,D3,4): + droneDX(D3,4)
 + 100 droneCollected(D3,P1,4) <= 102
 packageWasCollectedAtRightPositionX1(P1,D3,5): + droneDX(D3,5)
 + 100 droneCollected(D3,P1,5) <= 102
 packageWasCollectedAtRightPositionX1(P1,D4,1): + droneDX(D4,1)
 + 100 droneCollected(D4,P1,1) <= 102
 packageWasCollectedAtRightPositionX1(P1,D4,2): + droneDX(D4,2)
 + 100 droneCollected(D4,P1,2) <= 102
 packageWasCollectedAtRightPositionX1(P1,D4,3): + droneDX(D4,3)
 + 100 droneCollected(D4,P1,3) <= 102
 packageWasCollectedAtRightPositionX1(P1,D4,4): + droneDX(D4,4)
 + 100 droneCollected(D4,P1,4) <= 102
 packageWasCollectedAtRightPositionX1(P1,D4,5): + droneDX(D4,5)
 + 100 droneCollected(D4,P1,5) <= 102
 packageWasCollectedAtRightPositionX1(P1,D5,1): + droneDX(D5,1)
 + 100 droneCollected(D5,P1,1) <= 102
 packageWasCollectedAtRightPositionX1(P1,D5,2): + droneDX(D5,2)
 + 100 droneCollected(D5,P1,2) <= 102
 packageWasCollectedAtRightPositionX1(P1,D5,3): + droneDX(D5,3)
 + 100 droneCollected(D5,P1,3) <= 102
 packageWasCollectedAtRightPositionX1(P1,D5,4): + droneDX(D5,4)
 + 100 droneCollected(D5,P1,4) <= 102
 packageWasCollectedAtRightPositionX1(P1,D5,5): + droneDX(D5,5)
 + 100 droneCollected(D5,P1,5) <= 102
 packageWasCollectedAtRightPositionX1(P2,D1,1): + droneDX(D1,1)
 + 100 droneCollected(D1,P2,1) <= 102
 packageWasCollectedAtRightPositionX1(P2,D1,2): + droneDX(D1,2)
 + 100 droneCollected(D1,P2,2) <= 102
 packageWasCollectedAtRightPositionX1(P2,D1,3): + droneDX(D1,3)
 + 100 droneCollected(D1,P2,3) <= 102
 packageWasCollectedAtRightPositionX1(P2,D1,4): + droneDX(D1,4)
 + 100 droneCollected(D1,P2,4) <= 102
 packageWasCollectedAtRightPositionX1(P2,D1,5): + droneDX(D1,5)
 + 100 droneCollected(D1,P2,5) <= 102
 packageWasCollectedAtRightPositionX1(P2,D2,1): + droneDX(D2,1)
 + 100 droneCollected(D2,P2,1) <= 102
 packageWasCollectedAtRightPositionX1(P2,D2,2): + droneDX(D2,2)
 + 100 droneCollected(D2,P2,2) <= 102
 packageWasCollectedAtRightPositionX1(P2,D2,3): + droneDX(D2,3)
 + 100 droneCollected(D2,P2,3) <= 102
 packageWasCollectedAtRightPositionX1(P2,D2,4): + droneDX(D2,4)
 + 100 droneCollected(D2,P2,4) <= 102
 packageWasCollectedAtRightPositionX1(P2,D2,5): + droneDX(D2,5)
 + 100 droneCollected(D2,P2,5) <= 102
 packageWasCollectedAtRightPositionX1(P2,D3,1): + droneDX(D3,1)
 + 100 droneCollected(D3,P2,1) <= 102
 packageWasCollectedAtRightPositionX1(P2,D3,2): + droneDX(D3,2)
 + 100 droneCollected(D3,P2,2) <= 102
 packageWasCollectedAtRightPositionX1(P2,D3,3): + droneDX(D3,3)
 + 100 droneCollected(D3,P2,3) <= 102
 packageWasCollectedAtRightPositionX1(P2,D3,4): + droneDX(D3,4)
 + 100 droneCollected(D3,P2,4) <= 102
 packageWasCollectedAtRightPositionX1(P2,D3,5): + droneDX(D3,5)
 + 100 droneCollected(D3,P2,5) <= 102
 packageWasCollectedAtRightPositionX1(P2,D4,1): + droneDX(D4,1)
 + 100 droneCollected(D4,P2,1) <= 102
 packageWasCollectedAtRightPositionX1(P2,D4,2): + droneDX(D4,2)
 + 100 droneCollected(D4,P2,2) <= 102
 packageWasCollectedAtRightPositionX1(P2,D4,3): + droneDX(D4,3)
 + 100 droneCollected(D4,P2,3) <= 102
 packageWasCollectedAtRightPositionX1(P2,D4,4): + droneDX(D4,4)
 + 100 droneCollected(D4,P2,4) <= 102
 packageWasCollectedAtRightPositionX1(P2,D4,5): + droneDX(D4,5)
 + 100 droneCollected(D4,P2,5) <= 102
 packageWasCollectedAtRightPositionX1(P2,D5,1): + droneDX(D5,1)
 + 100 droneCollected(D5,P2,1) <= 102
 packageWasCollectedAtRightPositionX1(P2,D5,2): + droneDX(D5,2)
 + 100 droneCollected(D5,P2,2) <= 102
 packageWasCollectedAtRightPositionX1(P2,D5,3): + droneDX(D5,3)
 + 100 droneCollected(D5,P2,3) <= 102
 packageWasCollectedAtRightPositionX1(P2,D5,4): + droneDX(D5,4)
 + 100 droneCollected(D5,P2,4) <= 102
 packageWasCollectedAtRightPositionX1(P2,D5,5): + droneDX(D5,5)
 + 100 droneCollected(D5,P2,5) <= 102
 packageWasCollectedAtRightPositionX1(P3,D1,1): + droneDX(D1,1)
 + 100 droneCollected(D1,P3,1) <= 105
 packageWasCollectedAtRightPositionX1(P3,D1,2): + droneDX(D1,2)
 + 100 droneCollected(D1,P3,2) <= 105
 packageWasCollectedAtRightPositionX1(P3,D1,3): + droneDX(D1,3)
 + 100 droneCollected(D1,P3,3) <= 105
 packageWasCollectedAtRightPositionX1(P3,D1,4): + droneDX(D1,4)
 + 100 droneCollected(D1,P3,4) <= 105
 packageWasCollectedAtRightPositionX1(P3,D1,5): + droneDX(D1,5)
 + 100 droneCollected(D1,P3,5) <= 105
 packageWasCollectedAtRightPositionX1(P3,D2,1): + droneDX(D2,1)
 + 100 droneCollected(D2,P3,1) <= 105
 packageWasCollectedAtRightPositionX1(P3,D2,2): + droneDX(D2,2)
 + 100 droneCollected(D2,P3,2) <= 105
 packageWasCollectedAtRightPositionX1(P3,D2,3): + droneDX(D2,3)
 + 100 droneCollected(D2,P3,3) <= 105
 packageWasCollectedAtRightPositionX1(P3,D2,4): + droneDX(D2,4)
 + 100 droneCollected(D2,P3,4) <= 105
 packageWasCollectedAtRightPositionX1(P3,D2,5): + droneDX(D2,5)
 + 100 droneCollected(D2,P3,5) <= 105
 packageWasCollectedAtRightPositionX1(P3,D3,1): + droneDX(D3,1)
 + 100 droneCollected(D3,P3,1) <= 105
 packageWasCollectedAtRightPositionX1(P3,D3,2): + droneDX(D3,2)
 + 100 droneCollected(D3,P3,2) <= 105
 packageWasCollectedAtRightPositionX1(P3,D3,3): + droneDX(D3,3)
 + 100 droneCollected(D3,P3,3) <= 105
 packageWasCollectedAtRightPositionX1(P3,D3,4): + droneDX(D3,4)
 + 100 droneCollected(D3,P3,4) <= 105
 packageWasCollectedAtRightPositionX1(P3,D3,5): + droneDX(D3,5)
 + 100 droneCollected(D3,P3,5) <= 105
 packageWasCollectedAtRightPositionX1(P3,D4,1): + droneDX(D4,1)
 + 100 droneCollected(D4,P3,1) <= 105
 packageWasCollectedAtRightPositionX1(P3,D4,2): + droneDX(D4,2)
 + 100 droneCollected(D4,P3,2) <= 105
 packageWasCollectedAtRightPositionX1(P3,D4,3): + droneDX(D4,3)
 + 100 droneCollected(D4,P3,3) <= 105
 packageWasCollectedAtRightPositionX1(P3,D4,4): + droneDX(D4,4)
 + 100 droneCollected(D4,P3,4) <= 105
 packageWasCollectedAtRightPositionX1(P3,D4,5): + droneDX(D4,5)
 + 100 droneCollected(D4,P3,5) <= 105
 packageWasCollectedAtRightPositionX1(P3,D5,1): + droneDX(D5,1)
 + 100 droneCollected(D5,P3,1) <= 105
 packageWasCollectedAtRightPositionX1(P3,D5,2): + droneDX(D5,2)
 + 100 droneCollected(D5,P3,2) <= 105
 packageWasCollectedAtRightPositionX1(P3,D5,3): + droneDX(D5,3)
 + 100 droneCollected(D5,P3,3) <= 105
 packageWasCollectedAtRightPositionX1(P3,D5,4): + droneDX(D5,4)
 + 100 droneCollected(D5,P3,4) <= 105
 packageWasCollectedAtRightPositionX1(P3,D5,5): + droneDX(D5,5)
 + 100 droneCollected(D5,P3,5) <= 105
 packageWasCollectedAtRightPositionX1(P4,D1,1): + droneDX(D1,1)
 + 100 droneCollected(D1,P4,1) <= 100
 packageWasCollectedAtRightPositionX1(P4,D1,2): + droneDX(D1,2)
 + 100 droneCollected(D1,P4,2) <= 100
 packageWasCollectedAtRightPositionX1(P4,D1,3): + droneDX(D1,3)
 + 100 droneCollected(D1,P4,3) <= 100
 packageWasCollectedAtRightPositionX1(P4,D1,4): + droneDX(D1,4)
 + 100 droneCollected(D1,P4,4) <= 100
 packageWasCollectedAtRightPositionX1(P4,D1,5): + droneDX(D1,5)
 + 100 droneCollected(D1,P4,5) <= 100
 packageWasCollectedAtRightPositionX1(P4,D2,1): + droneDX(D2,1)
 + 100 droneCollected(D2,P4,1) <= 100
 packageWasCollectedAtRightPositionX1(P4,D2,2): + droneDX(D2,2)
 + 100 droneCollected(D2,P4,2) <= 100
 packageWasCollectedAtRightPositionX1(P4,D2,3): + droneDX(D2,3)
 + 100 droneCollected(D2,P4,3) <= 100
 packageWasCollectedAtRightPositionX1(P4,D2,4): + droneDX(D2,4)
 + 100 droneCollected(D2,P4,4) <= 100
 packageWasCollectedAtRightPositionX1(P4,D2,5): + droneDX(D2,5)
 + 100 droneCollected(D2,P4,5) <= 100
 packageWasCollectedAtRightPositionX1(P4,D3,1): + droneDX(D3,1)
 + 100 droneCollected(D3,P4,1) <= 100
 packageWasCollectedAtRightPositionX1(P4,D3,2): + droneDX(D3,2)
 + 100 droneCollected(D3,P4,2) <= 100
 packageWasCollectedAtRightPositionX1(P4,D3,3): + droneDX(D3,3)
 + 100 droneCollected(D3,P4,3) <= 100
 packageWasCollectedAtRightPositionX1(P4,D3,4): + droneDX(D3,4)
 + 100 droneCollected(D3,P4,4) <= 100
 packageWasCollectedAtRightPositionX1(P4,D3,5): + droneDX(D3,5)
 + 100 droneCollected(D3,P4,5) <= 100
 packageWasCollectedAtRightPositionX1(P4,D4,1): + droneDX(D4,1)
 + 100 droneCollected(D4,P4,1) <= 100
 packageWasCollectedAtRightPositionX1(P4,D4,2): + droneDX(D4,2)
 + 100 droneCollected(D4,P4,2) <= 100
 packageWasCollectedAtRightPositionX1(P4,D4,3): + droneDX(D4,3)
 + 100 droneCollected(D4,P4,3) <= 100
 packageWasCollectedAtRightPositionX1(P4,D4,4): + droneDX(D4,4)
 + 100 droneCollected(D4,P4,4) <= 100
 packageWasCollectedAtRightPositionX1(P4,D4,5): + droneDX(D4,5)
 + 100 droneCollected(D4,P4,5) <= 100
 packageWasCollectedAtRightPositionX1(P4,D5,1): + droneDX(D5,1)
 + 100 droneCollected(D5,P4,1) <= 100
 packageWasCollectedAtRightPositionX1(P4,D5,2): + droneDX(D5,2)
 + 100 droneCollected(D5,P4,2) <= 100
 packageWasCollectedAtRightPositionX1(P4,D5,3): + droneDX(D5,3)
 + 100 droneCollected(D5,P4,3) <= 100
 packageWasCollectedAtRightPositionX1(P4,D5,4): + droneDX(D5,4)
 + 100 droneCollected(D5,P4,4) <= 100
 packageWasCollectedAtRightPositionX1(P4,D5,5): + droneDX(D5,5)
 + 100 droneCollected(D5,P4,5) <= 100
 packageWasCollectedAtRightPositionX1(P5,D1,1): + droneDX(D1,1)
 + 100 droneCollected(D1,P5,1) <= 120
 packageWasCollectedAtRightPositionX1(P5,D1,2): + droneDX(D1,2)
 + 100 droneCollected(D1,P5,2) <= 120
 packageWasCollectedAtRightPositionX1(P5,D1,3): + droneDX(D1,3)
 + 100 droneCollected(D1,P5,3) <= 120
 packageWasCollectedAtRightPositionX1(P5,D1,4): + droneDX(D1,4)
 + 100 droneCollected(D1,P5,4) <= 120
 packageWasCollectedAtRightPositionX1(P5,D1,5): + droneDX(D1,5)
 + 100 droneCollected(D1,P5,5) <= 120
 packageWasCollectedAtRightPositionX1(P5,D2,1): + droneDX(D2,1)
 + 100 droneCollected(D2,P5,1) <= 120
 packageWasCollectedAtRightPositionX1(P5,D2,2): + droneDX(D2,2)
 + 100 droneCollected(D2,P5,2) <= 120
 packageWasCollectedAtRightPositionX1(P5,D2,3): + droneDX(D2,3)
 + 100 droneCollected(D2,P5,3) <= 120
 packageWasCollectedAtRightPositionX1(P5,D2,4): + droneDX(D2,4)
 + 100 droneCollected(D2,P5,4) <= 120
 packageWasCollectedAtRightPositionX1(P5,D2,5): + droneDX(D2,5)
 + 100 droneCollected(D2,P5,5) <= 120
 packageWasCollectedAtRightPositionX1(P5,D3,1): + droneDX(D3,1)
 + 100 droneCollected(D3,P5,1) <= 120
 packageWasCollectedAtRightPositionX1(P5,D3,2): + droneDX(D3,2)
 + 100 droneCollected(D3,P5,2) <= 120
 packageWasCollectedAtRightPositionX1(P5,D3,3): + droneDX(D3,3)
 + 100 droneCollected(D3,P5,3) <= 120
 packageWasCollectedAtRightPositionX1(P5,D3,4): + droneDX(D3,4)
 + 100 droneCollected(D3,P5,4) <= 120
 packageWasCollectedAtRightPositionX1(P5,D3,5): + droneDX(D3,5)
 + 100 droneCollected(D3,P5,5) <= 120
 packageWasCollectedAtRightPositionX1(P5,D4,1): + droneDX(D4,1)
 + 100 droneCollected(D4,P5,1) <= 120
 packageWasCollectedAtRightPositionX1(P5,D4,2): + droneDX(D4,2)
 + 100 droneCollected(D4,P5,2) <= 120
 packageWasCollectedAtRightPositionX1(P5,D4,3): + droneDX(D4,3)
 + 100 droneCollected(D4,P5,3) <= 120
 packageWasCollectedAtRightPositionX1(P5,D4,4): + droneDX(D4,4)
 + 100 droneCollected(D4,P5,4) <= 120
 packageWasCollectedAtRightPositionX1(P5,D4,5): + droneDX(D4,5)
 + 100 droneCollected(D4,P5,5) <= 120
 packageWasCollectedAtRightPositionX1(P5,D5,1): + droneDX(D5,1)
 + 100 droneCollected(D5,P5,1) <= 120
 packageWasCollectedAtRightPositionX1(P5,D5,2): + droneDX(D5,2)
 + 100 droneCollected(D5,P5,2) <= 120
 packageWasCollectedAtRightPositionX1(P5,D5,3): + droneDX(D5,3)
 + 100 droneCollected(D5,P5,3) <= 120
 packageWasCollectedAtRightPositionX1(P5,D5,4): + droneDX(D5,4)
 + 100 droneCollected(D5,P5,4) <= 120
 packageWasCollectedAtRightPositionX1(P5,D5,5): + droneDX(D5,5)
 + 100 droneCollected(D5,P5,5) <= 120
 packageWasCollectedAtRightPositionX1(P6,D1,1): + droneDX(D1,1)
 + 100 droneCollected(D1,P6,1) <= 110
 packageWasCollectedAtRightPositionX1(P6,D1,2): + droneDX(D1,2)
 + 100 droneCollected(D1,P6,2) <= 110
 packageWasCollectedAtRightPositionX1(P6,D1,3): + droneDX(D1,3)
 + 100 droneCollected(D1,P6,3) <= 110
 packageWasCollectedAtRightPositionX1(P6,D1,4): + droneDX(D1,4)
 + 100 droneCollected(D1,P6,4) <= 110
 packageWasCollectedAtRightPositionX1(P6,D1,5): + droneDX(D1,5)
 + 100 droneCollected(D1,P6,5) <= 110
 packageWasCollectedAtRightPositionX1(P6,D2,1): + droneDX(D2,1)
 + 100 droneCollected(D2,P6,1) <= 110
 packageWasCollectedAtRightPositionX1(P6,D2,2): + droneDX(D2,2)
 + 100 droneCollected(D2,P6,2) <= 110
 packageWasCollectedAtRightPositionX1(P6,D2,3): + droneDX(D2,3)
 + 100 droneCollected(D2,P6,3) <= 110
 packageWasCollectedAtRightPositionX1(P6,D2,4): + droneDX(D2,4)
 + 100 droneCollected(D2,P6,4) <= 110
 packageWasCollectedAtRightPositionX1(P6,D2,5): + droneDX(D2,5)
 + 100 droneCollected(D2,P6,5) <= 110
 packageWasCollectedAtRightPositionX1(P6,D3,1): + droneDX(D3,1)
 + 100 droneCollected(D3,P6,1) <= 110
 packageWasCollectedAtRightPositionX1(P6,D3,2): + droneDX(D3,2)
 + 100 droneCollected(D3,P6,2) <= 110
 packageWasCollectedAtRightPositionX1(P6,D3,3): + droneDX(D3,3)
 + 100 droneCollected(D3,P6,3) <= 110
 packageWasCollectedAtRightPositionX1(P6,D3,4): + droneDX(D3,4)
 + 100 droneCollected(D3,P6,4) <= 110
 packageWasCollectedAtRightPositionX1(P6,D3,5): + droneDX(D3,5)
 + 100 droneCollected(D3,P6,5) <= 110
 packageWasCollectedAtRightPositionX1(P6,D4,1): + droneDX(D4,1)
 + 100 droneCollected(D4,P6,1) <= 110
 packageWasCollectedAtRightPositionX1(P6,D4,2): + droneDX(D4,2)
 + 100 droneCollected(D4,P6,2) <= 110
 packageWasCollectedAtRightPositionX1(P6,D4,3): + droneDX(D4,3)
 + 100 droneCollected(D4,P6,3) <= 110
 packageWasCollectedAtRightPositionX1(P6,D4,4): + droneDX(D4,4)
 + 100 droneCollected(D4,P6,4) <= 110
 packageWasCollectedAtRightPositionX1(P6,D4,5): + droneDX(D4,5)
 + 100 droneCollected(D4,P6,5) <= 110
 packageWasCollectedAtRightPositionX1(P6,D5,1): + droneDX(D5,1)
 + 100 droneCollected(D5,P6,1) <= 110
 packageWasCollectedAtRightPositionX1(P6,D5,2): + droneDX(D5,2)
 + 100 droneCollected(D5,P6,2) <= 110
 packageWasCollectedAtRightPositionX1(P6,D5,3): + droneDX(D5,3)
 + 100 droneCollected(D5,P6,3) <= 110
 packageWasCollectedAtRightPositionX1(P6,D5,4): + droneDX(D5,4)
 + 100 droneCollected(D5,P6,4) <= 110
 packageWasCollectedAtRightPositionX1(P6,D5,5): + droneDX(D5,5)
 + 100 droneCollected(D5,P6,5) <= 110
 packageWasCollectedAtRightPositionX2(P1,D1,1): - droneDX(D1,1)
 + 100 droneCollected(D1,P1,1) <= 98
 packageWasCollectedAtRightPositionX2(P1,D1,2): - droneDX(D1,2)
 + 100 droneCollected(D1,P1,2) <= 98
 packageWasCollectedAtRightPositionX2(P1,D1,3): - droneDX(D1,3)
 + 100 droneCollected(D1,P1,3) <= 98
 packageWasCollectedAtRightPositionX2(P1,D1,4): - droneDX(D1,4)
 + 100 droneCollected(D1,P1,4) <= 98
 packageWasCollectedAtRightPositionX2(P1,D1,5): - droneDX(D1,5)
 + 100 droneCollected(D1,P1,5) <= 98
 packageWasCollectedAtRightPositionX2(P1,D2,1): - droneDX(D2,1)
 + 100 droneCollected(D2,P1,1) <= 98
 packageWasCollectedAtRightPositionX2(P1,D2,2): - droneDX(D2,2)
 + 100 droneCollected(D2,P1,2) <= 98
 packageWasCollectedAtRightPositionX2(P1,D2,3): - droneDX(D2,3)
 + 100 droneCollected(D2,P1,3) <= 98
 packageWasCollectedAtRightPositionX2(P1,D2,4): - droneDX(D2,4)
 + 100 droneCollected(D2,P1,4) <= 98
 packageWasCollectedAtRightPositionX2(P1,D2,5): - droneDX(D2,5)
 + 100 droneCollected(D2,P1,5) <= 98
 packageWasCollectedAtRightPositionX2(P1,D3,1): - droneDX(D3,1)
 + 100 droneCollected(D3,P1,1) <= 98
 packageWasCollectedAtRightPositionX2(P1,D3,2): - droneDX(D3,2)
 + 100 droneCollected(D3,P1,2) <= 98
 packageWasCollectedAtRightPositionX2(P1,D3,3): - droneDX(D3,3)
 + 100 droneCollected(D3,P1,3) <= 98
 packageWasCollectedAtRightPositionX2(P1,D3,4): - droneDX(D3,4)
 + 100 droneCollected(D3,P1,4) <= 98
 packageWasCollectedAtRightPositionX2(P1,D3,5): - droneDX(D3,5)
 + 100 droneCollected(D3,P1,5) <= 98
 packageWasCollectedAtRightPositionX2(P1,D4,1): - droneDX(D4,1)
 + 100 droneCollected(D4,P1,1) <= 98
 packageWasCollectedAtRightPositionX2(P1,D4,2): - droneDX(D4,2)
 + 100 droneCollected(D4,P1,2) <= 98
 packageWasCollectedAtRightPositionX2(P1,D4,3): - droneDX(D4,3)
 + 100 droneCollected(D4,P1,3) <= 98
 packageWasCollectedAtRightPositionX2(P1,D4,4): - droneDX(D4,4)
 + 100 droneCollected(D4,P1,4) <= 98
 packageWasCollectedAtRightPositionX2(P1,D4,5): - droneDX(D4,5)
 + 100 droneCollected(D4,P1,5) <= 98
 packageWasCollectedAtRightPositionX2(P1,D5,1): - droneDX(D5,1)
 + 100 droneCollected(D5,P1,1) <= 98
 packageWasCollectedAtRightPositionX2(P1,D5,2): - droneDX(D5,2)
 + 100 droneCollected(D5,P1,2) <= 98
 packageWasCollectedAtRightPositionX2(P1,D5,3): - droneDX(D5,3)
 + 100 droneCollected(D5,P1,3) <= 98
 packageWasCollectedAtRightPositionX2(P1,D5,4): - droneDX(D5,4)
 + 100 droneCollected(D5,P1,4) <= 98
 packageWasCollectedAtRightPositionX2(P1,D5,5): - droneDX(D5,5)
 + 100 droneCollected(D5,P1,5) <= 98
 packageWasCollectedAtRightPositionX2(P2,D1,1): - droneDX(D1,1)
 + 100 droneCollected(D1,P2,1) <= 98
 packageWasCollectedAtRightPositionX2(P2,D1,2): - droneDX(D1,2)
 + 100 droneCollected(D1,P2,2) <= 98
 packageWasCollectedAtRightPositionX2(P2,D1,3): - droneDX(D1,3)
 + 100 droneCollected(D1,P2,3) <= 98
 packageWasCollectedAtRightPositionX2(P2,D1,4): - droneDX(D1,4)
 + 100 droneCollected(D1,P2,4) <= 98
 packageWasCollectedAtRightPositionX2(P2,D1,5): - droneDX(D1,5)
 + 100 droneCollected(D1,P2,5) <= 98
 packageWasCollectedAtRightPositionX2(P2,D2,1): - droneDX(D2,1)
 + 100 droneCollected(D2,P2,1) <= 98
 packageWasCollectedAtRightPositionX2(P2,D2,2): - droneDX(D2,2)
 + 100 droneCollected(D2,P2,2) <= 98
 packageWasCollectedAtRightPositionX2(P2,D2,3): - droneDX(D2,3)
 + 100 droneCollected(D2,P2,3) <= 98
 packageWasCollectedAtRightPositionX2(P2,D2,4): - droneDX(D2,4)
 + 100 droneCollected(D2,P2,4) <= 98
 packageWasCollectedAtRightPositionX2(P2,D2,5): - droneDX(D2,5)
 + 100 droneCollected(D2,P2,5) <= 98
 packageWasCollectedAtRightPositionX2(P2,D3,1): - droneDX(D3,1)
 + 100 droneCollected(D3,P2,1) <= 98
 packageWasCollectedAtRightPositionX2(P2,D3,2): - droneDX(D3,2)
 + 100 droneCollected(D3,P2,2) <= 98
 packageWasCollectedAtRightPositionX2(P2,D3,3): - droneDX(D3,3)
 + 100 droneCollected(D3,P2,3) <= 98
 packageWasCollectedAtRightPositionX2(P2,D3,4): - droneDX(D3,4)
 + 100 droneCollected(D3,P2,4) <= 98
 packageWasCollectedAtRightPositionX2(P2,D3,5): - droneDX(D3,5)
 + 100 droneCollected(D3,P2,5) <= 98
 packageWasCollectedAtRightPositionX2(P2,D4,1): - droneDX(D4,1)
 + 100 droneCollected(D4,P2,1) <= 98
 packageWasCollectedAtRightPositionX2(P2,D4,2): - droneDX(D4,2)
 + 100 droneCollected(D4,P2,2) <= 98
 packageWasCollectedAtRightPositionX2(P2,D4,3): - droneDX(D4,3)
 + 100 droneCollected(D4,P2,3) <= 98
 packageWasCollectedAtRightPositionX2(P2,D4,4): - droneDX(D4,4)
 + 100 droneCollected(D4,P2,4) <= 98
 packageWasCollectedAtRightPositionX2(P2,D4,5): - droneDX(D4,5)
 + 100 droneCollected(D4,P2,5) <= 98
 packageWasCollectedAtRightPositionX2(P2,D5,1): - droneDX(D5,1)
 + 100 droneCollected(D5,P2,1) <= 98
 packageWasCollectedAtRightPositionX2(P2,D5,2): - droneDX(D5,2)
 + 100 droneCollected(D5,P2,2) <= 98
 packageWasCollectedAtRightPositionX2(P2,D5,3): - droneDX(D5,3)
 + 100 droneCollected(D5,P2,3) <= 98
 packageWasCollectedAtRightPositionX2(P2,D5,4): - droneDX(D5,4)
 + 100 droneCollected(D5,P2,4) <= 98
 packageWasCollectedAtRightPositionX2(P2,D5,5): - droneDX(D5,5)
 + 100 droneCollected(D5,P2,5) <= 98
 packageWasCollectedAtRightPositionX2(P3,D1,1): - droneDX(D1,1)
 + 100 droneCollected(D1,P3,1) <= 95
 packageWasCollectedAtRightPositionX2(P3,D1,2): - droneDX(D1,2)
 + 100 droneCollected(D1,P3,2) <= 95
 packageWasCollectedAtRightPositionX2(P3,D1,3): - droneDX(D1,3)
 + 100 droneCollected(D1,P3,3) <= 95
 packageWasCollectedAtRightPositionX2(P3,D1,4): - droneDX(D1,4)
 + 100 droneCollected(D1,P3,4) <= 95
 packageWasCollectedAtRightPositionX2(P3,D1,5): - droneDX(D1,5)
 + 100 droneCollected(D1,P3,5) <= 95
 packageWasCollectedAtRightPositionX2(P3,D2,1): - droneDX(D2,1)
 + 100 droneCollected(D2,P3,1) <= 95
 packageWasCollectedAtRightPositionX2(P3,D2,2): - droneDX(D2,2)
 + 100 droneCollected(D2,P3,2) <= 95
 packageWasCollectedAtRightPositionX2(P3,D2,3): - droneDX(D2,3)
 + 100 droneCollected(D2,P3,3) <= 95
 packageWasCollectedAtRightPositionX2(P3,D2,4): - droneDX(D2,4)
 + 100 droneCollected(D2,P3,4) <= 95
 packageWasCollectedAtRightPositionX2(P3,D2,5): - droneDX(D2,5)
 + 100 droneCollected(D2,P3,5) <= 95
 packageWasCollectedAtRightPositionX2(P3,D3,1): - droneDX(D3,1)
 + 100 droneCollected(D3,P3,1) <= 95
 packageWasCollectedAtRightPositionX2(P3,D3,2): - droneDX(D3,2)
 + 100 droneCollected(D3,P3,2) <= 95
 packageWasCollectedAtRightPositionX2(P3,D3,3): - droneDX(D3,3)
 + 100 droneCollected(D3,P3,3) <= 95
 packageWasCollectedAtRightPositionX2(P3,D3,4): - droneDX(D3,4)
 + 100 droneCollected(D3,P3,4) <= 95
 packageWasCollectedAtRightPositionX2(P3,D3,5): - droneDX(D3,5)
 + 100 droneCollected(D3,P3,5) <= 95
 packageWasCollectedAtRightPositionX2(P3,D4,1): - droneDX(D4,1)
 + 100 droneCollected(D4,P3,1) <= 95
 packageWasCollectedAtRightPositionX2(P3,D4,2): - droneDX(D4,2)
 + 100 droneCollected(D4,P3,2) <= 95
 packageWasCollectedAtRightPositionX2(P3,D4,3): - droneDX(D4,3)
 + 100 droneCollected(D4,P3,3) <= 95
 packageWasCollectedAtRightPositionX2(P3,D4,4): - droneDX(D4,4)
 + 100 droneCollected(D4,P3,4) <= 95
 packageWasCollectedAtRightPositionX2(P3,D4,5): - droneDX(D4,5)
 + 100 droneCollected(D4,P3,5) <= 95
 packageWasCollectedAtRightPositionX2(P3,D5,1): - droneDX(D5,1)
 + 100 droneCollected(D5,P3,1) <= 95
 packageWasCollectedAtRightPositionX2(P3,D5,2): - droneDX(D5,2)
 + 100 droneCollected(D5,P3,2) <= 95
 packageWasCollectedAtRightPositionX2(P3,D5,3): - droneDX(D5,3)
 + 100 droneCollected(D5,P3,3) <= 95
 packageWasCollectedAtRightPositionX2(P3,D5,4): - droneDX(D5,4)
 + 100 droneCollected(D5,P3,4) <= 95
 packageWasCollectedAtRightPositionX2(P3,D5,5): - droneDX(D5,5)
 + 100 droneCollected(D5,P3,5) <= 95
 packageWasCollectedAtRightPositionX2(P4,D1,1): - droneDX(D1,1)
 + 100 droneCollected(D1,P4,1) <= 100
 packageWasCollectedAtRightPositionX2(P4,D1,2): - droneDX(D1,2)
 + 100 droneCollected(D1,P4,2) <= 100
 packageWasCollectedAtRightPositionX2(P4,D1,3): - droneDX(D1,3)
 + 100 droneCollected(D1,P4,3) <= 100
 packageWasCollectedAtRightPositionX2(P4,D1,4): - droneDX(D1,4)
 + 100 droneCollected(D1,P4,4) <= 100
 packageWasCollectedAtRightPositionX2(P4,D1,5): - droneDX(D1,5)
 + 100 droneCollected(D1,P4,5) <= 100
 packageWasCollectedAtRightPositionX2(P4,D2,1): - droneDX(D2,1)
 + 100 droneCollected(D2,P4,1) <= 100
 packageWasCollectedAtRightPositionX2(P4,D2,2): - droneDX(D2,2)
 + 100 droneCollected(D2,P4,2) <= 100
 packageWasCollectedAtRightPositionX2(P4,D2,3): - droneDX(D2,3)
 + 100 droneCollected(D2,P4,3) <= 100
 packageWasCollectedAtRightPositionX2(P4,D2,4): - droneDX(D2,4)
 + 100 droneCollected(D2,P4,4) <= 100
 packageWasCollectedAtRightPositionX2(P4,D2,5): - droneDX(D2,5)
 + 100 droneCollected(D2,P4,5) <= 100
 packageWasCollectedAtRightPositionX2(P4,D3,1): - droneDX(D3,1)
 + 100 droneCollected(D3,P4,1) <= 100
 packageWasCollectedAtRightPositionX2(P4,D3,2): - droneDX(D3,2)
 + 100 droneCollected(D3,P4,2) <= 100
 packageWasCollectedAtRightPositionX2(P4,D3,3): - droneDX(D3,3)
 + 100 droneCollected(D3,P4,3) <= 100
 packageWasCollectedAtRightPositionX2(P4,D3,4): - droneDX(D3,4)
 + 100 droneCollected(D3,P4,4) <= 100
 packageWasCollectedAtRightPositionX2(P4,D3,5): - droneDX(D3,5)
 + 100 droneCollected(D3,P4,5) <= 100
 packageWasCollectedAtRightPositionX2(P4,D4,1): - droneDX(D4,1)
 + 100 droneCollected(D4,P4,1) <= 100
 packageWasCollectedAtRightPositionX2(P4,D4,2): - droneDX(D4,2)
 + 100 droneCollected(D4,P4,2) <= 100
 packageWasCollectedAtRightPositionX2(P4,D4,3): - droneDX(D4,3)
 + 100 droneCollected(D4,P4,3) <= 100
 packageWasCollectedAtRightPositionX2(P4,D4,4): - droneDX(D4,4)
 + 100 droneCollected(D4,P4,4) <= 100
 packageWasCollectedAtRightPositionX2(P4,D4,5): - droneDX(D4,5)
 + 100 droneCollected(D4,P4,5) <= 100
 packageWasCollectedAtRightPositionX2(P4,D5,1): - droneDX(D5,1)
 + 100 droneCollected(D5,P4,1) <= 100
 packageWasCollectedAtRightPositionX2(P4,D5,2): - droneDX(D5,2)
 + 100 droneCollected(D5,P4,2) <= 100
 packageWasCollectedAtRightPositionX2(P4,D5,3): - droneDX(D5,3)
 + 100 droneCollected(D5,P4,3) <= 100
 packageWasCollectedAtRightPositionX2(P4,D5,4): - droneDX(D5,4)
 + 100 droneCollected(D5,P4,4) <= 100
 packageWasCollectedAtRightPositionX2(P4,D5,5): - droneDX(D5,5)
 + 100 droneCollected(D5,P4,5) <= 100
 packageWasCollectedAtRightPositionX2(P5,D1,1): - droneDX(D1,1)
 + 100 droneCollected(D1,P5,1) <= 80
 packageWasCollectedAtRightPositionX2(P5,D1,2): - droneDX(D1,2)
 + 100 droneCollected(D1,P5,2) <= 80
 packageWasCollectedAtRightPositionX2(P5,D1,3): - droneDX(D1,3)
 + 100 droneCollected(D1,P5,3) <= 80
 packageWasCollectedAtRightPositionX2(P5,D1,4): - droneDX(D1,4)
 + 100 droneCollected(D1,P5,4) <= 80
 packageWasCollectedAtRightPositionX2(P5,D1,5): - droneDX(D1,5)
 + 100 droneCollected(D1,P5,5) <= 80
 packageWasCollectedAtRightPositionX2(P5,D2,1): - droneDX(D2,1)
 + 100 droneCollected(D2,P5,1) <= 80
 packageWasCollectedAtRightPositionX2(P5,D2,2): - droneDX(D2,2)
 + 100 droneCollected(D2,P5,2) <= 80
 packageWasCollectedAtRightPositionX2(P5,D2,3): - droneDX(D2,3)
 + 100 droneCollected(D2,P5,3) <= 80
 packageWasCollectedAtRightPositionX2(P5,D2,4): - droneDX(D2,4)
 + 100 droneCollected(D2,P5,4) <= 80
 packageWasCollectedAtRightPositionX2(P5,D2,5): - droneDX(D2,5)
 + 100 droneCollected(D2,P5,5) <= 80
 packageWasCollectedAtRightPositionX2(P5,D3,1): - droneDX(D3,1)
 + 100 droneCollected(D3,P5,1) <= 80
 packageWasCollectedAtRightPositionX2(P5,D3,2): - droneDX(D3,2)
 + 100 droneCollected(D3,P5,2) <= 80
 packageWasCollectedAtRightPositionX2(P5,D3,3): - droneDX(D3,3)
 + 100 droneCollected(D3,P5,3) <= 80
 packageWasCollectedAtRightPositionX2(P5,D3,4): - droneDX(D3,4)
 + 100 droneCollected(D3,P5,4) <= 80
 packageWasCollectedAtRightPositionX2(P5,D3,5): - droneDX(D3,5)
 + 100 droneCollected(D3,P5,5) <= 80
 packageWasCollectedAtRightPositionX2(P5,D4,1): - droneDX(D4,1)
 + 100 droneCollected(D4,P5,1) <= 80
 packageWasCollectedAtRightPositionX2(P5,D4,2): - droneDX(D4,2)
 + 100 droneCollected(D4,P5,2) <= 80
 packageWasCollectedAtRightPositionX2(P5,D4,3): - droneDX(D4,3)
 + 100 droneCollected(D4,P5,3) <= 80
 packageWasCollectedAtRightPositionX2(P5,D4,4): - droneDX(D4,4)
 + 100 droneCollected(D4,P5,4) <= 80
 packageWasCollectedAtRightPositionX2(P5,D4,5): - droneDX(D4,5)
 + 100 droneCollected(D4,P5,5) <= 80
 packageWasCollectedAtRightPositionX2(P5,D5,1): - droneDX(D5,1)
 + 100 droneCollected(D5,P5,1) <= 80
 packageWasCollectedAtRightPositionX2(P5,D5,2): - droneDX(D5,2)
 + 100 droneCollected(D5,P5,2) <= 80
 packageWasCollectedAtRightPositionX2(P5,D5,3): - droneDX(D5,3)
 + 100 droneCollected(D5,P5,3) <= 80
 packageWasCollectedAtRightPositionX2(P5,D5,4): - droneDX(D5,4)
 + 100 droneCollected(D5,P5,4) <= 80
 packageWasCollectedAtRightPositionX2(P5,D5,5): - droneDX(D5,5)
 + 100 droneCollected(D5,P5,5) <= 80
 packageWasCollectedAtRightPositionX2(P6,D1,1): - droneDX(D1,1)
 + 100 droneCollected(D1,P6,1) <= 90
 packageWasCollectedAtRightPositionX2(P6,D1,2): - droneDX(D1,2)
 + 100 droneCollected(D1,P6,2) <= 90
 packageWasCollectedAtRightPositionX2(P6,D1,3): - droneDX(D1,3)
 + 100 droneCollected(D1,P6,3) <= 90
 packageWasCollectedAtRightPositionX2(P6,D1,4): - droneDX(D1,4)
 + 100 droneCollected(D1,P6,4) <= 90
 packageWasCollectedAtRightPositionX2(P6,D1,5): - droneDX(D1,5)
 + 100 droneCollected(D1,P6,5) <= 90
 packageWasCollectedAtRightPositionX2(P6,D2,1): - droneDX(D2,1)
 + 100 droneCollected(D2,P6,1) <= 90
 packageWasCollectedAtRightPositionX2(P6,D2,2): - droneDX(D2,2)
 + 100 droneCollected(D2,P6,2) <= 90
 packageWasCollectedAtRightPositionX2(P6,D2,3): - droneDX(D2,3)
 + 100 droneCollected(D2,P6,3) <= 90
 packageWasCollectedAtRightPositionX2(P6,D2,4): - droneDX(D2,4)
 + 100 droneCollected(D2,P6,4) <= 90
 packageWasCollectedAtRightPositionX2(P6,D2,5): - droneDX(D2,5)
 + 100 droneCollected(D2,P6,5) <= 90
 packageWasCollectedAtRightPositionX2(P6,D3,1): - droneDX(D3,1)
 + 100 droneCollected(D3,P6,1) <= 90
 packageWasCollectedAtRightPositionX2(P6,D3,2): - droneDX(D3,2)
 + 100 droneCollected(D3,P6,2) <= 90
 packageWasCollectedAtRightPositionX2(P6,D3,3): - droneDX(D3,3)
 + 100 droneCollected(D3,P6,3) <= 90
 packageWasCollectedAtRightPositionX2(P6,D3,4): - droneDX(D3,4)
 + 100 droneCollected(D3,P6,4) <= 90
 packageWasCollectedAtRightPositionX2(P6,D3,5): - droneDX(D3,5)
 + 100 droneCollected(D3,P6,5) <= 90
 packageWasCollectedAtRightPositionX2(P6,D4,1): - droneDX(D4,1)
 + 100 droneCollected(D4,P6,1) <= 90
 packageWasCollectedAtRightPositionX2(P6,D4,2): - droneDX(D4,2)
 + 100 droneCollected(D4,P6,2) <= 90
 packageWasCollectedAtRightPositionX2(P6,D4,3): - droneDX(D4,3)
 + 100 droneCollected(D4,P6,3) <= 90
 packageWasCollectedAtRightPositionX2(P6,D4,4): - droneDX(D4,4)
 + 100 droneCollected(D4,P6,4) <= 90
 packageWasCollectedAtRightPositionX2(P6,D4,5): - droneDX(D4,5)
 + 100 droneCollected(D4,P6,5) <= 90
 packageWasCollectedAtRightPositionX2(P6,D5,1): - droneDX(D5,1)
 + 100 droneCollected(D5,P6,1) <= 90
 packageWasCollectedAtRightPositionX2(P6,D5,2): - droneDX(D5,2)
 + 100 droneCollected(D5,P6,2) <= 90
 packageWasCollectedAtRightPositionX2(P6,D5,3): - droneDX(D5,3)
 + 100 droneCollected(D5,P6,3) <= 90
 packageWasCollectedAtRightPositionX2(P6,D5,4): - droneDX(D5,4)
 + 100 droneCollected(D5,P6,4) <= 90
 packageWasCollectedAtRightPositionX2(P6,D5,5): - droneDX(D5,5)
 + 100 droneCollected(D5,P6,5) <= 90
 packageWasCollectedAtRightPositionY1(P1,D1,1): + droneDY(D1,1)
 + 100 droneCollected(D1,P1,1) <= 110
 packageWasCollectedAtRightPositionY1(P1,D1,2): + droneDY(D1,2)
 + 100 droneCollected(D1,P1,2) <= 110
 packageWasCollectedAtRightPositionY1(P1,D1,3): + droneDY(D1,3)
 + 100 droneCollected(D1,P1,3) <= 110
 packageWasCollectedAtRightPositionY1(P1,D1,4): + droneDY(D1,4)
 + 100 droneCollected(D1,P1,4) <= 110
 packageWasCollectedAtRightPositionY1(P1,D1,5): + droneDY(D1,5)
 + 100 droneCollected(D1,P1,5) <= 110
 packageWasCollectedAtRightPositionY1(P1,D2,1): + droneDY(D2,1)
 + 100 droneCollected(D2,P1,1) <= 110
 packageWasCollectedAtRightPositionY1(P1,D2,2): + droneDY(D2,2)
 + 100 droneCollected(D2,P1,2) <= 110
 packageWasCollectedAtRightPositionY1(P1,D2,3): + droneDY(D2,3)
 + 100 droneCollected(D2,P1,3) <= 110
 packageWasCollectedAtRightPositionY1(P1,D2,4): + droneDY(D2,4)
 + 100 droneCollected(D2,P1,4) <= 110
 packageWasCollectedAtRightPositionY1(P1,D2,5): + droneDY(D2,5)
 + 100 droneCollected(D2,P1,5) <= 110
 packageWasCollectedAtRightPositionY1(P1,D3,1): + droneDY(D3,1)
 + 100 droneCollected(D3,P1,1) <= 110
 packageWasCollectedAtRightPositionY1(P1,D3,2): + droneDY(D3,2)
 + 100 droneCollected(D3,P1,2) <= 110
 packageWasCollectedAtRightPositionY1(P1,D3,3): + droneDY(D3,3)
 + 100 droneCollected(D3,P1,3) <= 110
 packageWasCollectedAtRightPositionY1(P1,D3,4): + droneDY(D3,4)
 + 100 droneCollected(D3,P1,4) <= 110
 packageWasCollectedAtRightPositionY1(P1,D3,5): + droneDY(D3,5)
 + 100 droneCollected(D3,P1,5) <= 110
 packageWasCollectedAtRightPositionY1(P1,D4,1): + droneDY(D4,1)
 + 100 droneCollected(D4,P1,1) <= 110
 packageWasCollectedAtRightPositionY1(P1,D4,2): + droneDY(D4,2)
 + 100 droneCollected(D4,P1,2) <= 110
 packageWasCollectedAtRightPositionY1(P1,D4,3): + droneDY(D4,3)
 + 100 droneCollected(D4,P1,3) <= 110
 packageWasCollectedAtRightPositionY1(P1,D4,4): + droneDY(D4,4)
 + 100 droneCollected(D4,P1,4) <= 110
 packageWasCollectedAtRightPositionY1(P1,D4,5): + droneDY(D4,5)
 + 100 droneCollected(D4,P1,5) <= 110
 packageWasCollectedAtRightPositionY1(P1,D5,1): + droneDY(D5,1)
 + 100 droneCollected(D5,P1,1) <= 110
 packageWasCollectedAtRightPositionY1(P1,D5,2): + droneDY(D5,2)
 + 100 droneCollected(D5,P1,2) <= 110
 packageWasCollectedAtRightPositionY1(P1,D5,3): + droneDY(D5,3)
 + 100 droneCollected(D5,P1,3) <= 110
 packageWasCollectedAtRightPositionY1(P1,D5,4): + droneDY(D5,4)
 + 100 droneCollected(D5,P1,4) <= 110
 packageWasCollectedAtRightPositionY1(P1,D5,5): + droneDY(D5,5)
 + 100 droneCollected(D5,P1,5) <= 110
 packageWasCollectedAtRightPositionY1(P2,D1,1): + droneDY(D1,1)
 + 100 droneCollected(D1,P2,1) <= 110
 packageWasCollectedAtRightPositionY1(P2,D1,2): + droneDY(D1,2)
 + 100 droneCollected(D1,P2,2) <= 110
 packageWasCollectedAtRightPositionY1(P2,D1,3): + droneDY(D1,3)
 + 100 droneCollected(D1,P2,3) <= 110
 packageWasCollectedAtRightPositionY1(P2,D1,4): + droneDY(D1,4)
 + 100 droneCollected(D1,P2,4) <= 110
 packageWasCollectedAtRightPositionY1(P2,D1,5): + droneDY(D1,5)
 + 100 droneCollected(D1,P2,5) <= 110
 packageWasCollectedAtRightPositionY1(P2,D2,1): + droneDY(D2,1)
 + 100 droneCollected(D2,P2,1) <= 110
 packageWasCollectedAtRightPositionY1(P2,D2,2): + droneDY(D2,2)
 + 100 droneCollected(D2,P2,2) <= 110
 packageWasCollectedAtRightPositionY1(P2,D2,3): + droneDY(D2,3)
 + 100 droneCollected(D2,P2,3) <= 110
 packageWasCollectedAtRightPositionY1(P2,D2,4): + droneDY(D2,4)
 + 100 droneCollected(D2,P2,4) <= 110
 packageWasCollectedAtRightPositionY1(P2,D2,5): + droneDY(D2,5)
 + 100 droneCollected(D2,P2,5) <= 110
 packageWasCollectedAtRightPositionY1(P2,D3,1): + droneDY(D3,1)
 + 100 droneCollected(D3,P2,1) <= 110
 packageWasCollectedAtRightPositionY1(P2,D3,2): + droneDY(D3,2)
 + 100 droneCollected(D3,P2,2) <= 110
 packageWasCollectedAtRightPositionY1(P2,D3,3): + droneDY(D3,3)
 + 100 droneCollected(D3,P2,3) <= 110
 packageWasCollectedAtRightPositionY1(P2,D3,4): + droneDY(D3,4)
 + 100 droneCollected(D3,P2,4) <= 110
 packageWasCollectedAtRightPositionY1(P2,D3,5): + droneDY(D3,5)
 + 100 droneCollected(D3,P2,5) <= 110
 packageWasCollectedAtRightPositionY1(P2,D4,1): + droneDY(D4,1)
 + 100 droneCollected(D4,P2,1) <= 110
 packageWasCollectedAtRightPositionY1(P2,D4,2): + droneDY(D4,2)
 + 100 droneCollected(D4,P2,2) <= 110
 packageWasCollectedAtRightPositionY1(P2,D4,3): + droneDY(D4,3)
 + 100 droneCollected(D4,P2,3) <= 110
 packageWasCollectedAtRightPositionY1(P2,D4,4): + droneDY(D4,4)
 + 100 droneCollected(D4,P2,4) <= 110
 packageWasCollectedAtRightPositionY1(P2,D4,5): + droneDY(D4,5)
 + 100 droneCollected(D4,P2,5) <= 110
 packageWasCollectedAtRightPositionY1(P2,D5,1): + droneDY(D5,1)
 + 100 droneCollected(D5,P2,1) <= 110
 packageWasCollectedAtRightPositionY1(P2,D5,2): + droneDY(D5,2)
 + 100 droneCollected(D5,P2,2) <= 110
 packageWasCollectedAtRightPositionY1(P2,D5,3): + droneDY(D5,3)
 + 100 droneCollected(D5,P2,3) <= 110
 packageWasCollectedAtRightPositionY1(P2,D5,4): + droneDY(D5,4)
 + 100 droneCollected(D5,P2,4) <= 110
 packageWasCollectedAtRightPositionY1(P2,D5,5): + droneDY(D5,5)
 + 100 droneCollected(D5,P2,5) <= 110
 packageWasCollectedAtRightPositionY1(P3,D1,1): + droneDY(D1,1)
 + 100 droneCollected(D1,P3,1) <= 120
 packageWasCollectedAtRightPositionY1(P3,D1,2): + droneDY(D1,2)
 + 100 droneCollected(D1,P3,2) <= 120
 packageWasCollectedAtRightPositionY1(P3,D1,3): + droneDY(D1,3)
 + 100 droneCollected(D1,P3,3) <= 120
 packageWasCollectedAtRightPositionY1(P3,D1,4): + droneDY(D1,4)
 + 100 droneCollected(D1,P3,4) <= 120
 packageWasCollectedAtRightPositionY1(P3,D1,5): + droneDY(D1,5)
 + 100 droneCollected(D1,P3,5) <= 120
 packageWasCollectedAtRightPositionY1(P3,D2,1): + droneDY(D2,1)
 + 100 droneCollected(D2,P3,1) <= 120
 packageWasCollectedAtRightPositionY1(P3,D2,2): + droneDY(D2,2)
 + 100 droneCollected(D2,P3,2) <= 120
 packageWasCollectedAtRightPositionY1(P3,D2,3): + droneDY(D2,3)
 + 100 droneCollected(D2,P3,3) <= 120
 packageWasCollectedAtRightPositionY1(P3,D2,4): + droneDY(D2,4)
 + 100 droneCollected(D2,P3,4) <= 120
 packageWasCollectedAtRightPositionY1(P3,D2,5): + droneDY(D2,5)
 + 100 droneCollected(D2,P3,5) <= 120
 packageWasCollectedAtRightPositionY1(P3,D3,1): + droneDY(D3,1)
 + 100 droneCollected(D3,P3,1) <= 120
 packageWasCollectedAtRightPositionY1(P3,D3,2): + droneDY(D3,2)
 + 100 droneCollected(D3,P3,2) <= 120
 packageWasCollectedAtRightPositionY1(P3,D3,3): + droneDY(D3,3)
 + 100 droneCollected(D3,P3,3) <= 120
 packageWasCollectedAtRightPositionY1(P3,D3,4): + droneDY(D3,4)
 + 100 droneCollected(D3,P3,4) <= 120
 packageWasCollectedAtRightPositionY1(P3,D3,5): + droneDY(D3,5)
 + 100 droneCollected(D3,P3,5) <= 120
 packageWasCollectedAtRightPositionY1(P3,D4,1): + droneDY(D4,1)
 + 100 droneCollected(D4,P3,1) <= 120
 packageWasCollectedAtRightPositionY1(P3,D4,2): + droneDY(D4,2)
 + 100 droneCollected(D4,P3,2) <= 120
 packageWasCollectedAtRightPositionY1(P3,D4,3): + droneDY(D4,3)
 + 100 droneCollected(D4,P3,3) <= 120
 packageWasCollectedAtRightPositionY1(P3,D4,4): + droneDY(D4,4)
 + 100 droneCollected(D4,P3,4) <= 120
 packageWasCollectedAtRightPositionY1(P3,D4,5): + droneDY(D4,5)
 + 100 droneCollected(D4,P3,5) <= 120
 packageWasCollectedAtRightPositionY1(P3,D5,1): + droneDY(D5,1)
 + 100 droneCollected(D5,P3,1) <= 120
 packageWasCollectedAtRightPositionY1(P3,D5,2): + droneDY(D5,2)
 + 100 droneCollected(D5,P3,2) <= 120
 packageWasCollectedAtRightPositionY1(P3,D5,3): + droneDY(D5,3)
 + 100 droneCollected(D5,P3,3) <= 120
 packageWasCollectedAtRightPositionY1(P3,D5,4): + droneDY(D5,4)
 + 100 droneCollected(D5,P3,4) <= 120
 packageWasCollectedAtRightPositionY1(P3,D5,5): + droneDY(D5,5)
 + 100 droneCollected(D5,P3,5) <= 120
 packageWasCollectedAtRightPositionY1(P4,D1,1): + droneDY(D1,1)
 + 100 droneCollected(D1,P4,1) <= 110
 packageWasCollectedAtRightPositionY1(P4,D1,2): + droneDY(D1,2)
 + 100 droneCollected(D1,P4,2) <= 110
 packageWasCollectedAtRightPositionY1(P4,D1,3): + droneDY(D1,3)
 + 100 droneCollected(D1,P4,3) <= 110
 packageWasCollectedAtRightPositionY1(P4,D1,4): + droneDY(D1,4)
 + 100 droneCollected(D1,P4,4) <= 110
 packageWasCollectedAtRightPositionY1(P4,D1,5): + droneDY(D1,5)
 + 100 droneCollected(D1,P4,5) <= 110
 packageWasCollectedAtRightPositionY1(P4,D2,1): + droneDY(D2,1)
 + 100 droneCollected(D2,P4,1) <= 110
 packageWasCollectedAtRightPositionY1(P4,D2,2): + droneDY(D2,2)
 + 100 droneCollected(D2,P4,2) <= 110
 packageWasCollectedAtRightPositionY1(P4,D2,3): + droneDY(D2,3)
 + 100 droneCollected(D2,P4,3) <= 110
 packageWasCollectedAtRightPositionY1(P4,D2,4): + droneDY(D2,4)
 + 100 droneCollected(D2,P4,4) <= 110
 packageWasCollectedAtRightPositionY1(P4,D2,5): + droneDY(D2,5)
 + 100 droneCollected(D2,P4,5) <= 110
 packageWasCollectedAtRightPositionY1(P4,D3,1): + droneDY(D3,1)
 + 100 droneCollected(D3,P4,1) <= 110
 packageWasCollectedAtRightPositionY1(P4,D3,2): + droneDY(D3,2)
 + 100 droneCollected(D3,P4,2) <= 110
 packageWasCollectedAtRightPositionY1(P4,D3,3): + droneDY(D3,3)
 + 100 droneCollected(D3,P4,3) <= 110
 packageWasCollectedAtRightPositionY1(P4,D3,4): + droneDY(D3,4)
 + 100 droneCollected(D3,P4,4) <= 110
 packageWasCollectedAtRightPositionY1(P4,D3,5): + droneDY(D3,5)
 + 100 droneCollected(D3,P4,5) <= 110
 packageWasCollectedAtRightPositionY1(P4,D4,1): + droneDY(D4,1)
 + 100 droneCollected(D4,P4,1) <= 110
 packageWasCollectedAtRightPositionY1(P4,D4,2): + droneDY(D4,2)
 + 100 droneCollected(D4,P4,2) <= 110
 packageWasCollectedAtRightPositionY1(P4,D4,3): + droneDY(D4,3)
 + 100 droneCollected(D4,P4,3) <= 110
 packageWasCollectedAtRightPositionY1(P4,D4,4): + droneDY(D4,4)
 + 100 droneCollected(D4,P4,4) <= 110
 packageWasCollectedAtRightPositionY1(P4,D4,5): + droneDY(D4,5)
 + 100 droneCollected(D4,P4,5) <= 110
 packageWasCollectedAtRightPositionY1(P4,D5,1): + droneDY(D5,1)
 + 100 droneCollected(D5,P4,1) <= 110
 packageWasCollectedAtRightPositionY1(P4,D5,2): + droneDY(D5,2)
 + 100 droneCollected(D5,P4,2) <= 110
 packageWasCollectedAtRightPositionY1(P4,D5,3): + droneDY(D5,3)
 + 100 droneCollected(D5,P4,3) <= 110
 packageWasCollectedAtRightPositionY1(P4,D5,4): + droneDY(D5,4)
 + 100 droneCollected(D5,P4,4) <= 110
 packageWasCollectedAtRightPositionY1(P4,D5,5): + droneDY(D5,5)
 + 100 droneCollected(D5,P4,5) <= 110
 packageWasCollectedAtRightPositionY1(P5,D1,1): + droneDY(D1,1)
 + 100 droneCollected(D1,P5,1) <= 120
 packageWasCollectedAtRightPositionY1(P5,D1,2): + droneDY(D1,2)
 + 100 droneCollected(D1,P5,2) <= 120
 packageWasCollectedAtRightPositionY1(P5,D1,3): + droneDY(D1,3)
 + 100 droneCollected(D1,P5,3) <= 120
 packageWasCollectedAtRightPositionY1(P5,D1,4): + droneDY(D1,4)
 + 100 droneCollected(D1,P5,4) <= 120
 packageWasCollectedAtRightPositionY1(P5,D1,5): + droneDY(D1,5)
 + 100 droneCollected(D1,P5,5) <= 120
 packageWasCollectedAtRightPositionY1(P5,D2,1): + droneDY(D2,1)
 + 100 droneCollected(D2,P5,1) <= 120
 packageWasCollectedAtRightPositionY1(P5,D2,2): + droneDY(D2,2)
 + 100 droneCollected(D2,P5,2) <= 120
 packageWasCollectedAtRightPositionY1(P5,D2,3): + droneDY(D2,3)
 + 100 droneCollected(D2,P5,3) <= 120
 packageWasCollectedAtRightPositionY1(P5,D2,4): + droneDY(D2,4)
 + 100 droneCollected(D2,P5,4) <= 120
 packageWasCollectedAtRightPositionY1(P5,D2,5): + droneDY(D2,5)
 + 100 droneCollected(D2,P5,5) <= 120
 packageWasCollectedAtRightPositionY1(P5,D3,1): + droneDY(D3,1)
 + 100 droneCollected(D3,P5,1) <= 120
 packageWasCollectedAtRightPositionY1(P5,D3,2): + droneDY(D3,2)
 + 100 droneCollected(D3,P5,2) <= 120
 packageWasCollectedAtRightPositionY1(P5,D3,3): + droneDY(D3,3)
 + 100 droneCollected(D3,P5,3) <= 120
 packageWasCollectedAtRightPositionY1(P5,D3,4): + droneDY(D3,4)
 + 100 droneCollected(D3,P5,4) <= 120
 packageWasCollectedAtRightPositionY1(P5,D3,5): + droneDY(D3,5)
 + 100 droneCollected(D3,P5,5) <= 120
 packageWasCollectedAtRightPositionY1(P5,D4,1): + droneDY(D4,1)
 + 100 droneCollected(D4,P5,1) <= 120
 packageWasCollectedAtRightPositionY1(P5,D4,2): + droneDY(D4,2)
 + 100 droneCollected(D4,P5,2) <= 120
 packageWasCollectedAtRightPositionY1(P5,D4,3): + droneDY(D4,3)
 + 100 droneCollected(D4,P5,3) <= 120
 packageWasCollectedAtRightPositionY1(P5,D4,4): + droneDY(D4,4)
 + 100 droneCollected(D4,P5,4) <= 120
 packageWasCollectedAtRightPositionY1(P5,D4,5): + droneDY(D4,5)
 + 100 droneCollected(D4,P5,5) <= 120
 packageWasCollectedAtRightPositionY1(P5,D5,1): + droneDY(D5,1)
 + 100 droneCollected(D5,P5,1) <= 120
 packageWasCollectedAtRightPositionY1(P5,D5,2): + droneDY(D5,2)
 + 100 droneCollected(D5,P5,2) <= 120
 packageWasCollectedAtRightPositionY1(P5,D5,3): + droneDY(D5,3)
 + 100 droneCollected(D5,P5,3) <= 120
 packageWasCollectedAtRightPositionY1(P5,D5,4): + droneDY(D5,4)
 + 100 droneCollected(D5,P5,4) <= 120
 packageWasCollectedAtRightPositionY1(P5,D5,5): + droneDY(D5,5)
 + 100 droneCollected(D5,P5,5) <= 120
 packageWasCollectedAtRightPositionY1(P6,D1,1): + droneDY(D1,1)
 + 100 droneCollected(D1,P6,1) <= 110
 packageWasCollectedAtRightPositionY1(P6,D1,2): + droneDY(D1,2)
 + 100 droneCollected(D1,P6,2) <= 110
 packageWasCollectedAtRightPositionY1(P6,D1,3): + droneDY(D1,3)
 + 100 droneCollected(D1,P6,3) <= 110
 packageWasCollectedAtRightPositionY1(P6,D1,4): + droneDY(D1,4)
 + 100 droneCollected(D1,P6,4) <= 110
 packageWasCollectedAtRightPositionY1(P6,D1,5): + droneDY(D1,5)
 + 100 droneCollected(D1,P6,5) <= 110
 packageWasCollectedAtRightPositionY1(P6,D2,1): + droneDY(D2,1)
 + 100 droneCollected(D2,P6,1) <= 110
 packageWasCollectedAtRightPositionY1(P6,D2,2): + droneDY(D2,2)
 + 100 droneCollected(D2,P6,2) <= 110
 packageWasCollectedAtRightPositionY1(P6,D2,3): + droneDY(D2,3)
 + 100 droneCollected(D2,P6,3) <= 110
 packageWasCollectedAtRightPositionY1(P6,D2,4): + droneDY(D2,4)
 + 100 droneCollected(D2,P6,4) <= 110
 packageWasCollectedAtRightPositionY1(P6,D2,5): + droneDY(D2,5)
 + 100 droneCollected(D2,P6,5) <= 110
 packageWasCollectedAtRightPositionY1(P6,D3,1): + droneDY(D3,1)
 + 100 droneCollected(D3,P6,1) <= 110
 packageWasCollectedAtRightPositionY1(P6,D3,2): + droneDY(D3,2)
 + 100 droneCollected(D3,P6,2) <= 110
 packageWasCollectedAtRightPositionY1(P6,D3,3): + droneDY(D3,3)
 + 100 droneCollected(D3,P6,3) <= 110
 packageWasCollectedAtRightPositionY1(P6,D3,4): + droneDY(D3,4)
 + 100 droneCollected(D3,P6,4) <= 110
 packageWasCollectedAtRightPositionY1(P6,D3,5): + droneDY(D3,5)
 + 100 droneCollected(D3,P6,5) <= 110
 packageWasCollectedAtRightPositionY1(P6,D4,1): + droneDY(D4,1)
 + 100 droneCollected(D4,P6,1) <= 110
 packageWasCollectedAtRightPositionY1(P6,D4,2): + droneDY(D4,2)
 + 100 droneCollected(D4,P6,2) <= 110
 packageWasCollectedAtRightPositionY1(P6,D4,3): + droneDY(D4,3)
 + 100 droneCollected(D4,P6,3) <= 110
 packageWasCollectedAtRightPositionY1(P6,D4,4): + droneDY(D4,4)
 + 100 droneCollected(D4,P6,4) <= 110
 packageWasCollectedAtRightPositionY1(P6,D4,5): + droneDY(D4,5)
 + 100 droneCollected(D4,P6,5) <= 110
 packageWasCollectedAtRightPositionY1(P6,D5,1): + droneDY(D5,1)
 + 100 droneCollected(D5,P6,1) <= 110
 packageWasCollectedAtRightPositionY1(P6,D5,2): + droneDY(D5,2)
 + 100 droneCollected(D5,P6,2) <= 110
 packageWasCollectedAtRightPositionY1(P6,D5,3): + droneDY(D5,3)
 + 100 droneCollected(D5,P6,3) <= 110
 packageWasCollectedAtRightPositionY1(P6,D5,4): + droneDY(D5,4)
 + 100 droneCollected(D5,P6,4) <= 110
 packageWasCollectedAtRightPositionY1(P6,D5,5): + droneDY(D5,5)
 + 100 droneCollected(D5,P6,5) <= 110
 packageWasCollectedAtRightPositionY2(P1,D1,1): - droneDY(D1,1)
 + 100 droneCollected(D1,P1,1) <= 90
 packageWasCollectedAtRightPositionY2(P1,D1,2): - droneDY(D1,2)
 + 100 droneCollected(D1,P1,2) <= 90
 packageWasCollectedAtRightPositionY2(P1,D1,3): - droneDY(D1,3)
 + 100 droneCollected(D1,P1,3) <= 90
 packageWasCollectedAtRightPositionY2(P1,D1,4): - droneDY(D1,4)
 + 100 droneCollected(D1,P1,4) <= 90
 packageWasCollectedAtRightPositionY2(P1,D1,5): - droneDY(D1,5)
 + 100 droneCollected(D1,P1,5) <= 90
 packageWasCollectedAtRightPositionY2(P1,D2,1): - droneDY(D2,1)
 + 100 droneCollected(D2,P1,1) <= 90
 packageWasCollectedAtRightPositionY2(P1,D2,2): - droneDY(D2,2)
 + 100 droneCollected(D2,P1,2) <= 90
 packageWasCollectedAtRightPositionY2(P1,D2,3): - droneDY(D2,3)
 + 100 droneCollected(D2,P1,3) <= 90
 packageWasCollectedAtRightPositionY2(P1,D2,4): - droneDY(D2,4)
 + 100 droneCollected(D2,P1,4) <= 90
 packageWasCollectedAtRightPositionY2(P1,D2,5): - droneDY(D2,5)
 + 100 droneCollected(D2,P1,5) <= 90
 packageWasCollectedAtRightPositionY2(P1,D3,1): - droneDY(D3,1)
 + 100 droneCollected(D3,P1,1) <= 90
 packageWasCollectedAtRightPositionY2(P1,D3,2): - droneDY(D3,2)
 + 100 droneCollected(D3,P1,2) <= 90
 packageWasCollectedAtRightPositionY2(P1,D3,3): - droneDY(D3,3)
 + 100 droneCollected(D3,P1,3) <= 90
 packageWasCollectedAtRightPositionY2(P1,D3,4): - droneDY(D3,4)
 + 100 droneCollected(D3,P1,4) <= 90
 packageWasCollectedAtRightPositionY2(P1,D3,5): - droneDY(D3,5)
 + 100 droneCollected(D3,P1,5) <= 90
 packageWasCollectedAtRightPositionY2(P1,D4,1): - droneDY(D4,1)
 + 100 droneCollected(D4,P1,1) <= 90
 packageWasCollectedAtRightPositionY2(P1,D4,2): - droneDY(D4,2)
 + 100 droneCollected(D4,P1,2) <= 90
 packageWasCollectedAtRightPositionY2(P1,D4,3): - droneDY(D4,3)
 + 100 droneCollected(D4,P1,3) <= 90
 packageWasCollectedAtRightPositionY2(P1,D4,4): - droneDY(D4,4)
 + 100 droneCollected(D4,P1,4) <= 90
 packageWasCollectedAtRightPositionY2(P1,D4,5): - droneDY(D4,5)
 + 100 droneCollected(D4,P1,5) <= 90
 packageWasCollectedAtRightPositionY2(P1,D5,1): - droneDY(D5,1)
 + 100 droneCollected(D5,P1,1) <= 90
 packageWasCollectedAtRightPositionY2(P1,D5,2): - droneDY(D5,2)
 + 100 droneCollected(D5,P1,2) <= 90
 packageWasCollectedAtRightPositionY2(P1,D5,3): - droneDY(D5,3)
 + 100 droneCollected(D5,P1,3) <= 90
 packageWasCollectedAtRightPositionY2(P1,D5,4): - droneDY(D5,4)
 + 100 droneCollected(D5,P1,4) <= 90
 packageWasCollectedAtRightPositionY2(P1,D5,5): - droneDY(D5,5)
 + 100 droneCollected(D5,P1,5) <= 90
 packageWasCollectedAtRightPositionY2(P2,D1,1): - droneDY(D1,1)
 + 100 droneCollected(D1,P2,1) <= 90
 packageWasCollectedAtRightPositionY2(P2,D1,2): - droneDY(D1,2)
 + 100 droneCollected(D1,P2,2) <= 90
 packageWasCollectedAtRightPositionY2(P2,D1,3): - droneDY(D1,3)
 + 100 droneCollected(D1,P2,3) <= 90
 packageWasCollectedAtRightPositionY2(P2,D1,4): - droneDY(D1,4)
 + 100 droneCollected(D1,P2,4) <= 90
 packageWasCollectedAtRightPositionY2(P2,D1,5): - droneDY(D1,5)
 + 100 droneCollected(D1,P2,5) <= 90
 packageWasCollectedAtRightPositionY2(P2,D2,1): - droneDY(D2,1)
 + 100 droneCollected(D2,P2,1) <= 90
 packageWasCollectedAtRightPositionY2(P2,D2,2): - droneDY(D2,2)
 + 100 droneCollected(D2,P2,2) <= 90
 packageWasCollectedAtRightPositionY2(P2,D2,3): - droneDY(D2,3)
 + 100 droneCollected(D2,P2,3) <= 90
 packageWasCollectedAtRightPositionY2(P2,D2,4): - droneDY(D2,4)
 + 100 droneCollected(D2,P2,4) <= 90
 packageWasCollectedAtRightPositionY2(P2,D2,5): - droneDY(D2,5)
 + 100 droneCollected(D2,P2,5) <= 90
 packageWasCollectedAtRightPositionY2(P2,D3,1): - droneDY(D3,1)
 + 100 droneCollected(D3,P2,1) <= 90
 packageWasCollectedAtRightPositionY2(P2,D3,2): - droneDY(D3,2)
 + 100 droneCollected(D3,P2,2) <= 90
 packageWasCollectedAtRightPositionY2(P2,D3,3): - droneDY(D3,3)
 + 100 droneCollected(D3,P2,3) <= 90
 packageWasCollectedAtRightPositionY2(P2,D3,4): - droneDY(D3,4)
 + 100 droneCollected(D3,P2,4) <= 90
 packageWasCollectedAtRightPositionY2(P2,D3,5): - droneDY(D3,5)
 + 100 droneCollected(D3,P2,5) <= 90
 packageWasCollectedAtRightPositionY2(P2,D4,1): - droneDY(D4,1)
 + 100 droneCollected(D4,P2,1) <= 90
 packageWasCollectedAtRightPositionY2(P2,D4,2): - droneDY(D4,2)
 + 100 droneCollected(D4,P2,2) <= 90
 packageWasCollectedAtRightPositionY2(P2,D4,3): - droneDY(D4,3)
 + 100 droneCollected(D4,P2,3) <= 90
 packageWasCollectedAtRightPositionY2(P2,D4,4): - droneDY(D4,4)
 + 100 droneCollected(D4,P2,4) <= 90
 packageWasCollectedAtRightPositionY2(P2,D4,5): - droneDY(D4,5)
 + 100 droneCollected(D4,P2,5) <= 90
 packageWasCollectedAtRightPositionY2(P2,D5,1): - droneDY(D5,1)
 + 100 droneCollected(D5,P2,1) <= 90
 packageWasCollectedAtRightPositionY2(P2,D5,2): - droneDY(D5,2)
 + 100 droneCollected(D5,P2,2) <= 90
 packageWasCollectedAtRightPositionY2(P2,D5,3): - droneDY(D5,3)
 + 100 droneCollected(D5,P2,3) <= 90
 packageWasCollectedAtRightPositionY2(P2,D5,4): - droneDY(D5,4)
 + 100 droneCollected(D5,P2,4) <= 90
 packageWasCollectedAtRightPositionY2(P2,D5,5): - droneDY(D5,5)
 + 100 droneCollected(D5,P2,5) <= 90
 packageWasCollectedAtRightPositionY2(P3,D1,1): - droneDY(D1,1)
 + 100 droneCollected(D1,P3,1) <= 80
 packageWasCollectedAtRightPositionY2(P3,D1,2): - droneDY(D1,2)
 + 100 droneCollected(D1,P3,2) <= 80
 packageWasCollectedAtRightPositionY2(P3,D1,3): - droneDY(D1,3)
 + 100 droneCollected(D1,P3,3) <= 80
 packageWasCollectedAtRightPositionY2(P3,D1,4): - droneDY(D1,4)
 + 100 droneCollected(D1,P3,4) <= 80
 packageWasCollectedAtRightPositionY2(P3,D1,5): - droneDY(D1,5)
 + 100 droneCollected(D1,P3,5) <= 80
 packageWasCollectedAtRightPositionY2(P3,D2,1): - droneDY(D2,1)
 + 100 droneCollected(D2,P3,1) <= 80
 packageWasCollectedAtRightPositionY2(P3,D2,2): - droneDY(D2,2)
 + 100 droneCollected(D2,P3,2) <= 80
 packageWasCollectedAtRightPositionY2(P3,D2,3): - droneDY(D2,3)
 + 100 droneCollected(D2,P3,3) <= 80
 packageWasCollectedAtRightPositionY2(P3,D2,4): - droneDY(D2,4)
 + 100 droneCollected(D2,P3,4) <= 80
 packageWasCollectedAtRightPositionY2(P3,D2,5): - droneDY(D2,5)
 + 100 droneCollected(D2,P3,5) <= 80
 packageWasCollectedAtRightPositionY2(P3,D3,1): - droneDY(D3,1)
 + 100 droneCollected(D3,P3,1) <= 80
 packageWasCollectedAtRightPositionY2(P3,D3,2): - droneDY(D3,2)
 + 100 droneCollected(D3,P3,2) <= 80
 packageWasCollectedAtRightPositionY2(P3,D3,3): - droneDY(D3,3)
 + 100 droneCollected(D3,P3,3) <= 80
 packageWasCollectedAtRightPositionY2(P3,D3,4): - droneDY(D3,4)
 + 100 droneCollected(D3,P3,4) <= 80
 packageWasCollectedAtRightPositionY2(P3,D3,5): - droneDY(D3,5)
 + 100 droneCollected(D3,P3,5) <= 80
 packageWasCollectedAtRightPositionY2(P3,D4,1): - droneDY(D4,1)
 + 100 droneCollected(D4,P3,1) <= 80
 packageWasCollectedAtRightPositionY2(P3,D4,2): - droneDY(D4,2)
 + 100 droneCollected(D4,P3,2) <= 80
 packageWasCollectedAtRightPositionY2(P3,D4,3): - droneDY(D4,3)
 + 100 droneCollected(D4,P3,3) <= 80
 packageWasCollectedAtRightPositionY2(P3,D4,4): - droneDY(D4,4)
 + 100 droneCollected(D4,P3,4) <= 80
 packageWasCollectedAtRightPositionY2(P3,D4,5): - droneDY(D4,5)
 + 100 droneCollected(D4,P3,5) <= 80
 packageWasCollectedAtRightPositionY2(P3,D5,1): - droneDY(D5,1)
 + 100 droneCollected(D5,P3,1) <= 80
 packageWasCollectedAtRightPositionY2(P3,D5,2): - droneDY(D5,2)
 + 100 droneCollected(D5,P3,2) <= 80
 packageWasCollectedAtRightPositionY2(P3,D5,3): - droneDY(D5,3)
 + 100 droneCollected(D5,P3,3) <= 80
 packageWasCollectedAtRightPositionY2(P3,D5,4): - droneDY(D5,4)
 + 100 droneCollected(D5,P3,4) <= 80
 packageWasCollectedAtRightPositionY2(P3,D5,5): - droneDY(D5,5)
 + 100 droneCollected(D5,P3,5) <= 80
 packageWasCollectedAtRightPositionY2(P4,D1,1): - droneDY(D1,1)
 + 100 droneCollected(D1,P4,1) <= 90
 packageWasCollectedAtRightPositionY2(P4,D1,2): - droneDY(D1,2)
 + 100 droneCollected(D1,P4,2) <= 90
 packageWasCollectedAtRightPositionY2(P4,D1,3): - droneDY(D1,3)
 + 100 droneCollected(D1,P4,3) <= 90
 packageWasCollectedAtRightPositionY2(P4,D1,4): - droneDY(D1,4)
 + 100 droneCollected(D1,P4,4) <= 90
 packageWasCollectedAtRightPositionY2(P4,D1,5): - droneDY(D1,5)
 + 100 droneCollected(D1,P4,5) <= 90
 packageWasCollectedAtRightPositionY2(P4,D2,1): - droneDY(D2,1)
 + 100 droneCollected(D2,P4,1) <= 90
 packageWasCollectedAtRightPositionY2(P4,D2,2): - droneDY(D2,2)
 + 100 droneCollected(D2,P4,2) <= 90
 packageWasCollectedAtRightPositionY2(P4,D2,3): - droneDY(D2,3)
 + 100 droneCollected(D2,P4,3) <= 90
 packageWasCollectedAtRightPositionY2(P4,D2,4): - droneDY(D2,4)
 + 100 droneCollected(D2,P4,4) <= 90
 packageWasCollectedAtRightPositionY2(P4,D2,5): - droneDY(D2,5)
 + 100 droneCollected(D2,P4,5) <= 90
 packageWasCollectedAtRightPositionY2(P4,D3,1): - droneDY(D3,1)
 + 100 droneCollected(D3,P4,1) <= 90
 packageWasCollectedAtRightPositionY2(P4,D3,2): - droneDY(D3,2)
 + 100 droneCollected(D3,P4,2) <= 90
 packageWasCollectedAtRightPositionY2(P4,D3,3): - droneDY(D3,3)
 + 100 droneCollected(D3,P4,3) <= 90
 packageWasCollectedAtRightPositionY2(P4,D3,4): - droneDY(D3,4)
 + 100 droneCollected(D3,P4,4) <= 90
 packageWasCollectedAtRightPositionY2(P4,D3,5): - droneDY(D3,5)
 + 100 droneCollected(D3,P4,5) <= 90
 packageWasCollectedAtRightPositionY2(P4,D4,1): - droneDY(D4,1)
 + 100 droneCollected(D4,P4,1) <= 90
 packageWasCollectedAtRightPositionY2(P4,D4,2): - droneDY(D4,2)
 + 100 droneCollected(D4,P4,2) <= 90
 packageWasCollectedAtRightPositionY2(P4,D4,3): - droneDY(D4,3)
 + 100 droneCollected(D4,P4,3) <= 90
 packageWasCollectedAtRightPositionY2(P4,D4,4): - droneDY(D4,4)
 + 100 droneCollected(D4,P4,4) <= 90
 packageWasCollectedAtRightPositionY2(P4,D4,5): - droneDY(D4,5)
 + 100 droneCollected(D4,P4,5) <= 90
 packageWasCollectedAtRightPositionY2(P4,D5,1): - droneDY(D5,1)
 + 100 droneCollected(D5,P4,1) <= 90
 packageWasCollectedAtRightPositionY2(P4,D5,2): - droneDY(D5,2)
 + 100 droneCollected(D5,P4,2) <= 90
 packageWasCollectedAtRightPositionY2(P4,D5,3): - droneDY(D5,3)
 + 100 droneCollected(D5,P4,3) <= 90
 packageWasCollectedAtRightPositionY2(P4,D5,4): - droneDY(D5,4)
 + 100 droneCollected(D5,P4,4) <= 90
 packageWasCollectedAtRightPositionY2(P4,D5,5): - droneDY(D5,5)
 + 100 droneCollected(D5,P4,5) <= 90
 packageWasCollectedAtRightPositionY2(P5,D1,1): - droneDY(D1,1)
 + 100 droneCollected(D1,P5,1) <= 80
 packageWasCollectedAtRightPositionY2(P5,D1,2): - droneDY(D1,2)
 + 100 droneCollected(D1,P5,2) <= 80
 packageWasCollectedAtRightPositionY2(P5,D1,3): - droneDY(D1,3)
 + 100 droneCollected(D1,P5,3) <= 80
 packageWasCollectedAtRightPositionY2(P5,D1,4): - droneDY(D1,4)
 + 100 droneCollected(D1,P5,4) <= 80
 packageWasCollectedAtRightPositionY2(P5,D1,5): - droneDY(D1,5)
 + 100 droneCollected(D1,P5,5) <= 80
 packageWasCollectedAtRightPositionY2(P5,D2,1): - droneDY(D2,1)
 + 100 droneCollected(D2,P5,1) <= 80
 packageWasCollectedAtRightPositionY2(P5,D2,2): - droneDY(D2,2)
 + 100 droneCollected(D2,P5,2) <= 80
 packageWasCollectedAtRightPositionY2(P5,D2,3): - droneDY(D2,3)
 + 100 droneCollected(D2,P5,3) <= 80
 packageWasCollectedAtRightPositionY2(P5,D2,4): - droneDY(D2,4)
 + 100 droneCollected(D2,P5,4) <= 80
 packageWasCollectedAtRightPositionY2(P5,D2,5): - droneDY(D2,5)
 + 100 droneCollected(D2,P5,5) <= 80
 packageWasCollectedAtRightPositionY2(P5,D3,1): - droneDY(D3,1)
 + 100 droneCollected(D3,P5,1) <= 80
 packageWasCollectedAtRightPositionY2(P5,D3,2): - droneDY(D3,2)
 + 100 droneCollected(D3,P5,2) <= 80
 packageWasCollectedAtRightPositionY2(P5,D3,3): - droneDY(D3,3)
 + 100 droneCollected(D3,P5,3) <= 80
 packageWasCollectedAtRightPositionY2(P5,D3,4): - droneDY(D3,4)
 + 100 droneCollected(D3,P5,4) <= 80
 packageWasCollectedAtRightPositionY2(P5,D3,5): - droneDY(D3,5)
 + 100 droneCollected(D3,P5,5) <= 80
 packageWasCollectedAtRightPositionY2(P5,D4,1): - droneDY(D4,1)
 + 100 droneCollected(D4,P5,1) <= 80
 packageWasCollectedAtRightPositionY2(P5,D4,2): - droneDY(D4,2)
 + 100 droneCollected(D4,P5,2) <= 80
 packageWasCollectedAtRightPositionY2(P5,D4,3): - droneDY(D4,3)
 + 100 droneCollected(D4,P5,3) <= 80
 packageWasCollectedAtRightPositionY2(P5,D4,4): - droneDY(D4,4)
 + 100 droneCollected(D4,P5,4) <= 80
 packageWasCollectedAtRightPositionY2(P5,D4,5): - droneDY(D4,5)
 + 100 droneCollected(D4,P5,5) <= 80
 packageWasCollectedAtRightPositionY2(P5,D5,1): - droneDY(D5,1)
 + 100 droneCollected(D5,P5,1) <= 80
 packageWasCollectedAtRightPositionY2(P5,D5,2): - droneDY(D5,2)
 + 100 droneCollected(D5,P5,2) <= 80
 packageWasCollectedAtRightPositionY2(P5,D5,3): - droneDY(D5,3)
 + 100 droneCollected(D5,P5,3) <= 80
 packageWasCollectedAtRightPositionY2(P5,D5,4): - droneDY(D5,4)
 + 100 droneCollected(D5,P5,4) <= 80
 packageWasCollectedAtRightPositionY2(P5,D5,5): - droneDY(D5,5)
 + 100 droneCollected(D5,P5,5) <= 80
 packageWasCollectedAtRightPositionY2(P6,D1,1): - droneDY(D1,1)
 + 100 droneCollected(D1,P6,1) <= 90
 packageWasCollectedAtRightPositionY2(P6,D1,2): - droneDY(D1,2)
 + 100 droneCollected(D1,P6,2) <= 90
 packageWasCollectedAtRightPositionY2(P6,D1,3): - droneDY(D1,3)
 + 100 droneCollected(D1,P6,3) <= 90
 packageWasCollectedAtRightPositionY2(P6,D1,4): - droneDY(D1,4)
 + 100 droneCollected(D1,P6,4) <= 90
 packageWasCollectedAtRightPositionY2(P6,D1,5): - droneDY(D1,5)
 + 100 droneCollected(D1,P6,5) <= 90
 packageWasCollectedAtRightPositionY2(P6,D2,1): - droneDY(D2,1)
 + 100 droneCollected(D2,P6,1) <= 90
 packageWasCollectedAtRightPositionY2(P6,D2,2): - droneDY(D2,2)
 + 100 droneCollected(D2,P6,2) <= 90
 packageWasCollectedAtRightPositionY2(P6,D2,3): - droneDY(D2,3)
 + 100 droneCollected(D2,P6,3) <= 90
 packageWasCollectedAtRightPositionY2(P6,D2,4): - droneDY(D2,4)
 + 100 droneCollected(D2,P6,4) <= 90
 packageWasCollectedAtRightPositionY2(P6,D2,5): - droneDY(D2,5)
 + 100 droneCollected(D2,P6,5) <= 90
 packageWasCollectedAtRightPositionY2(P6,D3,1): - droneDY(D3,1)
 + 100 droneCollected(D3,P6,1) <= 90
 packageWasCollectedAtRightPositionY2(P6,D3,2): - droneDY(D3,2)
 + 100 droneCollected(D3,P6,2) <= 90
 packageWasCollectedAtRightPositionY2(P6,D3,3): - droneDY(D3,3)
 + 100 droneCollected(D3,P6,3) <= 90
 packageWasCollectedAtRightPositionY2(P6,D3,4): - droneDY(D3,4)
 + 100 droneCollected(D3,P6,4) <= 90
 packageWasCollectedAtRightPositionY2(P6,D3,5): - droneDY(D3,5)
 + 100 droneCollected(D3,P6,5) <= 90
 packageWasCollectedAtRightPositionY2(P6,D4,1): - droneDY(D4,1)
 + 100 droneCollected(D4,P6,1) <= 90
 packageWasCollectedAtRightPositionY2(P6,D4,2): - droneDY(D4,2)
 + 100 droneCollected(D4,P6,2) <= 90
 packageWasCollectedAtRightPositionY2(P6,D4,3): - droneDY(D4,3)
 + 100 droneCollected(D4,P6,3) <= 90
 packageWasCollectedAtRightPositionY2(P6,D4,4): - droneDY(D4,4)
 + 100 droneCollected(D4,P6,4) <= 90
 packageWasCollectedAtRightPositionY2(P6,D4,5): - droneDY(D4,5)
 + 100 droneCollected(D4,P6,5) <= 90
 packageWasCollectedAtRightPositionY2(P6,D5,1): - droneDY(D5,1)
 + 100 droneCollected(D5,P6,1) <= 90
 packageWasCollectedAtRightPositionY2(P6,D5,2): - droneDY(D5,2)
 + 100 droneCollected(D5,P6,2) <= 90
 packageWasCollectedAtRightPositionY2(P6,D5,3): - droneDY(D5,3)
 + 100 droneCollected(D5,P6,3) <= 90
 packageWasCollectedAtRightPositionY2(P6,D5,4): - droneDY(D5,4)
 + 100 droneCollected(D5,P6,4) <= 90
 packageWasCollectedAtRightPositionY2(P6,D5,5): - droneDY(D5,5)
 + 100 droneCollected(D5,P6,5) <= 90
 packageWasDeliveredAtRightPositionX1(P1,D1,1): + droneDX(D1,1)
 + 100 droneDelivered(D1,P1,1) <= 104
 packageWasDeliveredAtRightPositionX1(P1,D1,2): + droneDX(D1,2)
 + 100 droneDelivered(D1,P1,2) <= 104
 packageWasDeliveredAtRightPositionX1(P1,D1,3): + droneDX(D1,3)
 + 100 droneDelivered(D1,P1,3) <= 104
 packageWasDeliveredAtRightPositionX1(P1,D1,4): + droneDX(D1,4)
 + 100 droneDelivered(D1,P1,4) <= 104
 packageWasDeliveredAtRightPositionX1(P1,D1,5): + droneDX(D1,5)
 + 100 droneDelivered(D1,P1,5) <= 104
 packageWasDeliveredAtRightPositionX1(P1,D2,1): + droneDX(D2,1)
 + 100 droneDelivered(D2,P1,1) <= 104
 packageWasDeliveredAtRightPositionX1(P1,D2,2): + droneDX(D2,2)
 + 100 droneDelivered(D2,P1,2) <= 104
 packageWasDeliveredAtRightPositionX1(P1,D2,3): + droneDX(D2,3)
 + 100 droneDelivered(D2,P1,3) <= 104
 packageWasDeliveredAtRightPositionX1(P1,D2,4): + droneDX(D2,4)
 + 100 droneDelivered(D2,P1,4) <= 104
 packageWasDeliveredAtRightPositionX1(P1,D2,5): + droneDX(D2,5)
 + 100 droneDelivered(D2,P1,5) <= 104
 packageWasDeliveredAtRightPositionX1(P1,D3,1): + droneDX(D3,1)
 + 100 droneDelivered(D3,P1,1) <= 104
 packageWasDeliveredAtRightPositionX1(P1,D3,2): + droneDX(D3,2)
 + 100 droneDelivered(D3,P1,2) <= 104
 packageWasDeliveredAtRightPositionX1(P1,D3,3): + droneDX(D3,3)
 + 100 droneDelivered(D3,P1,3) <= 104
 packageWasDeliveredAtRightPositionX1(P1,D3,4): + droneDX(D3,4)
 + 100 droneDelivered(D3,P1,4) <= 104
 packageWasDeliveredAtRightPositionX1(P1,D3,5): + droneDX(D3,5)
 + 100 droneDelivered(D3,P1,5) <= 104
 packageWasDeliveredAtRightPositionX1(P1,D4,1): + droneDX(D4,1)
 + 100 droneDelivered(D4,P1,1) <= 104
 packageWasDeliveredAtRightPositionX1(P1,D4,2): + droneDX(D4,2)
 + 100 droneDelivered(D4,P1,2) <= 104
 packageWasDeliveredAtRightPositionX1(P1,D4,3): + droneDX(D4,3)
 + 100 droneDelivered(D4,P1,3) <= 104
 packageWasDeliveredAtRightPositionX1(P1,D4,4): + droneDX(D4,4)
 + 100 droneDelivered(D4,P1,4) <= 104
 packageWasDeliveredAtRightPositionX1(P1,D4,5): + droneDX(D4,5)
 + 100 droneDelivered(D4,P1,5) <= 104
 packageWasDeliveredAtRightPositionX1(P1,D5,1): + droneDX(D5,1)
 + 100 droneDelivered(D5,P1,1) <= 104
 packageWasDeliveredAtRightPositionX1(P1,D5,2): + droneDX(D5,2)
 + 100 droneDelivered(D5,P1,2) <= 104
 packageWasDeliveredAtRightPositionX1(P1,D5,3): + droneDX(D5,3)
 + 100 droneDelivered(D5,P1,3) <= 104
 packageWasDeliveredAtRightPositionX1(P1,D5,4): + droneDX(D5,4)
 + 100 droneDelivered(D5,P1,4) <= 104
 packageWasDeliveredAtRightPositionX1(P1,D5,5): + droneDX(D5,5)
 + 100 droneDelivered(D5,P1,5) <= 104
 packageWasDeliveredAtRightPositionX1(P2,D1,1): + droneDX(D1,1)
 + 100 droneDelivered(D1,P2,1) <= 105
 packageWasDeliveredAtRightPositionX1(P2,D1,2): + droneDX(D1,2)
 + 100 droneDelivered(D1,P2,2) <= 105
 packageWasDeliveredAtRightPositionX1(P2,D1,3): + droneDX(D1,3)
 + 100 droneDelivered(D1,P2,3) <= 105
 packageWasDeliveredAtRightPositionX1(P2,D1,4): + droneDX(D1,4)
 + 100 droneDelivered(D1,P2,4) <= 105
 packageWasDeliveredAtRightPositionX1(P2,D1,5): + droneDX(D1,5)
 + 100 droneDelivered(D1,P2,5) <= 105
 packageWasDeliveredAtRightPositionX1(P2,D2,1): + droneDX(D2,1)
 + 100 droneDelivered(D2,P2,1) <= 105
 packageWasDeliveredAtRightPositionX1(P2,D2,2): + droneDX(D2,2)
 + 100 droneDelivered(D2,P2,2) <= 105
 packageWasDeliveredAtRightPositionX1(P2,D2,3): + droneDX(D2,3)
 + 100 droneDelivered(D2,P2,3) <= 105
 packageWasDeliveredAtRightPositionX1(P2,D2,4): + droneDX(D2,4)
 + 100 droneDelivered(D2,P2,4) <= 105
 packageWasDeliveredAtRightPositionX1(P2,D2,5): + droneDX(D2,5)
 + 100 droneDelivered(D2,P2,5) <= 105
 packageWasDeliveredAtRightPositionX1(P2,D3,1): + droneDX(D3,1)
 + 100 droneDelivered(D3,P2,1) <= 105
 packageWasDeliveredAtRightPositionX1(P2,D3,2): + droneDX(D3,2)
 + 100 droneDelivered(D3,P2,2) <= 105
 packageWasDeliveredAtRightPositionX1(P2,D3,3): + droneDX(D3,3)
 + 100 droneDelivered(D3,P2,3) <= 105
 packageWasDeliveredAtRightPositionX1(P2,D3,4): + droneDX(D3,4)
 + 100 droneDelivered(D3,P2,4) <= 105
 packageWasDeliveredAtRightPositionX1(P2,D3,5): + droneDX(D3,5)
 + 100 droneDelivered(D3,P2,5) <= 105
 packageWasDeliveredAtRightPositionX1(P2,D4,1): + droneDX(D4,1)
 + 100 droneDelivered(D4,P2,1) <= 105
 packageWasDeliveredAtRightPositionX1(P2,D4,2): + droneDX(D4,2)
 + 100 droneDelivered(D4,P2,2) <= 105
 packageWasDeliveredAtRightPositionX1(P2,D4,3): + droneDX(D4,3)
 + 100 droneDelivered(D4,P2,3) <= 105
 packageWasDeliveredAtRightPositionX1(P2,D4,4): + droneDX(D4,4)
 + 100 droneDelivered(D4,P2,4) <= 105
 packageWasDeliveredAtRightPositionX1(P2,D4,5): + droneDX(D4,5)
 + 100 droneDelivered(D4,P2,5) <= 105
 packageWasDeliveredAtRightPositionX1(P2,D5,1): + droneDX(D5,1)
 + 100 droneDelivered(D5,P2,1) <= 105
 packageWasDeliveredAtRightPositionX1(P2,D5,2): + droneDX(D5,2)
 + 100 droneDelivered(D5,P2,2) <= 105
 packageWasDeliveredAtRightPositionX1(P2,D5,3): + droneDX(D5,3)
 + 100 droneDelivered(D5,P2,3) <= 105
 packageWasDeliveredAtRightPositionX1(P2,D5,4): + droneDX(D5,4)
 + 100 droneDelivered(D5,P2,4) <= 105
 packageWasDeliveredAtRightPositionX1(P2,D5,5): + droneDX(D5,5)
 + 100 droneDelivered(D5,P2,5) <= 105
 packageWasDeliveredAtRightPositionX1(P3,D1,1): + droneDX(D1,1)
 + 100 droneDelivered(D1,P3,1) <= 120
 packageWasDeliveredAtRightPositionX1(P3,D1,2): + droneDX(D1,2)
 + 100 droneDelivered(D1,P3,2) <= 120
 packageWasDeliveredAtRightPositionX1(P3,D1,3): + droneDX(D1,3)
 + 100 droneDelivered(D1,P3,3) <= 120
 packageWasDeliveredAtRightPositionX1(P3,D1,4): + droneDX(D1,4)
 + 100 droneDelivered(D1,P3,4) <= 120
 packageWasDeliveredAtRightPositionX1(P3,D1,5): + droneDX(D1,5)
 + 100 droneDelivered(D1,P3,5) <= 120
 packageWasDeliveredAtRightPositionX1(P3,D2,1): + droneDX(D2,1)
 + 100 droneDelivered(D2,P3,1) <= 120
 packageWasDeliveredAtRightPositionX1(P3,D2,2): + droneDX(D2,2)
 + 100 droneDelivered(D2,P3,2) <= 120
 packageWasDeliveredAtRightPositionX1(P3,D2,3): + droneDX(D2,3)
 + 100 droneDelivered(D2,P3,3) <= 120
 packageWasDeliveredAtRightPositionX1(P3,D2,4): + droneDX(D2,4)
 + 100 droneDelivered(D2,P3,4) <= 120
 packageWasDeliveredAtRightPositionX1(P3,D2,5): + droneDX(D2,5)
 + 100 droneDelivered(D2,P3,5) <= 120
 packageWasDeliveredAtRightPositionX1(P3,D3,1): + droneDX(D3,1)
 + 100 droneDelivered(D3,P3,1) <= 120
 packageWasDeliveredAtRightPositionX1(P3,D3,2): + droneDX(D3,2)
 + 100 droneDelivered(D3,P3,2) <= 120
 packageWasDeliveredAtRightPositionX1(P3,D3,3): + droneDX(D3,3)
 + 100 droneDelivered(D3,P3,3) <= 120
 packageWasDeliveredAtRightPositionX1(P3,D3,4): + droneDX(D3,4)
 + 100 droneDelivered(D3,P3,4) <= 120
 packageWasDeliveredAtRightPositionX1(P3,D3,5): + droneDX(D3,5)
 + 100 droneDelivered(D3,P3,5) <= 120
 packageWasDeliveredAtRightPositionX1(P3,D4,1): + droneDX(D4,1)
 + 100 droneDelivered(D4,P3,1) <= 120
 packageWasDeliveredAtRightPositionX1(P3,D4,2): + droneDX(D4,2)
 + 100 droneDelivered(D4,P3,2) <= 120
 packageWasDeliveredAtRightPositionX1(P3,D4,3): + droneDX(D4,3)
 + 100 droneDelivered(D4,P3,3) <= 120
 packageWasDeliveredAtRightPositionX1(P3,D4,4): + droneDX(D4,4)
 + 100 droneDelivered(D4,P3,4) <= 120
 packageWasDeliveredAtRightPositionX1(P3,D4,5): + droneDX(D4,5)
 + 100 droneDelivered(D4,P3,5) <= 120
 packageWasDeliveredAtRightPositionX1(P3,D5,1): + droneDX(D5,1)
 + 100 droneDelivered(D5,P3,1) <= 120
 packageWasDeliveredAtRightPositionX1(P3,D5,2): + droneDX(D5,2)
 + 100 droneDelivered(D5,P3,2) <= 120
 packageWasDeliveredAtRightPositionX1(P3,D5,3): + droneDX(D5,3)
 + 100 droneDelivered(D5,P3,3) <= 120
 packageWasDeliveredAtRightPositionX1(P3,D5,4): + droneDX(D5,4)
 + 100 droneDelivered(D5,P3,4) <= 120
 packageWasDeliveredAtRightPositionX1(P3,D5,5): + droneDX(D5,5)
 + 100 droneDelivered(D5,P3,5) <= 120
 packageWasDeliveredAtRightPositionX1(P4,D1,1): + droneDX(D1,1)
 + 100 droneDelivered(D1,P4,1) <= 105
 packageWasDeliveredAtRightPositionX1(P4,D1,2): + droneDX(D1,2)
 + 100 droneDelivered(D1,P4,2) <= 105
 packageWasDeliveredAtRightPositionX1(P4,D1,3): + droneDX(D1,3)
 + 100 droneDelivered(D1,P4,3) <= 105
 packageWasDeliveredAtRightPositionX1(P4,D1,4): + droneDX(D1,4)
 + 100 droneDelivered(D1,P4,4) <= 105
 packageWasDeliveredAtRightPositionX1(P4,D1,5): + droneDX(D1,5)
 + 100 droneDelivered(D1,P4,5) <= 105
 packageWasDeliveredAtRightPositionX1(P4,D2,1): + droneDX(D2,1)
 + 100 droneDelivered(D2,P4,1) <= 105
 packageWasDeliveredAtRightPositionX1(P4,D2,2): + droneDX(D2,2)
 + 100 droneDelivered(D2,P4,2) <= 105
 packageWasDeliveredAtRightPositionX1(P4,D2,3): + droneDX(D2,3)
 + 100 droneDelivered(D2,P4,3) <= 105
 packageWasDeliveredAtRightPositionX1(P4,D2,4): + droneDX(D2,4)
 + 100 droneDelivered(D2,P4,4) <= 105
 packageWasDeliveredAtRightPositionX1(P4,D2,5): + droneDX(D2,5)
 + 100 droneDelivered(D2,P4,5) <= 105
 packageWasDeliveredAtRightPositionX1(P4,D3,1): + droneDX(D3,1)
 + 100 droneDelivered(D3,P4,1) <= 105
 packageWasDeliveredAtRightPositionX1(P4,D3,2): + droneDX(D3,2)
 + 100 droneDelivered(D3,P4,2) <= 105
 packageWasDeliveredAtRightPositionX1(P4,D3,3): + droneDX(D3,3)
 + 100 droneDelivered(D3,P4,3) <= 105
 packageWasDeliveredAtRightPositionX1(P4,D3,4): + droneDX(D3,4)
 + 100 droneDelivered(D3,P4,4) <= 105
 packageWasDeliveredAtRightPositionX1(P4,D3,5): + droneDX(D3,5)
 + 100 droneDelivered(D3,P4,5) <= 105
 packageWasDeliveredAtRightPositionX1(P4,D4,1): + droneDX(D4,1)
 + 100 droneDelivered(D4,P4,1) <= 105
 packageWasDeliveredAtRightPositionX1(P4,D4,2): + droneDX(D4,2)
 + 100 droneDelivered(D4,P4,2) <= 105
 packageWasDeliveredAtRightPositionX1(P4,D4,3): + droneDX(D4,3)
 + 100 droneDelivered(D4,P4,3) <= 105
 packageWasDeliveredAtRightPositionX1(P4,D4,4): + droneDX(D4,4)
 + 100 droneDelivered(D4,P4,4) <= 105
 packageWasDeliveredAtRightPositionX1(P4,D4,5): + droneDX(D4,5)
 + 100 droneDelivered(D4,P4,5) <= 105
 packageWasDeliveredAtRightPositionX1(P4,D5,1): + droneDX(D5,1)
 + 100 droneDelivered(D5,P4,1) <= 105
 packageWasDeliveredAtRightPositionX1(P4,D5,2): + droneDX(D5,2)
 + 100 droneDelivered(D5,P4,2) <= 105
 packageWasDeliveredAtRightPositionX1(P4,D5,3): + droneDX(D5,3)
 + 100 droneDelivered(D5,P4,3) <= 105
 packageWasDeliveredAtRightPositionX1(P4,D5,4): + droneDX(D5,4)
 + 100 droneDelivered(D5,P4,4) <= 105
 packageWasDeliveredAtRightPositionX1(P4,D5,5): + droneDX(D5,5)
 + 100 droneDelivered(D5,P4,5) <= 105
 packageWasDeliveredAtRightPositionX1(P5,D1,1): + droneDX(D1,1)
 + 100 droneDelivered(D1,P5,1) <= 110
 packageWasDeliveredAtRightPositionX1(P5,D1,2): + droneDX(D1,2)
 + 100 droneDelivered(D1,P5,2) <= 110
 packageWasDeliveredAtRightPositionX1(P5,D1,3): + droneDX(D1,3)
 + 100 droneDelivered(D1,P5,3) <= 110
 packageWasDeliveredAtRightPositionX1(P5,D1,4): + droneDX(D1,4)
 + 100 droneDelivered(D1,P5,4) <= 110
 packageWasDeliveredAtRightPositionX1(P5,D1,5): + droneDX(D1,5)
 + 100 droneDelivered(D1,P5,5) <= 110
 packageWasDeliveredAtRightPositionX1(P5,D2,1): + droneDX(D2,1)
 + 100 droneDelivered(D2,P5,1) <= 110
 packageWasDeliveredAtRightPositionX1(P5,D2,2): + droneDX(D2,2)
 + 100 droneDelivered(D2,P5,2) <= 110
 packageWasDeliveredAtRightPositionX1(P5,D2,3): + droneDX(D2,3)
 + 100 droneDelivered(D2,P5,3) <= 110
 packageWasDeliveredAtRightPositionX1(P5,D2,4): + droneDX(D2,4)
 + 100 droneDelivered(D2,P5,4) <= 110
 packageWasDeliveredAtRightPositionX1(P5,D2,5): + droneDX(D2,5)
 + 100 droneDelivered(D2,P5,5) <= 110
 packageWasDeliveredAtRightPositionX1(P5,D3,1): + droneDX(D3,1)
 + 100 droneDelivered(D3,P5,1) <= 110
 packageWasDeliveredAtRightPositionX1(P5,D3,2): + droneDX(D3,2)
 + 100 droneDelivered(D3,P5,2) <= 110
 packageWasDeliveredAtRightPositionX1(P5,D3,3): + droneDX(D3,3)
 + 100 droneDelivered(D3,P5,3) <= 110
 packageWasDeliveredAtRightPositionX1(P5,D3,4): + droneDX(D3,4)
 + 100 droneDelivered(D3,P5,4) <= 110
 packageWasDeliveredAtRightPositionX1(P5,D3,5): + droneDX(D3,5)
 + 100 droneDelivered(D3,P5,5) <= 110
 packageWasDeliveredAtRightPositionX1(P5,D4,1): + droneDX(D4,1)
 + 100 droneDelivered(D4,P5,1) <= 110
 packageWasDeliveredAtRightPositionX1(P5,D4,2): + droneDX(D4,2)
 + 100 droneDelivered(D4,P5,2) <= 110
 packageWasDeliveredAtRightPositionX1(P5,D4,3): + droneDX(D4,3)
 + 100 droneDelivered(D4,P5,3) <= 110
 packageWasDeliveredAtRightPositionX1(P5,D4,4): + droneDX(D4,4)
 + 100 droneDelivered(D4,P5,4) <= 110
 packageWasDeliveredAtRightPositionX1(P5,D4,5): + droneDX(D4,5)
 + 100 droneDelivered(D4,P5,5) <= 110
 packageWasDeliveredAtRightPositionX1(P5,D5,1): + droneDX(D5,1)
 + 100 droneDelivered(D5,P5,1) <= 110
 packageWasDeliveredAtRightPositionX1(P5,D5,2): + droneDX(D5,2)
 + 100 droneDelivered(D5,P5,2) <= 110
 packageWasDeliveredAtRightPositionX1(P5,D5,3): + droneDX(D5,3)
 + 100 droneDelivered(D5,P5,3) <= 110
 packageWasDeliveredAtRightPositionX1(P5,D5,4): + droneDX(D5,4)
 + 100 droneDelivered(D5,P5,4) <= 110
 packageWasDeliveredAtRightPositionX1(P5,D5,5): + droneDX(D5,5)
 + 100 droneDelivered(D5,P5,5) <= 110
 packageWasDeliveredAtRightPositionX1(P6,D1,1): + droneDX(D1,1)
 + 100 droneDelivered(D1,P6,1) <= 100
 packageWasDeliveredAtRightPositionX1(P6,D1,2): + droneDX(D1,2)
 + 100 droneDelivered(D1,P6,2) <= 100
 packageWasDeliveredAtRightPositionX1(P6,D1,3): + droneDX(D1,3)
 + 100 droneDelivered(D1,P6,3) <= 100
 packageWasDeliveredAtRightPositionX1(P6,D1,4): + droneDX(D1,4)
 + 100 droneDelivered(D1,P6,4) <= 100
 packageWasDeliveredAtRightPositionX1(P6,D1,5): + droneDX(D1,5)
 + 100 droneDelivered(D1,P6,5) <= 100
 packageWasDeliveredAtRightPositionX1(P6,D2,1): + droneDX(D2,1)
 + 100 droneDelivered(D2,P6,1) <= 100
 packageWasDeliveredAtRightPositionX1(P6,D2,2): + droneDX(D2,2)
 + 100 droneDelivered(D2,P6,2) <= 100
 packageWasDeliveredAtRightPositionX1(P6,D2,3): + droneDX(D2,3)
 + 100 droneDelivered(D2,P6,3) <= 100
 packageWasDeliveredAtRightPositionX1(P6,D2,4): + droneDX(D2,4)
 + 100 droneDelivered(D2,P6,4) <= 100
 packageWasDeliveredAtRightPositionX1(P6,D2,5): + droneDX(D2,5)
 + 100 droneDelivered(D2,P6,5) <= 100
 packageWasDeliveredAtRightPositionX1(P6,D3,1): + droneDX(D3,1)
 + 100 droneDelivered(D3,P6,1) <= 100
 packageWasDeliveredAtRightPositionX1(P6,D3,2): + droneDX(D3,2)
 + 100 droneDelivered(D3,P6,2) <= 100
 packageWasDeliveredAtRightPositionX1(P6,D3,3): + droneDX(D3,3)
 + 100 droneDelivered(D3,P6,3) <= 100
 packageWasDeliveredAtRightPositionX1(P6,D3,4): + droneDX(D3,4)
 + 100 droneDelivered(D3,P6,4) <= 100
 packageWasDeliveredAtRightPositionX1(P6,D3,5): + droneDX(D3,5)
 + 100 droneDelivered(D3,P6,5) <= 100
 packageWasDeliveredAtRightPositionX1(P6,D4,1): + droneDX(D4,1)
 + 100 droneDelivered(D4,P6,1) <= 100
 packageWasDeliveredAtRightPositionX1(P6,D4,2): + droneDX(D4,2)
 + 100 droneDelivered(D4,P6,2) <= 100
 packageWasDeliveredAtRightPositionX1(P6,D4,3): + droneDX(D4,3)
 + 100 droneDelivered(D4,P6,3) <= 100
 packageWasDeliveredAtRightPositionX1(P6,D4,4): + droneDX(D4,4)
 + 100 droneDelivered(D4,P6,4) <= 100
 packageWasDeliveredAtRightPositionX1(P6,D4,5): + droneDX(D4,5)
 + 100 droneDelivered(D4,P6,5) <= 100
 packageWasDeliveredAtRightPositionX1(P6,D5,1): + droneDX(D5,1)
 + 100 droneDelivered(D5,P6,1) <= 100
 packageWasDeliveredAtRightPositionX1(P6,D5,2): + droneDX(D5,2)
 + 100 droneDelivered(D5,P6,2) <= 100
 packageWasDeliveredAtRightPositionX1(P6,D5,3): + droneDX(D5,3)
 + 100 droneDelivered(D5,P6,3) <= 100
 packageWasDeliveredAtRightPositionX1(P6,D5,4): + droneDX(D5,4)
 + 100 droneDelivered(D5,P6,4) <= 100
 packageWasDeliveredAtRightPositionX1(P6,D5,5): + droneDX(D5,5)
 + 100 droneDelivered(D5,P6,5) <= 100
 packageWasDeliveredAtRightPositionX2(P1,D1,1): - droneDX(D1,1)
 + 100 droneDelivered(D1,P1,1) <= 96
 packageWasDeliveredAtRightPositionX2(P1,D1,2): - droneDX(D1,2)
 + 100 droneDelivered(D1,P1,2) <= 96
 packageWasDeliveredAtRightPositionX2(P1,D1,3): - droneDX(D1,3)
 + 100 droneDelivered(D1,P1,3) <= 96
 packageWasDeliveredAtRightPositionX2(P1,D1,4): - droneDX(D1,4)
 + 100 droneDelivered(D1,P1,4) <= 96
 packageWasDeliveredAtRightPositionX2(P1,D1,5): - droneDX(D1,5)
 + 100 droneDelivered(D1,P1,5) <= 96
 packageWasDeliveredAtRightPositionX2(P1,D2,1): - droneDX(D2,1)
 + 100 droneDelivered(D2,P1,1) <= 96
 packageWasDeliveredAtRightPositionX2(P1,D2,2): - droneDX(D2,2)
 + 100 droneDelivered(D2,P1,2) <= 96
 packageWasDeliveredAtRightPositionX2(P1,D2,3): - droneDX(D2,3)
 + 100 droneDelivered(D2,P1,3) <= 96
 packageWasDeliveredAtRightPositionX2(P1,D2,4): - droneDX(D2,4)
 + 100 droneDelivered(D2,P1,4) <= 96
 packageWasDeliveredAtRightPositionX2(P1,D2,5): - droneDX(D2,5)
 + 100 droneDelivered(D2,P1,5) <= 96
 packageWasDeliveredAtRightPositionX2(P1,D3,1): - droneDX(D3,1)
 + 100 droneDelivered(D3,P1,1) <= 96
 packageWasDeliveredAtRightPositionX2(P1,D3,2): - droneDX(D3,2)
 + 100 droneDelivered(D3,P1,2) <= 96
 packageWasDeliveredAtRightPositionX2(P1,D3,3): - droneDX(D3,3)
 + 100 droneDelivered(D3,P1,3) <= 96
 packageWasDeliveredAtRightPositionX2(P1,D3,4): - droneDX(D3,4)
 + 100 droneDelivered(D3,P1,4) <= 96
 packageWasDeliveredAtRightPositionX2(P1,D3,5): - droneDX(D3,5)
 + 100 droneDelivered(D3,P1,5) <= 96
 packageWasDeliveredAtRightPositionX2(P1,D4,1): - droneDX(D4,1)
 + 100 droneDelivered(D4,P1,1) <= 96
 packageWasDeliveredAtRightPositionX2(P1,D4,2): - droneDX(D4,2)
 + 100 droneDelivered(D4,P1,2) <= 96
 packageWasDeliveredAtRightPositionX2(P1,D4,3): - droneDX(D4,3)
 + 100 droneDelivered(D4,P1,3) <= 96
 packageWasDeliveredAtRightPositionX2(P1,D4,4): - droneDX(D4,4)
 + 100 droneDelivered(D4,P1,4) <= 96
 packageWasDeliveredAtRightPositionX2(P1,D4,5): - droneDX(D4,5)
 + 100 droneDelivered(D4,P1,5) <= 96
 packageWasDeliveredAtRightPositionX2(P1,D5,1): - droneDX(D5,1)
 + 100 droneDelivered(D5,P1,1) <= 96
 packageWasDeliveredAtRightPositionX2(P1,D5,2): - droneDX(D5,2)
 + 100 droneDelivered(D5,P1,2) <= 96
 packageWasDeliveredAtRightPositionX2(P1,D5,3): - droneDX(D5,3)
 + 100 droneDelivered(D5,P1,3) <= 96
 packageWasDeliveredAtRightPositionX2(P1,D5,4): - droneDX(D5,4)
 + 100 droneDelivered(D5,P1,4) <= 96
 packageWasDeliveredAtRightPositionX2(P1,D5,5): - droneDX(D5,5)
 + 100 droneDelivered(D5,P1,5) <= 96
 packageWasDeliveredAtRightPositionX2(P2,D1,1): - droneDX(D1,1)
 + 100 droneDelivered(D1,P2,1) <= 95
 packageWasDeliveredAtRightPositionX2(P2,D1,2): - droneDX(D1,2)
 + 100 droneDelivered(D1,P2,2) <= 95
 packageWasDeliveredAtRightPositionX2(P2,D1,3): - droneDX(D1,3)
 + 100 droneDelivered(D1,P2,3) <= 95
 packageWasDeliveredAtRightPositionX2(P2,D1,4): - droneDX(D1,4)
 + 100 droneDelivered(D1,P2,4) <= 95
 packageWasDeliveredAtRightPositionX2(P2,D1,5): - droneDX(D1,5)
 + 100 droneDelivered(D1,P2,5) <= 95
 packageWasDeliveredAtRightPositionX2(P2,D2,1): - droneDX(D2,1)
 + 100 droneDelivered(D2,P2,1) <= 95
 packageWasDeliveredAtRightPositionX2(P2,D2,2): - droneDX(D2,2)
 + 100 droneDelivered(D2,P2,2) <= 95
 packageWasDeliveredAtRightPositionX2(P2,D2,3): - droneDX(D2,3)
 + 100 droneDelivered(D2,P2,3) <= 95
 packageWasDeliveredAtRightPositionX2(P2,D2,4): - droneDX(D2,4)
 + 100 droneDelivered(D2,P2,4) <= 95
 packageWasDeliveredAtRightPositionX2(P2,D2,5): - droneDX(D2,5)
 + 100 droneDelivered(D2,P2,5) <= 95
 packageWasDeliveredAtRightPositionX2(P2,D3,1): - droneDX(D3,1)
 + 100 droneDelivered(D3,P2,1) <= 95
 packageWasDeliveredAtRightPositionX2(P2,D3,2): - droneDX(D3,2)
 + 100 droneDelivered(D3,P2,2) <= 95
 packageWasDeliveredAtRightPositionX2(P2,D3,3): - droneDX(D3,3)
 + 100 droneDelivered(D3,P2,3) <= 95
 packageWasDeliveredAtRightPositionX2(P2,D3,4): - droneDX(D3,4)
 + 100 droneDelivered(D3,P2,4) <= 95
 packageWasDeliveredAtRightPositionX2(P2,D3,5): - droneDX(D3,5)
 + 100 droneDelivered(D3,P2,5) <= 95
 packageWasDeliveredAtRightPositionX2(P2,D4,1): - droneDX(D4,1)
 + 100 droneDelivered(D4,P2,1) <= 95
 packageWasDeliveredAtRightPositionX2(P2,D4,2): - droneDX(D4,2)
 + 100 droneDelivered(D4,P2,2) <= 95
 packageWasDeliveredAtRightPositionX2(P2,D4,3): - droneDX(D4,3)
 + 100 droneDelivered(D4,P2,3) <= 95
 packageWasDeliveredAtRightPositionX2(P2,D4,4): - droneDX(D4,4)
 + 100 droneDelivered(D4,P2,4) <= 95
 packageWasDeliveredAtRightPositionX2(P2,D4,5): - droneDX(D4,5)
 + 100 droneDelivered(D4,P2,5) <= 95
 packageWasDeliveredAtRightPositionX2(P2,D5,1): - droneDX(D5,1)
 + 100 droneDelivered(D5,P2,1) <= 95
 packageWasDeliveredAtRightPositionX2(P2,D5,2): - droneDX(D5,2)
 + 100 droneDelivered(D5,P2,2) <= 95
 packageWasDeliveredAtRightPositionX2(P2,D5,3): - droneDX(D5,3)
 + 100 droneDelivered(D5,P2,3) <= 95
 packageWasDeliveredAtRightPositionX2(P2,D5,4): - droneDX(D5,4)
 + 100 droneDelivered(D5,P2,4) <= 95
 packageWasDeliveredAtRightPositionX2(P2,D5,5): - droneDX(D5,5)
 + 100 droneDelivered(D5,P2,5) <= 95
 packageWasDeliveredAtRightPositionX2(P3,D1,1): - droneDX(D1,1)
 + 100 droneDelivered(D1,P3,1) <= 80
 packageWasDeliveredAtRightPositionX2(P3,D1,2): - droneDX(D1,2)
 + 100 droneDelivered(D1,P3,2) <= 80
 packageWasDeliveredAtRightPositionX2(P3,D1,3): - droneDX(D1,3)
 + 100 droneDelivered(D1,P3,3) <= 80
 packageWasDeliveredAtRightPositionX2(P3,D1,4): - droneDX(D1,4)
 + 100 droneDelivered(D1,P3,4) <= 80
 packageWasDeliveredAtRightPositionX2(P3,D1,5): - droneDX(D1,5)
 + 100 droneDelivered(D1,P3,5) <= 80
 packageWasDeliveredAtRightPositionX2(P3,D2,1): - droneDX(D2,1)
 + 100 droneDelivered(D2,P3,1) <= 80
 packageWasDeliveredAtRightPositionX2(P3,D2,2): - droneDX(D2,2)
 + 100 droneDelivered(D2,P3,2) <= 80
 packageWasDeliveredAtRightPositionX2(P3,D2,3): - droneDX(D2,3)
 + 100 droneDelivered(D2,P3,3) <= 80
 packageWasDeliveredAtRightPositionX2(P3,D2,4): - droneDX(D2,4)
 + 100 droneDelivered(D2,P3,4) <= 80
 packageWasDeliveredAtRightPositionX2(P3,D2,5): - droneDX(D2,5)
 + 100 droneDelivered(D2,P3,5) <= 80
 packageWasDeliveredAtRightPositionX2(P3,D3,1): - droneDX(D3,1)
 + 100 droneDelivered(D3,P3,1) <= 80
 packageWasDeliveredAtRightPositionX2(P3,D3,2): - droneDX(D3,2)
 + 100 droneDelivered(D3,P3,2) <= 80
 packageWasDeliveredAtRightPositionX2(P3,D3,3): - droneDX(D3,3)
 + 100 droneDelivered(D3,P3,3) <= 80
 packageWasDeliveredAtRightPositionX2(P3,D3,4): - droneDX(D3,4)
 + 100 droneDelivered(D3,P3,4) <= 80
 packageWasDeliveredAtRightPositionX2(P3,D3,5): - droneDX(D3,5)
 + 100 droneDelivered(D3,P3,5) <= 80
 packageWasDeliveredAtRightPositionX2(P3,D4,1): - droneDX(D4,1)
 + 100 droneDelivered(D4,P3,1) <= 80
 packageWasDeliveredAtRightPositionX2(P3,D4,2): - droneDX(D4,2)
 + 100 droneDelivered(D4,P3,2) <= 80
 packageWasDeliveredAtRightPositionX2(P3,D4,3): - droneDX(D4,3)
 + 100 droneDelivered(D4,P3,3) <= 80
 packageWasDeliveredAtRightPositionX2(P3,D4,4): - droneDX(D4,4)
 + 100 droneDelivered(D4,P3,4) <= 80
 packageWasDeliveredAtRightPositionX2(P3,D4,5): - droneDX(D4,5)
 + 100 droneDelivered(D4,P3,5) <= 80
 packageWasDeliveredAtRightPositionX2(P3,D5,1): - droneDX(D5,1)
 + 100 droneDelivered(D5,P3,1) <= 80
 packageWasDeliveredAtRightPositionX2(P3,D5,2): - droneDX(D5,2)
 + 100 droneDelivered(D5,P3,2) <= 80
 packageWasDeliveredAtRightPositionX2(P3,D5,3): - droneDX(D5,3)
 + 100 droneDelivered(D5,P3,3) <= 80
 packageWasDeliveredAtRightPositionX2(P3,D5,4): - droneDX(D5,4)
 + 100 droneDelivered(D5,P3,4) <= 80
 packageWasDeliveredAtRightPositionX2(P3,D5,5): - droneDX(D5,5)
 + 100 droneDelivered(D5,P3,5) <= 80
 packageWasDeliveredAtRightPositionX2(P4,D1,1): - droneDX(D1,1)
 + 100 droneDelivered(D1,P4,1) <= 95
 packageWasDeliveredAtRightPositionX2(P4,D1,2): - droneDX(D1,2)
 + 100 droneDelivered(D1,P4,2) <= 95
 packageWasDeliveredAtRightPositionX2(P4,D1,3): - droneDX(D1,3)
 + 100 droneDelivered(D1,P4,3) <= 95
 packageWasDeliveredAtRightPositionX2(P4,D1,4): - droneDX(D1,4)
 + 100 droneDelivered(D1,P4,4) <= 95
 packageWasDeliveredAtRightPositionX2(P4,D1,5): - droneDX(D1,5)
 + 100 droneDelivered(D1,P4,5) <= 95
 packageWasDeliveredAtRightPositionX2(P4,D2,1): - droneDX(D2,1)
 + 100 droneDelivered(D2,P4,1) <= 95
 packageWasDeliveredAtRightPositionX2(P4,D2,2): - droneDX(D2,2)
 + 100 droneDelivered(D2,P4,2) <= 95
 packageWasDeliveredAtRightPositionX2(P4,D2,3): - droneDX(D2,3)
 + 100 droneDelivered(D2,P4,3) <= 95
 packageWasDeliveredAtRightPositionX2(P4,D2,4): - droneDX(D2,4)
 + 100 droneDelivered(D2,P4,4) <= 95
 packageWasDeliveredAtRightPositionX2(P4,D2,5): - droneDX(D2,5)
 + 100 droneDelivered(D2,P4,5) <= 95
 packageWasDeliveredAtRightPositionX2(P4,D3,1): - droneDX(D3,1)
 + 100 droneDelivered(D3,P4,1) <= 95
 packageWasDeliveredAtRightPositionX2(P4,D3,2): - droneDX(D3,2)
 + 100 droneDelivered(D3,P4,2) <= 95
 packageWasDeliveredAtRightPositionX2(P4,D3,3): - droneDX(D3,3)
 + 100 droneDelivered(D3,P4,3) <= 95
 packageWasDeliveredAtRightPositionX2(P4,D3,4): - droneDX(D3,4)
 + 100 droneDelivered(D3,P4,4) <= 95
 packageWasDeliveredAtRightPositionX2(P4,D3,5): - droneDX(D3,5)
 + 100 droneDelivered(D3,P4,5) <= 95
 packageWasDeliveredAtRightPositionX2(P4,D4,1): - droneDX(D4,1)
 + 100 droneDelivered(D4,P4,1) <= 95
 packageWasDeliveredAtRightPositionX2(P4,D4,2): - droneDX(D4,2)
 + 100 droneDelivered(D4,P4,2) <= 95
 packageWasDeliveredAtRightPositionX2(P4,D4,3): - droneDX(D4,3)
 + 100 droneDelivered(D4,P4,3) <= 95
 packageWasDeliveredAtRightPositionX2(P4,D4,4): - droneDX(D4,4)
 + 100 droneDelivered(D4,P4,4) <= 95
 packageWasDeliveredAtRightPositionX2(P4,D4,5): - droneDX(D4,5)
 + 100 droneDelivered(D4,P4,5) <= 95
 packageWasDeliveredAtRightPositionX2(P4,D5,1): - droneDX(D5,1)
 + 100 droneDelivered(D5,P4,1) <= 95
 packageWasDeliveredAtRightPositionX2(P4,D5,2): - droneDX(D5,2)
 + 100 droneDelivered(D5,P4,2) <= 95
 packageWasDeliveredAtRightPositionX2(P4,D5,3): - droneDX(D5,3)
 + 100 droneDelivered(D5,P4,3) <= 95
 packageWasDeliveredAtRightPositionX2(P4,D5,4): - droneDX(D5,4)
 + 100 droneDelivered(D5,P4,4) <= 95
 packageWasDeliveredAtRightPositionX2(P4,D5,5): - droneDX(D5,5)
 + 100 droneDelivered(D5,P4,5) <= 95
 packageWasDeliveredAtRightPositionX2(P5,D1,1): - droneDX(D1,1)
 + 100 droneDelivered(D1,P5,1) <= 90
 packageWasDeliveredAtRightPositionX2(P5,D1,2): - droneDX(D1,2)
 + 100 droneDelivered(D1,P5,2) <= 90
 packageWasDeliveredAtRightPositionX2(P5,D1,3): - droneDX(D1,3)
 + 100 droneDelivered(D1,P5,3) <= 90
 packageWasDeliveredAtRightPositionX2(P5,D1,4): - droneDX(D1,4)
 + 100 droneDelivered(D1,P5,4) <= 90
 packageWasDeliveredAtRightPositionX2(P5,D1,5): - droneDX(D1,5)
 + 100 droneDelivered(D1,P5,5) <= 90
 packageWasDeliveredAtRightPositionX2(P5,D2,1): - droneDX(D2,1)
 + 100 droneDelivered(D2,P5,1) <= 90
 packageWasDeliveredAtRightPositionX2(P5,D2,2): - droneDX(D2,2)
 + 100 droneDelivered(D2,P5,2) <= 90
 packageWasDeliveredAtRightPositionX2(P5,D2,3): - droneDX(D2,3)
 + 100 droneDelivered(D2,P5,3) <= 90
 packageWasDeliveredAtRightPositionX2(P5,D2,4): - droneDX(D2,4)
 + 100 droneDelivered(D2,P5,4) <= 90
 packageWasDeliveredAtRightPositionX2(P5,D2,5): - droneDX(D2,5)
 + 100 droneDelivered(D2,P5,5) <= 90
 packageWasDeliveredAtRightPositionX2(P5,D3,1): - droneDX(D3,1)
 + 100 droneDelivered(D3,P5,1) <= 90
 packageWasDeliveredAtRightPositionX2(P5,D3,2): - droneDX(D3,2)
 + 100 droneDelivered(D3,P5,2) <= 90
 packageWasDeliveredAtRightPositionX2(P5,D3,3): - droneDX(D3,3)
 + 100 droneDelivered(D3,P5,3) <= 90
 packageWasDeliveredAtRightPositionX2(P5,D3,4): - droneDX(D3,4)
 + 100 droneDelivered(D3,P5,4) <= 90
 packageWasDeliveredAtRightPositionX2(P5,D3,5): - droneDX(D3,5)
 + 100 droneDelivered(D3,P5,5) <= 90
 packageWasDeliveredAtRightPositionX2(P5,D4,1): - droneDX(D4,1)
 + 100 droneDelivered(D4,P5,1) <= 90
 packageWasDeliveredAtRightPositionX2(P5,D4,2): - droneDX(D4,2)
 + 100 droneDelivered(D4,P5,2) <= 90
 packageWasDeliveredAtRightPositionX2(P5,D4,3): - droneDX(D4,3)
 + 100 droneDelivered(D4,P5,3) <= 90
 packageWasDeliveredAtRightPositionX2(P5,D4,4): - droneDX(D4,4)
 + 100 droneDelivered(D4,P5,4) <= 90
 packageWasDeliveredAtRightPositionX2(P5,D4,5): - droneDX(D4,5)
 + 100 droneDelivered(D4,P5,5) <= 90
 packageWasDeliveredAtRightPositionX2(P5,D5,1): - droneDX(D5,1)
 + 100 droneDelivered(D5,P5,1) <= 90
 packageWasDeliveredAtRightPositionX2(P5,D5,2): - droneDX(D5,2)
 + 100 droneDelivered(D5,P5,2) <= 90
 packageWasDeliveredAtRightPositionX2(P5,D5,3): - droneDX(D5,3)
 + 100 droneDelivered(D5,P5,3) <= 90
 packageWasDeliveredAtRightPositionX2(P5,D5,4): - droneDX(D5,4)
 + 100 droneDelivered(D5,P5,4) <= 90
 packageWasDeliveredAtRightPositionX2(P5,D5,5): - droneDX(D5,5)
 + 100 droneDelivered(D5,P5,5) <= 90
 packageWasDeliveredAtRightPositionX2(P6,D1,1): - droneDX(D1,1)
 + 100 droneDelivered(D1,P6,1) <= 100
 packageWasDeliveredAtRightPositionX2(P6,D1,2): - droneDX(D1,2)
 + 100 droneDelivered(D1,P6,2) <= 100
 packageWasDeliveredAtRightPositionX2(P6,D1,3): - droneDX(D1,3)
 + 100 droneDelivered(D1,P6,3) <= 100
 packageWasDeliveredAtRightPositionX2(P6,D1,4): - droneDX(D1,4)
 + 100 droneDelivered(D1,P6,4) <= 100
 packageWasDeliveredAtRightPositionX2(P6,D1,5): - droneDX(D1,5)
 + 100 droneDelivered(D1,P6,5) <= 100
 packageWasDeliveredAtRightPositionX2(P6,D2,1): - droneDX(D2,1)
 + 100 droneDelivered(D2,P6,1) <= 100
 packageWasDeliveredAtRightPositionX2(P6,D2,2): - droneDX(D2,2)
 + 100 droneDelivered(D2,P6,2) <= 100
 packageWasDeliveredAtRightPositionX2(P6,D2,3): - droneDX(D2,3)
 + 100 droneDelivered(D2,P6,3) <= 100
 packageWasDeliveredAtRightPositionX2(P6,D2,4): - droneDX(D2,4)
 + 100 droneDelivered(D2,P6,4) <= 100
 packageWasDeliveredAtRightPositionX2(P6,D2,5): - droneDX(D2,5)
 + 100 droneDelivered(D2,P6,5) <= 100
 packageWasDeliveredAtRightPositionX2(P6,D3,1): - droneDX(D3,1)
 + 100 droneDelivered(D3,P6,1) <= 100
 packageWasDeliveredAtRightPositionX2(P6,D3,2): - droneDX(D3,2)
 + 100 droneDelivered(D3,P6,2) <= 100
 packageWasDeliveredAtRightPositionX2(P6,D3,3): - droneDX(D3,3)
 + 100 droneDelivered(D3,P6,3) <= 100
 packageWasDeliveredAtRightPositionX2(P6,D3,4): - droneDX(D3,4)
 + 100 droneDelivered(D3,P6,4) <= 100
 packageWasDeliveredAtRightPositionX2(P6,D3,5): - droneDX(D3,5)
 + 100 droneDelivered(D3,P6,5) <= 100
 packageWasDeliveredAtRightPositionX2(P6,D4,1): - droneDX(D4,1)
 + 100 droneDelivered(D4,P6,1) <= 100
 packageWasDeliveredAtRightPositionX2(P6,D4,2): - droneDX(D4,2)
 + 100 droneDelivered(D4,P6,2) <= 100
 packageWasDeliveredAtRightPositionX2(P6,D4,3): - droneDX(D4,3)
 + 100 droneDelivered(D4,P6,3) <= 100
 packageWasDeliveredAtRightPositionX2(P6,D4,4): - droneDX(D4,4)
 + 100 droneDelivered(D4,P6,4) <= 100
 packageWasDeliveredAtRightPositionX2(P6,D4,5): - droneDX(D4,5)
 + 100 droneDelivered(D4,P6,5) <= 100
 packageWasDeliveredAtRightPositionX2(P6,D5,1): - droneDX(D5,1)
 + 100 droneDelivered(D5,P6,1) <= 100
 packageWasDeliveredAtRightPositionX2(P6,D5,2): - droneDX(D5,2)
 + 100 droneDelivered(D5,P6,2) <= 100
 packageWasDeliveredAtRightPositionX2(P6,D5,3): - droneDX(D5,3)
 + 100 droneDelivered(D5,P6,3) <= 100
 packageWasDeliveredAtRightPositionX2(P6,D5,4): - droneDX(D5,4)
 + 100 droneDelivered(D5,P6,4) <= 100
 packageWasDeliveredAtRightPositionX2(P6,D5,5): - droneDX(D5,5)
 + 100 droneDelivered(D5,P6,5) <= 100
 packageWasDeliveredAtRightPositionY1(P1,D1,1): + droneDY(D1,1)
 + 100 droneDelivered(D1,P1,1) <= 100
 packageWasDeliveredAtRightPositionY1(P1,D1,2): + droneDY(D1,2)
 + 100 droneDelivered(D1,P1,2) <= 100
 packageWasDeliveredAtRightPositionY1(P1,D1,3): + droneDY(D1,3)
 + 100 droneDelivered(D1,P1,3) <= 100
 packageWasDeliveredAtRightPositionY1(P1,D1,4): + droneDY(D1,4)
 + 100 droneDelivered(D1,P1,4) <= 100
 packageWasDeliveredAtRightPositionY1(P1,D1,5): + droneDY(D1,5)
 + 100 droneDelivered(D1,P1,5) <= 100
 packageWasDeliveredAtRightPositionY1(P1,D2,1): + droneDY(D2,1)
 + 100 droneDelivered(D2,P1,1) <= 100
 packageWasDeliveredAtRightPositionY1(P1,D2,2): + droneDY(D2,2)
 + 100 droneDelivered(D2,P1,2) <= 100
 packageWasDeliveredAtRightPositionY1(P1,D2,3): + droneDY(D2,3)
 + 100 droneDelivered(D2,P1,3) <= 100
 packageWasDeliveredAtRightPositionY1(P1,D2,4): + droneDY(D2,4)
 + 100 droneDelivered(D2,P1,4) <= 100
 packageWasDeliveredAtRightPositionY1(P1,D2,5): + droneDY(D2,5)
 + 100 droneDelivered(D2,P1,5) <= 100
 packageWasDeliveredAtRightPositionY1(P1,D3,1): + droneDY(D3,1)
 + 100 droneDelivered(D3,P1,1) <= 100
 packageWasDeliveredAtRightPositionY1(P1,D3,2): + droneDY(D3,2)
 + 100 droneDelivered(D3,P1,2) <= 100
 packageWasDeliveredAtRightPositionY1(P1,D3,3): + droneDY(D3,3)
 + 100 droneDelivered(D3,P1,3) <= 100
 packageWasDeliveredAtRightPositionY1(P1,D3,4): + droneDY(D3,4)
 + 100 droneDelivered(D3,P1,4) <= 100
 packageWasDeliveredAtRightPositionY1(P1,D3,5): + droneDY(D3,5)
 + 100 droneDelivered(D3,P1,5) <= 100
 packageWasDeliveredAtRightPositionY1(P1,D4,1): + droneDY(D4,1)
 + 100 droneDelivered(D4,P1,1) <= 100
 packageWasDeliveredAtRightPositionY1(P1,D4,2): + droneDY(D4,2)
 + 100 droneDelivered(D4,P1,2) <= 100
 packageWasDeliveredAtRightPositionY1(P1,D4,3): + droneDY(D4,3)
 + 100 droneDelivered(D4,P1,3) <= 100
 packageWasDeliveredAtRightPositionY1(P1,D4,4): + droneDY(D4,4)
 + 100 droneDelivered(D4,P1,4) <= 100
 packageWasDeliveredAtRightPositionY1(P1,D4,5): + droneDY(D4,5)
 + 100 droneDelivered(D4,P1,5) <= 100
 packageWasDeliveredAtRightPositionY1(P1,D5,1): + droneDY(D5,1)
 + 100 droneDelivered(D5,P1,1) <= 100
 packageWasDeliveredAtRightPositionY1(P1,D5,2): + droneDY(D5,2)
 + 100 droneDelivered(D5,P1,2) <= 100
 packageWasDeliveredAtRightPositionY1(P1,D5,3): + droneDY(D5,3)
 + 100 droneDelivered(D5,P1,3) <= 100
 packageWasDeliveredAtRightPositionY1(P1,D5,4): + droneDY(D5,4)
 + 100 droneDelivered(D5,P1,4) <= 100
 packageWasDeliveredAtRightPositionY1(P1,D5,5): + droneDY(D5,5)
 + 100 droneDelivered(D5,P1,5) <= 100
 packageWasDeliveredAtRightPositionY1(P2,D1,1): + droneDY(D1,1)
 + 100 droneDelivered(D1,P2,1) <= 100
 packageWasDeliveredAtRightPositionY1(P2,D1,2): + droneDY(D1,2)
 + 100 droneDelivered(D1,P2,2) <= 100
 packageWasDeliveredAtRightPositionY1(P2,D1,3): + droneDY(D1,3)
 + 100 droneDelivered(D1,P2,3) <= 100
 packageWasDeliveredAtRightPositionY1(P2,D1,4): + droneDY(D1,4)
 + 100 droneDelivered(D1,P2,4) <= 100
 packageWasDeliveredAtRightPositionY1(P2,D1,5): + droneDY(D1,5)
 + 100 droneDelivered(D1,P2,5) <= 100
 packageWasDeliveredAtRightPositionY1(P2,D2,1): + droneDY(D2,1)
 + 100 droneDelivered(D2,P2,1) <= 100
 packageWasDeliveredAtRightPositionY1(P2,D2,2): + droneDY(D2,2)
 + 100 droneDelivered(D2,P2,2) <= 100
 packageWasDeliveredAtRightPositionY1(P2,D2,3): + droneDY(D2,3)
 + 100 droneDelivered(D2,P2,3) <= 100
 packageWasDeliveredAtRightPositionY1(P2,D2,4): + droneDY(D2,4)
 + 100 droneDelivered(D2,P2,4) <= 100
 packageWasDeliveredAtRightPositionY1(P2,D2,5): + droneDY(D2,5)
 + 100 droneDelivered(D2,P2,5) <= 100
 packageWasDeliveredAtRightPositionY1(P2,D3,1): + droneDY(D3,1)
 + 100 droneDelivered(D3,P2,1) <= 100
 packageWasDeliveredAtRightPositionY1(P2,D3,2): + droneDY(D3,2)
 + 100 droneDelivered(D3,P2,2) <= 100
 packageWasDeliveredAtRightPositionY1(P2,D3,3): + droneDY(D3,3)
 + 100 droneDelivered(D3,P2,3) <= 100
 packageWasDeliveredAtRightPositionY1(P2,D3,4): + droneDY(D3,4)
 + 100 droneDelivered(D3,P2,4) <= 100
 packageWasDeliveredAtRightPositionY1(P2,D3,5): + droneDY(D3,5)
 + 100 droneDelivered(D3,P2,5) <= 100
 packageWasDeliveredAtRightPositionY1(P2,D4,1): + droneDY(D4,1)
 + 100 droneDelivered(D4,P2,1) <= 100
 packageWasDeliveredAtRightPositionY1(P2,D4,2): + droneDY(D4,2)
 + 100 droneDelivered(D4,P2,2) <= 100
 packageWasDeliveredAtRightPositionY1(P2,D4,3): + droneDY(D4,3)
 + 100 droneDelivered(D4,P2,3) <= 100
 packageWasDeliveredAtRightPositionY1(P2,D4,4): + droneDY(D4,4)
 + 100 droneDelivered(D4,P2,4) <= 100
 packageWasDeliveredAtRightPositionY1(P2,D4,5): + droneDY(D4,5)
 + 100 droneDelivered(D4,P2,5) <= 100
 packageWasDeliveredAtRightPositionY1(P2,D5,1): + droneDY(D5,1)
 + 100 droneDelivered(D5,P2,1) <= 100
 packageWasDeliveredAtRightPositionY1(P2,D5,2): + droneDY(D5,2)
 + 100 droneDelivered(D5,P2,2) <= 100
 packageWasDeliveredAtRightPositionY1(P2,D5,3): + droneDY(D5,3)
 + 100 droneDelivered(D5,P2,3) <= 100
 packageWasDeliveredAtRightPositionY1(P2,D5,4): + droneDY(D5,4)
 + 100 droneDelivered(D5,P2,4) <= 100
 packageWasDeliveredAtRightPositionY1(P2,D5,5): + droneDY(D5,5)
 + 100 droneDelivered(D5,P2,5) <= 100
 packageWasDeliveredAtRightPositionY1(P3,D1,1): + droneDY(D1,1)
 + 100 droneDelivered(D1,P3,1) <= 105
 packageWasDeliveredAtRightPositionY1(P3,D1,2): + droneDY(D1,2)
 + 100 droneDelivered(D1,P3,2) <= 105
 packageWasDeliveredAtRightPositionY1(P3,D1,3): + droneDY(D1,3)
 + 100 droneDelivered(D1,P3,3) <= 105
 packageWasDeliveredAtRightPositionY1(P3,D1,4): + droneDY(D1,4)
 + 100 droneDelivered(D1,P3,4) <= 105
 packageWasDeliveredAtRightPositionY1(P3,D1,5): + droneDY(D1,5)
 + 100 droneDelivered(D1,P3,5) <= 105
 packageWasDeliveredAtRightPositionY1(P3,D2,1): + droneDY(D2,1)
 + 100 droneDelivered(D2,P3,1) <= 105
 packageWasDeliveredAtRightPositionY1(P3,D2,2): + droneDY(D2,2)
 + 100 droneDelivered(D2,P3,2) <= 105
 packageWasDeliveredAtRightPositionY1(P3,D2,3): + droneDY(D2,3)
 + 100 droneDelivered(D2,P3,3) <= 105
 packageWasDeliveredAtRightPositionY1(P3,D2,4): + droneDY(D2,4)
 + 100 droneDelivered(D2,P3,4) <= 105
 packageWasDeliveredAtRightPositionY1(P3,D2,5): + droneDY(D2,5)
 + 100 droneDelivered(D2,P3,5) <= 105
 packageWasDeliveredAtRightPositionY1(P3,D3,1): + droneDY(D3,1)
 + 100 droneDelivered(D3,P3,1) <= 105
 packageWasDeliveredAtRightPositionY1(P3,D3,2): + droneDY(D3,2)
 + 100 droneDelivered(D3,P3,2) <= 105
 packageWasDeliveredAtRightPositionY1(P3,D3,3): + droneDY(D3,3)
 + 100 droneDelivered(D3,P3,3) <= 105
 packageWasDeliveredAtRightPositionY1(P3,D3,4): + droneDY(D3,4)
 + 100 droneDelivered(D3,P3,4) <= 105
 packageWasDeliveredAtRightPositionY1(P3,D3,5): + droneDY(D3,5)
 + 100 droneDelivered(D3,P3,5) <= 105
 packageWasDeliveredAtRightPositionY1(P3,D4,1): + droneDY(D4,1)
 + 100 droneDelivered(D4,P3,1) <= 105
 packageWasDeliveredAtRightPositionY1(P3,D4,2): + droneDY(D4,2)
 + 100 droneDelivered(D4,P3,2) <= 105
 packageWasDeliveredAtRightPositionY1(P3,D4,3): + droneDY(D4,3)
 + 100 droneDelivered(D4,P3,3) <= 105
 packageWasDeliveredAtRightPositionY1(P3,D4,4): + droneDY(D4,4)
 + 100 droneDelivered(D4,P3,4) <= 105
 packageWasDeliveredAtRightPositionY1(P3,D4,5): + droneDY(D4,5)
 + 100 droneDelivered(D4,P3,5) <= 105
 packageWasDeliveredAtRightPositionY1(P3,D5,1): + droneDY(D5,1)
 + 100 droneDelivered(D5,P3,1) <= 105
 packageWasDeliveredAtRightPositionY1(P3,D5,2): + droneDY(D5,2)
 + 100 droneDelivered(D5,P3,2) <= 105
 packageWasDeliveredAtRightPositionY1(P3,D5,3): + droneDY(D5,3)
 + 100 droneDelivered(D5,P3,3) <= 105
 packageWasDeliveredAtRightPositionY1(P3,D5,4): + droneDY(D5,4)
 + 100 droneDelivered(D5,P3,4) <= 105
 packageWasDeliveredAtRightPositionY1(P3,D5,5): + droneDY(D5,5)
 + 100 droneDelivered(D5,P3,5) <= 105
 packageWasDeliveredAtRightPositionY1(P4,D1,1): + droneDY(D1,1)
 + 100 droneDelivered(D1,P4,1) <= 110
 packageWasDeliveredAtRightPositionY1(P4,D1,2): + droneDY(D1,2)
 + 100 droneDelivered(D1,P4,2) <= 110
 packageWasDeliveredAtRightPositionY1(P4,D1,3): + droneDY(D1,3)
 + 100 droneDelivered(D1,P4,3) <= 110
 packageWasDeliveredAtRightPositionY1(P4,D1,4): + droneDY(D1,4)
 + 100 droneDelivered(D1,P4,4) <= 110
 packageWasDeliveredAtRightPositionY1(P4,D1,5): + droneDY(D1,5)
 + 100 droneDelivered(D1,P4,5) <= 110
 packageWasDeliveredAtRightPositionY1(P4,D2,1): + droneDY(D2,1)
 + 100 droneDelivered(D2,P4,1) <= 110
 packageWasDeliveredAtRightPositionY1(P4,D2,2): + droneDY(D2,2)
 + 100 droneDelivered(D2,P4,2) <= 110
 packageWasDeliveredAtRightPositionY1(P4,D2,3): + droneDY(D2,3)
 + 100 droneDelivered(D2,P4,3) <= 110
 packageWasDeliveredAtRightPositionY1(P4,D2,4): + droneDY(D2,4)
 + 100 droneDelivered(D2,P4,4) <= 110
 packageWasDeliveredAtRightPositionY1(P4,D2,5): + droneDY(D2,5)
 + 100 droneDelivered(D2,P4,5) <= 110
 packageWasDeliveredAtRightPositionY1(P4,D3,1): + droneDY(D3,1)
 + 100 droneDelivered(D3,P4,1) <= 110
 packageWasDeliveredAtRightPositionY1(P4,D3,2): + droneDY(D3,2)
 + 100 droneDelivered(D3,P4,2) <= 110
 packageWasDeliveredAtRightPositionY1(P4,D3,3): + droneDY(D3,3)
 + 100 droneDelivered(D3,P4,3) <= 110
 packageWasDeliveredAtRightPositionY1(P4,D3,4): + droneDY(D3,4)
 + 100 droneDelivered(D3,P4,4) <= 110
 packageWasDeliveredAtRightPositionY1(P4,D3,5): + droneDY(D3,5)
 + 100 droneDelivered(D3,P4,5) <= 110
 packageWasDeliveredAtRightPositionY1(P4,D4,1): + droneDY(D4,1)
 + 100 droneDelivered(D4,P4,1) <= 110
 packageWasDeliveredAtRightPositionY1(P4,D4,2): + droneDY(D4,2)
 + 100 droneDelivered(D4,P4,2) <= 110
 packageWasDeliveredAtRightPositionY1(P4,D4,3): + droneDY(D4,3)
 + 100 droneDelivered(D4,P4,3) <= 110
 packageWasDeliveredAtRightPositionY1(P4,D4,4): + droneDY(D4,4)
 + 100 droneDelivered(D4,P4,4) <= 110
 packageWasDeliveredAtRightPositionY1(P4,D4,5): + droneDY(D4,5)
 + 100 droneDelivered(D4,P4,5) <= 110
 packageWasDeliveredAtRightPositionY1(P4,D5,1): + droneDY(D5,1)
 + 100 droneDelivered(D5,P4,1) <= 110
 packageWasDeliveredAtRightPositionY1(P4,D5,2): + droneDY(D5,2)
 + 100 droneDelivered(D5,P4,2) <= 110
 packageWasDeliveredAtRightPositionY1(P4,D5,3): + droneDY(D5,3)
 + 100 droneDelivered(D5,P4,3) <= 110
 packageWasDeliveredAtRightPositionY1(P4,D5,4): + droneDY(D5,4)
 + 100 droneDelivered(D5,P4,4) <= 110
 packageWasDeliveredAtRightPositionY1(P4,D5,5): + droneDY(D5,5)
 + 100 droneDelivered(D5,P4,5) <= 110
 packageWasDeliveredAtRightPositionY1(P5,D1,1): + droneDY(D1,1)
 + 100 droneDelivered(D1,P5,1) <= 105
 packageWasDeliveredAtRightPositionY1(P5,D1,2): + droneDY(D1,2)
 + 100 droneDelivered(D1,P5,2) <= 105
 packageWasDeliveredAtRightPositionY1(P5,D1,3): + droneDY(D1,3)
 + 100 droneDelivered(D1,P5,3) <= 105
 packageWasDeliveredAtRightPositionY1(P5,D1,4): + droneDY(D1,4)
 + 100 droneDelivered(D1,P5,4) <= 105
 packageWasDeliveredAtRightPositionY1(P5,D1,5): + droneDY(D1,5)
 + 100 droneDelivered(D1,P5,5) <= 105
 packageWasDeliveredAtRightPositionY1(P5,D2,1): + droneDY(D2,1)
 + 100 droneDelivered(D2,P5,1) <= 105
 packageWasDeliveredAtRightPositionY1(P5,D2,2): + droneDY(D2,2)
 + 100 droneDelivered(D2,P5,2) <= 105
 packageWasDeliveredAtRightPositionY1(P5,D2,3): + droneDY(D2,3)
 + 100 droneDelivered(D2,P5,3) <= 105
 packageWasDeliveredAtRightPositionY1(P5,D2,4): + droneDY(D2,4)
 + 100 droneDelivered(D2,P5,4) <= 105
 packageWasDeliveredAtRightPositionY1(P5,D2,5): + droneDY(D2,5)
 + 100 droneDelivered(D2,P5,5) <= 105
 packageWasDeliveredAtRightPositionY1(P5,D3,1): + droneDY(D3,1)
 + 100 droneDelivered(D3,P5,1) <= 105
 packageWasDeliveredAtRightPositionY1(P5,D3,2): + droneDY(D3,2)
 + 100 droneDelivered(D3,P5,2) <= 105
 packageWasDeliveredAtRightPositionY1(P5,D3,3): + droneDY(D3,3)
 + 100 droneDelivered(D3,P5,3) <= 105
 packageWasDeliveredAtRightPositionY1(P5,D3,4): + droneDY(D3,4)
 + 100 droneDelivered(D3,P5,4) <= 105
 packageWasDeliveredAtRightPositionY1(P5,D3,5): + droneDY(D3,5)
 + 100 droneDelivered(D3,P5,5) <= 105
 packageWasDeliveredAtRightPositionY1(P5,D4,1): + droneDY(D4,1)
 + 100 droneDelivered(D4,P5,1) <= 105
 packageWasDeliveredAtRightPositionY1(P5,D4,2): + droneDY(D4,2)
 + 100 droneDelivered(D4,P5,2) <= 105
 packageWasDeliveredAtRightPositionY1(P5,D4,3): + droneDY(D4,3)
 + 100 droneDelivered(D4,P5,3) <= 105
 packageWasDeliveredAtRightPositionY1(P5,D4,4): + droneDY(D4,4)
 + 100 droneDelivered(D4,P5,4) <= 105
 packageWasDeliveredAtRightPositionY1(P5,D4,5): + droneDY(D4,5)
 + 100 droneDelivered(D4,P5,5) <= 105
 packageWasDeliveredAtRightPositionY1(P5,D5,1): + droneDY(D5,1)
 + 100 droneDelivered(D5,P5,1) <= 105
 packageWasDeliveredAtRightPositionY1(P5,D5,2): + droneDY(D5,2)
 + 100 droneDelivered(D5,P5,2) <= 105
 packageWasDeliveredAtRightPositionY1(P5,D5,3): + droneDY(D5,3)
 + 100 droneDelivered(D5,P5,3) <= 105
 packageWasDeliveredAtRightPositionY1(P5,D5,4): + droneDY(D5,4)
 + 100 droneDelivered(D5,P5,4) <= 105
 packageWasDeliveredAtRightPositionY1(P5,D5,5): + droneDY(D5,5)
 + 100 droneDelivered(D5,P5,5) <= 105
 packageWasDeliveredAtRightPositionY1(P6,D1,1): + droneDY(D1,1)
 + 100 droneDelivered(D1,P6,1) <= 100
 packageWasDeliveredAtRightPositionY1(P6,D1,2): + droneDY(D1,2)
 + 100 droneDelivered(D1,P6,2) <= 100
 packageWasDeliveredAtRightPositionY1(P6,D1,3): + droneDY(D1,3)
 + 100 droneDelivered(D1,P6,3) <= 100
 packageWasDeliveredAtRightPositionY1(P6,D1,4): + droneDY(D1,4)
 + 100 droneDelivered(D1,P6,4) <= 100
 packageWasDeliveredAtRightPositionY1(P6,D1,5): + droneDY(D1,5)
 + 100 droneDelivered(D1,P6,5) <= 100
 packageWasDeliveredAtRightPositionY1(P6,D2,1): + droneDY(D2,1)
 + 100 droneDelivered(D2,P6,1) <= 100
 packageWasDeliveredAtRightPositionY1(P6,D2,2): + droneDY(D2,2)
 + 100 droneDelivered(D2,P6,2) <= 100
 packageWasDeliveredAtRightPositionY1(P6,D2,3): + droneDY(D2,3)
 + 100 droneDelivered(D2,P6,3) <= 100
 packageWasDeliveredAtRightPositionY1(P6,D2,4): + droneDY(D2,4)
 + 100 droneDelivered(D2,P6,4) <= 100
 packageWasDeliveredAtRightPositionY1(P6,D2,5): + droneDY(D2,5)
 + 100 droneDelivered(D2,P6,5) <= 100
 packageWasDeliveredAtRightPositionY1(P6,D3,1): + droneDY(D3,1)
 + 100 droneDelivered(D3,P6,1) <= 100
 packageWasDeliveredAtRightPositionY1(P6,D3,2): + droneDY(D3,2)
 + 100 droneDelivered(D3,P6,2) <= 100
 packageWasDeliveredAtRightPositionY1(P6,D3,3): + droneDY(D3,3)
 + 100 droneDelivered(D3,P6,3) <= 100
 packageWasDeliveredAtRightPositionY1(P6,D3,4): + droneDY(D3,4)
 + 100 droneDelivered(D3,P6,4) <= 100
 packageWasDeliveredAtRightPositionY1(P6,D3,5): + droneDY(D3,5)
 + 100 droneDelivered(D3,P6,5) <= 100
 packageWasDeliveredAtRightPositionY1(P6,D4,1): + droneDY(D4,1)
 + 100 droneDelivered(D4,P6,1) <= 100
 packageWasDeliveredAtRightPositionY1(P6,D4,2): + droneDY(D4,2)
 + 100 droneDelivered(D4,P6,2) <= 100
 packageWasDeliveredAtRightPositionY1(P6,D4,3): + droneDY(D4,3)
 + 100 droneDelivered(D4,P6,3) <= 100
 packageWasDeliveredAtRightPositionY1(P6,D4,4): + droneDY(D4,4)
 + 100 droneDelivered(D4,P6,4) <= 100
 packageWasDeliveredAtRightPositionY1(P6,D4,5): + droneDY(D4,5)
 + 100 droneDelivered(D4,P6,5) <= 100
 packageWasDeliveredAtRightPositionY1(P6,D5,1): + droneDY(D5,1)
 + 100 droneDelivered(D5,P6,1) <= 100
 packageWasDeliveredAtRightPositionY1(P6,D5,2): + droneDY(D5,2)
 + 100 droneDelivered(D5,P6,2) <= 100
 packageWasDeliveredAtRightPositionY1(P6,D5,3): + droneDY(D5,3)
 + 100 droneDelivered(D5,P6,3) <= 100
 packageWasDeliveredAtRightPositionY1(P6,D5,4): + droneDY(D5,4)
 + 100 droneDelivered(D5,P6,4) <= 100
 packageWasDeliveredAtRightPositionY1(P6,D5,5): + droneDY(D5,5)
 + 100 droneDelivered(D5,P6,5) <= 100
 packageWasDeliveredAtRightPositionY2(P1,D1,1): - droneDY(D1,1)
 + 100 droneDelivered(D1,P1,1) <= 100
 packageWasDeliveredAtRightPositionY2(P1,D1,2): - droneDY(D1,2)
 + 100 droneDelivered(D1,P1,2) <= 100
 packageWasDeliveredAtRightPositionY2(P1,D1,3): - droneDY(D1,3)
 + 100 droneDelivered(D1,P1,3) <= 100
 packageWasDeliveredAtRightPositionY2(P1,D1,4): - droneDY(D1,4)
 + 100 droneDelivered(D1,P1,4) <= 100
 packageWasDeliveredAtRightPositionY2(P1,D1,5): - droneDY(D1,5)
 + 100 droneDelivered(D1,P1,5) <= 100
 packageWasDeliveredAtRightPositionY2(P1,D2,1): - droneDY(D2,1)
 + 100 droneDelivered(D2,P1,1) <= 100
 packageWasDeliveredAtRightPositionY2(P1,D2,2): - droneDY(D2,2)
 + 100 droneDelivered(D2,P1,2) <= 100
 packageWasDeliveredAtRightPositionY2(P1,D2,3): - droneDY(D2,3)
 + 100 droneDelivered(D2,P1,3) <= 100
 packageWasDeliveredAtRightPositionY2(P1,D2,4): - droneDY(D2,4)
 + 100 droneDelivered(D2,P1,4) <= 100
 packageWasDeliveredAtRightPositionY2(P1,D2,5): - droneDY(D2,5)
 + 100 droneDelivered(D2,P1,5) <= 100
 packageWasDeliveredAtRightPositionY2(P1,D3,1): - droneDY(D3,1)
 + 100 droneDelivered(D3,P1,1) <= 100
 packageWasDeliveredAtRightPositionY2(P1,D3,2): - droneDY(D3,2)
 + 100 droneDelivered(D3,P1,2) <= 100
 packageWasDeliveredAtRightPositionY2(P1,D3,3): - droneDY(D3,3)
 + 100 droneDelivered(D3,P1,3) <= 100
 packageWasDeliveredAtRightPositionY2(P1,D3,4): - droneDY(D3,4)
 + 100 droneDelivered(D3,P1,4) <= 100
 packageWasDeliveredAtRightPositionY2(P1,D3,5): - droneDY(D3,5)
 + 100 droneDelivered(D3,P1,5) <= 100
 packageWasDeliveredAtRightPositionY2(P1,D4,1): - droneDY(D4,1)
 + 100 droneDelivered(D4,P1,1) <= 100
 packageWasDeliveredAtRightPositionY2(P1,D4,2): - droneDY(D4,2)
 + 100 droneDelivered(D4,P1,2) <= 100
 packageWasDeliveredAtRightPositionY2(P1,D4,3): - droneDY(D4,3)
 + 100 droneDelivered(D4,P1,3) <= 100
 packageWasDeliveredAtRightPositionY2(P1,D4,4): - droneDY(D4,4)
 + 100 droneDelivered(D4,P1,4) <= 100
 packageWasDeliveredAtRightPositionY2(P1,D4,5): - droneDY(D4,5)
 + 100 droneDelivered(D4,P1,5) <= 100
 packageWasDeliveredAtRightPositionY2(P1,D5,1): - droneDY(D5,1)
 + 100 droneDelivered(D5,P1,1) <= 100
 packageWasDeliveredAtRightPositionY2(P1,D5,2): - droneDY(D5,2)
 + 100 droneDelivered(D5,P1,2) <= 100
 packageWasDeliveredAtRightPositionY2(P1,D5,3): - droneDY(D5,3)
 + 100 droneDelivered(D5,P1,3) <= 100
 packageWasDeliveredAtRightPositionY2(P1,D5,4): - droneDY(D5,4)
 + 100 droneDelivered(D5,P1,4) <= 100
 packageWasDeliveredAtRightPositionY2(P1,D5,5): - droneDY(D5,5)
 + 100 droneDelivered(D5,P1,5) <= 100
 packageWasDeliveredAtRightPositionY2(P2,D1,1): - droneDY(D1,1)
 + 100 droneDelivered(D1,P2,1) <= 100
 packageWasDeliveredAtRightPositionY2(P2,D1,2): - droneDY(D1,2)
 + 100 droneDelivered(D1,P2,2) <= 100
 packageWasDeliveredAtRightPositionY2(P2,D1,3): - droneDY(D1,3)
 + 100 droneDelivered(D1,P2,3) <= 100
 packageWasDeliveredAtRightPositionY2(P2,D1,4): - droneDY(D1,4)
 + 100 droneDelivered(D1,P2,4) <= 100
 packageWasDeliveredAtRightPositionY2(P2,D1,5): - droneDY(D1,5)
 + 100 droneDelivered(D1,P2,5) <= 100
 packageWasDeliveredAtRightPositionY2(P2,D2,1): - droneDY(D2,1)
 + 100 droneDelivered(D2,P2,1) <= 100
 packageWasDeliveredAtRightPositionY2(P2,D2,2): - droneDY(D2,2)
 + 100 droneDelivered(D2,P2,2) <= 100
 packageWasDeliveredAtRightPositionY2(P2,D2,3): - droneDY(D2,3)
 + 100 droneDelivered(D2,P2,3) <= 100
 packageWasDeliveredAtRightPositionY2(P2,D2,4): - droneDY(D2,4)
 + 100 droneDelivered(D2,P2,4) <= 100
 packageWasDeliveredAtRightPositionY2(P2,D2,5): - droneDY(D2,5)
 + 100 droneDelivered(D2,P2,5) <= 100
 packageWasDeliveredAtRightPositionY2(P2,D3,1): - droneDY(D3,1)
 + 100 droneDelivered(D3,P2,1) <= 100
 packageWasDeliveredAtRightPositionY2(P2,D3,2): - droneDY(D3,2)
 + 100 droneDelivered(D3,P2,2) <= 100
 packageWasDeliveredAtRightPositionY2(P2,D3,3): - droneDY(D3,3)
 + 100 droneDelivered(D3,P2,3) <= 100
 packageWasDeliveredAtRightPositionY2(P2,D3,4): - droneDY(D3,4)
 + 100 droneDelivered(D3,P2,4) <= 100
 packageWasDeliveredAtRightPositionY2(P2,D3,5): - droneDY(D3,5)
 + 100 droneDelivered(D3,P2,5) <= 100
 packageWasDeliveredAtRightPositionY2(P2,D4,1): - droneDY(D4,1)
 + 100 droneDelivered(D4,P2,1) <= 100
 packageWasDeliveredAtRightPositionY2(P2,D4,2): - droneDY(D4,2)
 + 100 droneDelivered(D4,P2,2) <= 100
 packageWasDeliveredAtRightPositionY2(P2,D4,3): - droneDY(D4,3)
 + 100 droneDelivered(D4,P2,3) <= 100
 packageWasDeliveredAtRightPositionY2(P2,D4,4): - droneDY(D4,4)
 + 100 droneDelivered(D4,P2,4) <= 100
 packageWasDeliveredAtRightPositionY2(P2,D4,5): - droneDY(D4,5)
 + 100 droneDelivered(D4,P2,5) <= 100
 packageWasDeliveredAtRightPositionY2(P2,D5,1): - droneDY(D5,1)
 + 100 droneDelivered(D5,P2,1) <= 100
 packageWasDeliveredAtRightPositionY2(P2,D5,2): - droneDY(D5,2)
 + 100 droneDelivered(D5,P2,2) <= 100
 packageWasDeliveredAtRightPositionY2(P2,D5,3): - droneDY(D5,3)
 + 100 droneDelivered(D5,P2,3) <= 100
 packageWasDeliveredAtRightPositionY2(P2,D5,4): - droneDY(D5,4)
 + 100 droneDelivered(D5,P2,4) <= 100
 packageWasDeliveredAtRightPositionY2(P2,D5,5): - droneDY(D5,5)
 + 100 droneDelivered(D5,P2,5) <= 100
 packageWasDeliveredAtRightPositionY2(P3,D1,1): - droneDY(D1,1)
 + 100 droneDelivered(D1,P3,1) <= 95
 packageWasDeliveredAtRightPositionY2(P3,D1,2): - droneDY(D1,2)
 + 100 droneDelivered(D1,P3,2) <= 95
 packageWasDeliveredAtRightPositionY2(P3,D1,3): - droneDY(D1,3)
 + 100 droneDelivered(D1,P3,3) <= 95
 packageWasDeliveredAtRightPositionY2(P3,D1,4): - droneDY(D1,4)
 + 100 droneDelivered(D1,P3,4) <= 95
 packageWasDeliveredAtRightPositionY2(P3,D1,5): - droneDY(D1,5)
 + 100 droneDelivered(D1,P3,5) <= 95
 packageWasDeliveredAtRightPositionY2(P3,D2,1): - droneDY(D2,1)
 + 100 droneDelivered(D2,P3,1) <= 95
 packageWasDeliveredAtRightPositionY2(P3,D2,2): - droneDY(D2,2)
 + 100 droneDelivered(D2,P3,2) <= 95
 packageWasDeliveredAtRightPositionY2(P3,D2,3): - droneDY(D2,3)
 + 100 droneDelivered(D2,P3,3) <= 95
 packageWasDeliveredAtRightPositionY2(P3,D2,4): - droneDY(D2,4)
 + 100 droneDelivered(D2,P3,4) <= 95
 packageWasDeliveredAtRightPositionY2(P3,D2,5): - droneDY(D2,5)
 + 100 droneDelivered(D2,P3,5) <= 95
 packageWasDeliveredAtRightPositionY2(P3,D3,1): - droneDY(D3,1)
 + 100 droneDelivered(D3,P3,1) <= 95
 packageWasDeliveredAtRightPositionY2(P3,D3,2): - droneDY(D3,2)
 + 100 droneDelivered(D3,P3,2) <= 95
 packageWasDeliveredAtRightPositionY2(P3,D3,3): - droneDY(D3,3)
 + 100 droneDelivered(D3,P3,3) <= 95
 packageWasDeliveredAtRightPositionY2(P3,D3,4): - droneDY(D3,4)
 + 100 droneDelivered(D3,P3,4) <= 95
 packageWasDeliveredAtRightPositionY2(P3,D3,5): - droneDY(D3,5)
 + 100 droneDelivered(D3,P3,5) <= 95
 packageWasDeliveredAtRightPositionY2(P3,D4,1): - droneDY(D4,1)
 + 100 droneDelivered(D4,P3,1) <= 95
 packageWasDeliveredAtRightPositionY2(P3,D4,2): - droneDY(D4,2)
 + 100 droneDelivered(D4,P3,2) <= 95
 packageWasDeliveredAtRightPositionY2(P3,D4,3): - droneDY(D4,3)
 + 100 droneDelivered(D4,P3,3) <= 95
 packageWasDeliveredAtRightPositionY2(P3,D4,4): - droneDY(D4,4)
 + 100 droneDelivered(D4,P3,4) <= 95
 packageWasDeliveredAtRightPositionY2(P3,D4,5): - droneDY(D4,5)
 + 100 droneDelivered(D4,P3,5) <= 95
 packageWasDeliveredAtRightPositionY2(P3,D5,1): - droneDY(D5,1)
 + 100 droneDelivered(D5,P3,1) <= 95
 packageWasDeliveredAtRightPositionY2(P3,D5,2): - droneDY(D5,2)
 + 100 droneDelivered(D5,P3,2) <= 95
 packageWasDeliveredAtRightPositionY2(P3,D5,3): - droneDY(D5,3)
 + 100 droneDelivered(D5,P3,3) <= 95
 packageWasDeliveredAtRightPositionY2(P3,D5,4): - droneDY(D5,4)
 + 100 droneDelivered(D5,P3,4) <= 95
 packageWasDeliveredAtRightPositionY2(P3,D5,5): - droneDY(D5,5)
 + 100 droneDelivered(D5,P3,5) <= 95
 packageWasDeliveredAtRightPositionY2(P4,D1,1): - droneDY(D1,1)
 + 100 droneDelivered(D1,P4,1) <= 90
 packageWasDeliveredAtRightPositionY2(P4,D1,2): - droneDY(D1,2)
 + 100 droneDelivered(D1,P4,2) <= 90
 packageWasDeliveredAtRightPositionY2(P4,D1,3): - droneDY(D1,3)
 + 100 droneDelivered(D1,P4,3) <= 90
 packageWasDeliveredAtRightPositionY2(P4,D1,4): - droneDY(D1,4)
 + 100 droneDelivered(D1,P4,4) <= 90
 packageWasDeliveredAtRightPositionY2(P4,D1,5): - droneDY(D1,5)
 + 100 droneDelivered(D1,P4,5) <= 90
 packageWasDeliveredAtRightPositionY2(P4,D2,1): - droneDY(D2,1)
 + 100 droneDelivered(D2,P4,1) <= 90
 packageWasDeliveredAtRightPositionY2(P4,D2,2): - droneDY(D2,2)
 + 100 droneDelivered(D2,P4,2) <= 90
 packageWasDeliveredAtRightPositionY2(P4,D2,3): - droneDY(D2,3)
 + 100 droneDelivered(D2,P4,3) <= 90
 packageWasDeliveredAtRightPositionY2(P4,D2,4): - droneDY(D2,4)
 + 100 droneDelivered(D2,P4,4) <= 90
 packageWasDeliveredAtRightPositionY2(P4,D2,5): - droneDY(D2,5)
 + 100 droneDelivered(D2,P4,5) <= 90
 packageWasDeliveredAtRightPositionY2(P4,D3,1): - droneDY(D3,1)
 + 100 droneDelivered(D3,P4,1) <= 90
 packageWasDeliveredAtRightPositionY2(P4,D3,2): - droneDY(D3,2)
 + 100 droneDelivered(D3,P4,2) <= 90
 packageWasDeliveredAtRightPositionY2(P4,D3,3): - droneDY(D3,3)
 + 100 droneDelivered(D3,P4,3) <= 90
 packageWasDeliveredAtRightPositionY2(P4,D3,4): - droneDY(D3,4)
 + 100 droneDelivered(D3,P4,4) <= 90
 packageWasDeliveredAtRightPositionY2(P4,D3,5): - droneDY(D3,5)
 + 100 droneDelivered(D3,P4,5) <= 90
 packageWasDeliveredAtRightPositionY2(P4,D4,1): - droneDY(D4,1)
 + 100 droneDelivered(D4,P4,1) <= 90
 packageWasDeliveredAtRightPositionY2(P4,D4,2): - droneDY(D4,2)
 + 100 droneDelivered(D4,P4,2) <= 90
 packageWasDeliveredAtRightPositionY2(P4,D4,3): - droneDY(D4,3)
 + 100 droneDelivered(D4,P4,3) <= 90
 packageWasDeliveredAtRightPositionY2(P4,D4,4): - droneDY(D4,4)
 + 100 droneDelivered(D4,P4,4) <= 90
 packageWasDeliveredAtRightPositionY2(P4,D4,5): - droneDY(D4,5)
 + 100 droneDelivered(D4,P4,5) <= 90
 packageWasDeliveredAtRightPositionY2(P4,D5,1): - droneDY(D5,1)
 + 100 droneDelivered(D5,P4,1) <= 90
 packageWasDeliveredAtRightPositionY2(P4,D5,2): - droneDY(D5,2)
 + 100 droneDelivered(D5,P4,2) <= 90
 packageWasDeliveredAtRightPositionY2(P4,D5,3): - droneDY(D5,3)
 + 100 droneDelivered(D5,P4,3) <= 90
 packageWasDeliveredAtRightPositionY2(P4,D5,4): - droneDY(D5,4)
 + 100 droneDelivered(D5,P4,4) <= 90
 packageWasDeliveredAtRightPositionY2(P4,D5,5): - droneDY(D5,5)
 + 100 droneDelivered(D5,P4,5) <= 90
 packageWasDeliveredAtRightPositionY2(P5,D1,1): - droneDY(D1,1)
 + 100 droneDelivered(D1,P5,1) <= 95
 packageWasDeliveredAtRightPositionY2(P5,D1,2): - droneDY(D1,2)
 + 100 droneDelivered(D1,P5,2) <= 95
 packageWasDeliveredAtRightPositionY2(P5,D1,3): - droneDY(D1,3)
 + 100 droneDelivered(D1,P5,3) <= 95
 packageWasDeliveredAtRightPositionY2(P5,D1,4): - droneDY(D1,4)
 + 100 droneDelivered(D1,P5,4) <= 95
 packageWasDeliveredAtRightPositionY2(P5,D1,5): - droneDY(D1,5)
 + 100 droneDelivered(D1,P5,5) <= 95
 packageWasDeliveredAtRightPositionY2(P5,D2,1): - droneDY(D2,1)
 + 100 droneDelivered(D2,P5,1) <= 95
 packageWasDeliveredAtRightPositionY2(P5,D2,2): - droneDY(D2,2)
 + 100 droneDelivered(D2,P5,2) <= 95
 packageWasDeliveredAtRightPositionY2(P5,D2,3): - droneDY(D2,3)
 + 100 droneDelivered(D2,P5,3) <= 95
 packageWasDeliveredAtRightPositionY2(P5,D2,4): - droneDY(D2,4)
 + 100 droneDelivered(D2,P5,4) <= 95
 packageWasDeliveredAtRightPositionY2(P5,D2,5): - droneDY(D2,5)
 + 100 droneDelivered(D2,P5,5) <= 95
 packageWasDeliveredAtRightPositionY2(P5,D3,1): - droneDY(D3,1)
 + 100 droneDelivered(D3,P5,1) <= 95
 packageWasDeliveredAtRightPositionY2(P5,D3,2): - droneDY(D3,2)
 + 100 droneDelivered(D3,P5,2) <= 95
 packageWasDeliveredAtRightPositionY2(P5,D3,3): - droneDY(D3,3)
 + 100 droneDelivered(D3,P5,3) <= 95
 packageWasDeliveredAtRightPositionY2(P5,D3,4): - droneDY(D3,4)
 + 100 droneDelivered(D3,P5,4) <= 95
 packageWasDeliveredAtRightPositionY2(P5,D3,5): - droneDY(D3,5)
 + 100 droneDelivered(D3,P5,5) <= 95
 packageWasDeliveredAtRightPositionY2(P5,D4,1): - droneDY(D4,1)
 + 100 droneDelivered(D4,P5,1) <= 95
 packageWasDeliveredAtRightPositionY2(P5,D4,2): - droneDY(D4,2)
 + 100 droneDelivered(D4,P5,2) <= 95
 packageWasDeliveredAtRightPositionY2(P5,D4,3): - droneDY(D4,3)
 + 100 droneDelivered(D4,P5,3) <= 95
 packageWasDeliveredAtRightPositionY2(P5,D4,4): - droneDY(D4,4)
 + 100 droneDelivered(D4,P5,4) <= 95
 packageWasDeliveredAtRightPositionY2(P5,D4,5): - droneDY(D4,5)
 + 100 droneDelivered(D4,P5,5) <= 95
 packageWasDeliveredAtRightPositionY2(P5,D5,1): - droneDY(D5,1)
 + 100 droneDelivered(D5,P5,1) <= 95
 packageWasDeliveredAtRightPositionY2(P5,D5,2): - droneDY(D5,2)
 + 100 droneDelivered(D5,P5,2) <= 95
 packageWasDeliveredAtRightPositionY2(P5,D5,3): - droneDY(D5,3)
 + 100 droneDelivered(D5,P5,3) <= 95
 packageWasDeliveredAtRightPositionY2(P5,D5,4): - droneDY(D5,4)
 + 100 droneDelivered(D5,P5,4) <= 95
 packageWasDeliveredAtRightPositionY2(P5,D5,5): - droneDY(D5,5)
 + 100 droneDelivered(D5,P5,5) <= 95
 packageWasDeliveredAtRightPositionY2(P6,D1,1): - droneDY(D1,1)
 + 100 droneDelivered(D1,P6,1) <= 100
 packageWasDeliveredAtRightPositionY2(P6,D1,2): - droneDY(D1,2)
 + 100 droneDelivered(D1,P6,2) <= 100
 packageWasDeliveredAtRightPositionY2(P6,D1,3): - droneDY(D1,3)
 + 100 droneDelivered(D1,P6,3) <= 100
 packageWasDeliveredAtRightPositionY2(P6,D1,4): - droneDY(D1,4)
 + 100 droneDelivered(D1,P6,4) <= 100
 packageWasDeliveredAtRightPositionY2(P6,D1,5): - droneDY(D1,5)
 + 100 droneDelivered(D1,P6,5) <= 100
 packageWasDeliveredAtRightPositionY2(P6,D2,1): - droneDY(D2,1)
 + 100 droneDelivered(D2,P6,1) <= 100
 packageWasDeliveredAtRightPositionY2(P6,D2,2): - droneDY(D2,2)
 + 100 droneDelivered(D2,P6,2) <= 100
 packageWasDeliveredAtRightPositionY2(P6,D2,3): - droneDY(D2,3)
 + 100 droneDelivered(D2,P6,3) <= 100
 packageWasDeliveredAtRightPositionY2(P6,D2,4): - droneDY(D2,4)
 + 100 droneDelivered(D2,P6,4) <= 100
 packageWasDeliveredAtRightPositionY2(P6,D2,5): - droneDY(D2,5)
 + 100 droneDelivered(D2,P6,5) <= 100
 packageWasDeliveredAtRightPositionY2(P6,D3,1): - droneDY(D3,1)
 + 100 droneDelivered(D3,P6,1) <= 100
 packageWasDeliveredAtRightPositionY2(P6,D3,2): - droneDY(D3,2)
 + 100 droneDelivered(D3,P6,2) <= 100
 packageWasDeliveredAtRightPositionY2(P6,D3,3): - droneDY(D3,3)
 + 100 droneDelivered(D3,P6,3) <= 100
 packageWasDeliveredAtRightPositionY2(P6,D3,4): - droneDY(D3,4)
 + 100 droneDelivered(D3,P6,4) <= 100
 packageWasDeliveredAtRightPositionY2(P6,D3,5): - droneDY(D3,5)
 + 100 droneDelivered(D3,P6,5) <= 100
 packageWasDeliveredAtRightPositionY2(P6,D4,1): - droneDY(D4,1)
 + 100 droneDelivered(D4,P6,1) <= 100
 packageWasDeliveredAtRightPositionY2(P6,D4,2): - droneDY(D4,2)
 + 100 droneDelivered(D4,P6,2) <= 100
 packageWasDeliveredAtRightPositionY2(P6,D4,3): - droneDY(D4,3)
 + 100 droneDelivered(D4,P6,3) <= 100
 packageWasDeliveredAtRightPositionY2(P6,D4,4): - droneDY(D4,4)
 + 100 droneDelivered(D4,P6,4) <= 100
 packageWasDeliveredAtRightPositionY2(P6,D4,5): - droneDY(D4,5)
 + 100 droneDelivered(D4,P6,5) <= 100
 packageWasDeliveredAtRightPositionY2(P6,D5,1): - droneDY(D5,1)
 + 100 droneDelivered(D5,P6,1) <= 100
 packageWasDeliveredAtRightPositionY2(P6,D5,2): - droneDY(D5,2)
 + 100 droneDelivered(D5,P6,2) <= 100
 packageWasDeliveredAtRightPositionY2(P6,D5,3): - droneDY(D5,3)
 + 100 droneDelivered(D5,P6,3) <= 100
 packageWasDeliveredAtRightPositionY2(P6,D5,4): - droneDY(D5,4)
 + 100 droneDelivered(D5,P6,4) <= 100
 packageWasDeliveredAtRightPositionY2(P6,D5,5): - droneDY(D5,5)
 + 100 droneDelivered(D5,P6,5) <= 100
 droneChargedAtRightPositionX1(1,D1,1): + droneDX(D1,1)
 + 100 droneIsChargingAtEP(D1,1,1) <= 105
 droneChargedAtRightPositionX1(1,D1,2): + droneDX(D1,2)
 + 100 droneIsChargingAtEP(D1,2,1) <= 105
 droneChargedAtRightPositionX1(1,D1,3): + droneDX(D1,3)
 + 100 droneIsChargingAtEP(D1,3,1) <= 105
 droneChargedAtRightPositionX1(1,D1,4): + droneDX(D1,4)
 + 100 droneIsChargingAtEP(D1,4,1) <= 105
 droneChargedAtRightPositionX1(1,D1,5): + droneDX(D1,5)
 + 100 droneIsChargingAtEP(D1,5,1) <= 105
 droneChargedAtRightPositionX1(1,D2,1): + droneDX(D2,1)
 + 100 droneIsChargingAtEP(D2,1,1) <= 105
 droneChargedAtRightPositionX1(1,D2,2): + droneDX(D2,2)
 + 100 droneIsChargingAtEP(D2,2,1) <= 105
 droneChargedAtRightPositionX1(1,D2,3): + droneDX(D2,3)
 + 100 droneIsChargingAtEP(D2,3,1) <= 105
 droneChargedAtRightPositionX1(1,D2,4): + droneDX(D2,4)
 + 100 droneIsChargingAtEP(D2,4,1) <= 105
 droneChargedAtRightPositionX1(1,D2,5): + droneDX(D2,5)
 + 100 droneIsChargingAtEP(D2,5,1) <= 105
 droneChargedAtRightPositionX1(1,D3,1): + droneDX(D3,1)
 + 100 droneIsChargingAtEP(D3,1,1) <= 105
 droneChargedAtRightPositionX1(1,D3,2): + droneDX(D3,2)
 + 100 droneIsChargingAtEP(D3,2,1) <= 105
 droneChargedAtRightPositionX1(1,D3,3): + droneDX(D3,3)
 + 100 droneIsChargingAtEP(D3,3,1) <= 105
 droneChargedAtRightPositionX1(1,D3,4): + droneDX(D3,4)
 + 100 droneIsChargingAtEP(D3,4,1) <= 105
 droneChargedAtRightPositionX1(1,D3,5): + droneDX(D3,5)
 + 100 droneIsChargingAtEP(D3,5,1) <= 105
 droneChargedAtRightPositionX1(1,D4,1): + droneDX(D4,1)
 + 100 droneIsChargingAtEP(D4,1,1) <= 105
 droneChargedAtRightPositionX1(1,D4,2): + droneDX(D4,2)
 + 100 droneIsChargingAtEP(D4,2,1) <= 105
 droneChargedAtRightPositionX1(1,D4,3): + droneDX(D4,3)
 + 100 droneIsChargingAtEP(D4,3,1) <= 105
 droneChargedAtRightPositionX1(1,D4,4): + droneDX(D4,4)
 + 100 droneIsChargingAtEP(D4,4,1) <= 105
 droneChargedAtRightPositionX1(1,D4,5): + droneDX(D4,5)
 + 100 droneIsChargingAtEP(D4,5,1) <= 105
 droneChargedAtRightPositionX1(1,D5,1): + droneDX(D5,1)
 + 100 droneIsChargingAtEP(D5,1,1) <= 105
 droneChargedAtRightPositionX1(1,D5,2): + droneDX(D5,2)
 + 100 droneIsChargingAtEP(D5,2,1) <= 105
 droneChargedAtRightPositionX1(1,D5,3): + droneDX(D5,3)
 + 100 droneIsChargingAtEP(D5,3,1) <= 105
 droneChargedAtRightPositionX1(1,D5,4): + droneDX(D5,4)
 + 100 droneIsChargingAtEP(D5,4,1) <= 105
 droneChargedAtRightPositionX1(1,D5,5): + droneDX(D5,5)
 + 100 droneIsChargingAtEP(D5,5,1) <= 105
 droneChargedAtRightPositionX1(2,D1,1): + droneDX(D1,1)
 + 100 droneIsChargingAtEP(D1,1,2) <= 100
 droneChargedAtRightPositionX1(2,D1,2): + droneDX(D1,2)
 + 100 droneIsChargingAtEP(D1,2,2) <= 100
 droneChargedAtRightPositionX1(2,D1,3): + droneDX(D1,3)
 + 100 droneIsChargingAtEP(D1,3,2) <= 100
 droneChargedAtRightPositionX1(2,D1,4): + droneDX(D1,4)
 + 100 droneIsChargingAtEP(D1,4,2) <= 100
 droneChargedAtRightPositionX1(2,D1,5): + droneDX(D1,5)
 + 100 droneIsChargingAtEP(D1,5,2) <= 100
 droneChargedAtRightPositionX1(2,D2,1): + droneDX(D2,1)
 + 100 droneIsChargingAtEP(D2,1,2) <= 100
 droneChargedAtRightPositionX1(2,D2,2): + droneDX(D2,2)
 + 100 droneIsChargingAtEP(D2,2,2) <= 100
 droneChargedAtRightPositionX1(2,D2,3): + droneDX(D2,3)
 + 100 droneIsChargingAtEP(D2,3,2) <= 100
 droneChargedAtRightPositionX1(2,D2,4): + droneDX(D2,4)
 + 100 droneIsChargingAtEP(D2,4,2) <= 100
 droneChargedAtRightPositionX1(2,D2,5): + droneDX(D2,5)
 + 100 droneIsChargingAtEP(D2,5,2) <= 100
 droneChargedAtRightPositionX1(2,D3,1): + droneDX(D3,1)
 + 100 droneIsChargingAtEP(D3,1,2) <= 100
 droneChargedAtRightPositionX1(2,D3,2): + droneDX(D3,2)
 + 100 droneIsChargingAtEP(D3,2,2) <= 100
 droneChargedAtRightPositionX1(2,D3,3): + droneDX(D3,3)
 + 100 droneIsChargingAtEP(D3,3,2) <= 100
 droneChargedAtRightPositionX1(2,D3,4): + droneDX(D3,4)
 + 100 droneIsChargingAtEP(D3,4,2) <= 100
 droneChargedAtRightPositionX1(2,D3,5): + droneDX(D3,5)
 + 100 droneIsChargingAtEP(D3,5,2) <= 100
 droneChargedAtRightPositionX1(2,D4,1): + droneDX(D4,1)
 + 100 droneIsChargingAtEP(D4,1,2) <= 100
 droneChargedAtRightPositionX1(2,D4,2): + droneDX(D4,2)
 + 100 droneIsChargingAtEP(D4,2,2) <= 100
 droneChargedAtRightPositionX1(2,D4,3): + droneDX(D4,3)
 + 100 droneIsChargingAtEP(D4,3,2) <= 100
 droneChargedAtRightPositionX1(2,D4,4): + droneDX(D4,4)
 + 100 droneIsChargingAtEP(D4,4,2) <= 100
 droneChargedAtRightPositionX1(2,D4,5): + droneDX(D4,5)
 + 100 droneIsChargingAtEP(D4,5,2) <= 100
 droneChargedAtRightPositionX1(2,D5,1): + droneDX(D5,1)
 + 100 droneIsChargingAtEP(D5,1,2) <= 100
 droneChargedAtRightPositionX1(2,D5,2): + droneDX(D5,2)
 + 100 droneIsChargingAtEP(D5,2,2) <= 100
 droneChargedAtRightPositionX1(2,D5,3): + droneDX(D5,3)
 + 100 droneIsChargingAtEP(D5,3,2) <= 100
 droneChargedAtRightPositionX1(2,D5,4): + droneDX(D5,4)
 + 100 droneIsChargingAtEP(D5,4,2) <= 100
 droneChargedAtRightPositionX1(2,D5,5): + droneDX(D5,5)
 + 100 droneIsChargingAtEP(D5,5,2) <= 100
 droneChargedAtRightPositionX1(3,D1,1): + droneDX(D1,1)
 + 100 droneIsChargingAtEP(D1,1,3) <= 120
 droneChargedAtRightPositionX1(3,D1,2): + droneDX(D1,2)
 + 100 droneIsChargingAtEP(D1,2,3) <= 120
 droneChargedAtRightPositionX1(3,D1,3): + droneDX(D1,3)
 + 100 droneIsChargingAtEP(D1,3,3) <= 120
 droneChargedAtRightPositionX1(3,D1,4): + droneDX(D1,4)
 + 100 droneIsChargingAtEP(D1,4,3) <= 120
 droneChargedAtRightPositionX1(3,D1,5): + droneDX(D1,5)
 + 100 droneIsChargingAtEP(D1,5,3) <= 120
 droneChargedAtRightPositionX1(3,D2,1): + droneDX(D2,1)
 + 100 droneIsChargingAtEP(D2,1,3) <= 120
 droneChargedAtRightPositionX1(3,D2,2): + droneDX(D2,2)
 + 100 droneIsChargingAtEP(D2,2,3) <= 120
 droneChargedAtRightPositionX1(3,D2,3): + droneDX(D2,3)
 + 100 droneIsChargingAtEP(D2,3,3) <= 120
 droneChargedAtRightPositionX1(3,D2,4): + droneDX(D2,4)
 + 100 droneIsChargingAtEP(D2,4,3) <= 120
 droneChargedAtRightPositionX1(3,D2,5): + droneDX(D2,5)
 + 100 droneIsChargingAtEP(D2,5,3) <= 120
 droneChargedAtRightPositionX1(3,D3,1): + droneDX(D3,1)
 + 100 droneIsChargingAtEP(D3,1,3) <= 120
 droneChargedAtRightPositionX1(3,D3,2): + droneDX(D3,2)
 + 100 droneIsChargingAtEP(D3,2,3) <= 120
 droneChargedAtRightPositionX1(3,D3,3): + droneDX(D3,3)
 + 100 droneIsChargingAtEP(D3,3,3) <= 120
 droneChargedAtRightPositionX1(3,D3,4): + droneDX(D3,4)
 + 100 droneIsChargingAtEP(D3,4,3) <= 120
 droneChargedAtRightPositionX1(3,D3,5): + droneDX(D3,5)
 + 100 droneIsChargingAtEP(D3,5,3) <= 120
 droneChargedAtRightPositionX1(3,D4,1): + droneDX(D4,1)
 + 100 droneIsChargingAtEP(D4,1,3) <= 120
 droneChargedAtRightPositionX1(3,D4,2): + droneDX(D4,2)
 + 100 droneIsChargingAtEP(D4,2,3) <= 120
 droneChargedAtRightPositionX1(3,D4,3): + droneDX(D4,3)
 + 100 droneIsChargingAtEP(D4,3,3) <= 120
 droneChargedAtRightPositionX1(3,D4,4): + droneDX(D4,4)
 + 100 droneIsChargingAtEP(D4,4,3) <= 120
 droneChargedAtRightPositionX1(3,D4,5): + droneDX(D4,5)
 + 100 droneIsChargingAtEP(D4,5,3) <= 120
 droneChargedAtRightPositionX1(3,D5,1): + droneDX(D5,1)
 + 100 droneIsChargingAtEP(D5,1,3) <= 120
 droneChargedAtRightPositionX1(3,D5,2): + droneDX(D5,2)
 + 100 droneIsChargingAtEP(D5,2,3) <= 120
 droneChargedAtRightPositionX1(3,D5,3): + droneDX(D5,3)
 + 100 droneIsChargingAtEP(D5,3,3) <= 120
 droneChargedAtRightPositionX1(3,D5,4): + droneDX(D5,4)
 + 100 droneIsChargingAtEP(D5,4,3) <= 120
 droneChargedAtRightPositionX1(3,D5,5): + droneDX(D5,5)
 + 100 droneIsChargingAtEP(D5,5,3) <= 120
 droneChargedAtRightPositionX1(4,D1,1): + droneDX(D1,1)
 + 100 droneIsChargingAtEP(D1,1,4) <= 110
 droneChargedAtRightPositionX1(4,D1,2): + droneDX(D1,2)
 + 100 droneIsChargingAtEP(D1,2,4) <= 110
 droneChargedAtRightPositionX1(4,D1,3): + droneDX(D1,3)
 + 100 droneIsChargingAtEP(D1,3,4) <= 110
 droneChargedAtRightPositionX1(4,D1,4): + droneDX(D1,4)
 + 100 droneIsChargingAtEP(D1,4,4) <= 110
 droneChargedAtRightPositionX1(4,D1,5): + droneDX(D1,5)
 + 100 droneIsChargingAtEP(D1,5,4) <= 110
 droneChargedAtRightPositionX1(4,D2,1): + droneDX(D2,1)
 + 100 droneIsChargingAtEP(D2,1,4) <= 110
 droneChargedAtRightPositionX1(4,D2,2): + droneDX(D2,2)
 + 100 droneIsChargingAtEP(D2,2,4) <= 110
 droneChargedAtRightPositionX1(4,D2,3): + droneDX(D2,3)
 + 100 droneIsChargingAtEP(D2,3,4) <= 110
 droneChargedAtRightPositionX1(4,D2,4): + droneDX(D2,4)
 + 100 droneIsChargingAtEP(D2,4,4) <= 110
 droneChargedAtRightPositionX1(4,D2,5): + droneDX(D2,5)
 + 100 droneIsChargingAtEP(D2,5,4) <= 110
 droneChargedAtRightPositionX1(4,D3,1): + droneDX(D3,1)
 + 100 droneIsChargingAtEP(D3,1,4) <= 110
 droneChargedAtRightPositionX1(4,D3,2): + droneDX(D3,2)
 + 100 droneIsChargingAtEP(D3,2,4) <= 110
 droneChargedAtRightPositionX1(4,D3,3): + droneDX(D3,3)
 + 100 droneIsChargingAtEP(D3,3,4) <= 110
 droneChargedAtRightPositionX1(4,D3,4): + droneDX(D3,4)
 + 100 droneIsChargingAtEP(D3,4,4) <= 110
 droneChargedAtRightPositionX1(4,D3,5): + droneDX(D3,5)
 + 100 droneIsChargingAtEP(D3,5,4) <= 110
 droneChargedAtRightPositionX1(4,D4,1): + droneDX(D4,1)
 + 100 droneIsChargingAtEP(D4,1,4) <= 110
 droneChargedAtRightPositionX1(4,D4,2): + droneDX(D4,2)
 + 100 droneIsChargingAtEP(D4,2,4) <= 110
 droneChargedAtRightPositionX1(4,D4,3): + droneDX(D4,3)
 + 100 droneIsChargingAtEP(D4,3,4) <= 110
 droneChargedAtRightPositionX1(4,D4,4): + droneDX(D4,4)
 + 100 droneIsChargingAtEP(D4,4,4) <= 110
 droneChargedAtRightPositionX1(4,D4,5): + droneDX(D4,5)
 + 100 droneIsChargingAtEP(D4,5,4) <= 110
 droneChargedAtRightPositionX1(4,D5,1): + droneDX(D5,1)
 + 100 droneIsChargingAtEP(D5,1,4) <= 110
 droneChargedAtRightPositionX1(4,D5,2): + droneDX(D5,2)
 + 100 droneIsChargingAtEP(D5,2,4) <= 110
 droneChargedAtRightPositionX1(4,D5,3): + droneDX(D5,3)
 + 100 droneIsChargingAtEP(D5,3,4) <= 110
 droneChargedAtRightPositionX1(4,D5,4): + droneDX(D5,4)
 + 100 droneIsChargingAtEP(D5,4,4) <= 110
 droneChargedAtRightPositionX1(4,D5,5): + droneDX(D5,5)
 + 100 droneIsChargingAtEP(D5,5,4) <= 110
 droneChargedRightPositionX2(1,D1,1): - droneDX(D1,1)
 + 100 droneIsChargingAtEP(D1,1,1) <= 95
 droneChargedRightPositionX2(1,D1,2): - droneDX(D1,2)
 + 100 droneIsChargingAtEP(D1,2,1) <= 95
 droneChargedRightPositionX2(1,D1,3): - droneDX(D1,3)
 + 100 droneIsChargingAtEP(D1,3,1) <= 95
 droneChargedRightPositionX2(1,D1,4): - droneDX(D1,4)
 + 100 droneIsChargingAtEP(D1,4,1) <= 95
 droneChargedRightPositionX2(1,D1,5): - droneDX(D1,5)
 + 100 droneIsChargingAtEP(D1,5,1) <= 95
 droneChargedRightPositionX2(1,D2,1): - droneDX(D2,1)
 + 100 droneIsChargingAtEP(D2,1,1) <= 95
 droneChargedRightPositionX2(1,D2,2): - droneDX(D2,2)
 + 100 droneIsChargingAtEP(D2,2,1) <= 95
 droneChargedRightPositionX2(1,D2,3): - droneDX(D2,3)
 + 100 droneIsChargingAtEP(D2,3,1) <= 95
 droneChargedRightPositionX2(1,D2,4): - droneDX(D2,4)
 + 100 droneIsChargingAtEP(D2,4,1) <= 95
 droneChargedRightPositionX2(1,D2,5): - droneDX(D2,5)
 + 100 droneIsChargingAtEP(D2,5,1) <= 95
 droneChargedRightPositionX2(1,D3,1): - droneDX(D3,1)
 + 100 droneIsChargingAtEP(D3,1,1) <= 95
 droneChargedRightPositionX2(1,D3,2): - droneDX(D3,2)
 + 100 droneIsChargingAtEP(D3,2,1) <= 95
 droneChargedRightPositionX2(1,D3,3): - droneDX(D3,3)
 + 100 droneIsChargingAtEP(D3,3,1) <= 95
 droneChargedRightPositionX2(1,D3,4): - droneDX(D3,4)
 + 100 droneIsChargingAtEP(D3,4,1) <= 95
 droneChargedRightPositionX2(1,D3,5): - droneDX(D3,5)
 + 100 droneIsChargingAtEP(D3,5,1) <= 95
 droneChargedRightPositionX2(1,D4,1): - droneDX(D4,1)
 + 100 droneIsChargingAtEP(D4,1,1) <= 95
 droneChargedRightPositionX2(1,D4,2): - droneDX(D4,2)
 + 100 droneIsChargingAtEP(D4,2,1) <= 95
 droneChargedRightPositionX2(1,D4,3): - droneDX(D4,3)
 + 100 droneIsChargingAtEP(D4,3,1) <= 95
 droneChargedRightPositionX2(1,D4,4): - droneDX(D4,4)
 + 100 droneIsChargingAtEP(D4,4,1) <= 95
 droneChargedRightPositionX2(1,D4,5): - droneDX(D4,5)
 + 100 droneIsChargingAtEP(D4,5,1) <= 95
 droneChargedRightPositionX2(1,D5,1): - droneDX(D5,1)
 + 100 droneIsChargingAtEP(D5,1,1) <= 95
 droneChargedRightPositionX2(1,D5,2): - droneDX(D5,2)
 + 100 droneIsChargingAtEP(D5,2,1) <= 95
 droneChargedRightPositionX2(1,D5,3): - droneDX(D5,3)
 + 100 droneIsChargingAtEP(D5,3,1) <= 95
 droneChargedRightPositionX2(1,D5,4): - droneDX(D5,4)
 + 100 droneIsChargingAtEP(D5,4,1) <= 95
 droneChargedRightPositionX2(1,D5,5): - droneDX(D5,5)
 + 100 droneIsChargingAtEP(D5,5,1) <= 95
 droneChargedRightPositionX2(2,D1,1): - droneDX(D1,1)
 + 100 droneIsChargingAtEP(D1,1,2) <= 100
 droneChargedRightPositionX2(2,D1,2): - droneDX(D1,2)
 + 100 droneIsChargingAtEP(D1,2,2) <= 100
 droneChargedRightPositionX2(2,D1,3): - droneDX(D1,3)
 + 100 droneIsChargingAtEP(D1,3,2) <= 100
 droneChargedRightPositionX2(2,D1,4): - droneDX(D1,4)
 + 100 droneIsChargingAtEP(D1,4,2) <= 100
 droneChargedRightPositionX2(2,D1,5): - droneDX(D1,5)
 + 100 droneIsChargingAtEP(D1,5,2) <= 100
 droneChargedRightPositionX2(2,D2,1): - droneDX(D2,1)
 + 100 droneIsChargingAtEP(D2,1,2) <= 100
 droneChargedRightPositionX2(2,D2,2): - droneDX(D2,2)
 + 100 droneIsChargingAtEP(D2,2,2) <= 100
 droneChargedRightPositionX2(2,D2,3): - droneDX(D2,3)
 + 100 droneIsChargingAtEP(D2,3,2) <= 100
 droneChargedRightPositionX2(2,D2,4): - droneDX(D2,4)
 + 100 droneIsChargingAtEP(D2,4,2) <= 100
 droneChargedRightPositionX2(2,D2,5): - droneDX(D2,5)
 + 100 droneIsChargingAtEP(D2,5,2) <= 100
 droneChargedRightPositionX2(2,D3,1): - droneDX(D3,1)
 + 100 droneIsChargingAtEP(D3,1,2) <= 100
 droneChargedRightPositionX2(2,D3,2): - droneDX(D3,2)
 + 100 droneIsChargingAtEP(D3,2,2) <= 100
 droneChargedRightPositionX2(2,D3,3): - droneDX(D3,3)
 + 100 droneIsChargingAtEP(D3,3,2) <= 100
 droneChargedRightPositionX2(2,D3,4): - droneDX(D3,4)
 + 100 droneIsChargingAtEP(D3,4,2) <= 100
 droneChargedRightPositionX2(2,D3,5): - droneDX(D3,5)
 + 100 droneIsChargingAtEP(D3,5,2) <= 100
 droneChargedRightPositionX2(2,D4,1): - droneDX(D4,1)
 + 100 droneIsChargingAtEP(D4,1,2) <= 100
 droneChargedRightPositionX2(2,D4,2): - droneDX(D4,2)
 + 100 droneIsChargingAtEP(D4,2,2) <= 100
 droneChargedRightPositionX2(2,D4,3): - droneDX(D4,3)
 + 100 droneIsChargingAtEP(D4,3,2) <= 100
 droneChargedRightPositionX2(2,D4,4): - droneDX(D4,4)
 + 100 droneIsChargingAtEP(D4,4,2) <= 100
 droneChargedRightPositionX2(2,D4,5): - droneDX(D4,5)
 + 100 droneIsChargingAtEP(D4,5,2) <= 100
 droneChargedRightPositionX2(2,D5,1): - droneDX(D5,1)
 + 100 droneIsChargingAtEP(D5,1,2) <= 100
 droneChargedRightPositionX2(2,D5,2): - droneDX(D5,2)
 + 100 droneIsChargingAtEP(D5,2,2) <= 100
 droneChargedRightPositionX2(2,D5,3): - droneDX(D5,3)
 + 100 droneIsChargingAtEP(D5,3,2) <= 100
 droneChargedRightPositionX2(2,D5,4): - droneDX(D5,4)
 + 100 droneIsChargingAtEP(D5,4,2) <= 100
 droneChargedRightPositionX2(2,D5,5): - droneDX(D5,5)
 + 100 droneIsChargingAtEP(D5,5,2) <= 100
 droneChargedRightPositionX2(3,D1,1): - droneDX(D1,1)
 + 100 droneIsChargingAtEP(D1,1,3) <= 80
 droneChargedRightPositionX2(3,D1,2): - droneDX(D1,2)
 + 100 droneIsChargingAtEP(D1,2,3) <= 80
 droneChargedRightPositionX2(3,D1,3): - droneDX(D1,3)
 + 100 droneIsChargingAtEP(D1,3,3) <= 80
 droneChargedRightPositionX2(3,D1,4): - droneDX(D1,4)
 + 100 droneIsChargingAtEP(D1,4,3) <= 80
 droneChargedRightPositionX2(3,D1,5): - droneDX(D1,5)
 + 100 droneIsChargingAtEP(D1,5,3) <= 80
 droneChargedRightPositionX2(3,D2,1): - droneDX(D2,1)
 + 100 droneIsChargingAtEP(D2,1,3) <= 80
 droneChargedRightPositionX2(3,D2,2): - droneDX(D2,2)
 + 100 droneIsChargingAtEP(D2,2,3) <= 80
 droneChargedRightPositionX2(3,D2,3): - droneDX(D2,3)
 + 100 droneIsChargingAtEP(D2,3,3) <= 80
 droneChargedRightPositionX2(3,D2,4): - droneDX(D2,4)
 + 100 droneIsChargingAtEP(D2,4,3) <= 80
 droneChargedRightPositionX2(3,D2,5): - droneDX(D2,5)
 + 100 droneIsChargingAtEP(D2,5,3) <= 80
 droneChargedRightPositionX2(3,D3,1): - droneDX(D3,1)
 + 100 droneIsChargingAtEP(D3,1,3) <= 80
 droneChargedRightPositionX2(3,D3,2): - droneDX(D3,2)
 + 100 droneIsChargingAtEP(D3,2,3) <= 80
 droneChargedRightPositionX2(3,D3,3): - droneDX(D3,3)
 + 100 droneIsChargingAtEP(D3,3,3) <= 80
 droneChargedRightPositionX2(3,D3,4): - droneDX(D3,4)
 + 100 droneIsChargingAtEP(D3,4,3) <= 80
 droneChargedRightPositionX2(3,D3,5): - droneDX(D3,5)
 + 100 droneIsChargingAtEP(D3,5,3) <= 80
 droneChargedRightPositionX2(3,D4,1): - droneDX(D4,1)
 + 100 droneIsChargingAtEP(D4,1,3) <= 80
 droneChargedRightPositionX2(3,D4,2): - droneDX(D4,2)
 + 100 droneIsChargingAtEP(D4,2,3) <= 80
 droneChargedRightPositionX2(3,D4,3): - droneDX(D4,3)
 + 100 droneIsChargingAtEP(D4,3,3) <= 80
 droneChargedRightPositionX2(3,D4,4): - droneDX(D4,4)
 + 100 droneIsChargingAtEP(D4,4,3) <= 80
 droneChargedRightPositionX2(3,D4,5): - droneDX(D4,5)
 + 100 droneIsChargingAtEP(D4,5,3) <= 80
 droneChargedRightPositionX2(3,D5,1): - droneDX(D5,1)
 + 100 droneIsChargingAtEP(D5,1,3) <= 80
 droneChargedRightPositionX2(3,D5,2): - droneDX(D5,2)
 + 100 droneIsChargingAtEP(D5,2,3) <= 80
 droneChargedRightPositionX2(3,D5,3): - droneDX(D5,3)
 + 100 droneIsChargingAtEP(D5,3,3) <= 80
 droneChargedRightPositionX2(3,D5,4): - droneDX(D5,4)
 + 100 droneIsChargingAtEP(D5,4,3) <= 80
 droneChargedRightPositionX2(3,D5,5): - droneDX(D5,5)
 + 100 droneIsChargingAtEP(D5,5,3) <= 80
 droneChargedRightPositionX2(4,D1,1): - droneDX(D1,1)
 + 100 droneIsChargingAtEP(D1,1,4) <= 90
 droneChargedRightPositionX2(4,D1,2): - droneDX(D1,2)
 + 100 droneIsChargingAtEP(D1,2,4) <= 90
 droneChargedRightPositionX2(4,D1,3): - droneDX(D1,3)
 + 100 droneIsChargingAtEP(D1,3,4) <= 90
 droneChargedRightPositionX2(4,D1,4): - droneDX(D1,4)
 + 100 droneIsChargingAtEP(D1,4,4) <= 90
 droneChargedRightPositionX2(4,D1,5): - droneDX(D1,5)
 + 100 droneIsChargingAtEP(D1,5,4) <= 90
 droneChargedRightPositionX2(4,D2,1): - droneDX(D2,1)
 + 100 droneIsChargingAtEP(D2,1,4) <= 90
 droneChargedRightPositionX2(4,D2,2): - droneDX(D2,2)
 + 100 droneIsChargingAtEP(D2,2,4) <= 90
 droneChargedRightPositionX2(4,D2,3): - droneDX(D2,3)
 + 100 droneIsChargingAtEP(D2,3,4) <= 90
 droneChargedRightPositionX2(4,D2,4): - droneDX(D2,4)
 + 100 droneIsChargingAtEP(D2,4,4) <= 90
 droneChargedRightPositionX2(4,D2,5): - droneDX(D2,5)
 + 100 droneIsChargingAtEP(D2,5,4) <= 90
 droneChargedRightPositionX2(4,D3,1): - droneDX(D3,1)
 + 100 droneIsChargingAtEP(D3,1,4) <= 90
 droneChargedRightPositionX2(4,D3,2): - droneDX(D3,2)
 + 100 droneIsChargingAtEP(D3,2,4) <= 90
 droneChargedRightPositionX2(4,D3,3): - droneDX(D3,3)
 + 100 droneIsChargingAtEP(D3,3,4) <= 90
 droneChargedRightPositionX2(4,D3,4): - droneDX(D3,4)
 + 100 droneIsChargingAtEP(D3,4,4) <= 90
 droneChargedRightPositionX2(4,D3,5): - droneDX(D3,5)
 + 100 droneIsChargingAtEP(D3,5,4) <= 90
 droneChargedRightPositionX2(4,D4,1): - droneDX(D4,1)
 + 100 droneIsChargingAtEP(D4,1,4) <= 90
 droneChargedRightPositionX2(4,D4,2): - droneDX(D4,2)
 + 100 droneIsChargingAtEP(D4,2,4) <= 90
 droneChargedRightPositionX2(4,D4,3): - droneDX(D4,3)
 + 100 droneIsChargingAtEP(D4,3,4) <= 90
 droneChargedRightPositionX2(4,D4,4): - droneDX(D4,4)
 + 100 droneIsChargingAtEP(D4,4,4) <= 90
 droneChargedRightPositionX2(4,D4,5): - droneDX(D4,5)
 + 100 droneIsChargingAtEP(D4,5,4) <= 90
 droneChargedRightPositionX2(4,D5,1): - droneDX(D5,1)
 + 100 droneIsChargingAtEP(D5,1,4) <= 90
 droneChargedRightPositionX2(4,D5,2): - droneDX(D5,2)
 + 100 droneIsChargingAtEP(D5,2,4) <= 90
 droneChargedRightPositionX2(4,D5,3): - droneDX(D5,3)
 + 100 droneIsChargingAtEP(D5,3,4) <= 90
 droneChargedRightPositionX2(4,D5,4): - droneDX(D5,4)
 + 100 droneIsChargingAtEP(D5,4,4) <= 90
 droneChargedRightPositionX2(4,D5,5): - droneDX(D5,5)
 + 100 droneIsChargingAtEP(D5,5,4) <= 90
 droneChargedAtRightPositionY1(1,D1,1): + droneDY(D1,1)
 + 100 droneIsChargingAtEP(D1,1,1) <= 105
 droneChargedAtRightPositionY1(1,D1,2): + droneDY(D1,2)
 + 100 droneIsChargingAtEP(D1,2,1) <= 105
 droneChargedAtRightPositionY1(1,D1,3): + droneDY(D1,3)
 + 100 droneIsChargingAtEP(D1,3,1) <= 105
 droneChargedAtRightPositionY1(1,D1,4): + droneDY(D1,4)
 + 100 droneIsChargingAtEP(D1,4,1) <= 105
 droneChargedAtRightPositionY1(1,D1,5): + droneDY(D1,5)
 + 100 droneIsChargingAtEP(D1,5,1) <= 105
 droneChargedAtRightPositionY1(1,D2,1): + droneDY(D2,1)
 + 100 droneIsChargingAtEP(D2,1,1) <= 105
 droneChargedAtRightPositionY1(1,D2,2): + droneDY(D2,2)
 + 100 droneIsChargingAtEP(D2,2,1) <= 105
 droneChargedAtRightPositionY1(1,D2,3): + droneDY(D2,3)
 + 100 droneIsChargingAtEP(D2,3,1) <= 105
 droneChargedAtRightPositionY1(1,D2,4): + droneDY(D2,4)
 + 100 droneIsChargingAtEP(D2,4,1) <= 105
 droneChargedAtRightPositionY1(1,D2,5): + droneDY(D2,5)
 + 100 droneIsChargingAtEP(D2,5,1) <= 105
 droneChargedAtRightPositionY1(1,D3,1): + droneDY(D3,1)
 + 100 droneIsChargingAtEP(D3,1,1) <= 105
 droneChargedAtRightPositionY1(1,D3,2): + droneDY(D3,2)
 + 100 droneIsChargingAtEP(D3,2,1) <= 105
 droneChargedAtRightPositionY1(1,D3,3): + droneDY(D3,3)
 + 100 droneIsChargingAtEP(D3,3,1) <= 105
 droneChargedAtRightPositionY1(1,D3,4): + droneDY(D3,4)
 + 100 droneIsChargingAtEP(D3,4,1) <= 105
 droneChargedAtRightPositionY1(1,D3,5): + droneDY(D3,5)
 + 100 droneIsChargingAtEP(D3,5,1) <= 105
 droneChargedAtRightPositionY1(1,D4,1): + droneDY(D4,1)
 + 100 droneIsChargingAtEP(D4,1,1) <= 105
 droneChargedAtRightPositionY1(1,D4,2): + droneDY(D4,2)
 + 100 droneIsChargingAtEP(D4,2,1) <= 105
 droneChargedAtRightPositionY1(1,D4,3): + droneDY(D4,3)
 + 100 droneIsChargingAtEP(D4,3,1) <= 105
 droneChargedAtRightPositionY1(1,D4,4): + droneDY(D4,4)
 + 100 droneIsChargingAtEP(D4,4,1) <= 105
 droneChargedAtRightPositionY1(1,D4,5): + droneDY(D4,5)
 + 100 droneIsChargingAtEP(D4,5,1) <= 105
 droneChargedAtRightPositionY1(1,D5,1): + droneDY(D5,1)
 + 100 droneIsChargingAtEP(D5,1,1) <= 105
 droneChargedAtRightPositionY1(1,D5,2): + droneDY(D5,2)
 + 100 droneIsChargingAtEP(D5,2,1) <= 105
 droneChargedAtRightPositionY1(1,D5,3): + droneDY(D5,3)
 + 100 droneIsChargingAtEP(D5,3,1) <= 105
 droneChargedAtRightPositionY1(1,D5,4): + droneDY(D5,4)
 + 100 droneIsChargingAtEP(D5,4,1) <= 105
 droneChargedAtRightPositionY1(1,D5,5): + droneDY(D5,5)
 + 100 droneIsChargingAtEP(D5,5,1) <= 105
 droneChargedAtRightPositionY1(2,D1,1): + droneDY(D1,1)
 + 100 droneIsChargingAtEP(D1,1,2) <= 100
 droneChargedAtRightPositionY1(2,D1,2): + droneDY(D1,2)
 + 100 droneIsChargingAtEP(D1,2,2) <= 100
 droneChargedAtRightPositionY1(2,D1,3): + droneDY(D1,3)
 + 100 droneIsChargingAtEP(D1,3,2) <= 100
 droneChargedAtRightPositionY1(2,D1,4): + droneDY(D1,4)
 + 100 droneIsChargingAtEP(D1,4,2) <= 100
 droneChargedAtRightPositionY1(2,D1,5): + droneDY(D1,5)
 + 100 droneIsChargingAtEP(D1,5,2) <= 100
 droneChargedAtRightPositionY1(2,D2,1): + droneDY(D2,1)
 + 100 droneIsChargingAtEP(D2,1,2) <= 100
 droneChargedAtRightPositionY1(2,D2,2): + droneDY(D2,2)
 + 100 droneIsChargingAtEP(D2,2,2) <= 100
 droneChargedAtRightPositionY1(2,D2,3): + droneDY(D2,3)
 + 100 droneIsChargingAtEP(D2,3,2) <= 100
 droneChargedAtRightPositionY1(2,D2,4): + droneDY(D2,4)
 + 100 droneIsChargingAtEP(D2,4,2) <= 100
 droneChargedAtRightPositionY1(2,D2,5): + droneDY(D2,5)
 + 100 droneIsChargingAtEP(D2,5,2) <= 100
 droneChargedAtRightPositionY1(2,D3,1): + droneDY(D3,1)
 + 100 droneIsChargingAtEP(D3,1,2) <= 100
 droneChargedAtRightPositionY1(2,D3,2): + droneDY(D3,2)
 + 100 droneIsChargingAtEP(D3,2,2) <= 100
 droneChargedAtRightPositionY1(2,D3,3): + droneDY(D3,3)
 + 100 droneIsChargingAtEP(D3,3,2) <= 100
 droneChargedAtRightPositionY1(2,D3,4): + droneDY(D3,4)
 + 100 droneIsChargingAtEP(D3,4,2) <= 100
 droneChargedAtRightPositionY1(2,D3,5): + droneDY(D3,5)
 + 100 droneIsChargingAtEP(D3,5,2) <= 100
 droneChargedAtRightPositionY1(2,D4,1): + droneDY(D4,1)
 + 100 droneIsChargingAtEP(D4,1,2) <= 100
 droneChargedAtRightPositionY1(2,D4,2): + droneDY(D4,2)
 + 100 droneIsChargingAtEP(D4,2,2) <= 100
 droneChargedAtRightPositionY1(2,D4,3): + droneDY(D4,3)
 + 100 droneIsChargingAtEP(D4,3,2) <= 100
 droneChargedAtRightPositionY1(2,D4,4): + droneDY(D4,4)
 + 100 droneIsChargingAtEP(D4,4,2) <= 100
 droneChargedAtRightPositionY1(2,D4,5): + droneDY(D4,5)
 + 100 droneIsChargingAtEP(D4,5,2) <= 100
 droneChargedAtRightPositionY1(2,D5,1): + droneDY(D5,1)
 + 100 droneIsChargingAtEP(D5,1,2) <= 100
 droneChargedAtRightPositionY1(2,D5,2): + droneDY(D5,2)
 + 100 droneIsChargingAtEP(D5,2,2) <= 100
 droneChargedAtRightPositionY1(2,D5,3): + droneDY(D5,3)
 + 100 droneIsChargingAtEP(D5,3,2) <= 100
 droneChargedAtRightPositionY1(2,D5,4): + droneDY(D5,4)
 + 100 droneIsChargingAtEP(D5,4,2) <= 100
 droneChargedAtRightPositionY1(2,D5,5): + droneDY(D5,5)
 + 100 droneIsChargingAtEP(D5,5,2) <= 100
 droneChargedAtRightPositionY1(3,D1,1): + droneDY(D1,1)
 + 100 droneIsChargingAtEP(D1,1,3) <= 120
 droneChargedAtRightPositionY1(3,D1,2): + droneDY(D1,2)
 + 100 droneIsChargingAtEP(D1,2,3) <= 120
 droneChargedAtRightPositionY1(3,D1,3): + droneDY(D1,3)
 + 100 droneIsChargingAtEP(D1,3,3) <= 120
 droneChargedAtRightPositionY1(3,D1,4): + droneDY(D1,4)
 + 100 droneIsChargingAtEP(D1,4,3) <= 120
 droneChargedAtRightPositionY1(3,D1,5): + droneDY(D1,5)
 + 100 droneIsChargingAtEP(D1,5,3) <= 120
 droneChargedAtRightPositionY1(3,D2,1): + droneDY(D2,1)
 + 100 droneIsChargingAtEP(D2,1,3) <= 120
 droneChargedAtRightPositionY1(3,D2,2): + droneDY(D2,2)
 + 100 droneIsChargingAtEP(D2,2,3) <= 120
 droneChargedAtRightPositionY1(3,D2,3): + droneDY(D2,3)
 + 100 droneIsChargingAtEP(D2,3,3) <= 120
 droneChargedAtRightPositionY1(3,D2,4): + droneDY(D2,4)
 + 100 droneIsChargingAtEP(D2,4,3) <= 120
 droneChargedAtRightPositionY1(3,D2,5): + droneDY(D2,5)
 + 100 droneIsChargingAtEP(D2,5,3) <= 120
 droneChargedAtRightPositionY1(3,D3,1): + droneDY(D3,1)
 + 100 droneIsChargingAtEP(D3,1,3) <= 120
 droneChargedAtRightPositionY1(3,D3,2): + droneDY(D3,2)
 + 100 droneIsChargingAtEP(D3,2,3) <= 120
 droneChargedAtRightPositionY1(3,D3,3): + droneDY(D3,3)
 + 100 droneIsChargingAtEP(D3,3,3) <= 120
 droneChargedAtRightPositionY1(3,D3,4): + droneDY(D3,4)
 + 100 droneIsChargingAtEP(D3,4,3) <= 120
 droneChargedAtRightPositionY1(3,D3,5): + droneDY(D3,5)
 + 100 droneIsChargingAtEP(D3,5,3) <= 120
 droneChargedAtRightPositionY1(3,D4,1): + droneDY(D4,1)
 + 100 droneIsChargingAtEP(D4,1,3) <= 120
 droneChargedAtRightPositionY1(3,D4,2): + droneDY(D4,2)
 + 100 droneIsChargingAtEP(D4,2,3) <= 120
 droneChargedAtRightPositionY1(3,D4,3): + droneDY(D4,3)
 + 100 droneIsChargingAtEP(D4,3,3) <= 120
 droneChargedAtRightPositionY1(3,D4,4): + droneDY(D4,4)
 + 100 droneIsChargingAtEP(D4,4,3) <= 120
 droneChargedAtRightPositionY1(3,D4,5): + droneDY(D4,5)
 + 100 droneIsChargingAtEP(D4,5,3) <= 120
 droneChargedAtRightPositionY1(3,D5,1): + droneDY(D5,1)
 + 100 droneIsChargingAtEP(D5,1,3) <= 120
 droneChargedAtRightPositionY1(3,D5,2): + droneDY(D5,2)
 + 100 droneIsChargingAtEP(D5,2,3) <= 120
 droneChargedAtRightPositionY1(3,D5,3): + droneDY(D5,3)
 + 100 droneIsChargingAtEP(D5,3,3) <= 120
 droneChargedAtRightPositionY1(3,D5,4): + droneDY(D5,4)
 + 100 droneIsChargingAtEP(D5,4,3) <= 120
 droneChargedAtRightPositionY1(3,D5,5): + droneDY(D5,5)
 + 100 droneIsChargingAtEP(D5,5,3) <= 120
 droneChargedAtRightPositionY1(4,D1,1): + droneDY(D1,1)
 + 100 droneIsChargingAtEP(D1,1,4) <= 110
 droneChargedAtRightPositionY1(4,D1,2): + droneDY(D1,2)
 + 100 droneIsChargingAtEP(D1,2,4) <= 110
 droneChargedAtRightPositionY1(4,D1,3): + droneDY(D1,3)
 + 100 droneIsChargingAtEP(D1,3,4) <= 110
 droneChargedAtRightPositionY1(4,D1,4): + droneDY(D1,4)
 + 100 droneIsChargingAtEP(D1,4,4) <= 110
 droneChargedAtRightPositionY1(4,D1,5): + droneDY(D1,5)
 + 100 droneIsChargingAtEP(D1,5,4) <= 110
 droneChargedAtRightPositionY1(4,D2,1): + droneDY(D2,1)
 + 100 droneIsChargingAtEP(D2,1,4) <= 110
 droneChargedAtRightPositionY1(4,D2,2): + droneDY(D2,2)
 + 100 droneIsChargingAtEP(D2,2,4) <= 110
 droneChargedAtRightPositionY1(4,D2,3): + droneDY(D2,3)
 + 100 droneIsChargingAtEP(D2,3,4) <= 110
 droneChargedAtRightPositionY1(4,D2,4): + droneDY(D2,4)
 + 100 droneIsChargingAtEP(D2,4,4) <= 110
 droneChargedAtRightPositionY1(4,D2,5): + droneDY(D2,5)
 + 100 droneIsChargingAtEP(D2,5,4) <= 110
 droneChargedAtRightPositionY1(4,D3,1): + droneDY(D3,1)
 + 100 droneIsChargingAtEP(D3,1,4) <= 110
 droneChargedAtRightPositionY1(4,D3,2): + droneDY(D3,2)
 + 100 droneIsChargingAtEP(D3,2,4) <= 110
 droneChargedAtRightPositionY1(4,D3,3): + droneDY(D3,3)
 + 100 droneIsChargingAtEP(D3,3,4) <= 110
 droneChargedAtRightPositionY1(4,D3,4): + droneDY(D3,4)
 + 100 droneIsChargingAtEP(D3,4,4) <= 110
 droneChargedAtRightPositionY1(4,D3,5): + droneDY(D3,5)
 + 100 droneIsChargingAtEP(D3,5,4) <= 110
 droneChargedAtRightPositionY1(4,D4,1): + droneDY(D4,1)
 + 100 droneIsChargingAtEP(D4,1,4) <= 110
 droneChargedAtRightPositionY1(4,D4,2): + droneDY(D4,2)
 + 100 droneIsChargingAtEP(D4,2,4) <= 110
 droneChargedAtRightPositionY1(4,D4,3): + droneDY(D4,3)
 + 100 droneIsChargingAtEP(D4,3,4) <= 110
 droneChargedAtRightPositionY1(4,D4,4): + droneDY(D4,4)
 + 100 droneIsChargingAtEP(D4,4,4) <= 110
 droneChargedAtRightPositionY1(4,D4,5): + droneDY(D4,5)
 + 100 droneIsChargingAtEP(D4,5,4) <= 110
 droneChargedAtRightPositionY1(4,D5,1): + droneDY(D5,1)
 + 100 droneIsChargingAtEP(D5,1,4) <= 110
 droneChargedAtRightPositionY1(4,D5,2): + droneDY(D5,2)
 + 100 droneIsChargingAtEP(D5,2,4) <= 110
 droneChargedAtRightPositionY1(4,D5,3): + droneDY(D5,3)
 + 100 droneIsChargingAtEP(D5,3,4) <= 110
 droneChargedAtRightPositionY1(4,D5,4): + droneDY(D5,4)
 + 100 droneIsChargingAtEP(D5,4,4) <= 110
 droneChargedAtRightPositionY1(4,D5,5): + droneDY(D5,5)
 + 100 droneIsChargingAtEP(D5,5,4) <= 110
 droneChargedAtRightPositionY2(1,D1,1): - droneDY(D1,1)
 + 100 droneIsChargingAtEP(D1,1,1) <= 95
 droneChargedAtRightPositionY2(1,D1,2): - droneDY(D1,2)
 + 100 droneIsChargingAtEP(D1,2,1) <= 95
 droneChargedAtRightPositionY2(1,D1,3): - droneDY(D1,3)
 + 100 droneIsChargingAtEP(D1,3,1) <= 95
 droneChargedAtRightPositionY2(1,D1,4): - droneDY(D1,4)
 + 100 droneIsChargingAtEP(D1,4,1) <= 95
 droneChargedAtRightPositionY2(1,D1,5): - droneDY(D1,5)
 + 100 droneIsChargingAtEP(D1,5,1) <= 95
 droneChargedAtRightPositionY2(1,D2,1): - droneDY(D2,1)
 + 100 droneIsChargingAtEP(D2,1,1) <= 95
 droneChargedAtRightPositionY2(1,D2,2): - droneDY(D2,2)
 + 100 droneIsChargingAtEP(D2,2,1) <= 95
 droneChargedAtRightPositionY2(1,D2,3): - droneDY(D2,3)
 + 100 droneIsChargingAtEP(D2,3,1) <= 95
 droneChargedAtRightPositionY2(1,D2,4): - droneDY(D2,4)
 + 100 droneIsChargingAtEP(D2,4,1) <= 95
 droneChargedAtRightPositionY2(1,D2,5): - droneDY(D2,5)
 + 100 droneIsChargingAtEP(D2,5,1) <= 95
 droneChargedAtRightPositionY2(1,D3,1): - droneDY(D3,1)
 + 100 droneIsChargingAtEP(D3,1,1) <= 95
 droneChargedAtRightPositionY2(1,D3,2): - droneDY(D3,2)
 + 100 droneIsChargingAtEP(D3,2,1) <= 95
 droneChargedAtRightPositionY2(1,D3,3): - droneDY(D3,3)
 + 100 droneIsChargingAtEP(D3,3,1) <= 95
 droneChargedAtRightPositionY2(1,D3,4): - droneDY(D3,4)
 + 100 droneIsChargingAtEP(D3,4,1) <= 95
 droneChargedAtRightPositionY2(1,D3,5): - droneDY(D3,5)
 + 100 droneIsChargingAtEP(D3,5,1) <= 95
 droneChargedAtRightPositionY2(1,D4,1): - droneDY(D4,1)
 + 100 droneIsChargingAtEP(D4,1,1) <= 95
 droneChargedAtRightPositionY2(1,D4,2): - droneDY(D4,2)
 + 100 droneIsChargingAtEP(D4,2,1) <= 95
 droneChargedAtRightPositionY2(1,D4,3): - droneDY(D4,3)
 + 100 droneIsChargingAtEP(D4,3,1) <= 95
 droneChargedAtRightPositionY2(1,D4,4): - droneDY(D4,4)
 + 100 droneIsChargingAtEP(D4,4,1) <= 95
 droneChargedAtRightPositionY2(1,D4,5): - droneDY(D4,5)
 + 100 droneIsChargingAtEP(D4,5,1) <= 95
 droneChargedAtRightPositionY2(1,D5,1): - droneDY(D5,1)
 + 100 droneIsChargingAtEP(D5,1,1) <= 95
 droneChargedAtRightPositionY2(1,D5,2): - droneDY(D5,2)
 + 100 droneIsChargingAtEP(D5,2,1) <= 95
 droneChargedAtRightPositionY2(1,D5,3): - droneDY(D5,3)
 + 100 droneIsChargingAtEP(D5,3,1) <= 95
 droneChargedAtRightPositionY2(1,D5,4): - droneDY(D5,4)
 + 100 droneIsChargingAtEP(D5,4,1) <= 95
 droneChargedAtRightPositionY2(1,D5,5): - droneDY(D5,5)
 + 100 droneIsChargingAtEP(D5,5,1) <= 95
 droneChargedAtRightPositionY2(2,D1,1): - droneDY(D1,1)
 + 100 droneIsChargingAtEP(D1,1,2) <= 100
 droneChargedAtRightPositionY2(2,D1,2): - droneDY(D1,2)
 + 100 droneIsChargingAtEP(D1,2,2) <= 100
 droneChargedAtRightPositionY2(2,D1,3): - droneDY(D1,3)
 + 100 droneIsChargingAtEP(D1,3,2) <= 100
 droneChargedAtRightPositionY2(2,D1,4): - droneDY(D1,4)
 + 100 droneIsChargingAtEP(D1,4,2) <= 100
 droneChargedAtRightPositionY2(2,D1,5): - droneDY(D1,5)
 + 100 droneIsChargingAtEP(D1,5,2) <= 100
 droneChargedAtRightPositionY2(2,D2,1): - droneDY(D2,1)
 + 100 droneIsChargingAtEP(D2,1,2) <= 100
 droneChargedAtRightPositionY2(2,D2,2): - droneDY(D2,2)
 + 100 droneIsChargingAtEP(D2,2,2) <= 100
 droneChargedAtRightPositionY2(2,D2,3): - droneDY(D2,3)
 + 100 droneIsChargingAtEP(D2,3,2) <= 100
 droneChargedAtRightPositionY2(2,D2,4): - droneDY(D2,4)
 + 100 droneIsChargingAtEP(D2,4,2) <= 100
 droneChargedAtRightPositionY2(2,D2,5): - droneDY(D2,5)
 + 100 droneIsChargingAtEP(D2,5,2) <= 100
 droneChargedAtRightPositionY2(2,D3,1): - droneDY(D3,1)
 + 100 droneIsChargingAtEP(D3,1,2) <= 100
 droneChargedAtRightPositionY2(2,D3,2): - droneDY(D3,2)
 + 100 droneIsChargingAtEP(D3,2,2) <= 100
 droneChargedAtRightPositionY2(2,D3,3): - droneDY(D3,3)
 + 100 droneIsChargingAtEP(D3,3,2) <= 100
 droneChargedAtRightPositionY2(2,D3,4): - droneDY(D3,4)
 + 100 droneIsChargingAtEP(D3,4,2) <= 100
 droneChargedAtRightPositionY2(2,D3,5): - droneDY(D3,5)
 + 100 droneIsChargingAtEP(D3,5,2) <= 100
 droneChargedAtRightPositionY2(2,D4,1): - droneDY(D4,1)
 + 100 droneIsChargingAtEP(D4,1,2) <= 100
 droneChargedAtRightPositionY2(2,D4,2): - droneDY(D4,2)
 + 100 droneIsChargingAtEP(D4,2,2) <= 100
 droneChargedAtRightPositionY2(2,D4,3): - droneDY(D4,3)
 + 100 droneIsChargingAtEP(D4,3,2) <= 100
 droneChargedAtRightPositionY2(2,D4,4): - droneDY(D4,4)
 + 100 droneIsChargingAtEP(D4,4,2) <= 100
 droneChargedAtRightPositionY2(2,D4,5): - droneDY(D4,5)
 + 100 droneIsChargingAtEP(D4,5,2) <= 100
 droneChargedAtRightPositionY2(2,D5,1): - droneDY(D5,1)
 + 100 droneIsChargingAtEP(D5,1,2) <= 100
 droneChargedAtRightPositionY2(2,D5,2): - droneDY(D5,2)
 + 100 droneIsChargingAtEP(D5,2,2) <= 100
 droneChargedAtRightPositionY2(2,D5,3): - droneDY(D5,3)
 + 100 droneIsChargingAtEP(D5,3,2) <= 100
 droneChargedAtRightPositionY2(2,D5,4): - droneDY(D5,4)
 + 100 droneIsChargingAtEP(D5,4,2) <= 100
 droneChargedAtRightPositionY2(2,D5,5): - droneDY(D5,5)
 + 100 droneIsChargingAtEP(D5,5,2) <= 100
 droneChargedAtRightPositionY2(3,D1,1): - droneDY(D1,1)
 + 100 droneIsChargingAtEP(D1,1,3) <= 80
 droneChargedAtRightPositionY2(3,D1,2): - droneDY(D1,2)
 + 100 droneIsChargingAtEP(D1,2,3) <= 80
 droneChargedAtRightPositionY2(3,D1,3): - droneDY(D1,3)
 + 100 droneIsChargingAtEP(D1,3,3) <= 80
 droneChargedAtRightPositionY2(3,D1,4): - droneDY(D1,4)
 + 100 droneIsChargingAtEP(D1,4,3) <= 80
 droneChargedAtRightPositionY2(3,D1,5): - droneDY(D1,5)
 + 100 droneIsChargingAtEP(D1,5,3) <= 80
 droneChargedAtRightPositionY2(3,D2,1): - droneDY(D2,1)
 + 100 droneIsChargingAtEP(D2,1,3) <= 80
 droneChargedAtRightPositionY2(3,D2,2): - droneDY(D2,2)
 + 100 droneIsChargingAtEP(D2,2,3) <= 80
 droneChargedAtRightPositionY2(3,D2,3): - droneDY(D2,3)
 + 100 droneIsChargingAtEP(D2,3,3) <= 80
 droneChargedAtRightPositionY2(3,D2,4): - droneDY(D2,4)
 + 100 droneIsChargingAtEP(D2,4,3) <= 80
 droneChargedAtRightPositionY2(3,D2,5): - droneDY(D2,5)
 + 100 droneIsChargingAtEP(D2,5,3) <= 80
 droneChargedAtRightPositionY2(3,D3,1): - droneDY(D3,1)
 + 100 droneIsChargingAtEP(D3,1,3) <= 80
 droneChargedAtRightPositionY2(3,D3,2): - droneDY(D3,2)
 + 100 droneIsChargingAtEP(D3,2,3) <= 80
 droneChargedAtRightPositionY2(3,D3,3): - droneDY(D3,3)
 + 100 droneIsChargingAtEP(D3,3,3) <= 80
 droneChargedAtRightPositionY2(3,D3,4): - droneDY(D3,4)
 + 100 droneIsChargingAtEP(D3,4,3) <= 80
 droneChargedAtRightPositionY2(3,D3,5): - droneDY(D3,5)
 + 100 droneIsChargingAtEP(D3,5,3) <= 80
 droneChargedAtRightPositionY2(3,D4,1): - droneDY(D4,1)
 + 100 droneIsChargingAtEP(D4,1,3) <= 80
 droneChargedAtRightPositionY2(3,D4,2): - droneDY(D4,2)
 + 100 droneIsChargingAtEP(D4,2,3) <= 80
 droneChargedAtRightPositionY2(3,D4,3): - droneDY(D4,3)
 + 100 droneIsChargingAtEP(D4,3,3) <= 80
 droneChargedAtRightPositionY2(3,D4,4): - droneDY(D4,4)
 + 100 droneIsChargingAtEP(D4,4,3) <= 80
 droneChargedAtRightPositionY2(3,D4,5): - droneDY(D4,5)
 + 100 droneIsChargingAtEP(D4,5,3) <= 80
 droneChargedAtRightPositionY2(3,D5,1): - droneDY(D5,1)
 + 100 droneIsChargingAtEP(D5,1,3) <= 80
 droneChargedAtRightPositionY2(3,D5,2): - droneDY(D5,2)
 + 100 droneIsChargingAtEP(D5,2,3) <= 80
 droneChargedAtRightPositionY2(3,D5,3): - droneDY(D5,3)
 + 100 droneIsChargingAtEP(D5,3,3) <= 80
 droneChargedAtRightPositionY2(3,D5,4): - droneDY(D5,4)
 + 100 droneIsChargingAtEP(D5,4,3) <= 80
 droneChargedAtRightPositionY2(3,D5,5): - droneDY(D5,5)
 + 100 droneIsChargingAtEP(D5,5,3) <= 80
 droneChargedAtRightPositionY2(4,D1,1): - droneDY(D1,1)
 + 100 droneIsChargingAtEP(D1,1,4) <= 90
 droneChargedAtRightPositionY2(4,D1,2): - droneDY(D1,2)
 + 100 droneIsChargingAtEP(D1,2,4) <= 90
 droneChargedAtRightPositionY2(4,D1,3): - droneDY(D1,3)
 + 100 droneIsChargingAtEP(D1,3,4) <= 90
 droneChargedAtRightPositionY2(4,D1,4): - droneDY(D1,4)
 + 100 droneIsChargingAtEP(D1,4,4) <= 90
 droneChargedAtRightPositionY2(4,D1,5): - droneDY(D1,5)
 + 100 droneIsChargingAtEP(D1,5,4) <= 90
 droneChargedAtRightPositionY2(4,D2,1): - droneDY(D2,1)
 + 100 droneIsChargingAtEP(D2,1,4) <= 90
 droneChargedAtRightPositionY2(4,D2,2): - droneDY(D2,2)
 + 100 droneIsChargingAtEP(D2,2,4) <= 90
 droneChargedAtRightPositionY2(4,D2,3): - droneDY(D2,3)
 + 100 droneIsChargingAtEP(D2,3,4) <= 90
 droneChargedAtRightPositionY2(4,D2,4): - droneDY(D2,4)
 + 100 droneIsChargingAtEP(D2,4,4) <= 90
 droneChargedAtRightPositionY2(4,D2,5): - droneDY(D2,5)
 + 100 droneIsChargingAtEP(D2,5,4) <= 90
 droneChargedAtRightPositionY2(4,D3,1): - droneDY(D3,1)
 + 100 droneIsChargingAtEP(D3,1,4) <= 90
 droneChargedAtRightPositionY2(4,D3,2): - droneDY(D3,2)
 + 100 droneIsChargingAtEP(D3,2,4) <= 90
 droneChargedAtRightPositionY2(4,D3,3): - droneDY(D3,3)
 + 100 droneIsChargingAtEP(D3,3,4) <= 90
 droneChargedAtRightPositionY2(4,D3,4): - droneDY(D3,4)
 + 100 droneIsChargingAtEP(D3,4,4) <= 90
 droneChargedAtRightPositionY2(4,D3,5): - droneDY(D3,5)
 + 100 droneIsChargingAtEP(D3,5,4) <= 90
 droneChargedAtRightPositionY2(4,D4,1): - droneDY(D4,1)
 + 100 droneIsChargingAtEP(D4,1,4) <= 90
 droneChargedAtRightPositionY2(4,D4,2): - droneDY(D4,2)
 + 100 droneIsChargingAtEP(D4,2,4) <= 90
 droneChargedAtRightPositionY2(4,D4,3): - droneDY(D4,3)
 + 100 droneIsChargingAtEP(D4,3,4) <= 90
 droneChargedAtRightPositionY2(4,D4,4): - droneDY(D4,4)
 + 100 droneIsChargingAtEP(D4,4,4) <= 90
 droneChargedAtRightPositionY2(4,D4,5): - droneDY(D4,5)
 + 100 droneIsChargingAtEP(D4,5,4) <= 90
 droneChargedAtRightPositionY2(4,D5,1): - droneDY(D5,1)
 + 100 droneIsChargingAtEP(D5,1,4) <= 90
 droneChargedAtRightPositionY2(4,D5,2): - droneDY(D5,2)
 + 100 droneIsChargingAtEP(D5,2,4) <= 90
 droneChargedAtRightPositionY2(4,D5,3): - droneDY(D5,3)
 + 100 droneIsChargingAtEP(D5,3,4) <= 90
 droneChargedAtRightPositionY2(4,D5,4): - droneDY(D5,4)
 + 100 droneIsChargingAtEP(D5,4,4) <= 90
 droneChargedAtRightPositionY2(4,D5,5): - droneDY(D5,5)
 + 100 droneIsChargingAtEP(D5,5,4) <= 90
 calcABSMoveX1(D1,2): - droneDX(D1,1) + droneDX(D1,2) + diffPosX(D1,2)
 >= -0
 calcABSMoveX1(D1,3): - droneDX(D1,2) + droneDX(D1,3) + diffPosX(D1,3)
 >= -0
 calcABSMoveX1(D1,4): - droneDX(D1,3) + droneDX(D1,4) + diffPosX(D1,4)
 >= -0
 calcABSMoveX1(D1,5): - droneDX(D1,4) + droneDX(D1,5) + diffPosX(D1,5)
 >= -0
 calcABSMoveX1(D2,2): - droneDX(D2,1) + droneDX(D2,2) + diffPosX(D2,2)
 >= -0
 calcABSMoveX1(D2,3): - droneDX(D2,2) + droneDX(D2,3) + diffPosX(D2,3)
 >= -0
 calcABSMoveX1(D2,4): - droneDX(D2,3) + droneDX(D2,4) + diffPosX(D2,4)
 >= -0
 calcABSMoveX1(D2,5): - droneDX(D2,4) + droneDX(D2,5) + diffPosX(D2,5)
 >= -0
 calcABSMoveX1(D3,2): - droneDX(D3,1) + droneDX(D3,2) + diffPosX(D3,2)
 >= -0
 calcABSMoveX1(D3,3): - droneDX(D3,2) + droneDX(D3,3) + diffPosX(D3,3)
 >= -0
 calcABSMoveX1(D3,4): - droneDX(D3,3) + droneDX(D3,4) + diffPosX(D3,4)
 >= -0
 calcABSMoveX1(D3,5): - droneDX(D3,4) + droneDX(D3,5) + diffPosX(D3,5)
 >= -0
 calcABSMoveX1(D4,2): - droneDX(D4,1) + droneDX(D4,2) + diffPosX(D4,2)
 >= -0
 calcABSMoveX1(D4,3): - droneDX(D4,2) + droneDX(D4,3) + diffPosX(D4,3)
 >= -0
 calcABSMoveX1(D4,4): - droneDX(D4,3) + droneDX(D4,4) + diffPosX(D4,4)
 >= -0
 calcABSMoveX1(D4,5): - droneDX(D4,4) + droneDX(D4,5) + diffPosX(D4,5)
 >= -0
 calcABSMoveX1(D5,2): - droneDX(D5,1) + droneDX(D5,2) + diffPosX(D5,2)
 >= -0
 calcABSMoveX1(D5,3): - droneDX(D5,2) + droneDX(D5,3) + diffPosX(D5,3)
 >= -0
 calcABSMoveX1(D5,4): - droneDX(D5,3) + droneDX(D5,4) + diffPosX(D5,4)
 >= -0
 calcABSMoveX1(D5,5): - droneDX(D5,4) + droneDX(D5,5) + diffPosX(D5,5)
 >= -0
 calcABSMoveX2(D1,2): - droneDX(D1,1) + droneDX(D1,2) - diffNegX(D1,2)
 <= -0
 calcABSMoveX2(D1,3): - droneDX(D1,2) + droneDX(D1,3) - diffNegX(D1,3)
 <= -0
 calcABSMoveX2(D1,4): - droneDX(D1,3) + droneDX(D1,4) - diffNegX(D1,4)
 <= -0
 calcABSMoveX2(D1,5): - droneDX(D1,4) + droneDX(D1,5) - diffNegX(D1,5)
 <= -0
 calcABSMoveX2(D2,2): - droneDX(D2,1) + droneDX(D2,2) - diffNegX(D2,2)
 <= -0
 calcABSMoveX2(D2,3): - droneDX(D2,2) + droneDX(D2,3) - diffNegX(D2,3)
 <= -0
 calcABSMoveX2(D2,4): - droneDX(D2,3) + droneDX(D2,4) - diffNegX(D2,4)
 <= -0
 calcABSMoveX2(D2,5): - droneDX(D2,4) + droneDX(D2,5) - diffNegX(D2,5)
 <= -0
 calcABSMoveX2(D3,2): - droneDX(D3,1) + droneDX(D3,2) - diffNegX(D3,2)
 <= -0
 calcABSMoveX2(D3,3): - droneDX(D3,2) + droneDX(D3,3) - diffNegX(D3,3)
 <= -0
 calcABSMoveX2(D3,4): - droneDX(D3,3) + droneDX(D3,4) - diffNegX(D3,4)
 <= -0
 calcABSMoveX2(D3,5): - droneDX(D3,4) + droneDX(D3,5) - diffNegX(D3,5)
 <= -0
 calcABSMoveX2(D4,2): - droneDX(D4,1) + droneDX(D4,2) - diffNegX(D4,2)
 <= -0
 calcABSMoveX2(D4,3): - droneDX(D4,2) + droneDX(D4,3) - diffNegX(D4,3)
 <= -0
 calcABSMoveX2(D4,4): - droneDX(D4,3) + droneDX(D4,4) - diffNegX(D4,4)
 <= -0
 calcABSMoveX2(D4,5): - droneDX(D4,4) + droneDX(D4,5) - diffNegX(D4,5)
 <= -0
 calcABSMoveX2(D5,2): - droneDX(D5,1) + droneDX(D5,2) - diffNegX(D5,2)
 <= -0
 calcABSMoveX2(D5,3): - droneDX(D5,2) + droneDX(D5,3) - diffNegX(D5,3)
 <= -0
 calcABSMoveX2(D5,4): - droneDX(D5,3) + droneDX(D5,4) - diffNegX(D5,4)
 <= -0
 calcABSMoveX2(D5,5): - droneDX(D5,4) + droneDX(D5,5) - diffNegX(D5,5)
 <= -0
 calcABSMoveY1(D1,2): - droneDY(D1,1) + droneDY(D1,2) + diffPosY(D1,2)
 >= -0
 calcABSMoveY1(D1,3): - droneDY(D1,2) + droneDY(D1,3) + diffPosY(D1,3)
 >= -0
 calcABSMoveY1(D1,4): - droneDY(D1,3) + droneDY(D1,4) + diffPosY(D1,4)
 >= -0
 calcABSMoveY1(D1,5): - droneDY(D1,4) + droneDY(D1,5) + diffPosY(D1,5)
 >= -0
 calcABSMoveY1(D2,2): - droneDY(D2,1) + droneDY(D2,2) + diffPosY(D2,2)
 >= -0
 calcABSMoveY1(D2,3): - droneDY(D2,2) + droneDY(D2,3) + diffPosY(D2,3)
 >= -0
 calcABSMoveY1(D2,4): - droneDY(D2,3) + droneDY(D2,4) + diffPosY(D2,4)
 >= -0
 calcABSMoveY1(D2,5): - droneDY(D2,4) + droneDY(D2,5) + diffPosY(D2,5)
 >= -0
 calcABSMoveY1(D3,2): - droneDY(D3,1) + droneDY(D3,2) + diffPosY(D3,2)
 >= -0
 calcABSMoveY1(D3,3): - droneDY(D3,2) + droneDY(D3,3) + diffPosY(D3,3)
 >= -0
 calcABSMoveY1(D3,4): - droneDY(D3,3) + droneDY(D3,4) + diffPosY(D3,4)
 >= -0
 calcABSMoveY1(D3,5): - droneDY(D3,4) + droneDY(D3,5) + diffPosY(D3,5)
 >= -0
 calcABSMoveY1(D4,2): - droneDY(D4,1) + droneDY(D4,2) + diffPosY(D4,2)
 >= -0
 calcABSMoveY1(D4,3): - droneDY(D4,2) + droneDY(D4,3) + diffPosY(D4,3)
 >= -0
 calcABSMoveY1(D4,4): - droneDY(D4,3) + droneDY(D4,4) + diffPosY(D4,4)
 >= -0
 calcABSMoveY1(D4,5): - droneDY(D4,4) + droneDY(D4,5) + diffPosY(D4,5)
 >= -0
 calcABSMoveY1(D5,2): - droneDY(D5,1) + droneDY(D5,2) + diffPosY(D5,2)
 >= -0
 calcABSMoveY1(D5,3): - droneDY(D5,2) + droneDY(D5,3) + diffPosY(D5,3)
 >= -0
 calcABSMoveY1(D5,4): - droneDY(D5,3) + droneDY(D5,4) + diffPosY(D5,4)
 >= -0
 calcABSMoveY1(D5,5): - droneDY(D5,4) + droneDY(D5,5) + diffPosY(D5,5)
 >= -0
 calcABSMoveY2(D1,2): - droneDY(D1,1) + droneDY(D1,2) - diffNegY(D1,2)
 <= -0
 calcABSMoveY2(D1,3): - droneDY(D1,2) + droneDY(D1,3) - diffNegY(D1,3)
 <= -0
 calcABSMoveY2(D1,4): - droneDY(D1,3) + droneDY(D1,4) - diffNegY(D1,4)
 <= -0
 calcABSMoveY2(D1,5): - droneDY(D1,4) + droneDY(D1,5) - diffNegY(D1,5)
 <= -0
 calcABSMoveY2(D2,2): - droneDY(D2,1) + droneDY(D2,2) - diffNegY(D2,2)
 <= -0
 calcABSMoveY2(D2,3): - droneDY(D2,2) + droneDY(D2,3) - diffNegY(D2,3)
 <= -0
 calcABSMoveY2(D2,4): - droneDY(D2,3) + droneDY(D2,4) - diffNegY(D2,4)
 <= -0
 calcABSMoveY2(D2,5): - droneDY(D2,4) + droneDY(D2,5) - diffNegY(D2,5)
 <= -0
 calcABSMoveY2(D3,2): - droneDY(D3,1) + droneDY(D3,2) - diffNegY(D3,2)
 <= -0
 calcABSMoveY2(D3,3): - droneDY(D3,2) + droneDY(D3,3) - diffNegY(D3,3)
 <= -0
 calcABSMoveY2(D3,4): - droneDY(D3,3) + droneDY(D3,4) - diffNegY(D3,4)
 <= -0
 calcABSMoveY2(D3,5): - droneDY(D3,4) + droneDY(D3,5) - diffNegY(D3,5)
 <= -0
 calcABSMoveY2(D4,2): - droneDY(D4,1) + droneDY(D4,2) - diffNegY(D4,2)
 <= -0
 calcABSMoveY2(D4,3): - droneDY(D4,2) + droneDY(D4,3) - diffNegY(D4,3)
 <= -0
 calcABSMoveY2(D4,4): - droneDY(D4,3) + droneDY(D4,4) - diffNegY(D4,4)
 <= -0
 calcABSMoveY2(D4,5): - droneDY(D4,4) + droneDY(D4,5) - diffNegY(D4,5)
 <= -0
 calcABSMoveY2(D5,2): - droneDY(D5,1) + droneDY(D5,2) - diffNegY(D5,2)
 <= -0
 calcABSMoveY2(D5,3): - droneDY(D5,2) + droneDY(D5,3) - diffNegY(D5,3)
 <= -0
 calcABSMoveY2(D5,4): - droneDY(D5,3) + droneDY(D5,4) - diffNegY(D5,4)
 <= -0
 calcABSMoveY2(D5,5): - droneDY(D5,4) + droneDY(D5,5) - diffNegY(D5,5)
 <= -0
 calcDronesSpeed(D1,2): - 2.4 diffPosX(D1,2) - 1.2 diffPosY(D1,2)
 - 1.2 diffNegY(D1,2) + droneSpeed(D1,2) = -0
 calcDronesSpeed(D1,3): - 2.4 diffPosX(D1,3) - 1.2 diffPosY(D1,3)
 - 1.2 diffNegY(D1,3) + droneSpeed(D1,3) = -0
 calcDronesSpeed(D1,4): - 2.4 diffPosX(D1,4) - 1.2 diffPosY(D1,4)
 - 1.2 diffNegY(D1,4) + droneSpeed(D1,4) = -0
 calcDronesSpeed(D1,5): - 2.4 diffPosX(D1,5) - 1.2 diffPosY(D1,5)
 - 1.2 diffNegY(D1,5) + droneSpeed(D1,5) = -0
 calcDronesSpeed(D2,2): - 2.4 diffPosX(D2,2) - 1.2 diffPosY(D2,2)
 - 1.2 diffNegY(D2,2) + droneSpeed(D2,2) = -0
 calcDronesSpeed(D2,3): - 2.4 diffPosX(D2,3) - 1.2 diffPosY(D2,3)
 - 1.2 diffNegY(D2,3) + droneSpeed(D2,3) = -0
 calcDronesSpeed(D2,4): - 2.4 diffPosX(D2,4) - 1.2 diffPosY(D2,4)
 - 1.2 diffNegY(D2,4) + droneSpeed(D2,4) = -0
 calcDronesSpeed(D2,5): - 2.4 diffPosX(D2,5) - 1.2 diffPosY(D2,5)
 - 1.2 diffNegY(D2,5) + droneSpeed(D2,5) = -0
 calcDronesSpeed(D3,2): - 2.4 diffPosX(D3,2) - 1.2 diffPosY(D3,2)
 - 1.2 diffNegY(D3,2) + droneSpeed(D3,2) = -0
 calcDronesSpeed(D3,3): - 2.4 diffPosX(D3,3) - 1.2 diffPosY(D3,3)
 - 1.2 diffNegY(D3,3) + droneSpeed(D3,3) = -0
 calcDronesSpeed(D3,4): - 2.4 diffPosX(D3,4) - 1.2 diffPosY(D3,4)
 - 1.2 diffNegY(D3,4) + droneSpeed(D3,4) = -0
 calcDronesSpeed(D3,5): - 2.4 diffPosX(D3,5) - 1.2 diffPosY(D3,5)
 - 1.2 diffNegY(D3,5) + droneSpeed(D3,5) = -0
 calcDronesSpeed(D4,2): - 2.4 diffPosX(D4,2) - 1.2 diffPosY(D4,2)
 - 1.2 diffNegY(D4,2) + droneSpeed(D4,2) = -0
 calcDronesSpeed(D4,3): - 2.4 diffPosX(D4,3) - 1.2 diffPosY(D4,3)
 - 1.2 diffNegY(D4,3) + droneSpeed(D4,3) = -0
 calcDronesSpeed(D4,4): - 2.4 diffPosX(D4,4) - 1.2 diffPosY(D4,4)
 - 1.2 diffNegY(D4,4) + droneSpeed(D4,4) = -0
 calcDronesSpeed(D4,5): - 2.4 diffPosX(D4,5) - 1.2 diffPosY(D4,5)
 - 1.2 diffNegY(D4,5) + droneSpeed(D4,5) = -0
 calcDronesSpeed(D5,2): - 2.4 diffPosX(D5,2) - 1.2 diffPosY(D5,2)
 - 1.2 diffNegY(D5,2) + droneSpeed(D5,2) = -0
 calcDronesSpeed(D5,3): - 2.4 diffPosX(D5,3) - 1.2 diffPosY(D5,3)
 - 1.2 diffNegY(D5,3) + droneSpeed(D5,3) = -0
 calcDronesSpeed(D5,4): - 2.4 diffPosX(D5,4) - 1.2 diffPosY(D5,4)
 - 1.2 diffNegY(D5,4) + droneSpeed(D5,4) = -0
 calcDronesSpeed(D5,5): - 2.4 diffPosX(D5,5) - 1.2 diffPosY(D5,5)
 - 1.2 diffNegY(D5,5) + droneSpeed(D5,5) = -0
 respectDroneMaxSpeed(D1,2): + droneSpeed(D1,2) <= 70
 respectDroneMaxSpeed(D1,3): + droneSpeed(D1,3) <= 70
 respectDroneMaxSpeed(D1,4): + droneSpeed(D1,4) <= 70
 respectDroneMaxSpeed(D1,5): + droneSpeed(D1,5) <= 70
 respectDroneMaxSpeed(D2,2): + droneSpeed(D2,2) <= 80
 respectDroneMaxSpeed(D2,3): + droneSpeed(D2,3) <= 80
 respectDroneMaxSpeed(D2,4): + droneSpeed(D2,4) <= 80
 respectDroneMaxSpeed(D2,5): + droneSpeed(D2,5) <= 80
 respectDroneMaxSpeed(D3,2): + droneSpeed(D3,2) <= 40
 respectDroneMaxSpeed(D3,3): + droneSpeed(D3,3) <= 40
 respectDroneMaxSpeed(D3,4): + droneSpeed(D3,4) <= 40
 respectDroneMaxSpeed(D3,5): + droneSpeed(D3,5) <= 40
 respectDroneMaxSpeed(D4,2): + droneSpeed(D4,2) <= 30
 respectDroneMaxSpeed(D4,3): + droneSpeed(D4,3) <= 30
 respectDroneMaxSpeed(D4,4): + droneSpeed(D4,4) <= 30
 respectDroneMaxSpeed(D4,5): + droneSpeed(D4,5) <= 30
 respectDroneMaxSpeed(D5,2): + droneSpeed(D5,2) <= 20
 respectDroneMaxSpeed(D5,3): + droneSpeed(D5,3) <= 20
 respectDroneMaxSpeed(D5,4): + droneSpeed(D5,4) <= 20
 respectDroneMaxSpeed(D5,5): + droneSpeed(D5,5) <= 20

Bounds
 0 <= droneDX(D1,1) <= 100
 0 <= droneDX(D2,1) <= 100
 0 <= droneDX(D3,1) <= 100
 0 <= droneDX(D4,1) <= 100
 0 <= droneDX(D5,1) <= 100
 0 <= droneDX(D1,2) <= 100
 0 <= droneDX(D1,3) <= 100
 0 <= droneDX(D1,4) <= 100
 0 <= droneDX(D1,5) <= 100
 0 <= droneDX(D2,2) <= 100
 0 <= droneDX(D2,3) <= 100
 0 <= droneDX(D2,4) <= 100
 0 <= droneDX(D2,5) <= 100
 0 <= droneDX(D3,2) <= 100
 0 <= droneDX(D3,3) <= 100
 0 <= droneDX(D3,4) <= 100
 0 <= droneDX(D3,5) <= 100
 0 <= droneDX(D4,2) <= 100
 0 <= droneDX(D4,3) <= 100
 0 <= droneDX(D4,4) <= 100
 0 <= droneDX(D4,5) <= 100
 0 <= droneDX(D5,2) <= 100
 0 <= droneDX(D5,3) <= 100
 0 <= droneDX(D5,4) <= 100
 0 <= droneDX(D5,5) <= 100
 0 <= droneDY(D1,1) <= 100
 0 <= droneDY(D2,1) <= 100
 0 <= droneDY(D3,1) <= 100
 0 <= droneDY(D4,1) <= 100
 0 <= droneDY(D5,1) <= 100
 0 <= droneDY(D1,2) <= 100
 0 <= droneDY(D1,3) <= 100
 0 <= droneDY(D1,4) <= 100
 0 <= droneDY(D1,5) <= 100
 0 <= droneDY(D2,2) <= 100
 0 <= droneDY(D2,3) <= 100
 0 <= droneDY(D2,4) <= 100
 0 <= droneDY(D2,5) <= 100
 0 <= droneDY(D3,2) <= 100
 0 <= droneDY(D3,3) <= 100
 0 <= droneDY(D3,4) <= 100
 0 <= droneDY(D3,5) <= 100
 0 <= droneDY(D4,2) <= 100
 0 <= droneDY(D4,3) <= 100
 0 <= droneDY(D4,4) <= 100
 0 <= droneDY(D4,5) <= 100
 0 <= droneDY(D5,2) <= 100
 0 <= droneDY(D5,3) <= 100
 0 <= droneDY(D5,4) <= 100
 0 <= droneDY(D5,5) <= 100
 0 <= droneCollected(D1,P1,2) <= 1
 0 <= droneCollected(D1,P2,2) <= 1
 0 <= droneCollected(D1,P3,2) <= 1
 0 <= droneCollected(D1,P4,2) <= 1
 0 <= droneCollected(D1,P5,2) <= 1
 0 <= droneCollected(D1,P6,2) <= 1
 0 <= droneCollected(D1,P1,3) <= 1
 0 <= droneCollected(D1,P2,3) <= 1
 0 <= droneCollected(D1,P3,3) <= 1
 0 <= droneCollected(D1,P4,3) <= 1
 0 <= droneCollected(D1,P5,3) <= 1
 0 <= droneCollected(D1,P6,3) <= 1
 0 <= droneCollected(D1,P1,4) <= 1
 0 <= droneCollected(D1,P2,4) <= 1
 0 <= droneCollected(D1,P3,4) <= 1
 0 <= droneCollected(D1,P4,4) <= 1
 0 <= droneCollected(D1,P5,4) <= 1
 0 <= droneCollected(D1,P6,4) <= 1
 0 <= droneCollected(D1,P1,5) <= 1
 0 <= droneCollected(D1,P2,5) <= 1
 0 <= droneCollected(D1,P3,5) <= 1
 0 <= droneCollected(D1,P4,5) <= 1
 0 <= droneCollected(D1,P5,5) <= 1
 0 <= droneCollected(D1,P6,5) <= 1
 0 <= droneCollected(D2,P1,2) <= 1
 0 <= droneCollected(D2,P2,2) <= 1
 0 <= droneCollected(D2,P3,2) <= 1
 0 <= droneCollected(D2,P4,2) <= 1
 0 <= droneCollected(D2,P5,2) <= 1
 0 <= droneCollected(D2,P6,2) <= 1
 0 <= droneCollected(D2,P1,3) <= 1
 0 <= droneCollected(D2,P2,3) <= 1
 0 <= droneCollected(D2,P3,3) <= 1
 0 <= droneCollected(D2,P4,3) <= 1
 0 <= droneCollected(D2,P5,3) <= 1
 0 <= droneCollected(D2,P6,3) <= 1
 0 <= droneCollected(D2,P1,4) <= 1
 0 <= droneCollected(D2,P2,4) <= 1
 0 <= droneCollected(D2,P3,4) <= 1
 0 <= droneCollected(D2,P4,4) <= 1
 0 <= droneCollected(D2,P5,4) <= 1
 0 <= droneCollected(D2,P6,4) <= 1
 0 <= droneCollected(D2,P1,5) <= 1
 0 <= droneCollected(D2,P2,5) <= 1
 0 <= droneCollected(D2,P3,5) <= 1
 0 <= droneCollected(D2,P4,5) <= 1
 0 <= droneCollected(D2,P5,5) <= 1
 0 <= droneCollected(D2,P6,5) <= 1
 0 <= droneCollected(D3,P1,2) <= 1
 0 <= droneCollected(D3,P2,2) <= 1
 0 <= droneCollected(D3,P3,2) <= 1
 0 <= droneCollected(D3,P4,2) <= 1
 0 <= droneCollected(D3,P5,2) <= 1
 0 <= droneCollected(D3,P6,2) <= 1
 0 <= droneCollected(D3,P1,3) <= 1
 0 <= droneCollected(D3,P2,3) <= 1
 0 <= droneCollected(D3,P3,3) <= 1
 0 <= droneCollected(D3,P4,3) <= 1
 0 <= droneCollected(D3,P5,3) <= 1
 0 <= droneCollected(D3,P6,3) <= 1
 0 <= droneCollected(D3,P1,4) <= 1
 0 <= droneCollected(D3,P2,4) <= 1
 0 <= droneCollected(D3,P3,4) <= 1
 0 <= droneCollected(D3,P4,4) <= 1
 0 <= droneCollected(D3,P5,4) <= 1
 0 <= droneCollected(D3,P6,4) <= 1
 0 <= droneCollected(D3,P1,5) <= 1
 0 <= droneCollected(D3,P2,5) <= 1
 0 <= droneCollected(D3,P3,5) <= 1
 0 <= droneCollected(D3,P4,5) <= 1
 0 <= droneCollected(D3,P5,5) <= 1
 0 <= droneCollected(D3,P6,5) <= 1
 0 <= droneCollected(D4,P1,2) <= 1
 0 <= droneCollected(D4,P2,2) <= 1
 0 <= droneCollected(D4,P3,2) <= 1
 0 <= droneCollected(D4,P4,2) <= 1
 0 <= droneCollected(D4,P5,2) <= 1
 0 <= droneCollected(D4,P6,2) <= 1
 0 <= droneCollected(D4,P1,3) <= 1
 0 <= droneCollected(D4,P2,3) <= 1
 0 <= droneCollected(D4,P3,3) <= 1
 0 <= droneCollected(D4,P4,3) <= 1
 0 <= droneCollected(D4,P5,3) <= 1
 0 <= droneCollected(D4,P6,3) <= 1
 0 <= droneCollected(D4,P1,4) <= 1
 0 <= droneCollected(D4,P2,4) <= 1
 0 <= droneCollected(D4,P3,4) <= 1
 0 <= droneCollected(D4,P4,4) <= 1
 0 <= droneCollected(D4,P5,4) <= 1
 0 <= droneCollected(D4,P6,4) <= 1
 0 <= droneCollected(D4,P1,5) <= 1
 0 <= droneCollected(D4,P2,5) <= 1
 0 <= droneCollected(D4,P3,5) <= 1
 0 <= droneCollected(D4,P4,5) <= 1
 0 <= droneCollected(D4,P5,5) <= 1
 0 <= droneCollected(D4,P6,5) <= 1
 0 <= droneCollected(D5,P1,2) <= 1
 0 <= droneCollected(D5,P2,2) <= 1
 0 <= droneCollected(D5,P3,2) <= 1
 0 <= droneCollected(D5,P4,2) <= 1
 0 <= droneCollected(D5,P5,2) <= 1
 0 <= droneCollected(D5,P6,2) <= 1
 0 <= droneCollected(D5,P1,3) <= 1
 0 <= droneCollected(D5,P2,3) <= 1
 0 <= droneCollected(D5,P3,3) <= 1
 0 <= droneCollected(D5,P4,3) <= 1
 0 <= droneCollected(D5,P5,3) <= 1
 0 <= droneCollected(D5,P6,3) <= 1
 0 <= droneCollected(D5,P1,4) <= 1
 0 <= droneCollected(D5,P2,4) <= 1
 0 <= droneCollected(D5,P3,4) <= 1
 0 <= droneCollected(D5,P4,4) <= 1
 0 <= droneCollected(D5,P5,4) <= 1
 0 <= droneCollected(D5,P6,4) <= 1
 0 <= droneCollected(D5,P1,5) <= 1
 0 <= droneCollected(D5,P2,5) <= 1
 0 <= droneCollected(D5,P3,5) <= 1
 0 <= droneCollected(D5,P4,5) <= 1
 0 <= droneCollected(D5,P5,5) <= 1
 0 <= droneCollected(D5,P6,5) <= 1
 0 <= droneCollected(D1,P1,1) <= 1
 0 <= droneCollected(D2,P1,1) <= 1
 0 <= droneCollected(D3,P1,1) <= 1
 0 <= droneCollected(D4,P1,1) <= 1
 0 <= droneCollected(D5,P1,1) <= 1
 0 <= droneCollected(D1,P2,1) <= 1
 0 <= droneCollected(D2,P2,1) <= 1
 0 <= droneCollected(D3,P2,1) <= 1
 0 <= droneCollected(D4,P2,1) <= 1
 0 <= droneCollected(D5,P2,1) <= 1
 0 <= droneCollected(D1,P3,1) <= 1
 0 <= droneCollected(D2,P3,1) <= 1
 0 <= droneCollected(D3,P3,1) <= 1
 0 <= droneCollected(D4,P3,1) <= 1
 0 <= droneCollected(D5,P3,1) <= 1
 0 <= droneCollected(D1,P4,1) <= 1
 0 <= droneCollected(D2,P4,1) <= 1
 0 <= droneCollected(D3,P4,1) <= 1
 0 <= droneCollected(D4,P4,1) <= 1
 0 <= droneCollected(D5,P4,1) <= 1
 0 <= droneCollected(D1,P5,1) <= 1
 0 <= droneCollected(D2,P5,1) <= 1
 0 <= droneCollected(D3,P5,1) <= 1
 0 <= droneCollected(D4,P5,1) <= 1
 0 <= droneCollected(D5,P5,1) <= 1
 0 <= droneCollected(D1,P6,1) <= 1
 0 <= droneCollected(D2,P6,1) <= 1
 0 <= droneCollected(D3,P6,1) <= 1
 0 <= droneCollected(D4,P6,1) <= 1
 0 <= droneCollected(D5,P6,1) <= 1
 0 <= droneDelivered(D1,P1,2) <= 1
 0 <= droneDelivered(D1,P2,2) <= 1
 0 <= droneDelivered(D1,P3,2) <= 1
 0 <= droneDelivered(D1,P4,2) <= 1
 0 <= droneDelivered(D1,P5,2) <= 1
 0 <= droneDelivered(D1,P6,2) <= 1
 0 <= droneDelivered(D1,P1,3) <= 1
 0 <= droneDelivered(D1,P2,3) <= 1
 0 <= droneDelivered(D1,P3,3) <= 1
 0 <= droneDelivered(D1,P4,3) <= 1
 0 <= droneDelivered(D1,P5,3) <= 1
 0 <= droneDelivered(D1,P6,3) <= 1
 0 <= droneDelivered(D1,P1,4) <= 1
 0 <= droneDelivered(D1,P2,4) <= 1
 0 <= droneDelivered(D1,P3,4) <= 1
 0 <= droneDelivered(D1,P4,4) <= 1
 0 <= droneDelivered(D1,P5,4) <= 1
 0 <= droneDelivered(D1,P6,4) <= 1
 0 <= droneDelivered(D1,P1,5) <= 1
 0 <= droneDelivered(D1,P2,5) <= 1
 0 <= droneDelivered(D1,P3,5) <= 1
 0 <= droneDelivered(D1,P4,5) <= 1
 0 <= droneDelivered(D1,P5,5) <= 1
 0 <= droneDelivered(D1,P6,5) <= 1
 0 <= droneDelivered(D2,P1,2) <= 1
 0 <= droneDelivered(D2,P2,2) <= 1
 0 <= droneDelivered(D2,P3,2) <= 1
 0 <= droneDelivered(D2,P4,2) <= 1
 0 <= droneDelivered(D2,P5,2) <= 1
 0 <= droneDelivered(D2,P6,2) <= 1
 0 <= droneDelivered(D2,P1,3) <= 1
 0 <= droneDelivered(D2,P2,3) <= 1
 0 <= droneDelivered(D2,P3,3) <= 1
 0 <= droneDelivered(D2,P4,3) <= 1
 0 <= droneDelivered(D2,P5,3) <= 1
 0 <= droneDelivered(D2,P6,3) <= 1
 0 <= droneDelivered(D2,P1,4) <= 1
 0 <= droneDelivered(D2,P2,4) <= 1
 0 <= droneDelivered(D2,P3,4) <= 1
 0 <= droneDelivered(D2,P4,4) <= 1
 0 <= droneDelivered(D2,P5,4) <= 1
 0 <= droneDelivered(D2,P6,4) <= 1
 0 <= droneDelivered(D2,P1,5) <= 1
 0 <= droneDelivered(D2,P2,5) <= 1
 0 <= droneDelivered(D2,P3,5) <= 1
 0 <= droneDelivered(D2,P4,5) <= 1
 0 <= droneDelivered(D2,P5,5) <= 1
 0 <= droneDelivered(D2,P6,5) <= 1
 0 <= droneDelivered(D3,P1,2) <= 1
 0 <= droneDelivered(D3,P2,2) <= 1
 0 <= droneDelivered(D3,P3,2) <= 1
 0 <= droneDelivered(D3,P4,2) <= 1
 0 <= droneDelivered(D3,P5,2) <= 1
 0 <= droneDelivered(D3,P6,2) <= 1
 0 <= droneDelivered(D3,P1,3) <= 1
 0 <= droneDelivered(D3,P2,3) <= 1
 0 <= droneDelivered(D3,P3,3) <= 1
 0 <= droneDelivered(D3,P4,3) <= 1
 0 <= droneDelivered(D3,P5,3) <= 1
 0 <= droneDelivered(D3,P6,3) <= 1
 0 <= droneDelivered(D3,P1,4) <= 1
 0 <= droneDelivered(D3,P2,4) <= 1
 0 <= droneDelivered(D3,P3,4) <= 1
 0 <= droneDelivered(D3,P4,4) <= 1
 0 <= droneDelivered(D3,P5,4) <= 1
 0 <= droneDelivered(D3,P6,4) <= 1
 0 <= droneDelivered(D3,P1,5) <= 1
 0 <= droneDelivered(D3,P2,5) <= 1
 0 <= droneDelivered(D3,P3,5) <= 1
 0 <= droneDelivered(D3,P4,5) <= 1
 0 <= droneDelivered(D3,P5,5) <= 1
 0 <= droneDelivered(D3,P6,5) <= 1
 0 <= droneDelivered(D4,P1,2) <= 1
 0 <= droneDelivered(D4,P2,2) <= 1
 0 <= droneDelivered(D4,P3,2) <= 1
 0 <= droneDelivered(D4,P4,2) <= 1
 0 <= droneDelivered(D4,P5,2) <= 1
 0 <= droneDelivered(D4,P6,2) <= 1
 0 <= droneDelivered(D4,P1,3) <= 1
 0 <= droneDelivered(D4,P2,3) <= 1
 0 <= droneDelivered(D4,P3,3) <= 1
 0 <= droneDelivered(D4,P4,3) <= 1
 0 <= droneDelivered(D4,P5,3) <= 1
 0 <= droneDelivered(D4,P6,3) <= 1
 0 <= droneDelivered(D4,P1,4) <= 1
 0 <= droneDelivered(D4,P2,4) <= 1
 0 <= droneDelivered(D4,P3,4) <= 1
 0 <= droneDelivered(D4,P4,4) <= 1
 0 <= droneDelivered(D4,P5,4) <= 1
 0 <= droneDelivered(D4,P6,4) <= 1
 0 <= droneDelivered(D4,P1,5) <= 1
 0 <= droneDelivered(D4,P2,5) <= 1
 0 <= droneDelivered(D4,P3,5) <= 1
 0 <= droneDelivered(D4,P4,5) <= 1
 0 <= droneDelivered(D4,P5,5) <= 1
 0 <= droneDelivered(D4,P6,5) <= 1
 0 <= droneDelivered(D5,P1,2) <= 1
 0 <= droneDelivered(D5,P2,2) <= 1
 0 <= droneDelivered(D5,P3,2) <= 1
 0 <= droneDelivered(D5,P4,2) <= 1
 0 <= droneDelivered(D5,P5,2) <= 1
 0 <= droneDelivered(D5,P6,2) <= 1
 0 <= droneDelivered(D5,P1,3) <= 1
 0 <= droneDelivered(D5,P2,3) <= 1
 0 <= droneDelivered(D5,P3,3) <= 1
 0 <= droneDelivered(D5,P4,3) <= 1
 0 <= droneDelivered(D5,P5,3) <= 1
 0 <= droneDelivered(D5,P6,3) <= 1
 0 <= droneDelivered(D5,P1,4) <= 1
 0 <= droneDelivered(D5,P2,4) <= 1
 0 <= droneDelivered(D5,P3,4) <= 1
 0 <= droneDelivered(D5,P4,4) <= 1
 0 <= droneDelivered(D5,P5,4) <= 1
 0 <= droneDelivered(D5,P6,4) <= 1
 0 <= droneDelivered(D5,P1,5) <= 1
 0 <= droneDelivered(D5,P2,5) <= 1
 0 <= droneDelivered(D5,P3,5) <= 1
 0 <= droneDelivered(D5,P4,5) <= 1
 0 <= droneDelivered(D5,P5,5) <= 1
 0 <= droneDelivered(D5,P6,5) <= 1
 0 <= droneDelivered(D1,P1,1) <= 1
 0 <= droneDelivered(D2,P1,1) <= 1
 0 <= droneDelivered(D3,P1,1) <= 1
 0 <= droneDelivered(D4,P1,1) <= 1
 0 <= droneDelivered(D5,P1,1) <= 1
 0 <= droneDelivered(D1,P2,1) <= 1
 0 <= droneDelivered(D2,P2,1) <= 1
 0 <= droneDelivered(D3,P2,1) <= 1
 0 <= droneDelivered(D4,P2,1) <= 1
 0 <= droneDelivered(D5,P2,1) <= 1
 0 <= droneDelivered(D1,P3,1) <= 1
 0 <= droneDelivered(D2,P3,1) <= 1
 0 <= droneDelivered(D3,P3,1) <= 1
 0 <= droneDelivered(D4,P3,1) <= 1
 0 <= droneDelivered(D5,P3,1) <= 1
 0 <= droneDelivered(D1,P4,1) <= 1
 0 <= droneDelivered(D2,P4,1) <= 1
 0 <= droneDelivered(D3,P4,1) <= 1
 0 <= droneDelivered(D4,P4,1) <= 1
 0 <= droneDelivered(D5,P4,1) <= 1
 0 <= droneDelivered(D1,P5,1) <= 1
 0 <= droneDelivered(D2,P5,1) <= 1
 0 <= droneDelivered(D3,P5,1) <= 1
 0 <= droneDelivered(D4,P5,1) <= 1
 0 <= droneDelivered(D5,P5,1) <= 1
 0 <= droneDelivered(D1,P6,1) <= 1
 0 <= droneDelivered(D2,P6,1) <= 1
 0 <= droneDelivered(D3,P6,1) <= 1
 0 <= droneDelivered(D4,P6,1) <= 1
 0 <= droneDelivered(D5,P6,1) <= 1
 0 <= droneIsDelivering(D1,P1,2) <= 1
 0 <= droneIsDelivering(D1,P1,1) <= 1
 0 <= droneIsDelivering(D1,P1,3) <= 1
 0 <= droneIsDelivering(D1,P1,4) <= 1
 0 <= droneIsDelivering(D1,P1,5) <= 1
 0 <= droneIsDelivering(D2,P1,2) <= 1
 0 <= droneIsDelivering(D2,P1,1) <= 1
 0 <= droneIsDelivering(D2,P1,3) <= 1
 0 <= droneIsDelivering(D2,P1,4) <= 1
 0 <= droneIsDelivering(D2,P1,5) <= 1
 0 <= droneIsDelivering(D3,P1,2) <= 1
 0 <= droneIsDelivering(D3,P1,1) <= 1
 0 <= droneIsDelivering(D3,P1,3) <= 1
 0 <= droneIsDelivering(D3,P1,4) <= 1
 0 <= droneIsDelivering(D3,P1,5) <= 1
 0 <= droneIsDelivering(D4,P1,2) <= 1
 0 <= droneIsDelivering(D4,P1,1) <= 1
 0 <= droneIsDelivering(D4,P1,3) <= 1
 0 <= droneIsDelivering(D4,P1,4) <= 1
 0 <= droneIsDelivering(D4,P1,5) <= 1
 0 <= droneIsDelivering(D5,P1,2) <= 1
 0 <= droneIsDelivering(D5,P1,1) <= 1
 0 <= droneIsDelivering(D5,P1,3) <= 1
 0 <= droneIsDelivering(D5,P1,4) <= 1
 0 <= droneIsDelivering(D5,P1,5) <= 1
 0 <= droneIsDelivering(D1,P2,2) <= 1
 0 <= droneIsDelivering(D1,P2,1) <= 1
 0 <= droneIsDelivering(D1,P2,3) <= 1
 0 <= droneIsDelivering(D1,P2,4) <= 1
 0 <= droneIsDelivering(D1,P2,5) <= 1
 0 <= droneIsDelivering(D2,P2,2) <= 1
 0 <= droneIsDelivering(D2,P2,1) <= 1
 0 <= droneIsDelivering(D2,P2,3) <= 1
 0 <= droneIsDelivering(D2,P2,4) <= 1
 0 <= droneIsDelivering(D2,P2,5) <= 1
 0 <= droneIsDelivering(D3,P2,2) <= 1
 0 <= droneIsDelivering(D3,P2,1) <= 1
 0 <= droneIsDelivering(D3,P2,3) <= 1
 0 <= droneIsDelivering(D3,P2,4) <= 1
 0 <= droneIsDelivering(D3,P2,5) <= 1
 0 <= droneIsDelivering(D4,P2,2) <= 1
 0 <= droneIsDelivering(D4,P2,1) <= 1
 0 <= droneIsDelivering(D4,P2,3) <= 1
 0 <= droneIsDelivering(D4,P2,4) <= 1
 0 <= droneIsDelivering(D4,P2,5) <= 1
 0 <= droneIsDelivering(D5,P2,2) <= 1
 0 <= droneIsDelivering(D5,P2,1) <= 1
 0 <= droneIsDelivering(D5,P2,3) <= 1
 0 <= droneIsDelivering(D5,P2,4) <= 1
 0 <= droneIsDelivering(D5,P2,5) <= 1
 0 <= droneIsDelivering(D1,P3,2) <= 1
 0 <= droneIsDelivering(D1,P3,1) <= 1
 0 <= droneIsDelivering(D1,P3,3) <= 1
 0 <= droneIsDelivering(D1,P3,4) <= 1
 0 <= droneIsDelivering(D1,P3,5) <= 1
 0 <= droneIsDelivering(D2,P3,2) <= 1
 0 <= droneIsDelivering(D2,P3,1) <= 1
 0 <= droneIsDelivering(D2,P3,3) <= 1
 0 <= droneIsDelivering(D2,P3,4) <= 1
 0 <= droneIsDelivering(D2,P3,5) <= 1
 0 <= droneIsDelivering(D3,P3,2) <= 1
 0 <= droneIsDelivering(D3,P3,1) <= 1
 0 <= droneIsDelivering(D3,P3,3) <= 1
 0 <= droneIsDelivering(D3,P3,4) <= 1
 0 <= droneIsDelivering(D3,P3,5) <= 1
 0 <= droneIsDelivering(D4,P3,2) <= 1
 0 <= droneIsDelivering(D4,P3,1) <= 1
 0 <= droneIsDelivering(D4,P3,3) <= 1
 0 <= droneIsDelivering(D4,P3,4) <= 1
 0 <= droneIsDelivering(D4,P3,5) <= 1
 0 <= droneIsDelivering(D5,P3,2) <= 1
 0 <= droneIsDelivering(D5,P3,1) <= 1
 0 <= droneIsDelivering(D5,P3,3) <= 1
 0 <= droneIsDelivering(D5,P3,4) <= 1
 0 <= droneIsDelivering(D5,P3,5) <= 1
 0 <= droneIsDelivering(D1,P4,2) <= 1
 0 <= droneIsDelivering(D1,P4,1) <= 1
 0 <= droneIsDelivering(D1,P4,3) <= 1
 0 <= droneIsDelivering(D1,P4,4) <= 1
 0 <= droneIsDelivering(D1,P4,5) <= 1
 0 <= droneIsDelivering(D2,P4,2) <= 1
 0 <= droneIsDelivering(D2,P4,1) <= 1
 0 <= droneIsDelivering(D2,P4,3) <= 1
 0 <= droneIsDelivering(D2,P4,4) <= 1
 0 <= droneIsDelivering(D2,P4,5) <= 1
 0 <= droneIsDelivering(D3,P4,2) <= 1
 0 <= droneIsDelivering(D3,P4,1) <= 1
 0 <= droneIsDelivering(D3,P4,3) <= 1
 0 <= droneIsDelivering(D3,P4,4) <= 1
 0 <= droneIsDelivering(D3,P4,5) <= 1
 0 <= droneIsDelivering(D4,P4,2) <= 1
 0 <= droneIsDelivering(D4,P4,1) <= 1
 0 <= droneIsDelivering(D4,P4,3) <= 1
 0 <= droneIsDelivering(D4,P4,4) <= 1
 0 <= droneIsDelivering(D4,P4,5) <= 1
 0 <= droneIsDelivering(D5,P4,2) <= 1
 0 <= droneIsDelivering(D5,P4,1) <= 1
 0 <= droneIsDelivering(D5,P4,3) <= 1
 0 <= droneIsDelivering(D5,P4,4) <= 1
 0 <= droneIsDelivering(D5,P4,5) <= 1
 0 <= droneIsDelivering(D1,P5,2) <= 1
 0 <= droneIsDelivering(D1,P5,1) <= 1
 0 <= droneIsDelivering(D1,P5,3) <= 1
 0 <= droneIsDelivering(D1,P5,4) <= 1
 0 <= droneIsDelivering(D1,P5,5) <= 1
 0 <= droneIsDelivering(D2,P5,2) <= 1
 0 <= droneIsDelivering(D2,P5,1) <= 1
 0 <= droneIsDelivering(D2,P5,3) <= 1
 0 <= droneIsDelivering(D2,P5,4) <= 1
 0 <= droneIsDelivering(D2,P5,5) <= 1
 0 <= droneIsDelivering(D3,P5,2) <= 1
 0 <= droneIsDelivering(D3,P5,1) <= 1
 0 <= droneIsDelivering(D3,P5,3) <= 1
 0 <= droneIsDelivering(D3,P5,4) <= 1
 0 <= droneIsDelivering(D3,P5,5) <= 1
 0 <= droneIsDelivering(D4,P5,2) <= 1
 0 <= droneIsDelivering(D4,P5,1) <= 1
 0 <= droneIsDelivering(D4,P5,3) <= 1
 0 <= droneIsDelivering(D4,P5,4) <= 1
 0 <= droneIsDelivering(D4,P5,5) <= 1
 0 <= droneIsDelivering(D5,P5,2) <= 1
 0 <= droneIsDelivering(D5,P5,1) <= 1
 0 <= droneIsDelivering(D5,P5,3) <= 1
 0 <= droneIsDelivering(D5,P5,4) <= 1
 0 <= droneIsDelivering(D5,P5,5) <= 1
 0 <= droneIsDelivering(D1,P6,2) <= 1
 0 <= droneIsDelivering(D1,P6,1) <= 1
 0 <= droneIsDelivering(D1,P6,3) <= 1
 0 <= droneIsDelivering(D1,P6,4) <= 1
 0 <= droneIsDelivering(D1,P6,5) <= 1
 0 <= droneIsDelivering(D2,P6,2) <= 1
 0 <= droneIsDelivering(D2,P6,1) <= 1
 0 <= droneIsDelivering(D2,P6,3) <= 1
 0 <= droneIsDelivering(D2,P6,4) <= 1
 0 <= droneIsDelivering(D2,P6,5) <= 1
 0 <= droneIsDelivering(D3,P6,2) <= 1
 0 <= droneIsDelivering(D3,P6,1) <= 1
 0 <= droneIsDelivering(D3,P6,3) <= 1
 0 <= droneIsDelivering(D3,P6,4) <= 1
 0 <= droneIsDelivering(D3,P6,5) <= 1
 0 <= droneIsDelivering(D4,P6,2) <= 1
 0 <= droneIsDelivering(D4,P6,1) <= 1
 0 <= droneIsDelivering(D4,P6,3) <= 1
 0 <= droneIsDelivering(D4,P6,4) <= 1
 0 <= droneIsDelivering(D4,P6,5) <= 1
 0 <= droneIsDelivering(D5,P6,2) <= 1
 0 <= droneIsDelivering(D5,P6,1) <= 1
 0 <= droneIsDelivering(D5,P6,3) <= 1
 0 <= droneIsDelivering(D5,P6,4) <= 1
 0 <= droneIsDelivering(D5,P6,5) <= 1
 0 <= droneIsUsed(D1) <= 1
 0 <= droneIsUsed(D2) <= 1
 0 <= droneIsUsed(D3) <= 1
 0 <= droneIsUsed(D4) <= 1
 0 <= droneIsUsed(D5) <= 1
 0 <= droneBatteryRate(D1,2) <= 100
 0 <= droneBatteryRate(D1,3) <= 100
 0 <= droneBatteryRate(D1,4) <= 100
 0 <= droneBatteryRate(D1,5) <= 100
 0 <= droneBatteryRate(D2,2) <= 100
 0 <= droneBatteryRate(D2,3) <= 100
 0 <= droneBatteryRate(D2,4) <= 100
 0 <= droneBatteryRate(D2,5) <= 100
 0 <= droneBatteryRate(D3,2) <= 100
 0 <= droneBatteryRate(D3,3) <= 100
 0 <= droneBatteryRate(D3,4) <= 100
 0 <= droneBatteryRate(D3,5) <= 100
 0 <= droneBatteryRate(D4,2) <= 100
 0 <= droneBatteryRate(D4,3) <= 100
 0 <= droneBatteryRate(D4,4) <= 100
 0 <= droneBatteryRate(D4,5) <= 100
 0 <= droneBatteryRate(D5,2) <= 100
 0 <= droneBatteryRate(D5,3) <= 100
 0 <= droneBatteryRate(D5,4) <= 100
 0 <= droneBatteryRate(D5,5) <= 100
 0 <= droneBatteryRate(D1,1) <= 100
 0 <= droneBatteryRate(D2,1) <= 100
 0 <= droneBatteryRate(D3,1) <= 100
 0 <= droneBatteryRate(D4,1) <= 100
 0 <= droneBatteryRate(D5,1) <= 100
 0 <= excessOfEnergy(D1,1) <= 20
 0 <= excessOfEnergy(D1,2) <= 20
 0 <= excessOfEnergy(D1,3) <= 20
 0 <= excessOfEnergy(D1,4) <= 20
 0 <= excessOfEnergy(D1,5) <= 20
 0 <= excessOfEnergy(D2,1) <= 20
 0 <= excessOfEnergy(D2,2) <= 20
 0 <= excessOfEnergy(D2,3) <= 20
 0 <= excessOfEnergy(D2,4) <= 20
 0 <= excessOfEnergy(D2,5) <= 20
 0 <= excessOfEnergy(D3,1) <= 20
 0 <= excessOfEnergy(D3,2) <= 20
 0 <= excessOfEnergy(D3,3) <= 20
 0 <= excessOfEnergy(D3,4) <= 20
 0 <= excessOfEnergy(D3,5) <= 20
 0 <= excessOfEnergy(D4,1) <= 20
 0 <= excessOfEnergy(D4,2) <= 20
 0 <= excessOfEnergy(D4,3) <= 20
 0 <= excessOfEnergy(D4,4) <= 20
 0 <= excessOfEnergy(D4,5) <= 20
 0 <= excessOfEnergy(D5,1) <= 20
 0 <= excessOfEnergy(D5,2) <= 20
 0 <= excessOfEnergy(D5,3) <= 20
 0 <= excessOfEnergy(D5,4) <= 20
 0 <= excessOfEnergy(D5,5) <= 20
 0 <= droneIsChargingAtEP(D1,2,1) <= 1
 0 <= droneIsChargingAtEP(D1,2,2) <= 1
 0 <= droneIsChargingAtEP(D1,2,3) <= 1
 0 <= droneIsChargingAtEP(D1,2,4) <= 1
 0 <= droneIsChargingAtEP(D1,3,1) <= 1
 0 <= droneIsChargingAtEP(D1,3,2) <= 1
 0 <= droneIsChargingAtEP(D1,3,3) <= 1
 0 <= droneIsChargingAtEP(D1,3,4) <= 1
 0 <= droneIsChargingAtEP(D1,4,1) <= 1
 0 <= droneIsChargingAtEP(D1,4,2) <= 1
 0 <= droneIsChargingAtEP(D1,4,3) <= 1
 0 <= droneIsChargingAtEP(D1,4,4) <= 1
 0 <= droneIsChargingAtEP(D1,5,1) <= 1
 0 <= droneIsChargingAtEP(D1,5,2) <= 1
 0 <= droneIsChargingAtEP(D1,5,3) <= 1
 0 <= droneIsChargingAtEP(D1,5,4) <= 1
 0 <= droneIsChargingAtEP(D2,2,1) <= 1
 0 <= droneIsChargingAtEP(D2,2,2) <= 1
 0 <= droneIsChargingAtEP(D2,2,3) <= 1
 0 <= droneIsChargingAtEP(D2,2,4) <= 1
 0 <= droneIsChargingAtEP(D2,3,1) <= 1
 0 <= droneIsChargingAtEP(D2,3,2) <= 1
 0 <= droneIsChargingAtEP(D2,3,3) <= 1
 0 <= droneIsChargingAtEP(D2,3,4) <= 1
 0 <= droneIsChargingAtEP(D2,4,1) <= 1
 0 <= droneIsChargingAtEP(D2,4,2) <= 1
 0 <= droneIsChargingAtEP(D2,4,3) <= 1
 0 <= droneIsChargingAtEP(D2,4,4) <= 1
 0 <= droneIsChargingAtEP(D2,5,1) <= 1
 0 <= droneIsChargingAtEP(D2,5,2) <= 1
 0 <= droneIsChargingAtEP(D2,5,3) <= 1
 0 <= droneIsChargingAtEP(D2,5,4) <= 1
 0 <= droneIsChargingAtEP(D3,2,1) <= 1
 0 <= droneIsChargingAtEP(D3,2,2) <= 1
 0 <= droneIsChargingAtEP(D3,2,3) <= 1
 0 <= droneIsChargingAtEP(D3,2,4) <= 1
 0 <= droneIsChargingAtEP(D3,3,1) <= 1
 0 <= droneIsChargingAtEP(D3,3,2) <= 1
 0 <= droneIsChargingAtEP(D3,3,3) <= 1
 0 <= droneIsChargingAtEP(D3,3,4) <= 1
 0 <= droneIsChargingAtEP(D3,4,1) <= 1
 0 <= droneIsChargingAtEP(D3,4,2) <= 1
 0 <= droneIsChargingAtEP(D3,4,3) <= 1
 0 <= droneIsChargingAtEP(D3,4,4) <= 1
 0 <= droneIsChargingAtEP(D3,5,1) <= 1
 0 <= droneIsChargingAtEP(D3,5,2) <= 1
 0 <= droneIsChargingAtEP(D3,5,3) <= 1
 0 <= droneIsChargingAtEP(D3,5,4) <= 1
 0 <= droneIsChargingAtEP(D4,2,1) <= 1
 0 <= droneIsChargingAtEP(D4,2,2) <= 1
 0 <= droneIsChargingAtEP(D4,2,3) <= 1
 0 <= droneIsChargingAtEP(D4,2,4) <= 1
 0 <= droneIsChargingAtEP(D4,3,1) <= 1
 0 <= droneIsChargingAtEP(D4,3,2) <= 1
 0 <= droneIsChargingAtEP(D4,3,3) <= 1
 0 <= droneIsChargingAtEP(D4,3,4) <= 1
 0 <= droneIsChargingAtEP(D4,4,1) <= 1
 0 <= droneIsChargingAtEP(D4,4,2) <= 1
 0 <= droneIsChargingAtEP(D4,4,3) <= 1
 0 <= droneIsChargingAtEP(D4,4,4) <= 1
 0 <= droneIsChargingAtEP(D4,5,1) <= 1
 0 <= droneIsChargingAtEP(D4,5,2) <= 1
 0 <= droneIsChargingAtEP(D4,5,3) <= 1
 0 <= droneIsChargingAtEP(D4,5,4) <= 1
 0 <= droneIsChargingAtEP(D5,2,1) <= 1
 0 <= droneIsChargingAtEP(D5,2,2) <= 1
 0 <= droneIsChargingAtEP(D5,2,3) <= 1
 0 <= droneIsChargingAtEP(D5,2,4) <= 1
 0 <= droneIsChargingAtEP(D5,3,1) <= 1
 0 <= droneIsChargingAtEP(D5,3,2) <= 1
 0 <= droneIsChargingAtEP(D5,3,3) <= 1
 0 <= droneIsChargingAtEP(D5,3,4) <= 1
 0 <= droneIsChargingAtEP(D5,4,1) <= 1
 0 <= droneIsChargingAtEP(D5,4,2) <= 1
 0 <= droneIsChargingAtEP(D5,4,3) <= 1
 0 <= droneIsChargingAtEP(D5,4,4) <= 1
 0 <= droneIsChargingAtEP(D5,5,1) <= 1
 0 <= droneIsChargingAtEP(D5,5,2) <= 1
 0 <= droneIsChargingAtEP(D5,5,3) <= 1
 0 <= droneIsChargingAtEP(D5,5,4) <= 1
 0 <= droneIsChargingAtEP(D1,1,1) <= 1
 0 <= droneIsChargingAtEP(D2,1,1) <= 1
 0 <= droneIsChargingAtEP(D3,1,1) <= 1
 0 <= droneIsChargingAtEP(D4,1,1) <= 1
 0 <= droneIsChargingAtEP(D5,1,1) <= 1
 0 <= droneIsChargingAtEP(D1,1,2) <= 1
 0 <= droneIsChargingAtEP(D2,1,2) <= 1
 0 <= droneIsChargingAtEP(D3,1,2) <= 1
 0 <= droneIsChargingAtEP(D4,1,2) <= 1
 0 <= droneIsChargingAtEP(D5,1,2) <= 1
 0 <= droneIsChargingAtEP(D1,1,3) <= 1
 0 <= droneIsChargingAtEP(D2,1,3) <= 1
 0 <= droneIsChargingAtEP(D3,1,3) <= 1
 0 <= droneIsChargingAtEP(D4,1,3) <= 1
 0 <= droneIsChargingAtEP(D5,1,3) <= 1
 0 <= droneIsChargingAtEP(D1,1,4) <= 1
 0 <= droneIsChargingAtEP(D2,1,4) <= 1
 0 <= droneIsChargingAtEP(D3,1,4) <= 1
 0 <= droneIsChargingAtEP(D4,1,4) <= 1
 0 <= droneIsChargingAtEP(D5,1,4) <= 1
 0 <= droneIsOn(D1,1) <= 1
 0 <= droneIsOn(D1,2) <= 1
 0 <= droneIsOn(D1,3) <= 1
 0 <= droneIsOn(D1,4) <= 1
 0 <= droneIsOn(D1,5) <= 1
 0 <= droneIsOn(D2,1) <= 1
 0 <= droneIsOn(D2,2) <= 1
 0 <= droneIsOn(D2,3) <= 1
 0 <= droneIsOn(D2,4) <= 1
 0 <= droneIsOn(D2,5) <= 1
 0 <= droneIsOn(D3,1) <= 1
 0 <= droneIsOn(D3,2) <= 1
 0 <= droneIsOn(D3,3) <= 1
 0 <= droneIsOn(D3,4) <= 1
 0 <= droneIsOn(D3,5) <= 1
 0 <= droneIsOn(D4,1) <= 1
 0 <= droneIsOn(D4,2) <= 1
 0 <= droneIsOn(D4,3) <= 1
 0 <= droneIsOn(D4,4) <= 1
 0 <= droneIsOn(D4,5) <= 1
 0 <= droneIsOn(D5,1) <= 1
 0 <= droneIsOn(D5,2) <= 1
 0 <= droneIsOn(D5,3) <= 1
 0 <= droneIsOn(D5,4) <= 1
 0 <= droneIsOn(D5,5) <= 1

Generals
 droneCollected(D1,P1,2)
 droneCollected(D1,P2,2)
 droneCollected(D1,P3,2)
 droneCollected(D1,P4,2)
 droneCollected(D1,P5,2)
 droneCollected(D1,P6,2)
 droneCollected(D1,P1,3)
 droneCollected(D1,P2,3)
 droneCollected(D1,P3,3)
 droneCollected(D1,P4,3)
 droneCollected(D1,P5,3)
 droneCollected(D1,P6,3)
 droneCollected(D1,P1,4)
 droneCollected(D1,P2,4)
 droneCollected(D1,P3,4)
 droneCollected(D1,P4,4)
 droneCollected(D1,P5,4)
 droneCollected(D1,P6,4)
 droneCollected(D1,P1,5)
 droneCollected(D1,P2,5)
 droneCollected(D1,P3,5)
 droneCollected(D1,P4,5)
 droneCollected(D1,P5,5)
 droneCollected(D1,P6,5)
 droneCollected(D2,P1,2)
 droneCollected(D2,P2,2)
 droneCollected(D2,P3,2)
 droneCollected(D2,P4,2)
 droneCollected(D2,P5,2)
 droneCollected(D2,P6,2)
 droneCollected(D2,P1,3)
 droneCollected(D2,P2,3)
 droneCollected(D2,P3,3)
 droneCollected(D2,P4,3)
 droneCollected(D2,P5,3)
 droneCollected(D2,P6,3)
 droneCollected(D2,P1,4)
 droneCollected(D2,P2,4)
 droneCollected(D2,P3,4)
 droneCollected(D2,P4,4)
 droneCollected(D2,P5,4)
 droneCollected(D2,P6,4)
 droneCollected(D2,P1,5)
 droneCollected(D2,P2,5)
 droneCollected(D2,P3,5)
 droneCollected(D2,P4,5)
 droneCollected(D2,P5,5)
 droneCollected(D2,P6,5)
 droneCollected(D3,P1,2)
 droneCollected(D3,P2,2)
 droneCollected(D3,P3,2)
 droneCollected(D3,P4,2)
 droneCollected(D3,P5,2)
 droneCollected(D3,P6,2)
 droneCollected(D3,P1,3)
 droneCollected(D3,P2,3)
 droneCollected(D3,P3,3)
 droneCollected(D3,P4,3)
 droneCollected(D3,P5,3)
 droneCollected(D3,P6,3)
 droneCollected(D3,P1,4)
 droneCollected(D3,P2,4)
 droneCollected(D3,P3,4)
 droneCollected(D3,P4,4)
 droneCollected(D3,P5,4)
 droneCollected(D3,P6,4)
 droneCollected(D3,P1,5)
 droneCollected(D3,P2,5)
 droneCollected(D3,P3,5)
 droneCollected(D3,P4,5)
 droneCollected(D3,P5,5)
 droneCollected(D3,P6,5)
 droneCollected(D4,P1,2)
 droneCollected(D4,P2,2)
 droneCollected(D4,P3,2)
 droneCollected(D4,P4,2)
 droneCollected(D4,P5,2)
 droneCollected(D4,P6,2)
 droneCollected(D4,P1,3)
 droneCollected(D4,P2,3)
 droneCollected(D4,P3,3)
 droneCollected(D4,P4,3)
 droneCollected(D4,P5,3)
 droneCollected(D4,P6,3)
 droneCollected(D4,P1,4)
 droneCollected(D4,P2,4)
 droneCollected(D4,P3,4)
 droneCollected(D4,P4,4)
 droneCollected(D4,P5,4)
 droneCollected(D4,P6,4)
 droneCollected(D4,P1,5)
 droneCollected(D4,P2,5)
 droneCollected(D4,P3,5)
 droneCollected(D4,P4,5)
 droneCollected(D4,P5,5)
 droneCollected(D4,P6,5)
 droneCollected(D5,P1,2)
 droneCollected(D5,P2,2)
 droneCollected(D5,P3,2)
 droneCollected(D5,P4,2)
 droneCollected(D5,P5,2)
 droneCollected(D5,P6,2)
 droneCollected(D5,P1,3)
 droneCollected(D5,P2,3)
 droneCollected(D5,P3,3)
 droneCollected(D5,P4,3)
 droneCollected(D5,P5,3)
 droneCollected(D5,P6,3)
 droneCollected(D5,P1,4)
 droneCollected(D5,P2,4)
 droneCollected(D5,P3,4)
 droneCollected(D5,P4,4)
 droneCollected(D5,P5,4)
 droneCollected(D5,P6,4)
 droneCollected(D5,P1,5)
 droneCollected(D5,P2,5)
 droneCollected(D5,P3,5)
 droneCollected(D5,P4,5)
 droneCollected(D5,P5,5)
 droneCollected(D5,P6,5)
 droneCollected(D1,P1,1)
 droneCollected(D2,P1,1)
 droneCollected(D3,P1,1)
 droneCollected(D4,P1,1)
 droneCollected(D5,P1,1)
 droneCollected(D1,P2,1)
 droneCollected(D2,P2,1)
 droneCollected(D3,P2,1)
 droneCollected(D4,P2,1)
 droneCollected(D5,P2,1)
 droneCollected(D1,P3,1)
 droneCollected(D2,P3,1)
 droneCollected(D3,P3,1)
 droneCollected(D4,P3,1)
 droneCollected(D5,P3,1)
 droneCollected(D1,P4,1)
 droneCollected(D2,P4,1)
 droneCollected(D3,P4,1)
 droneCollected(D4,P4,1)
 droneCollected(D5,P4,1)
 droneCollected(D1,P5,1)
 droneCollected(D2,P5,1)
 droneCollected(D3,P5,1)
 droneCollected(D4,P5,1)
 droneCollected(D5,P5,1)
 droneCollected(D1,P6,1)
 droneCollected(D2,P6,1)
 droneCollected(D3,P6,1)
 droneCollected(D4,P6,1)
 droneCollected(D5,P6,1)
 droneDelivered(D1,P1,2)
 droneDelivered(D1,P2,2)
 droneDelivered(D1,P3,2)
 droneDelivered(D1,P4,2)
 droneDelivered(D1,P5,2)
 droneDelivered(D1,P6,2)
 droneDelivered(D1,P1,3)
 droneDelivered(D1,P2,3)
 droneDelivered(D1,P3,3)
 droneDelivered(D1,P4,3)
 droneDelivered(D1,P5,3)
 droneDelivered(D1,P6,3)
 droneDelivered(D1,P1,4)
 droneDelivered(D1,P2,4)
 droneDelivered(D1,P3,4)
 droneDelivered(D1,P4,4)
 droneDelivered(D1,P5,4)
 droneDelivered(D1,P6,4)
 droneDelivered(D1,P1,5)
 droneDelivered(D1,P2,5)
 droneDelivered(D1,P3,5)
 droneDelivered(D1,P4,5)
 droneDelivered(D1,P5,5)
 droneDelivered(D1,P6,5)
 droneDelivered(D2,P1,2)
 droneDelivered(D2,P2,2)
 droneDelivered(D2,P3,2)
 droneDelivered(D2,P4,2)
 droneDelivered(D2,P5,2)
 droneDelivered(D2,P6,2)
 droneDelivered(D2,P1,3)
 droneDelivered(D2,P2,3)
 droneDelivered(D2,P3,3)
 droneDelivered(D2,P4,3)
 droneDelivered(D2,P5,3)
 droneDelivered(D2,P6,3)
 droneDelivered(D2,P1,4)
 droneDelivered(D2,P2,4)
 droneDelivered(D2,P3,4)
 droneDelivered(D2,P4,4)
 droneDelivered(D2,P5,4)
 droneDelivered(D2,P6,4)
 droneDelivered(D2,P1,5)
 droneDelivered(D2,P2,5)
 droneDelivered(D2,P3,5)
 droneDelivered(D2,P4,5)
 droneDelivered(D2,P5,5)
 droneDelivered(D2,P6,5)
 droneDelivered(D3,P1,2)
 droneDelivered(D3,P2,2)
 droneDelivered(D3,P3,2)
 droneDelivered(D3,P4,2)
 droneDelivered(D3,P5,2)
 droneDelivered(D3,P6,2)
 droneDelivered(D3,P1,3)
 droneDelivered(D3,P2,3)
 droneDelivered(D3,P3,3)
 droneDelivered(D3,P4,3)
 droneDelivered(D3,P5,3)
 droneDelivered(D3,P6,3)
 droneDelivered(D3,P1,4)
 droneDelivered(D3,P2,4)
 droneDelivered(D3,P3,4)
 droneDelivered(D3,P4,4)
 droneDelivered(D3,P5,4)
 droneDelivered(D3,P6,4)
 droneDelivered(D3,P1,5)
 droneDelivered(D3,P2,5)
 droneDelivered(D3,P3,5)
 droneDelivered(D3,P4,5)
 droneDelivered(D3,P5,5)
 droneDelivered(D3,P6,5)
 droneDelivered(D4,P1,2)
 droneDelivered(D4,P2,2)
 droneDelivered(D4,P3,2)
 droneDelivered(D4,P4,2)
 droneDelivered(D4,P5,2)
 droneDelivered(D4,P6,2)
 droneDelivered(D4,P1,3)
 droneDelivered(D4,P2,3)
 droneDelivered(D4,P3,3)
 droneDelivered(D4,P4,3)
 droneDelivered(D4,P5,3)
 droneDelivered(D4,P6,3)
 droneDelivered(D4,P1,4)
 droneDelivered(D4,P2,4)
 droneDelivered(D4,P3,4)
 droneDelivered(D4,P4,4)
 droneDelivered(D4,P5,4)
 droneDelivered(D4,P6,4)
 droneDelivered(D4,P1,5)
 droneDelivered(D4,P2,5)
 droneDelivered(D4,P3,5)
 droneDelivered(D4,P4,5)
 droneDelivered(D4,P5,5)
 droneDelivered(D4,P6,5)
 droneDelivered(D5,P1,2)
 droneDelivered(D5,P2,2)
 droneDelivered(D5,P3,2)
 droneDelivered(D5,P4,2)
 droneDelivered(D5,P5,2)
 droneDelivered(D5,P6,2)
 droneDelivered(D5,P1,3)
 droneDelivered(D5,P2,3)
 droneDelivered(D5,P3,3)
 droneDelivered(D5,P4,3)
 droneDelivered(D5,P5,3)
 droneDelivered(D5,P6,3)
 droneDelivered(D5,P1,4)
 droneDelivered(D5,P2,4)
 droneDelivered(D5,P3,4)
 droneDelivered(D5,P4,4)
 droneDelivered(D5,P5,4)
 droneDelivered(D5,P6,4)
 droneDelivered(D5,P1,5)
 droneDelivered(D5,P2,5)
 droneDelivered(D5,P3,5)
 droneDelivered(D5,P4,5)
 droneDelivered(D5,P5,5)
 droneDelivered(D5,P6,5)
 droneDelivered(D1,P1,1)
 droneDelivered(D2,P1,1)
 droneDelivered(D3,P1,1)
 droneDelivered(D4,P1,1)
 droneDelivered(D5,P1,1)
 droneDelivered(D1,P2,1)
 droneDelivered(D2,P2,1)
 droneDelivered(D3,P2,1)
 droneDelivered(D4,P2,1)
 droneDelivered(D5,P2,1)
 droneDelivered(D1,P3,1)
 droneDelivered(D2,P3,1)
 droneDelivered(D3,P3,1)
 droneDelivered(D4,P3,1)
 droneDelivered(D5,P3,1)
 droneDelivered(D1,P4,1)
 droneDelivered(D2,P4,1)
 droneDelivered(D3,P4,1)
 droneDelivered(D4,P4,1)
 droneDelivered(D5,P4,1)
 droneDelivered(D1,P5,1)
 droneDelivered(D2,P5,1)
 droneDelivered(D3,P5,1)
 droneDelivered(D4,P5,1)
 droneDelivered(D5,P5,1)
 droneDelivered(D1,P6,1)
 droneDelivered(D2,P6,1)
 droneDelivered(D3,P6,1)
 droneDelivered(D4,P6,1)
 droneDelivered(D5,P6,1)
 droneIsDelivering(D1,P1,2)
 droneIsDelivering(D1,P1,1)
 droneIsDelivering(D1,P1,3)
 droneIsDelivering(D1,P1,4)
 droneIsDelivering(D1,P1,5)
 droneIsDelivering(D2,P1,2)
 droneIsDelivering(D2,P1,1)
 droneIsDelivering(D2,P1,3)
 droneIsDelivering(D2,P1,4)
 droneIsDelivering(D2,P1,5)
 droneIsDelivering(D3,P1,2)
 droneIsDelivering(D3,P1,1)
 droneIsDelivering(D3,P1,3)
 droneIsDelivering(D3,P1,4)
 droneIsDelivering(D3,P1,5)
 droneIsDelivering(D4,P1,2)
 droneIsDelivering(D4,P1,1)
 droneIsDelivering(D4,P1,3)
 droneIsDelivering(D4,P1,4)
 droneIsDelivering(D4,P1,5)
 droneIsDelivering(D5,P1,2)
 droneIsDelivering(D5,P1,1)
 droneIsDelivering(D5,P1,3)
 droneIsDelivering(D5,P1,4)
 droneIsDelivering(D5,P1,5)
 droneIsDelivering(D1,P2,2)
 droneIsDelivering(D1,P2,1)
 droneIsDelivering(D1,P2,3)
 droneIsDelivering(D1,P2,4)
 droneIsDelivering(D1,P2,5)
 droneIsDelivering(D2,P2,2)
 droneIsDelivering(D2,P2,1)
 droneIsDelivering(D2,P2,3)
 droneIsDelivering(D2,P2,4)
 droneIsDelivering(D2,P2,5)
 droneIsDelivering(D3,P2,2)
 droneIsDelivering(D3,P2,1)
 droneIsDelivering(D3,P2,3)
 droneIsDelivering(D3,P2,4)
 droneIsDelivering(D3,P2,5)
 droneIsDelivering(D4,P2,2)
 droneIsDelivering(D4,P2,1)
 droneIsDelivering(D4,P2,3)
 droneIsDelivering(D4,P2,4)
 droneIsDelivering(D4,P2,5)
 droneIsDelivering(D5,P2,2)
 droneIsDelivering(D5,P2,1)
 droneIsDelivering(D5,P2,3)
 droneIsDelivering(D5,P2,4)
 droneIsDelivering(D5,P2,5)
 droneIsDelivering(D1,P3,2)
 droneIsDelivering(D1,P3,1)
 droneIsDelivering(D1,P3,3)
 droneIsDelivering(D1,P3,4)
 droneIsDelivering(D1,P3,5)
 droneIsDelivering(D2,P3,2)
 droneIsDelivering(D2,P3,1)
 droneIsDelivering(D2,P3,3)
 droneIsDelivering(D2,P3,4)
 droneIsDelivering(D2,P3,5)
 droneIsDelivering(D3,P3,2)
 droneIsDelivering(D3,P3,1)
 droneIsDelivering(D3,P3,3)
 droneIsDelivering(D3,P3,4)
 droneIsDelivering(D3,P3,5)
 droneIsDelivering(D4,P3,2)
 droneIsDelivering(D4,P3,1)
 droneIsDelivering(D4,P3,3)
 droneIsDelivering(D4,P3,4)
 droneIsDelivering(D4,P3,5)
 droneIsDelivering(D5,P3,2)
 droneIsDelivering(D5,P3,1)
 droneIsDelivering(D5,P3,3)
 droneIsDelivering(D5,P3,4)
 droneIsDelivering(D5,P3,5)
 droneIsDelivering(D1,P4,2)
 droneIsDelivering(D1,P4,1)
 droneIsDelivering(D1,P4,3)
 droneIsDelivering(D1,P4,4)
 droneIsDelivering(D1,P4,5)
 droneIsDelivering(D2,P4,2)
 droneIsDelivering(D2,P4,1)
 droneIsDelivering(D2,P4,3)
 droneIsDelivering(D2,P4,4)
 droneIsDelivering(D2,P4,5)
 droneIsDelivering(D3,P4,2)
 droneIsDelivering(D3,P4,1)
 droneIsDelivering(D3,P4,3)
 droneIsDelivering(D3,P4,4)
 droneIsDelivering(D3,P4,5)
 droneIsDelivering(D4,P4,2)
 droneIsDelivering(D4,P4,1)
 droneIsDelivering(D4,P4,3)
 droneIsDelivering(D4,P4,4)
 droneIsDelivering(D4,P4,5)
 droneIsDelivering(D5,P4,2)
 droneIsDelivering(D5,P4,1)
 droneIsDelivering(D5,P4,3)
 droneIsDelivering(D5,P4,4)
 droneIsDelivering(D5,P4,5)
 droneIsDelivering(D1,P5,2)
 droneIsDelivering(D1,P5,1)
 droneIsDelivering(D1,P5,3)
 droneIsDelivering(D1,P5,4)
 droneIsDelivering(D1,P5,5)
 droneIsDelivering(D2,P5,2)
 droneIsDelivering(D2,P5,1)
 droneIsDelivering(D2,P5,3)
 droneIsDelivering(D2,P5,4)
 droneIsDelivering(D2,P5,5)
 droneIsDelivering(D3,P5,2)
 droneIsDelivering(D3,P5,1)
 droneIsDelivering(D3,P5,3)
 droneIsDelivering(D3,P5,4)
 droneIsDelivering(D3,P5,5)
 droneIsDelivering(D4,P5,2)
 droneIsDelivering(D4,P5,1)
 droneIsDelivering(D4,P5,3)
 droneIsDelivering(D4,P5,4)
 droneIsDelivering(D4,P5,5)
 droneIsDelivering(D5,P5,2)
 droneIsDelivering(D5,P5,1)
 droneIsDelivering(D5,P5,3)
 droneIsDelivering(D5,P5,4)
 droneIsDelivering(D5,P5,5)
 droneIsDelivering(D1,P6,2)
 droneIsDelivering(D1,P6,1)
 droneIsDelivering(D1,P6,3)
 droneIsDelivering(D1,P6,4)
 droneIsDelivering(D1,P6,5)
 droneIsDelivering(D2,P6,2)
 droneIsDelivering(D2,P6,1)
 droneIsDelivering(D2,P6,3)
 droneIsDelivering(D2,P6,4)
 droneIsDelivering(D2,P6,5)
 droneIsDelivering(D3,P6,2)
 droneIsDelivering(D3,P6,1)
 droneIsDelivering(D3,P6,3)
 droneIsDelivering(D3,P6,4)
 droneIsDelivering(D3,P6,5)
 droneIsDelivering(D4,P6,2)
 droneIsDelivering(D4,P6,1)
 droneIsDelivering(D4,P6,3)
 droneIsDelivering(D4,P6,4)
 droneIsDelivering(D4,P6,5)
 droneIsDelivering(D5,P6,2)
 droneIsDelivering(D5,P6,1)
 droneIsDelivering(D5,P6,3)
 droneIsDelivering(D5,P6,4)
 droneIsDelivering(D5,P6,5)
 droneIsUsed(D1)
 droneIsUsed(D2)
 droneIsUsed(D3)
 droneIsUsed(D4)
 droneIsUsed(D5)
 droneIsChargingAtEP(D1,2,1)
 droneIsChargingAtEP(D1,2,2)
 droneIsChargingAtEP(D1,2,3)
 droneIsChargingAtEP(D1,2,4)
 droneIsChargingAtEP(D1,3,1)
 droneIsChargingAtEP(D1,3,2)
 droneIsChargingAtEP(D1,3,3)
 droneIsChargingAtEP(D1,3,4)
 droneIsChargingAtEP(D1,4,1)
 droneIsChargingAtEP(D1,4,2)
 droneIsChargingAtEP(D1,4,3)
 droneIsChargingAtEP(D1,4,4)
 droneIsChargingAtEP(D1,5,1)
 droneIsChargingAtEP(D1,5,2)
 droneIsChargingAtEP(D1,5,3)
 droneIsChargingAtEP(D1,5,4)
 droneIsChargingAtEP(D2,2,1)
 droneIsChargingAtEP(D2,2,2)
 droneIsChargingAtEP(D2,2,3)
 droneIsChargingAtEP(D2,2,4)
 droneIsChargingAtEP(D2,3,1)
 droneIsChargingAtEP(D2,3,2)
 droneIsChargingAtEP(D2,3,3)
 droneIsChargingAtEP(D2,3,4)
 droneIsChargingAtEP(D2,4,1)
 droneIsChargingAtEP(D2,4,2)
 droneIsChargingAtEP(D2,4,3)
 droneIsChargingAtEP(D2,4,4)
 droneIsChargingAtEP(D2,5,1)
 droneIsChargingAtEP(D2,5,2)
 droneIsChargingAtEP(D2,5,3)
 droneIsChargingAtEP(D2,5,4)
 droneIsChargingAtEP(D3,2,1)
 droneIsChargingAtEP(D3,2,2)
 droneIsChargingAtEP(D3,2,3)
 droneIsChargingAtEP(D3,2,4)
 droneIsChargingAtEP(D3,3,1)
 droneIsChargingAtEP(D3,3,2)
 droneIsChargingAtEP(D3,3,3)
 droneIsChargingAtEP(D3,3,4)
 droneIsChargingAtEP(D3,4,1)
 droneIsChargingAtEP(D3,4,2)
 droneIsChargingAtEP(D3,4,3)
 droneIsChargingAtEP(D3,4,4)
 droneIsChargingAtEP(D3,5,1)
 droneIsChargingAtEP(D3,5,2)
 droneIsChargingAtEP(D3,5,3)
 droneIsChargingAtEP(D3,5,4)
 droneIsChargingAtEP(D4,2,1)
 droneIsChargingAtEP(D4,2,2)
 droneIsChargingAtEP(D4,2,3)
 droneIsChargingAtEP(D4,2,4)
 droneIsChargingAtEP(D4,3,1)
 droneIsChargingAtEP(D4,3,2)
 droneIsChargingAtEP(D4,3,3)
 droneIsChargingAtEP(D4,3,4)
 droneIsChargingAtEP(D4,4,1)
 droneIsChargingAtEP(D4,4,2)
 droneIsChargingAtEP(D4,4,3)
 droneIsChargingAtEP(D4,4,4)
 droneIsChargingAtEP(D4,5,1)
 droneIsChargingAtEP(D4,5,2)
 droneIsChargingAtEP(D4,5,3)
 droneIsChargingAtEP(D4,5,4)
 droneIsChargingAtEP(D5,2,1)
 droneIsChargingAtEP(D5,2,2)
 droneIsChargingAtEP(D5,2,3)
 droneIsChargingAtEP(D5,2,4)
 droneIsChargingAtEP(D5,3,1)
 droneIsChargingAtEP(D5,3,2)
 droneIsChargingAtEP(D5,3,3)
 droneIsChargingAtEP(D5,3,4)
 droneIsChargingAtEP(D5,4,1)
 droneIsChargingAtEP(D5,4,2)
 droneIsChargingAtEP(D5,4,3)
 droneIsChargingAtEP(D5,4,4)
 droneIsChargingAtEP(D5,5,1)
 droneIsChargingAtEP(D5,5,2)
 droneIsChargingAtEP(D5,5,3)
 droneIsChargingAtEP(D5,5,4)
 droneIsChargingAtEP(D1,1,1)
 droneIsChargingAtEP(D2,1,1)
 droneIsChargingAtEP(D3,1,1)
 droneIsChargingAtEP(D4,1,1)
 droneIsChargingAtEP(D5,1,1)
 droneIsChargingAtEP(D1,1,2)
 droneIsChargingAtEP(D2,1,2)
 droneIsChargingAtEP(D3,1,2)
 droneIsChargingAtEP(D4,1,2)
 droneIsChargingAtEP(D5,1,2)
 droneIsChargingAtEP(D1,1,3)
 droneIsChargingAtEP(D2,1,3)
 droneIsChargingAtEP(D3,1,3)
 droneIsChargingAtEP(D4,1,3)
 droneIsChargingAtEP(D5,1,3)
 droneIsChargingAtEP(D1,1,4)
 droneIsChargingAtEP(D2,1,4)
 droneIsChargingAtEP(D3,1,4)
 droneIsChargingAtEP(D4,1,4)
 droneIsChargingAtEP(D5,1,4)
 droneIsOn(D1,1)
 droneIsOn(D1,2)
 droneIsOn(D1,3)
 droneIsOn(D1,4)
 droneIsOn(D1,5)
 droneIsOn(D2,1)
 droneIsOn(D2,2)
 droneIsOn(D2,3)
 droneIsOn(D2,4)
 droneIsOn(D2,5)
 droneIsOn(D3,1)
 droneIsOn(D3,2)
 droneIsOn(D3,3)
 droneIsOn(D3,4)
 droneIsOn(D3,5)
 droneIsOn(D4,1)
 droneIsOn(D4,2)
 droneIsOn(D4,3)
 droneIsOn(D4,4)
 droneIsOn(D4,5)
 droneIsOn(D5,1)
 droneIsOn(D5,2)
 droneIsOn(D5,3)
 droneIsOn(D5,4)
 droneIsOn(D5,5)

End

\* Problem: MILP_UAVRouting2 *\

Minimize
 obj: + totalDist + timePerDeliver + nUsedDrones + 100 dronesMaxSpeed

Subject To
 respectDroneCap(D1,1): + droneCurrentCap(D1,1) <= 20
 respectDroneCap(D1,2): + droneCurrentCap(D1,2) <= 20
 respectDroneCap(D1,3): + droneCurrentCap(D1,3) <= 20
 respectDroneCap(D1,4): + droneCurrentCap(D1,4) <= 20
 respectDroneCap(D1,5): + droneCurrentCap(D1,5) <= 20
 respectDroneCap(D2,1): + droneCurrentCap(D2,1) <= 30
 respectDroneCap(D2,2): + droneCurrentCap(D2,2) <= 30
 respectDroneCap(D2,3): + droneCurrentCap(D2,3) <= 30
 respectDroneCap(D2,4): + droneCurrentCap(D2,4) <= 30
 respectDroneCap(D2,5): + droneCurrentCap(D2,5) <= 30
 respectDroneCap(D3,1): + droneCurrentCap(D3,1) <= 20
 respectDroneCap(D3,2): + droneCurrentCap(D3,2) <= 20
 respectDroneCap(D3,3): + droneCurrentCap(D3,3) <= 20
 respectDroneCap(D3,4): + droneCurrentCap(D3,4) <= 20
 respectDroneCap(D3,5): + droneCurrentCap(D3,5) <= 20
 updatedCAP(D1,2): - droneCurrentCap(D1,1) + droneCurrentCap(D1,2)
 - 10 droneCollected(D1,P1,2) - 10 droneCollected(D1,P2,2)
 - 5 droneCollected(D1,P3,2) - 5 droneCollected(D1,P4,2)
 - droneCollected(D1,P6,2) + 10 droneDelivered(D1,P1,2)
 + 10 droneDelivered(D1,P2,2) + 5 droneDelivered(D1,P3,2)
 + 5 droneDelivered(D1,P4,2) + droneDelivered(D1,P6,2) = -0
 updatedCAP(D1,3): - droneCurrentCap(D1,2) + droneCurrentCap(D1,3)
 - 10 droneCollected(D1,P1,3) - 10 droneCollected(D1,P2,3)
 - 5 droneCollected(D1,P3,3) - 5 droneCollected(D1,P4,3)
 - droneCollected(D1,P6,3) + 10 droneDelivered(D1,P1,3)
 + 10 droneDelivered(D1,P2,3) + 5 droneDelivered(D1,P3,3)
 + 5 droneDelivered(D1,P4,3) + droneDelivered(D1,P6,3) = -0
 updatedCAP(D1,4): - droneCurrentCap(D1,3) + droneCurrentCap(D1,4)
 - 10 droneCollected(D1,P1,4) - 10 droneCollected(D1,P2,4)
 - 5 droneCollected(D1,P3,4) - 5 droneCollected(D1,P4,4)
 - droneCollected(D1,P6,4) + 10 droneDelivered(D1,P1,4)
 + 10 droneDelivered(D1,P2,4) + 5 droneDelivered(D1,P3,4)
 + 5 droneDelivered(D1,P4,4) + droneDelivered(D1,P6,4) = -0
 updatedCAP(D1,5): - droneCurrentCap(D1,4) + droneCurrentCap(D1,5)
 - 10 droneCollected(D1,P1,5) - 10 droneCollected(D1,P2,5)
 - 5 droneCollected(D1,P3,5) - 5 droneCollected(D1,P4,5)
 - droneCollected(D1,P6,5) + 10 droneDelivered(D1,P1,5)
 + 10 droneDelivered(D1,P2,5) + 5 droneDelivered(D1,P3,5)
 + 5 droneDelivered(D1,P4,5) + droneDelivered(D1,P6,5) = -0
 updatedCAP(D2,2): - droneCurrentCap(D2,1) + droneCurrentCap(D2,2)
 - 10 droneCollected(D2,P1,2) - 10 droneCollected(D2,P2,2)
 - 5 droneCollected(D2,P3,2) - 5 droneCollected(D2,P4,2)
 - droneCollected(D2,P6,2) + 10 droneDelivered(D2,P1,2)
 + 10 droneDelivered(D2,P2,2) + 5 droneDelivered(D2,P3,2)
 + 5 droneDelivered(D2,P4,2) + droneDelivered(D2,P6,2) = -0
 updatedCAP(D2,3): - droneCurrentCap(D2,2) + droneCurrentCap(D2,3)
 - 10 droneCollected(D2,P1,3) - 10 droneCollected(D2,P2,3)
 - 5 droneCollected(D2,P3,3) - 5 droneCollected(D2,P4,3)
 - droneCollected(D2,P6,3) + 10 droneDelivered(D2,P1,3)
 + 10 droneDelivered(D2,P2,3) + 5 droneDelivered(D2,P3,3)
 + 5 droneDelivered(D2,P4,3) + droneDelivered(D2,P6,3) = -0
 updatedCAP(D2,4): - droneCurrentCap(D2,3) + droneCurrentCap(D2,4)
 - 10 droneCollected(D2,P1,4) - 10 droneCollected(D2,P2,4)
 - 5 droneCollected(D2,P3,4) - 5 droneCollected(D2,P4,4)
 - droneCollected(D2,P6,4) + 10 droneDelivered(D2,P1,4)
 + 10 droneDelivered(D2,P2,4) + 5 droneDelivered(D2,P3,4)
 + 5 droneDelivered(D2,P4,4) + droneDelivered(D2,P6,4) = -0
 updatedCAP(D2,5): - droneCurrentCap(D2,4) + droneCurrentCap(D2,5)
 - 10 droneCollected(D2,P1,5) - 10 droneCollected(D2,P2,5)
 - 5 droneCollected(D2,P3,5) - 5 droneCollected(D2,P4,5)
 - droneCollected(D2,P6,5) + 10 droneDelivered(D2,P1,5)
 + 10 droneDelivered(D2,P2,5) + 5 droneDelivered(D2,P3,5)
 + 5 droneDelivered(D2,P4,5) + droneDelivered(D2,P6,5) = -0
 updatedCAP(D3,2): - droneCurrentCap(D3,1) + droneCurrentCap(D3,2)
 - 10 droneCollected(D3,P1,2) - 10 droneCollected(D3,P2,2)
 - 5 droneCollected(D3,P3,2) - 5 droneCollected(D3,P4,2)
 - droneCollected(D3,P6,2) + 10 droneDelivered(D3,P1,2)
 + 10 droneDelivered(D3,P2,2) + 5 droneDelivered(D3,P3,2)
 + 5 droneDelivered(D3,P4,2) + droneDelivered(D3,P6,2) = -0
 updatedCAP(D3,3): - droneCurrentCap(D3,2) + droneCurrentCap(D3,3)
 - 10 droneCollected(D3,P1,3) - 10 droneCollected(D3,P2,3)
 - 5 droneCollected(D3,P3,3) - 5 droneCollected(D3,P4,3)
 - droneCollected(D3,P6,3) + 10 droneDelivered(D3,P1,3)
 + 10 droneDelivered(D3,P2,3) + 5 droneDelivered(D3,P3,3)
 + 5 droneDelivered(D3,P4,3) + droneDelivered(D3,P6,3) = -0
 updatedCAP(D3,4): - droneCurrentCap(D3,3) + droneCurrentCap(D3,4)
 - 10 droneCollected(D3,P1,4) - 10 droneCollected(D3,P2,4)
 - 5 droneCollected(D3,P3,4) - 5 droneCollected(D3,P4,4)
 - droneCollected(D3,P6,4) + 10 droneDelivered(D3,P1,4)
 + 10 droneDelivered(D3,P2,4) + 5 droneDelivered(D3,P3,4)
 + 5 droneDelivered(D3,P4,4) + droneDelivered(D3,P6,4) = -0
 updatedCAP(D3,5): - droneCurrentCap(D3,4) + droneCurrentCap(D3,5)
 - 10 droneCollected(D3,P1,5) - 10 droneCollected(D3,P2,5)
 - 5 droneCollected(D3,P3,5) - 5 droneCollected(D3,P4,5)
 - droneCollected(D3,P6,5) + 10 droneDelivered(D3,P1,5)
 + 10 droneDelivered(D3,P2,5) + 5 droneDelivered(D3,P3,5)
 + 5 droneDelivered(D3,P4,5) + droneDelivered(D3,P6,5) = -0
 updatedCAP1(D1,1): + droneCurrentCap(D1,1) - 10 droneCollected(D1,P1,1)
 - 10 droneCollected(D1,P2,1) - 5 droneCollected(D1,P3,1)
 - 5 droneCollected(D1,P4,1) - droneCollected(D1,P6,1)
 + 10 droneDelivered(D1,P1,1) + 10 droneDelivered(D1,P2,1)
 + 5 droneDelivered(D1,P3,1) + 5 droneDelivered(D1,P4,1)
 + droneDelivered(D1,P6,1) = -0
 updatedCAP1(D2,1): + droneCurrentCap(D2,1) - 10 droneCollected(D2,P1,1)
 - 10 droneCollected(D2,P2,1) - 5 droneCollected(D2,P3,1)
 - 5 droneCollected(D2,P4,1) - droneCollected(D2,P6,1)
 + 10 droneDelivered(D2,P1,1) + 10 droneDelivered(D2,P2,1)
 + 5 droneDelivered(D2,P3,1) + 5 droneDelivered(D2,P4,1)
 + droneDelivered(D2,P6,1) = -0
 updatedCAP1(D3,1): + droneCurrentCap(D3,1) - 10 droneCollected(D3,P1,1)
 - 10 droneCollected(D3,P2,1) - 5 droneCollected(D3,P3,1)
 - 5 droneCollected(D3,P4,1) - droneCollected(D3,P6,1)
 + 10 droneDelivered(D3,P1,1) + 10 droneDelivered(D3,P2,1)
 + 5 droneDelivered(D3,P3,1) + 5 droneDelivered(D3,P4,1)
 + droneDelivered(D3,P6,1) = -0
 droneShouldCollect(P1): + 1000000 droneCollected(D1,P1,2)
 + 1000000 droneCollected(D1,P1,3) + 1000000 droneCollected(D1,P1,4)
 + 1000000 droneCollected(D1,P1,5) + 1000000 droneCollected(D2,P1,2)
 + 1000000 droneCollected(D2,P1,3) + 1000000 droneCollected(D2,P1,4)
 + 1000000 droneCollected(D2,P1,5) + 1000000 droneCollected(D3,P1,2)
 + 1000000 droneCollected(D3,P1,3) + 1000000 droneCollected(D3,P1,4)
 + 1000000 droneCollected(D3,P1,5) + 1000000 droneCollected(D1,P1,1)
 + 1000000 droneCollected(D2,P1,1) + 1000000 droneCollected(D3,P1,1)
 >= 10
 droneShouldCollect(P2): + 1000000 droneCollected(D1,P2,2)
 + 1000000 droneCollected(D1,P2,3) + 1000000 droneCollected(D1,P2,4)
 + 1000000 droneCollected(D1,P2,5) + 1000000 droneCollected(D2,P2,2)
 + 1000000 droneCollected(D2,P2,3) + 1000000 droneCollected(D2,P2,4)
 + 1000000 droneCollected(D2,P2,5) + 1000000 droneCollected(D3,P2,2)
 + 1000000 droneCollected(D3,P2,3) + 1000000 droneCollected(D3,P2,4)
 + 1000000 droneCollected(D3,P2,5) + 1000000 droneCollected(D1,P2,1)
 + 1000000 droneCollected(D2,P2,1) + 1000000 droneCollected(D3,P2,1)
 >= 10
 droneShouldCollect(P3): + 1000000 droneCollected(D1,P3,2)
 + 1000000 droneCollected(D1,P3,3) + 1000000 droneCollected(D1,P3,4)
 + 1000000 droneCollected(D1,P3,5) + 1000000 droneCollected(D2,P3,2)
 + 1000000 droneCollected(D2,P3,3) + 1000000 droneCollected(D2,P3,4)
 + 1000000 droneCollected(D2,P3,5) + 1000000 droneCollected(D3,P3,2)
 + 1000000 droneCollected(D3,P3,3) + 1000000 droneCollected(D3,P3,4)
 + 1000000 droneCollected(D3,P3,5) + 1000000 droneCollected(D1,P3,1)
 + 1000000 droneCollected(D2,P3,1) + 1000000 droneCollected(D3,P3,1)
 >= 5
 droneShouldCollect(P4): + 1000000 droneCollected(D1,P4,2)
 + 1000000 droneCollected(D1,P4,3) + 1000000 droneCollected(D1,P4,4)
 + 1000000 droneCollected(D1,P4,5) + 1000000 droneCollected(D2,P4,2)
 + 1000000 droneCollected(D2,P4,3) + 1000000 droneCollected(D2,P4,4)
 + 1000000 droneCollected(D2,P4,5) + 1000000 droneCollected(D3,P4,2)
 + 1000000 droneCollected(D3,P4,3) + 1000000 droneCollected(D3,P4,4)
 + 1000000 droneCollected(D3,P4,5) + 1000000 droneCollected(D1,P4,1)
 + 1000000 droneCollected(D2,P4,1) + 1000000 droneCollected(D3,P4,1)
 >= 5
 droneShouldCollect(P5): + 1000000 droneCollected(D1,P5,2)
 + 1000000 droneCollected(D1,P5,3) + 1000000 droneCollected(D1,P5,4)
 + 1000000 droneCollected(D1,P5,5) + 1000000 droneCollected(D2,P5,2)
 + 1000000 droneCollected(D2,P5,3) + 1000000 droneCollected(D2,P5,4)
 + 1000000 droneCollected(D2,P5,5) + 1000000 droneCollected(D3,P5,2)
 + 1000000 droneCollected(D3,P5,3) + 1000000 droneCollected(D3,P5,4)
 + 1000000 droneCollected(D3,P5,5) + 1000000 droneCollected(D1,P5,1)
 + 1000000 droneCollected(D2,P5,1) + 1000000 droneCollected(D3,P5,1)
 >= -0
 droneShouldCollect(P6): + 1000000 droneCollected(D1,P6,2)
 + 1000000 droneCollected(D1,P6,3) + 1000000 droneCollected(D1,P6,4)
 + 1000000 droneCollected(D1,P6,5) + 1000000 droneCollected(D2,P6,2)
 + 1000000 droneCollected(D2,P6,3) + 1000000 droneCollected(D2,P6,4)
 + 1000000 droneCollected(D2,P6,5) + 1000000 droneCollected(D3,P6,2)
 + 1000000 droneCollected(D3,P6,3) + 1000000 droneCollected(D3,P6,4)
 + 1000000 droneCollected(D3,P6,5) + 1000000 droneCollected(D1,P6,1)
 + 1000000 droneCollected(D2,P6,1) + 1000000 droneCollected(D3,P6,1)
 >= 1
 droneShouldCollectAlone(P1): + droneCollected(D1,P1,2)
 + droneCollected(D1,P1,3) + droneCollected(D1,P1,4)
 + droneCollected(D1,P1,5) + droneCollected(D2,P1,2)
 + droneCollected(D2,P1,3) + droneCollected(D2,P1,4)
 + droneCollected(D2,P1,5) + droneCollected(D3,P1,2)
 + droneCollected(D3,P1,3) + droneCollected(D3,P1,4)
 + droneCollected(D3,P1,5) + droneCollected(D1,P1,1)
 + droneCollected(D2,P1,1) + droneCollected(D3,P1,1) <= 1
 droneShouldCollectAlone(P2): + droneCollected(D1,P2,2)
 + droneCollected(D1,P2,3) + droneCollected(D1,P2,4)
 + droneCollected(D1,P2,5) + droneCollected(D2,P2,2)
 + droneCollected(D2,P2,3) + droneCollected(D2,P2,4)
 + droneCollected(D2,P2,5) + droneCollected(D3,P2,2)
 + droneCollected(D3,P2,3) + droneCollected(D3,P2,4)
 + droneCollected(D3,P2,5) + droneCollected(D1,P2,1)
 + droneCollected(D2,P2,1) + droneCollected(D3,P2,1) <= 1
 droneShouldCollectAlone(P3): + droneCollected(D1,P3,2)
 + droneCollected(D1,P3,3) + droneCollected(D1,P3,4)
 + droneCollected(D1,P3,5) + droneCollected(D2,P3,2)
 + droneCollected(D2,P3,3) + droneCollected(D2,P3,4)
 + droneCollected(D2,P3,5) + droneCollected(D3,P3,2)
 + droneCollected(D3,P3,3) + droneCollected(D3,P3,4)
 + droneCollected(D3,P3,5) + droneCollected(D1,P3,1)
 + droneCollected(D2,P3,1) + droneCollected(D3,P3,1) <= 1
 droneShouldCollectAlone(P4): + droneCollected(D1,P4,2)
 + droneCollected(D1,P4,3) + droneCollected(D1,P4,4)
 + droneCollected(D1,P4,5) + droneCollected(D2,P4,2)
 + droneCollected(D2,P4,3) + droneCollected(D2,P4,4)
 + droneCollected(D2,P4,5) + droneCollected(D3,P4,2)
 + droneCollected(D3,P4,3) + droneCollected(D3,P4,4)
 + droneCollected(D3,P4,5) + droneCollected(D1,P4,1)
 + droneCollected(D2,P4,1) + droneCollected(D3,P4,1) <= 1
 droneShouldCollectAlone(P5): + droneCollected(D1,P5,2)
 + droneCollected(D1,P5,3) + droneCollected(D1,P5,4)
 + droneCollected(D1,P5,5) + droneCollected(D2,P5,2)
 + droneCollected(D2,P5,3) + droneCollected(D2,P5,4)
 + droneCollected(D2,P5,5) + droneCollected(D3,P5,2)
 + droneCollected(D3,P5,3) + droneCollected(D3,P5,4)
 + droneCollected(D3,P5,5) + droneCollected(D1,P5,1)
 + droneCollected(D2,P5,1) + droneCollected(D3,P5,1) <= 1
 droneShouldCollectAlone(P6): + droneCollected(D1,P6,2)
 + droneCollected(D1,P6,3) + droneCollected(D1,P6,4)
 + droneCollected(D1,P6,5) + droneCollected(D2,P6,2)
 + droneCollected(D2,P6,3) + droneCollected(D2,P6,4)
 + droneCollected(D2,P6,5) + droneCollected(D3,P6,2)
 + droneCollected(D3,P6,3) + droneCollected(D3,P6,4)
 + droneCollected(D3,P6,5) + droneCollected(D1,P6,1)
 + droneCollected(D2,P6,1) + droneCollected(D3,P6,1) <= 1
 droneShouldCollectAndDeliver(P1,D1): - droneCollected(D1,P1,2)
 - droneCollected(D1,P1,3) - droneCollected(D1,P1,4)
 - droneCollected(D1,P1,5) - droneCollected(D1,P1,1)
 + droneDelivered(D1,P1,2) + droneDelivered(D1,P1,3)
 + droneDelivered(D1,P1,4) + droneDelivered(D1,P1,5)
 + droneDelivered(D1,P1,1) = -0
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
 droneDeliverAfterCollecting(P1,D1,2): + droneDelivered(D1,P1,2)
 - droneIsDelivering(D1,P1,1) >= -0
 droneDeliverAfterCollecting(P1,D1,3): + droneDelivered(D1,P1,3)
 - droneIsDelivering(D1,P1,2) >= -0
 droneDeliverAfterCollecting(P1,D1,4): + droneDelivered(D1,P1,4)
 - droneIsDelivering(D1,P1,3) >= -0
 droneDeliverAfterCollecting(P1,D1,5): + droneDelivered(D1,P1,5)
 - droneIsDelivering(D1,P1,4) >= -0
 droneDeliverAfterCollecting(P1,D2,2): + droneDelivered(D2,P1,2)
 - droneIsDelivering(D2,P1,1) >= -0
 droneDeliverAfterCollecting(P1,D2,3): + droneDelivered(D2,P1,3)
 - droneIsDelivering(D2,P1,2) >= -0
 droneDeliverAfterCollecting(P1,D2,4): + droneDelivered(D2,P1,4)
 - droneIsDelivering(D2,P1,3) >= -0
 droneDeliverAfterCollecting(P1,D2,5): + droneDelivered(D2,P1,5)
 - droneIsDelivering(D2,P1,4) >= -0
 droneDeliverAfterCollecting(P1,D3,2): + droneDelivered(D3,P1,2)
 - droneIsDelivering(D3,P1,1) >= -0
 droneDeliverAfterCollecting(P1,D3,3): + droneDelivered(D3,P1,3)
 - droneIsDelivering(D3,P1,2) >= -0
 droneDeliverAfterCollecting(P1,D3,4): + droneDelivered(D3,P1,4)
 - droneIsDelivering(D3,P1,3) >= -0
 droneDeliverAfterCollecting(P1,D3,5): + droneDelivered(D3,P1,5)
 - droneIsDelivering(D3,P1,4) >= -0
 droneDeliverAfterCollecting(P2,D1,2): + droneDelivered(D1,P2,2)
 - droneIsDelivering(D1,P2,1) >= -0
 droneDeliverAfterCollecting(P2,D1,3): + droneDelivered(D1,P2,3)
 - droneIsDelivering(D1,P2,2) >= -0
 droneDeliverAfterCollecting(P2,D1,4): + droneDelivered(D1,P2,4)
 - droneIsDelivering(D1,P2,3) >= -0
 droneDeliverAfterCollecting(P2,D1,5): + droneDelivered(D1,P2,5)
 - droneIsDelivering(D1,P2,4) >= -0
 droneDeliverAfterCollecting(P2,D2,2): + droneDelivered(D2,P2,2)
 - droneIsDelivering(D2,P2,1) >= -0
 droneDeliverAfterCollecting(P2,D2,3): + droneDelivered(D2,P2,3)
 - droneIsDelivering(D2,P2,2) >= -0
 droneDeliverAfterCollecting(P2,D2,4): + droneDelivered(D2,P2,4)
 - droneIsDelivering(D2,P2,3) >= -0
 droneDeliverAfterCollecting(P2,D2,5): + droneDelivered(D2,P2,5)
 - droneIsDelivering(D2,P2,4) >= -0
 droneDeliverAfterCollecting(P2,D3,2): + droneDelivered(D3,P2,2)
 - droneIsDelivering(D3,P2,1) >= -0
 droneDeliverAfterCollecting(P2,D3,3): + droneDelivered(D3,P2,3)
 - droneIsDelivering(D3,P2,2) >= -0
 droneDeliverAfterCollecting(P2,D3,4): + droneDelivered(D3,P2,4)
 - droneIsDelivering(D3,P2,3) >= -0
 droneDeliverAfterCollecting(P2,D3,5): + droneDelivered(D3,P2,5)
 - droneIsDelivering(D3,P2,4) >= -0
 droneDeliverAfterCollecting(P3,D1,2): + droneDelivered(D1,P3,2)
 - droneIsDelivering(D1,P3,1) >= -0
 droneDeliverAfterCollecting(P3,D1,3): + droneDelivered(D1,P3,3)
 - droneIsDelivering(D1,P3,2) >= -0
 droneDeliverAfterCollecting(P3,D1,4): + droneDelivered(D1,P3,4)
 - droneIsDelivering(D1,P3,3) >= -0
 droneDeliverAfterCollecting(P3,D1,5): + droneDelivered(D1,P3,5)
 - droneIsDelivering(D1,P3,4) >= -0
 droneDeliverAfterCollecting(P3,D2,2): + droneDelivered(D2,P3,2)
 - droneIsDelivering(D2,P3,1) >= -0
 droneDeliverAfterCollecting(P3,D2,3): + droneDelivered(D2,P3,3)
 - droneIsDelivering(D2,P3,2) >= -0
 droneDeliverAfterCollecting(P3,D2,4): + droneDelivered(D2,P3,4)
 - droneIsDelivering(D2,P3,3) >= -0
 droneDeliverAfterCollecting(P3,D2,5): + droneDelivered(D2,P3,5)
 - droneIsDelivering(D2,P3,4) >= -0
 droneDeliverAfterCollecting(P3,D3,2): + droneDelivered(D3,P3,2)
 - droneIsDelivering(D3,P3,1) >= -0
 droneDeliverAfterCollecting(P3,D3,3): + droneDelivered(D3,P3,3)
 - droneIsDelivering(D3,P3,2) >= -0
 droneDeliverAfterCollecting(P3,D3,4): + droneDelivered(D3,P3,4)
 - droneIsDelivering(D3,P3,3) >= -0
 droneDeliverAfterCollecting(P3,D3,5): + droneDelivered(D3,P3,5)
 - droneIsDelivering(D3,P3,4) >= -0
 droneDeliverAfterCollecting(P4,D1,2): + droneDelivered(D1,P4,2)
 - droneIsDelivering(D1,P4,1) >= -0
 droneDeliverAfterCollecting(P4,D1,3): + droneDelivered(D1,P4,3)
 - droneIsDelivering(D1,P4,2) >= -0
 droneDeliverAfterCollecting(P4,D1,4): + droneDelivered(D1,P4,4)
 - droneIsDelivering(D1,P4,3) >= -0
 droneDeliverAfterCollecting(P4,D1,5): + droneDelivered(D1,P4,5)
 - droneIsDelivering(D1,P4,4) >= -0
 droneDeliverAfterCollecting(P4,D2,2): + droneDelivered(D2,P4,2)
 - droneIsDelivering(D2,P4,1) >= -0
 droneDeliverAfterCollecting(P4,D2,3): + droneDelivered(D2,P4,3)
 - droneIsDelivering(D2,P4,2) >= -0
 droneDeliverAfterCollecting(P4,D2,4): + droneDelivered(D2,P4,4)
 - droneIsDelivering(D2,P4,3) >= -0
 droneDeliverAfterCollecting(P4,D2,5): + droneDelivered(D2,P4,5)
 - droneIsDelivering(D2,P4,4) >= -0
 droneDeliverAfterCollecting(P4,D3,2): + droneDelivered(D3,P4,2)
 - droneIsDelivering(D3,P4,1) >= -0
 droneDeliverAfterCollecting(P4,D3,3): + droneDelivered(D3,P4,3)
 - droneIsDelivering(D3,P4,2) >= -0
 droneDeliverAfterCollecting(P4,D3,4): + droneDelivered(D3,P4,4)
 - droneIsDelivering(D3,P4,3) >= -0
 droneDeliverAfterCollecting(P4,D3,5): + droneDelivered(D3,P4,5)
 - droneIsDelivering(D3,P4,4) >= -0
 droneDeliverAfterCollecting(P5,D1,2): + droneDelivered(D1,P5,2)
 - droneIsDelivering(D1,P5,1) >= -0
 droneDeliverAfterCollecting(P5,D1,3): + droneDelivered(D1,P5,3)
 - droneIsDelivering(D1,P5,2) >= -0
 droneDeliverAfterCollecting(P5,D1,4): + droneDelivered(D1,P5,4)
 - droneIsDelivering(D1,P5,3) >= -0
 droneDeliverAfterCollecting(P5,D1,5): + droneDelivered(D1,P5,5)
 - droneIsDelivering(D1,P5,4) >= -0
 droneDeliverAfterCollecting(P5,D2,2): + droneDelivered(D2,P5,2)
 - droneIsDelivering(D2,P5,1) >= -0
 droneDeliverAfterCollecting(P5,D2,3): + droneDelivered(D2,P5,3)
 - droneIsDelivering(D2,P5,2) >= -0
 droneDeliverAfterCollecting(P5,D2,4): + droneDelivered(D2,P5,4)
 - droneIsDelivering(D2,P5,3) >= -0
 droneDeliverAfterCollecting(P5,D2,5): + droneDelivered(D2,P5,5)
 - droneIsDelivering(D2,P5,4) >= -0
 droneDeliverAfterCollecting(P5,D3,2): + droneDelivered(D3,P5,2)
 - droneIsDelivering(D3,P5,1) >= -0
 droneDeliverAfterCollecting(P5,D3,3): + droneDelivered(D3,P5,3)
 - droneIsDelivering(D3,P5,2) >= -0
 droneDeliverAfterCollecting(P5,D3,4): + droneDelivered(D3,P5,4)
 - droneIsDelivering(D3,P5,3) >= -0
 droneDeliverAfterCollecting(P5,D3,5): + droneDelivered(D3,P5,5)
 - droneIsDelivering(D3,P5,4) >= -0
 droneDeliverAfterCollecting(P6,D1,2): + droneDelivered(D1,P6,2)
 - droneIsDelivering(D1,P6,1) >= -0
 droneDeliverAfterCollecting(P6,D1,3): + droneDelivered(D1,P6,3)
 - droneIsDelivering(D1,P6,2) >= -0
 droneDeliverAfterCollecting(P6,D1,4): + droneDelivered(D1,P6,4)
 - droneIsDelivering(D1,P6,3) >= -0
 droneDeliverAfterCollecting(P6,D1,5): + droneDelivered(D1,P6,5)
 - droneIsDelivering(D1,P6,4) >= -0
 droneDeliverAfterCollecting(P6,D2,2): + droneDelivered(D2,P6,2)
 - droneIsDelivering(D2,P6,1) >= -0
 droneDeliverAfterCollecting(P6,D2,3): + droneDelivered(D2,P6,3)
 - droneIsDelivering(D2,P6,2) >= -0
 droneDeliverAfterCollecting(P6,D2,4): + droneDelivered(D2,P6,4)
 - droneIsDelivering(D2,P6,3) >= -0
 droneDeliverAfterCollecting(P6,D2,5): + droneDelivered(D2,P6,5)
 - droneIsDelivering(D2,P6,4) >= -0
 droneDeliverAfterCollecting(P6,D3,2): + droneDelivered(D3,P6,2)
 - droneIsDelivering(D3,P6,1) >= -0
 droneDeliverAfterCollecting(P6,D3,3): + droneDelivered(D3,P6,3)
 - droneIsDelivering(D3,P6,2) >= -0
 droneDeliverAfterCollecting(P6,D3,4): + droneDelivered(D3,P6,4)
 - droneIsDelivering(D3,P6,3) >= -0
 droneDeliverAfterCollecting(P6,D3,5): + droneDelivered(D3,P6,5)
 - droneIsDelivering(D3,P6,4) >= -0
 droneIsBeingUsed(D1): - droneCollected(D1,P1,2)
 - droneCollected(D1,P2,2) - droneCollected(D1,P3,2)
 - droneCollected(D1,P4,2) - droneCollected(D1,P5,2)
 - droneCollected(D1,P6,2) - droneCollected(D1,P1,3)
 - droneCollected(D1,P2,3) - droneCollected(D1,P3,3)
 - droneCollected(D1,P4,3) - droneCollected(D1,P5,3)
 - droneCollected(D1,P6,3) - droneCollected(D1,P1,4)
 - droneCollected(D1,P2,4) - droneCollected(D1,P3,4)
 - droneCollected(D1,P4,4) - droneCollected(D1,P5,4)
 - droneCollected(D1,P6,4) - droneCollected(D1,P1,5)
 - droneCollected(D1,P2,5) - droneCollected(D1,P3,5)
 - droneCollected(D1,P4,5) - droneCollected(D1,P5,5)
 - droneCollected(D1,P6,5) - droneCollected(D1,P1,1)
 - droneCollected(D1,P2,1) - droneCollected(D1,P3,1)
 - droneCollected(D1,P4,1) - droneCollected(D1,P5,1)
 - droneCollected(D1,P6,1) + 1000000 droneIsUsed(D1) >= -0
 droneIsBeingUsed(D2): - droneCollected(D2,P1,2)
 - droneCollected(D2,P2,2) - droneCollected(D2,P3,2)
 - droneCollected(D2,P4,2) - droneCollected(D2,P5,2)
 - droneCollected(D2,P6,2) - droneCollected(D2,P1,3)
 - droneCollected(D2,P2,3) - droneCollected(D2,P3,3)
 - droneCollected(D2,P4,3) - droneCollected(D2,P5,3)
 - droneCollected(D2,P6,3) - droneCollected(D2,P1,4)
 - droneCollected(D2,P2,4) - droneCollected(D2,P3,4)
 - droneCollected(D2,P4,4) - droneCollected(D2,P5,4)
 - droneCollected(D2,P6,4) - droneCollected(D2,P1,5)
 - droneCollected(D2,P2,5) - droneCollected(D2,P3,5)
 - droneCollected(D2,P4,5) - droneCollected(D2,P5,5)
 - droneCollected(D2,P6,5) - droneCollected(D2,P1,1)
 - droneCollected(D2,P2,1) - droneCollected(D2,P3,1)
 - droneCollected(D2,P4,1) - droneCollected(D2,P5,1)
 - droneCollected(D2,P6,1) + 1000000 droneIsUsed(D2) >= -0
 droneIsBeingUsed(D3): - droneCollected(D3,P1,2)
 - droneCollected(D3,P2,2) - droneCollected(D3,P3,2)
 - droneCollected(D3,P4,2) - droneCollected(D3,P5,2)
 - droneCollected(D3,P6,2) - droneCollected(D3,P1,3)
 - droneCollected(D3,P2,3) - droneCollected(D3,P3,3)
 - droneCollected(D3,P4,3) - droneCollected(D3,P5,3)
 - droneCollected(D3,P6,3) - droneCollected(D3,P1,4)
 - droneCollected(D3,P2,4) - droneCollected(D3,P3,4)
 - droneCollected(D3,P4,4) - droneCollected(D3,P5,4)
 - droneCollected(D3,P6,4) - droneCollected(D3,P1,5)
 - droneCollected(D3,P2,5) - droneCollected(D3,P3,5)
 - droneCollected(D3,P4,5) - droneCollected(D3,P5,5)
 - droneCollected(D3,P6,5) - droneCollected(D3,P1,1)
 - droneCollected(D3,P2,1) - droneCollected(D3,P3,1)
 - droneCollected(D3,P4,1) - droneCollected(D3,P5,1)
 - droneCollected(D3,P6,1) + 1000000 droneIsUsed(D3) >= -0
 updatedDelivering(P1,D1,2): - droneCollected(D1,P1,2)
 + droneDelivered(D1,P1,2) - droneIsDelivering(D1,P1,1)
 + droneIsDelivering(D1,P1,2) = -0
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
 + droneDelivered(D2,P1,2) - droneIsDelivering(D2,P1,1)
 + droneIsDelivering(D2,P1,2) = -0
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
 + droneDelivered(D3,P1,2) - droneIsDelivering(D3,P1,1)
 + droneIsDelivering(D3,P1,2) = -0
 updatedDelivering(P1,D3,3): - droneCollected(D3,P1,3)
 + droneDelivered(D3,P1,3) - droneIsDelivering(D3,P1,2)
 + droneIsDelivering(D3,P1,3) = -0
 updatedDelivering(P1,D3,4): - droneCollected(D3,P1,4)
 + droneDelivered(D3,P1,4) - droneIsDelivering(D3,P1,3)
 + droneIsDelivering(D3,P1,4) = -0
 updatedDelivering(P1,D3,5): - droneCollected(D3,P1,5)
 + droneDelivered(D3,P1,5) - droneIsDelivering(D3,P1,4)
 + droneIsDelivering(D3,P1,5) = -0
 updatedDelivering(P2,D1,2): - droneCollected(D1,P2,2)
 + droneDelivered(D1,P2,2) - droneIsDelivering(D1,P2,1)
 + droneIsDelivering(D1,P2,2) = -0
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
 + droneDelivered(D2,P2,2) - droneIsDelivering(D2,P2,1)
 + droneIsDelivering(D2,P2,2) = -0
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
 + droneDelivered(D3,P2,2) - droneIsDelivering(D3,P2,1)
 + droneIsDelivering(D3,P2,2) = -0
 updatedDelivering(P2,D3,3): - droneCollected(D3,P2,3)
 + droneDelivered(D3,P2,3) - droneIsDelivering(D3,P2,2)
 + droneIsDelivering(D3,P2,3) = -0
 updatedDelivering(P2,D3,4): - droneCollected(D3,P2,4)
 + droneDelivered(D3,P2,4) - droneIsDelivering(D3,P2,3)
 + droneIsDelivering(D3,P2,4) = -0
 updatedDelivering(P2,D3,5): - droneCollected(D3,P2,5)
 + droneDelivered(D3,P2,5) - droneIsDelivering(D3,P2,4)
 + droneIsDelivering(D3,P2,5) = -0
 updatedDelivering(P3,D1,2): - droneCollected(D1,P3,2)
 + droneDelivered(D1,P3,2) - droneIsDelivering(D1,P3,1)
 + droneIsDelivering(D1,P3,2) = -0
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
 + droneDelivered(D2,P3,2) - droneIsDelivering(D2,P3,1)
 + droneIsDelivering(D2,P3,2) = -0
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
 + droneDelivered(D3,P3,2) - droneIsDelivering(D3,P3,1)
 + droneIsDelivering(D3,P3,2) = -0
 updatedDelivering(P3,D3,3): - droneCollected(D3,P3,3)
 + droneDelivered(D3,P3,3) - droneIsDelivering(D3,P3,2)
 + droneIsDelivering(D3,P3,3) = -0
 updatedDelivering(P3,D3,4): - droneCollected(D3,P3,4)
 + droneDelivered(D3,P3,4) - droneIsDelivering(D3,P3,3)
 + droneIsDelivering(D3,P3,4) = -0
 updatedDelivering(P3,D3,5): - droneCollected(D3,P3,5)
 + droneDelivered(D3,P3,5) - droneIsDelivering(D3,P3,4)
 + droneIsDelivering(D3,P3,5) = -0
 updatedDelivering(P4,D1,2): - droneCollected(D1,P4,2)
 + droneDelivered(D1,P4,2) - droneIsDelivering(D1,P4,1)
 + droneIsDelivering(D1,P4,2) = -0
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
 + droneDelivered(D2,P4,2) - droneIsDelivering(D2,P4,1)
 + droneIsDelivering(D2,P4,2) = -0
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
 + droneDelivered(D3,P4,2) - droneIsDelivering(D3,P4,1)
 + droneIsDelivering(D3,P4,2) = -0
 updatedDelivering(P4,D3,3): - droneCollected(D3,P4,3)
 + droneDelivered(D3,P4,3) - droneIsDelivering(D3,P4,2)
 + droneIsDelivering(D3,P4,3) = -0
 updatedDelivering(P4,D3,4): - droneCollected(D3,P4,4)
 + droneDelivered(D3,P4,4) - droneIsDelivering(D3,P4,3)
 + droneIsDelivering(D3,P4,4) = -0
 updatedDelivering(P4,D3,5): - droneCollected(D3,P4,5)
 + droneDelivered(D3,P4,5) - droneIsDelivering(D3,P4,4)
 + droneIsDelivering(D3,P4,5) = -0
 updatedDelivering(P5,D1,2): - droneCollected(D1,P5,2)
 + droneDelivered(D1,P5,2) - droneIsDelivering(D1,P5,1)
 + droneIsDelivering(D1,P5,2) = -0
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
 + droneDelivered(D2,P5,2) - droneIsDelivering(D2,P5,1)
 + droneIsDelivering(D2,P5,2) = -0
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
 + droneDelivered(D3,P5,2) - droneIsDelivering(D3,P5,1)
 + droneIsDelivering(D3,P5,2) = -0
 updatedDelivering(P5,D3,3): - droneCollected(D3,P5,3)
 + droneDelivered(D3,P5,3) - droneIsDelivering(D3,P5,2)
 + droneIsDelivering(D3,P5,3) = -0
 updatedDelivering(P5,D3,4): - droneCollected(D3,P5,4)
 + droneDelivered(D3,P5,4) - droneIsDelivering(D3,P5,3)
 + droneIsDelivering(D3,P5,4) = -0
 updatedDelivering(P5,D3,5): - droneCollected(D3,P5,5)
 + droneDelivered(D3,P5,5) - droneIsDelivering(D3,P5,4)
 + droneIsDelivering(D3,P5,5) = -0
 updatedDelivering(P6,D1,2): - droneCollected(D1,P6,2)
 + droneDelivered(D1,P6,2) - droneIsDelivering(D1,P6,1)
 + droneIsDelivering(D1,P6,2) = -0
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
 + droneDelivered(D2,P6,2) - droneIsDelivering(D2,P6,1)
 + droneIsDelivering(D2,P6,2) = -0
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
 + droneDelivered(D3,P6,2) - droneIsDelivering(D3,P6,1)
 + droneIsDelivering(D3,P6,2) = -0
 updatedDelivering(P6,D3,3): - droneCollected(D3,P6,3)
 + droneDelivered(D3,P6,3) - droneIsDelivering(D3,P6,2)
 + droneIsDelivering(D3,P6,3) = -0
 updatedDelivering(P6,D3,4): - droneCollected(D3,P6,4)
 + droneDelivered(D3,P6,4) - droneIsDelivering(D3,P6,3)
 + droneIsDelivering(D3,P6,4) = -0
 updatedDelivering(P6,D3,5): - droneCollected(D3,P6,5)
 + droneDelivered(D3,P6,5) - droneIsDelivering(D3,P6,4)
 + droneIsDelivering(D3,P6,5) = -0
 updatedDelivering2(P1,D1,1): - droneCollected(D1,P1,1)
 + droneIsDelivering(D1,P1,1) = -0
 updatedDelivering2(P1,D2,1): - droneCollected(D2,P1,1)
 + droneIsDelivering(D2,P1,1) = -0
 updatedDelivering2(P1,D3,1): - droneCollected(D3,P1,1)
 + droneIsDelivering(D3,P1,1) = -0
 updatedDelivering2(P2,D1,1): - droneCollected(D1,P2,1)
 + droneIsDelivering(D1,P2,1) = -0
 updatedDelivering2(P2,D2,1): - droneCollected(D2,P2,1)
 + droneIsDelivering(D2,P2,1) = -0
 updatedDelivering2(P2,D3,1): - droneCollected(D3,P2,1)
 + droneIsDelivering(D3,P2,1) = -0
 updatedDelivering2(P3,D1,1): - droneCollected(D1,P3,1)
 + droneIsDelivering(D1,P3,1) = -0
 updatedDelivering2(P3,D2,1): - droneCollected(D2,P3,1)
 + droneIsDelivering(D2,P3,1) = -0
 updatedDelivering2(P3,D3,1): - droneCollected(D3,P3,1)
 + droneIsDelivering(D3,P3,1) = -0
 updatedDelivering2(P4,D1,1): - droneCollected(D1,P4,1)
 + droneIsDelivering(D1,P4,1) = -0
 updatedDelivering2(P4,D2,1): - droneCollected(D2,P4,1)
 + droneIsDelivering(D2,P4,1) = -0
 updatedDelivering2(P4,D3,1): - droneCollected(D3,P4,1)
 + droneIsDelivering(D3,P4,1) = -0
 updatedDelivering2(P5,D1,1): - droneCollected(D1,P5,1)
 + droneIsDelivering(D1,P5,1) = -0
 updatedDelivering2(P5,D2,1): - droneCollected(D2,P5,1)
 + droneIsDelivering(D2,P5,1) = -0
 updatedDelivering2(P5,D3,1): - droneCollected(D3,P5,1)
 + droneIsDelivering(D3,P5,1) = -0
 updatedDelivering2(P6,D1,1): - droneCollected(D1,P6,1)
 + droneIsDelivering(D1,P6,1) = -0
 updatedDelivering2(P6,D2,1): - droneCollected(D2,P6,1)
 + droneIsDelivering(D2,P6,1) = -0
 updatedDelivering2(P6,D3,1): - droneCollected(D3,P6,1)
 + droneIsDelivering(D3,P6,1) = -0
 productDeliveringTime(P1): - droneIsDelivering(D1,P1,1)
 - droneIsDelivering(D1,P1,2) - droneIsDelivering(D1,P1,3)
 - droneIsDelivering(D1,P1,4) - droneIsDelivering(D2,P1,1)
 - droneIsDelivering(D2,P1,2) - droneIsDelivering(D2,P1,3)
 - droneIsDelivering(D2,P1,4) - droneIsDelivering(D3,P1,1)
 - droneIsDelivering(D3,P1,2) - droneIsDelivering(D3,P1,3)
 - droneIsDelivering(D3,P1,4) - droneIsDelivering(D1,P1,5)
 - droneIsDelivering(D2,P1,5) - droneIsDelivering(D3,P1,5)
 + timePerProductDelivered(P1) = -0
 productDeliveringTime(P2): - droneIsDelivering(D1,P2,1)
 - droneIsDelivering(D1,P2,2) - droneIsDelivering(D1,P2,3)
 - droneIsDelivering(D1,P2,4) - droneIsDelivering(D2,P2,1)
 - droneIsDelivering(D2,P2,2) - droneIsDelivering(D2,P2,3)
 - droneIsDelivering(D2,P2,4) - droneIsDelivering(D3,P2,1)
 - droneIsDelivering(D3,P2,2) - droneIsDelivering(D3,P2,3)
 - droneIsDelivering(D3,P2,4) - droneIsDelivering(D1,P2,5)
 - droneIsDelivering(D2,P2,5) - droneIsDelivering(D3,P2,5)
 + timePerProductDelivered(P2) = -0
 productDeliveringTime(P3): - droneIsDelivering(D1,P3,1)
 - droneIsDelivering(D1,P3,2) - droneIsDelivering(D1,P3,3)
 - droneIsDelivering(D1,P3,4) - droneIsDelivering(D2,P3,1)
 - droneIsDelivering(D2,P3,2) - droneIsDelivering(D2,P3,3)
 - droneIsDelivering(D2,P3,4) - droneIsDelivering(D3,P3,1)
 - droneIsDelivering(D3,P3,2) - droneIsDelivering(D3,P3,3)
 - droneIsDelivering(D3,P3,4) - droneIsDelivering(D1,P3,5)
 - droneIsDelivering(D2,P3,5) - droneIsDelivering(D3,P3,5)
 + timePerProductDelivered(P3) = -0
 productDeliveringTime(P4): - droneIsDelivering(D1,P4,1)
 - droneIsDelivering(D1,P4,2) - droneIsDelivering(D1,P4,3)
 - droneIsDelivering(D1,P4,4) - droneIsDelivering(D2,P4,1)
 - droneIsDelivering(D2,P4,2) - droneIsDelivering(D2,P4,3)
 - droneIsDelivering(D2,P4,4) - droneIsDelivering(D3,P4,1)
 - droneIsDelivering(D3,P4,2) - droneIsDelivering(D3,P4,3)
 - droneIsDelivering(D3,P4,4) - droneIsDelivering(D1,P4,5)
 - droneIsDelivering(D2,P4,5) - droneIsDelivering(D3,P4,5)
 + timePerProductDelivered(P4) = -0
 productDeliveringTime(P5): - droneIsDelivering(D1,P5,1)
 - droneIsDelivering(D1,P5,2) - droneIsDelivering(D1,P5,3)
 - droneIsDelivering(D1,P5,4) - droneIsDelivering(D2,P5,1)
 - droneIsDelivering(D2,P5,2) - droneIsDelivering(D2,P5,3)
 - droneIsDelivering(D2,P5,4) - droneIsDelivering(D3,P5,1)
 - droneIsDelivering(D3,P5,2) - droneIsDelivering(D3,P5,3)
 - droneIsDelivering(D3,P5,4) - droneIsDelivering(D1,P5,5)
 - droneIsDelivering(D2,P5,5) - droneIsDelivering(D3,P5,5)
 + timePerProductDelivered(P5) = -0
 productDeliveringTime(P6): - droneIsDelivering(D1,P6,1)
 - droneIsDelivering(D1,P6,2) - droneIsDelivering(D1,P6,3)
 - droneIsDelivering(D1,P6,4) - droneIsDelivering(D2,P6,1)
 - droneIsDelivering(D2,P6,2) - droneIsDelivering(D2,P6,3)
 - droneIsDelivering(D2,P6,4) - droneIsDelivering(D3,P6,1)
 - droneIsDelivering(D3,P6,2) - droneIsDelivering(D3,P6,3)
 - droneIsDelivering(D3,P6,4) - droneIsDelivering(D1,P6,5)
 - droneIsDelivering(D2,P6,5) - droneIsDelivering(D3,P6,5)
 + timePerProductDelivered(P6) = -0
 initializePositionX(D1,1): + droneDX(D1,1) = 1
 initializePositionX(D2,1): + droneDX(D2,1) = 5
 initializePositionX(D3,1): + droneDX(D3,1) = -0
 initializePositionY(D1,1): + droneDY(D1,1) = 2
 initializePositionY(D2,1): + droneDY(D2,1) = 5
 initializePositionY(D3,1): + droneDY(D3,1) = -0
 packageWasCollectedAtRightPositionX1(P1,D1,1): + droneDX(D1,1)
 + 30 droneCollected(D1,P1,1) <= 32
 packageWasCollectedAtRightPositionX1(P1,D1,2): + droneDX(D1,2)
 + 30 droneCollected(D1,P1,2) <= 32
 packageWasCollectedAtRightPositionX1(P1,D1,3): + droneDX(D1,3)
 + 30 droneCollected(D1,P1,3) <= 32
 packageWasCollectedAtRightPositionX1(P1,D1,4): + droneDX(D1,4)
 + 30 droneCollected(D1,P1,4) <= 32
 packageWasCollectedAtRightPositionX1(P1,D1,5): + droneDX(D1,5)
 + 30 droneCollected(D1,P1,5) <= 32
 packageWasCollectedAtRightPositionX1(P1,D2,1): + droneDX(D2,1)
 + 30 droneCollected(D2,P1,1) <= 32
 packageWasCollectedAtRightPositionX1(P1,D2,2): + droneDX(D2,2)
 + 30 droneCollected(D2,P1,2) <= 32
 packageWasCollectedAtRightPositionX1(P1,D2,3): + droneDX(D2,3)
 + 30 droneCollected(D2,P1,3) <= 32
 packageWasCollectedAtRightPositionX1(P1,D2,4): + droneDX(D2,4)
 + 30 droneCollected(D2,P1,4) <= 32
 packageWasCollectedAtRightPositionX1(P1,D2,5): + droneDX(D2,5)
 + 30 droneCollected(D2,P1,5) <= 32
 packageWasCollectedAtRightPositionX1(P1,D3,1): + droneDX(D3,1)
 + 30 droneCollected(D3,P1,1) <= 32
 packageWasCollectedAtRightPositionX1(P1,D3,2): + droneDX(D3,2)
 + 30 droneCollected(D3,P1,2) <= 32
 packageWasCollectedAtRightPositionX1(P1,D3,3): + droneDX(D3,3)
 + 30 droneCollected(D3,P1,3) <= 32
 packageWasCollectedAtRightPositionX1(P1,D3,4): + droneDX(D3,4)
 + 30 droneCollected(D3,P1,4) <= 32
 packageWasCollectedAtRightPositionX1(P1,D3,5): + droneDX(D3,5)
 + 30 droneCollected(D3,P1,5) <= 32
 packageWasCollectedAtRightPositionX1(P2,D1,1): + droneDX(D1,1)
 + 30 droneCollected(D1,P2,1) <= 32
 packageWasCollectedAtRightPositionX1(P2,D1,2): + droneDX(D1,2)
 + 30 droneCollected(D1,P2,2) <= 32
 packageWasCollectedAtRightPositionX1(P2,D1,3): + droneDX(D1,3)
 + 30 droneCollected(D1,P2,3) <= 32
 packageWasCollectedAtRightPositionX1(P2,D1,4): + droneDX(D1,4)
 + 30 droneCollected(D1,P2,4) <= 32
 packageWasCollectedAtRightPositionX1(P2,D1,5): + droneDX(D1,5)
 + 30 droneCollected(D1,P2,5) <= 32
 packageWasCollectedAtRightPositionX1(P2,D2,1): + droneDX(D2,1)
 + 30 droneCollected(D2,P2,1) <= 32
 packageWasCollectedAtRightPositionX1(P2,D2,2): + droneDX(D2,2)
 + 30 droneCollected(D2,P2,2) <= 32
 packageWasCollectedAtRightPositionX1(P2,D2,3): + droneDX(D2,3)
 + 30 droneCollected(D2,P2,3) <= 32
 packageWasCollectedAtRightPositionX1(P2,D2,4): + droneDX(D2,4)
 + 30 droneCollected(D2,P2,4) <= 32
 packageWasCollectedAtRightPositionX1(P2,D2,5): + droneDX(D2,5)
 + 30 droneCollected(D2,P2,5) <= 32
 packageWasCollectedAtRightPositionX1(P2,D3,1): + droneDX(D3,1)
 + 30 droneCollected(D3,P2,1) <= 32
 packageWasCollectedAtRightPositionX1(P2,D3,2): + droneDX(D3,2)
 + 30 droneCollected(D3,P2,2) <= 32
 packageWasCollectedAtRightPositionX1(P2,D3,3): + droneDX(D3,3)
 + 30 droneCollected(D3,P2,3) <= 32
 packageWasCollectedAtRightPositionX1(P2,D3,4): + droneDX(D3,4)
 + 30 droneCollected(D3,P2,4) <= 32
 packageWasCollectedAtRightPositionX1(P2,D3,5): + droneDX(D3,5)
 + 30 droneCollected(D3,P2,5) <= 32
 packageWasCollectedAtRightPositionX1(P3,D1,1): + droneDX(D1,1)
 + 30 droneCollected(D1,P3,1) <= 35
 packageWasCollectedAtRightPositionX1(P3,D1,2): + droneDX(D1,2)
 + 30 droneCollected(D1,P3,2) <= 35
 packageWasCollectedAtRightPositionX1(P3,D1,3): + droneDX(D1,3)
 + 30 droneCollected(D1,P3,3) <= 35
 packageWasCollectedAtRightPositionX1(P3,D1,4): + droneDX(D1,4)
 + 30 droneCollected(D1,P3,4) <= 35
 packageWasCollectedAtRightPositionX1(P3,D1,5): + droneDX(D1,5)
 + 30 droneCollected(D1,P3,5) <= 35
 packageWasCollectedAtRightPositionX1(P3,D2,1): + droneDX(D2,1)
 + 30 droneCollected(D2,P3,1) <= 35
 packageWasCollectedAtRightPositionX1(P3,D2,2): + droneDX(D2,2)
 + 30 droneCollected(D2,P3,2) <= 35
 packageWasCollectedAtRightPositionX1(P3,D2,3): + droneDX(D2,3)
 + 30 droneCollected(D2,P3,3) <= 35
 packageWasCollectedAtRightPositionX1(P3,D2,4): + droneDX(D2,4)
 + 30 droneCollected(D2,P3,4) <= 35
 packageWasCollectedAtRightPositionX1(P3,D2,5): + droneDX(D2,5)
 + 30 droneCollected(D2,P3,5) <= 35
 packageWasCollectedAtRightPositionX1(P3,D3,1): + droneDX(D3,1)
 + 30 droneCollected(D3,P3,1) <= 35
 packageWasCollectedAtRightPositionX1(P3,D3,2): + droneDX(D3,2)
 + 30 droneCollected(D3,P3,2) <= 35
 packageWasCollectedAtRightPositionX1(P3,D3,3): + droneDX(D3,3)
 + 30 droneCollected(D3,P3,3) <= 35
 packageWasCollectedAtRightPositionX1(P3,D3,4): + droneDX(D3,4)
 + 30 droneCollected(D3,P3,4) <= 35
 packageWasCollectedAtRightPositionX1(P3,D3,5): + droneDX(D3,5)
 + 30 droneCollected(D3,P3,5) <= 35
 packageWasCollectedAtRightPositionX1(P4,D1,1): + droneDX(D1,1)
 + 30 droneCollected(D1,P4,1) <= 30
 packageWasCollectedAtRightPositionX1(P4,D1,2): + droneDX(D1,2)
 + 30 droneCollected(D1,P4,2) <= 30
 packageWasCollectedAtRightPositionX1(P4,D1,3): + droneDX(D1,3)
 + 30 droneCollected(D1,P4,3) <= 30
 packageWasCollectedAtRightPositionX1(P4,D1,4): + droneDX(D1,4)
 + 30 droneCollected(D1,P4,4) <= 30
 packageWasCollectedAtRightPositionX1(P4,D1,5): + droneDX(D1,5)
 + 30 droneCollected(D1,P4,5) <= 30
 packageWasCollectedAtRightPositionX1(P4,D2,1): + droneDX(D2,1)
 + 30 droneCollected(D2,P4,1) <= 30
 packageWasCollectedAtRightPositionX1(P4,D2,2): + droneDX(D2,2)
 + 30 droneCollected(D2,P4,2) <= 30
 packageWasCollectedAtRightPositionX1(P4,D2,3): + droneDX(D2,3)
 + 30 droneCollected(D2,P4,3) <= 30
 packageWasCollectedAtRightPositionX1(P4,D2,4): + droneDX(D2,4)
 + 30 droneCollected(D2,P4,4) <= 30
 packageWasCollectedAtRightPositionX1(P4,D2,5): + droneDX(D2,5)
 + 30 droneCollected(D2,P4,5) <= 30
 packageWasCollectedAtRightPositionX1(P4,D3,1): + droneDX(D3,1)
 + 30 droneCollected(D3,P4,1) <= 30
 packageWasCollectedAtRightPositionX1(P4,D3,2): + droneDX(D3,2)
 + 30 droneCollected(D3,P4,2) <= 30
 packageWasCollectedAtRightPositionX1(P4,D3,3): + droneDX(D3,3)
 + 30 droneCollected(D3,P4,3) <= 30
 packageWasCollectedAtRightPositionX1(P4,D3,4): + droneDX(D3,4)
 + 30 droneCollected(D3,P4,4) <= 30
 packageWasCollectedAtRightPositionX1(P4,D3,5): + droneDX(D3,5)
 + 30 droneCollected(D3,P4,5) <= 30
 packageWasCollectedAtRightPositionX1(P5,D1,1): + droneDX(D1,1)
 + 30 droneCollected(D1,P5,1) <= 50
 packageWasCollectedAtRightPositionX1(P5,D1,2): + droneDX(D1,2)
 + 30 droneCollected(D1,P5,2) <= 50
 packageWasCollectedAtRightPositionX1(P5,D1,3): + droneDX(D1,3)
 + 30 droneCollected(D1,P5,3) <= 50
 packageWasCollectedAtRightPositionX1(P5,D1,4): + droneDX(D1,4)
 + 30 droneCollected(D1,P5,4) <= 50
 packageWasCollectedAtRightPositionX1(P5,D1,5): + droneDX(D1,5)
 + 30 droneCollected(D1,P5,5) <= 50
 packageWasCollectedAtRightPositionX1(P5,D2,1): + droneDX(D2,1)
 + 30 droneCollected(D2,P5,1) <= 50
 packageWasCollectedAtRightPositionX1(P5,D2,2): + droneDX(D2,2)
 + 30 droneCollected(D2,P5,2) <= 50
 packageWasCollectedAtRightPositionX1(P5,D2,3): + droneDX(D2,3)
 + 30 droneCollected(D2,P5,3) <= 50
 packageWasCollectedAtRightPositionX1(P5,D2,4): + droneDX(D2,4)
 + 30 droneCollected(D2,P5,4) <= 50
 packageWasCollectedAtRightPositionX1(P5,D2,5): + droneDX(D2,5)
 + 30 droneCollected(D2,P5,5) <= 50
 packageWasCollectedAtRightPositionX1(P5,D3,1): + droneDX(D3,1)
 + 30 droneCollected(D3,P5,1) <= 50
 packageWasCollectedAtRightPositionX1(P5,D3,2): + droneDX(D3,2)
 + 30 droneCollected(D3,P5,2) <= 50
 packageWasCollectedAtRightPositionX1(P5,D3,3): + droneDX(D3,3)
 + 30 droneCollected(D3,P5,3) <= 50
 packageWasCollectedAtRightPositionX1(P5,D3,4): + droneDX(D3,4)
 + 30 droneCollected(D3,P5,4) <= 50
 packageWasCollectedAtRightPositionX1(P5,D3,5): + droneDX(D3,5)
 + 30 droneCollected(D3,P5,5) <= 50
 packageWasCollectedAtRightPositionX1(P6,D1,1): + droneDX(D1,1)
 + 30 droneCollected(D1,P6,1) <= 40
 packageWasCollectedAtRightPositionX1(P6,D1,2): + droneDX(D1,2)
 + 30 droneCollected(D1,P6,2) <= 40
 packageWasCollectedAtRightPositionX1(P6,D1,3): + droneDX(D1,3)
 + 30 droneCollected(D1,P6,3) <= 40
 packageWasCollectedAtRightPositionX1(P6,D1,4): + droneDX(D1,4)
 + 30 droneCollected(D1,P6,4) <= 40
 packageWasCollectedAtRightPositionX1(P6,D1,5): + droneDX(D1,5)
 + 30 droneCollected(D1,P6,5) <= 40
 packageWasCollectedAtRightPositionX1(P6,D2,1): + droneDX(D2,1)
 + 30 droneCollected(D2,P6,1) <= 40
 packageWasCollectedAtRightPositionX1(P6,D2,2): + droneDX(D2,2)
 + 30 droneCollected(D2,P6,2) <= 40
 packageWasCollectedAtRightPositionX1(P6,D2,3): + droneDX(D2,3)
 + 30 droneCollected(D2,P6,3) <= 40
 packageWasCollectedAtRightPositionX1(P6,D2,4): + droneDX(D2,4)
 + 30 droneCollected(D2,P6,4) <= 40
 packageWasCollectedAtRightPositionX1(P6,D2,5): + droneDX(D2,5)
 + 30 droneCollected(D2,P6,5) <= 40
 packageWasCollectedAtRightPositionX1(P6,D3,1): + droneDX(D3,1)
 + 30 droneCollected(D3,P6,1) <= 40
 packageWasCollectedAtRightPositionX1(P6,D3,2): + droneDX(D3,2)
 + 30 droneCollected(D3,P6,2) <= 40
 packageWasCollectedAtRightPositionX1(P6,D3,3): + droneDX(D3,3)
 + 30 droneCollected(D3,P6,3) <= 40
 packageWasCollectedAtRightPositionX1(P6,D3,4): + droneDX(D3,4)
 + 30 droneCollected(D3,P6,4) <= 40
 packageWasCollectedAtRightPositionX1(P6,D3,5): + droneDX(D3,5)
 + 30 droneCollected(D3,P6,5) <= 40
 packageWasCollectedAtRightPositionX2(P1,D1,1): - droneDX(D1,1)
 + 30 droneCollected(D1,P1,1) <= 28
 packageWasCollectedAtRightPositionX2(P1,D1,2): - droneDX(D1,2)
 + 30 droneCollected(D1,P1,2) <= 28
 packageWasCollectedAtRightPositionX2(P1,D1,3): - droneDX(D1,3)
 + 30 droneCollected(D1,P1,3) <= 28
 packageWasCollectedAtRightPositionX2(P1,D1,4): - droneDX(D1,4)
 + 30 droneCollected(D1,P1,4) <= 28
 packageWasCollectedAtRightPositionX2(P1,D1,5): - droneDX(D1,5)
 + 30 droneCollected(D1,P1,5) <= 28
 packageWasCollectedAtRightPositionX2(P1,D2,1): - droneDX(D2,1)
 + 30 droneCollected(D2,P1,1) <= 28
 packageWasCollectedAtRightPositionX2(P1,D2,2): - droneDX(D2,2)
 + 30 droneCollected(D2,P1,2) <= 28
 packageWasCollectedAtRightPositionX2(P1,D2,3): - droneDX(D2,3)
 + 30 droneCollected(D2,P1,3) <= 28
 packageWasCollectedAtRightPositionX2(P1,D2,4): - droneDX(D2,4)
 + 30 droneCollected(D2,P1,4) <= 28
 packageWasCollectedAtRightPositionX2(P1,D2,5): - droneDX(D2,5)
 + 30 droneCollected(D2,P1,5) <= 28
 packageWasCollectedAtRightPositionX2(P1,D3,1): - droneDX(D3,1)
 + 30 droneCollected(D3,P1,1) <= 28
 packageWasCollectedAtRightPositionX2(P1,D3,2): - droneDX(D3,2)
 + 30 droneCollected(D3,P1,2) <= 28
 packageWasCollectedAtRightPositionX2(P1,D3,3): - droneDX(D3,3)
 + 30 droneCollected(D3,P1,3) <= 28
 packageWasCollectedAtRightPositionX2(P1,D3,4): - droneDX(D3,4)
 + 30 droneCollected(D3,P1,4) <= 28
 packageWasCollectedAtRightPositionX2(P1,D3,5): - droneDX(D3,5)
 + 30 droneCollected(D3,P1,5) <= 28
 packageWasCollectedAtRightPositionX2(P2,D1,1): - droneDX(D1,1)
 + 30 droneCollected(D1,P2,1) <= 28
 packageWasCollectedAtRightPositionX2(P2,D1,2): - droneDX(D1,2)
 + 30 droneCollected(D1,P2,2) <= 28
 packageWasCollectedAtRightPositionX2(P2,D1,3): - droneDX(D1,3)
 + 30 droneCollected(D1,P2,3) <= 28
 packageWasCollectedAtRightPositionX2(P2,D1,4): - droneDX(D1,4)
 + 30 droneCollected(D1,P2,4) <= 28
 packageWasCollectedAtRightPositionX2(P2,D1,5): - droneDX(D1,5)
 + 30 droneCollected(D1,P2,5) <= 28
 packageWasCollectedAtRightPositionX2(P2,D2,1): - droneDX(D2,1)
 + 30 droneCollected(D2,P2,1) <= 28
 packageWasCollectedAtRightPositionX2(P2,D2,2): - droneDX(D2,2)
 + 30 droneCollected(D2,P2,2) <= 28
 packageWasCollectedAtRightPositionX2(P2,D2,3): - droneDX(D2,3)
 + 30 droneCollected(D2,P2,3) <= 28
 packageWasCollectedAtRightPositionX2(P2,D2,4): - droneDX(D2,4)
 + 30 droneCollected(D2,P2,4) <= 28
 packageWasCollectedAtRightPositionX2(P2,D2,5): - droneDX(D2,5)
 + 30 droneCollected(D2,P2,5) <= 28
 packageWasCollectedAtRightPositionX2(P2,D3,1): - droneDX(D3,1)
 + 30 droneCollected(D3,P2,1) <= 28
 packageWasCollectedAtRightPositionX2(P2,D3,2): - droneDX(D3,2)
 + 30 droneCollected(D3,P2,2) <= 28
 packageWasCollectedAtRightPositionX2(P2,D3,3): - droneDX(D3,3)
 + 30 droneCollected(D3,P2,3) <= 28
 packageWasCollectedAtRightPositionX2(P2,D3,4): - droneDX(D3,4)
 + 30 droneCollected(D3,P2,4) <= 28
 packageWasCollectedAtRightPositionX2(P2,D3,5): - droneDX(D3,5)
 + 30 droneCollected(D3,P2,5) <= 28
 packageWasCollectedAtRightPositionX2(P3,D1,1): - droneDX(D1,1)
 + 30 droneCollected(D1,P3,1) <= 25
 packageWasCollectedAtRightPositionX2(P3,D1,2): - droneDX(D1,2)
 + 30 droneCollected(D1,P3,2) <= 25
 packageWasCollectedAtRightPositionX2(P3,D1,3): - droneDX(D1,3)
 + 30 droneCollected(D1,P3,3) <= 25
 packageWasCollectedAtRightPositionX2(P3,D1,4): - droneDX(D1,4)
 + 30 droneCollected(D1,P3,4) <= 25
 packageWasCollectedAtRightPositionX2(P3,D1,5): - droneDX(D1,5)
 + 30 droneCollected(D1,P3,5) <= 25
 packageWasCollectedAtRightPositionX2(P3,D2,1): - droneDX(D2,1)
 + 30 droneCollected(D2,P3,1) <= 25
 packageWasCollectedAtRightPositionX2(P3,D2,2): - droneDX(D2,2)
 + 30 droneCollected(D2,P3,2) <= 25
 packageWasCollectedAtRightPositionX2(P3,D2,3): - droneDX(D2,3)
 + 30 droneCollected(D2,P3,3) <= 25
 packageWasCollectedAtRightPositionX2(P3,D2,4): - droneDX(D2,4)
 + 30 droneCollected(D2,P3,4) <= 25
 packageWasCollectedAtRightPositionX2(P3,D2,5): - droneDX(D2,5)
 + 30 droneCollected(D2,P3,5) <= 25
 packageWasCollectedAtRightPositionX2(P3,D3,1): - droneDX(D3,1)
 + 30 droneCollected(D3,P3,1) <= 25
 packageWasCollectedAtRightPositionX2(P3,D3,2): - droneDX(D3,2)
 + 30 droneCollected(D3,P3,2) <= 25
 packageWasCollectedAtRightPositionX2(P3,D3,3): - droneDX(D3,3)
 + 30 droneCollected(D3,P3,3) <= 25
 packageWasCollectedAtRightPositionX2(P3,D3,4): - droneDX(D3,4)
 + 30 droneCollected(D3,P3,4) <= 25
 packageWasCollectedAtRightPositionX2(P3,D3,5): - droneDX(D3,5)
 + 30 droneCollected(D3,P3,5) <= 25
 packageWasCollectedAtRightPositionX2(P4,D1,1): - droneDX(D1,1)
 + 30 droneCollected(D1,P4,1) <= 30
 packageWasCollectedAtRightPositionX2(P4,D1,2): - droneDX(D1,2)
 + 30 droneCollected(D1,P4,2) <= 30
 packageWasCollectedAtRightPositionX2(P4,D1,3): - droneDX(D1,3)
 + 30 droneCollected(D1,P4,3) <= 30
 packageWasCollectedAtRightPositionX2(P4,D1,4): - droneDX(D1,4)
 + 30 droneCollected(D1,P4,4) <= 30
 packageWasCollectedAtRightPositionX2(P4,D1,5): - droneDX(D1,5)
 + 30 droneCollected(D1,P4,5) <= 30
 packageWasCollectedAtRightPositionX2(P4,D2,1): - droneDX(D2,1)
 + 30 droneCollected(D2,P4,1) <= 30
 packageWasCollectedAtRightPositionX2(P4,D2,2): - droneDX(D2,2)
 + 30 droneCollected(D2,P4,2) <= 30
 packageWasCollectedAtRightPositionX2(P4,D2,3): - droneDX(D2,3)
 + 30 droneCollected(D2,P4,3) <= 30
 packageWasCollectedAtRightPositionX2(P4,D2,4): - droneDX(D2,4)
 + 30 droneCollected(D2,P4,4) <= 30
 packageWasCollectedAtRightPositionX2(P4,D2,5): - droneDX(D2,5)
 + 30 droneCollected(D2,P4,5) <= 30
 packageWasCollectedAtRightPositionX2(P4,D3,1): - droneDX(D3,1)
 + 30 droneCollected(D3,P4,1) <= 30
 packageWasCollectedAtRightPositionX2(P4,D3,2): - droneDX(D3,2)
 + 30 droneCollected(D3,P4,2) <= 30
 packageWasCollectedAtRightPositionX2(P4,D3,3): - droneDX(D3,3)
 + 30 droneCollected(D3,P4,3) <= 30
 packageWasCollectedAtRightPositionX2(P4,D3,4): - droneDX(D3,4)
 + 30 droneCollected(D3,P4,4) <= 30
 packageWasCollectedAtRightPositionX2(P4,D3,5): - droneDX(D3,5)
 + 30 droneCollected(D3,P4,5) <= 30
 packageWasCollectedAtRightPositionX2(P5,D1,1): - droneDX(D1,1)
 + 30 droneCollected(D1,P5,1) <= 10
 packageWasCollectedAtRightPositionX2(P5,D1,2): - droneDX(D1,2)
 + 30 droneCollected(D1,P5,2) <= 10
 packageWasCollectedAtRightPositionX2(P5,D1,3): - droneDX(D1,3)
 + 30 droneCollected(D1,P5,3) <= 10
 packageWasCollectedAtRightPositionX2(P5,D1,4): - droneDX(D1,4)
 + 30 droneCollected(D1,P5,4) <= 10
 packageWasCollectedAtRightPositionX2(P5,D1,5): - droneDX(D1,5)
 + 30 droneCollected(D1,P5,5) <= 10
 packageWasCollectedAtRightPositionX2(P5,D2,1): - droneDX(D2,1)
 + 30 droneCollected(D2,P5,1) <= 10
 packageWasCollectedAtRightPositionX2(P5,D2,2): - droneDX(D2,2)
 + 30 droneCollected(D2,P5,2) <= 10
 packageWasCollectedAtRightPositionX2(P5,D2,3): - droneDX(D2,3)
 + 30 droneCollected(D2,P5,3) <= 10
 packageWasCollectedAtRightPositionX2(P5,D2,4): - droneDX(D2,4)
 + 30 droneCollected(D2,P5,4) <= 10
 packageWasCollectedAtRightPositionX2(P5,D2,5): - droneDX(D2,5)
 + 30 droneCollected(D2,P5,5) <= 10
 packageWasCollectedAtRightPositionX2(P5,D3,1): - droneDX(D3,1)
 + 30 droneCollected(D3,P5,1) <= 10
 packageWasCollectedAtRightPositionX2(P5,D3,2): - droneDX(D3,2)
 + 30 droneCollected(D3,P5,2) <= 10
 packageWasCollectedAtRightPositionX2(P5,D3,3): - droneDX(D3,3)
 + 30 droneCollected(D3,P5,3) <= 10
 packageWasCollectedAtRightPositionX2(P5,D3,4): - droneDX(D3,4)
 + 30 droneCollected(D3,P5,4) <= 10
 packageWasCollectedAtRightPositionX2(P5,D3,5): - droneDX(D3,5)
 + 30 droneCollected(D3,P5,5) <= 10
 packageWasCollectedAtRightPositionX2(P6,D1,1): - droneDX(D1,1)
 + 30 droneCollected(D1,P6,1) <= 20
 packageWasCollectedAtRightPositionX2(P6,D1,2): - droneDX(D1,2)
 + 30 droneCollected(D1,P6,2) <= 20
 packageWasCollectedAtRightPositionX2(P6,D1,3): - droneDX(D1,3)
 + 30 droneCollected(D1,P6,3) <= 20
 packageWasCollectedAtRightPositionX2(P6,D1,4): - droneDX(D1,4)
 + 30 droneCollected(D1,P6,4) <= 20
 packageWasCollectedAtRightPositionX2(P6,D1,5): - droneDX(D1,5)
 + 30 droneCollected(D1,P6,5) <= 20
 packageWasCollectedAtRightPositionX2(P6,D2,1): - droneDX(D2,1)
 + 30 droneCollected(D2,P6,1) <= 20
 packageWasCollectedAtRightPositionX2(P6,D2,2): - droneDX(D2,2)
 + 30 droneCollected(D2,P6,2) <= 20
 packageWasCollectedAtRightPositionX2(P6,D2,3): - droneDX(D2,3)
 + 30 droneCollected(D2,P6,3) <= 20
 packageWasCollectedAtRightPositionX2(P6,D2,4): - droneDX(D2,4)
 + 30 droneCollected(D2,P6,4) <= 20
 packageWasCollectedAtRightPositionX2(P6,D2,5): - droneDX(D2,5)
 + 30 droneCollected(D2,P6,5) <= 20
 packageWasCollectedAtRightPositionX2(P6,D3,1): - droneDX(D3,1)
 + 30 droneCollected(D3,P6,1) <= 20
 packageWasCollectedAtRightPositionX2(P6,D3,2): - droneDX(D3,2)
 + 30 droneCollected(D3,P6,2) <= 20
 packageWasCollectedAtRightPositionX2(P6,D3,3): - droneDX(D3,3)
 + 30 droneCollected(D3,P6,3) <= 20
 packageWasCollectedAtRightPositionX2(P6,D3,4): - droneDX(D3,4)
 + 30 droneCollected(D3,P6,4) <= 20
 packageWasCollectedAtRightPositionX2(P6,D3,5): - droneDX(D3,5)
 + 30 droneCollected(D3,P6,5) <= 20
 packageWasCollectedAtRightPositionY1(P1,D1,1): + droneDY(D1,1)
 + 30 droneCollected(D1,P1,1) <= 40
 packageWasCollectedAtRightPositionY1(P1,D1,2): + droneDY(D1,2)
 + 30 droneCollected(D1,P1,2) <= 40
 packageWasCollectedAtRightPositionY1(P1,D1,3): + droneDY(D1,3)
 + 30 droneCollected(D1,P1,3) <= 40
 packageWasCollectedAtRightPositionY1(P1,D1,4): + droneDY(D1,4)
 + 30 droneCollected(D1,P1,4) <= 40
 packageWasCollectedAtRightPositionY1(P1,D1,5): + droneDY(D1,5)
 + 30 droneCollected(D1,P1,5) <= 40
 packageWasCollectedAtRightPositionY1(P1,D2,1): + droneDY(D2,1)
 + 30 droneCollected(D2,P1,1) <= 40
 packageWasCollectedAtRightPositionY1(P1,D2,2): + droneDY(D2,2)
 + 30 droneCollected(D2,P1,2) <= 40
 packageWasCollectedAtRightPositionY1(P1,D2,3): + droneDY(D2,3)
 + 30 droneCollected(D2,P1,3) <= 40
 packageWasCollectedAtRightPositionY1(P1,D2,4): + droneDY(D2,4)
 + 30 droneCollected(D2,P1,4) <= 40
 packageWasCollectedAtRightPositionY1(P1,D2,5): + droneDY(D2,5)
 + 30 droneCollected(D2,P1,5) <= 40
 packageWasCollectedAtRightPositionY1(P1,D3,1): + droneDY(D3,1)
 + 30 droneCollected(D3,P1,1) <= 40
 packageWasCollectedAtRightPositionY1(P1,D3,2): + droneDY(D3,2)
 + 30 droneCollected(D3,P1,2) <= 40
 packageWasCollectedAtRightPositionY1(P1,D3,3): + droneDY(D3,3)
 + 30 droneCollected(D3,P1,3) <= 40
 packageWasCollectedAtRightPositionY1(P1,D3,4): + droneDY(D3,4)
 + 30 droneCollected(D3,P1,4) <= 40
 packageWasCollectedAtRightPositionY1(P1,D3,5): + droneDY(D3,5)
 + 30 droneCollected(D3,P1,5) <= 40
 packageWasCollectedAtRightPositionY1(P2,D1,1): + droneDY(D1,1)
 + 30 droneCollected(D1,P2,1) <= 35
 packageWasCollectedAtRightPositionY1(P2,D1,2): + droneDY(D1,2)
 + 30 droneCollected(D1,P2,2) <= 35
 packageWasCollectedAtRightPositionY1(P2,D1,3): + droneDY(D1,3)
 + 30 droneCollected(D1,P2,3) <= 35
 packageWasCollectedAtRightPositionY1(P2,D1,4): + droneDY(D1,4)
 + 30 droneCollected(D1,P2,4) <= 35
 packageWasCollectedAtRightPositionY1(P2,D1,5): + droneDY(D1,5)
 + 30 droneCollected(D1,P2,5) <= 35
 packageWasCollectedAtRightPositionY1(P2,D2,1): + droneDY(D2,1)
 + 30 droneCollected(D2,P2,1) <= 35
 packageWasCollectedAtRightPositionY1(P2,D2,2): + droneDY(D2,2)
 + 30 droneCollected(D2,P2,2) <= 35
 packageWasCollectedAtRightPositionY1(P2,D2,3): + droneDY(D2,3)
 + 30 droneCollected(D2,P2,3) <= 35
 packageWasCollectedAtRightPositionY1(P2,D2,4): + droneDY(D2,4)
 + 30 droneCollected(D2,P2,4) <= 35
 packageWasCollectedAtRightPositionY1(P2,D2,5): + droneDY(D2,5)
 + 30 droneCollected(D2,P2,5) <= 35
 packageWasCollectedAtRightPositionY1(P2,D3,1): + droneDY(D3,1)
 + 30 droneCollected(D3,P2,1) <= 35
 packageWasCollectedAtRightPositionY1(P2,D3,2): + droneDY(D3,2)
 + 30 droneCollected(D3,P2,2) <= 35
 packageWasCollectedAtRightPositionY1(P2,D3,3): + droneDY(D3,3)
 + 30 droneCollected(D3,P2,3) <= 35
 packageWasCollectedAtRightPositionY1(P2,D3,4): + droneDY(D3,4)
 + 30 droneCollected(D3,P2,4) <= 35
 packageWasCollectedAtRightPositionY1(P2,D3,5): + droneDY(D3,5)
 + 30 droneCollected(D3,P2,5) <= 35
 packageWasCollectedAtRightPositionY1(P3,D1,1): + droneDY(D1,1)
 + 30 droneCollected(D1,P3,1) <= 50
 packageWasCollectedAtRightPositionY1(P3,D1,2): + droneDY(D1,2)
 + 30 droneCollected(D1,P3,2) <= 50
 packageWasCollectedAtRightPositionY1(P3,D1,3): + droneDY(D1,3)
 + 30 droneCollected(D1,P3,3) <= 50
 packageWasCollectedAtRightPositionY1(P3,D1,4): + droneDY(D1,4)
 + 30 droneCollected(D1,P3,4) <= 50
 packageWasCollectedAtRightPositionY1(P3,D1,5): + droneDY(D1,5)
 + 30 droneCollected(D1,P3,5) <= 50
 packageWasCollectedAtRightPositionY1(P3,D2,1): + droneDY(D2,1)
 + 30 droneCollected(D2,P3,1) <= 50
 packageWasCollectedAtRightPositionY1(P3,D2,2): + droneDY(D2,2)
 + 30 droneCollected(D2,P3,2) <= 50
 packageWasCollectedAtRightPositionY1(P3,D2,3): + droneDY(D2,3)
 + 30 droneCollected(D2,P3,3) <= 50
 packageWasCollectedAtRightPositionY1(P3,D2,4): + droneDY(D2,4)
 + 30 droneCollected(D2,P3,4) <= 50
 packageWasCollectedAtRightPositionY1(P3,D2,5): + droneDY(D2,5)
 + 30 droneCollected(D2,P3,5) <= 50
 packageWasCollectedAtRightPositionY1(P3,D3,1): + droneDY(D3,1)
 + 30 droneCollected(D3,P3,1) <= 50
 packageWasCollectedAtRightPositionY1(P3,D3,2): + droneDY(D3,2)
 + 30 droneCollected(D3,P3,2) <= 50
 packageWasCollectedAtRightPositionY1(P3,D3,3): + droneDY(D3,3)
 + 30 droneCollected(D3,P3,3) <= 50
 packageWasCollectedAtRightPositionY1(P3,D3,4): + droneDY(D3,4)
 + 30 droneCollected(D3,P3,4) <= 50
 packageWasCollectedAtRightPositionY1(P3,D3,5): + droneDY(D3,5)
 + 30 droneCollected(D3,P3,5) <= 50
 packageWasCollectedAtRightPositionY1(P4,D1,1): + droneDY(D1,1)
 + 30 droneCollected(D1,P4,1) <= 40
 packageWasCollectedAtRightPositionY1(P4,D1,2): + droneDY(D1,2)
 + 30 droneCollected(D1,P4,2) <= 40
 packageWasCollectedAtRightPositionY1(P4,D1,3): + droneDY(D1,3)
 + 30 droneCollected(D1,P4,3) <= 40
 packageWasCollectedAtRightPositionY1(P4,D1,4): + droneDY(D1,4)
 + 30 droneCollected(D1,P4,4) <= 40
 packageWasCollectedAtRightPositionY1(P4,D1,5): + droneDY(D1,5)
 + 30 droneCollected(D1,P4,5) <= 40
 packageWasCollectedAtRightPositionY1(P4,D2,1): + droneDY(D2,1)
 + 30 droneCollected(D2,P4,1) <= 40
 packageWasCollectedAtRightPositionY1(P4,D2,2): + droneDY(D2,2)
 + 30 droneCollected(D2,P4,2) <= 40
 packageWasCollectedAtRightPositionY1(P4,D2,3): + droneDY(D2,3)
 + 30 droneCollected(D2,P4,3) <= 40
 packageWasCollectedAtRightPositionY1(P4,D2,4): + droneDY(D2,4)
 + 30 droneCollected(D2,P4,4) <= 40
 packageWasCollectedAtRightPositionY1(P4,D2,5): + droneDY(D2,5)
 + 30 droneCollected(D2,P4,5) <= 40
 packageWasCollectedAtRightPositionY1(P4,D3,1): + droneDY(D3,1)
 + 30 droneCollected(D3,P4,1) <= 40
 packageWasCollectedAtRightPositionY1(P4,D3,2): + droneDY(D3,2)
 + 30 droneCollected(D3,P4,2) <= 40
 packageWasCollectedAtRightPositionY1(P4,D3,3): + droneDY(D3,3)
 + 30 droneCollected(D3,P4,3) <= 40
 packageWasCollectedAtRightPositionY1(P4,D3,4): + droneDY(D3,4)
 + 30 droneCollected(D3,P4,4) <= 40
 packageWasCollectedAtRightPositionY1(P4,D3,5): + droneDY(D3,5)
 + 30 droneCollected(D3,P4,5) <= 40
 packageWasCollectedAtRightPositionY1(P5,D1,1): + droneDY(D1,1)
 + 30 droneCollected(D1,P5,1) <= 50
 packageWasCollectedAtRightPositionY1(P5,D1,2): + droneDY(D1,2)
 + 30 droneCollected(D1,P5,2) <= 50
 packageWasCollectedAtRightPositionY1(P5,D1,3): + droneDY(D1,3)
 + 30 droneCollected(D1,P5,3) <= 50
 packageWasCollectedAtRightPositionY1(P5,D1,4): + droneDY(D1,4)
 + 30 droneCollected(D1,P5,4) <= 50
 packageWasCollectedAtRightPositionY1(P5,D1,5): + droneDY(D1,5)
 + 30 droneCollected(D1,P5,5) <= 50
 packageWasCollectedAtRightPositionY1(P5,D2,1): + droneDY(D2,1)
 + 30 droneCollected(D2,P5,1) <= 50
 packageWasCollectedAtRightPositionY1(P5,D2,2): + droneDY(D2,2)
 + 30 droneCollected(D2,P5,2) <= 50
 packageWasCollectedAtRightPositionY1(P5,D2,3): + droneDY(D2,3)
 + 30 droneCollected(D2,P5,3) <= 50
 packageWasCollectedAtRightPositionY1(P5,D2,4): + droneDY(D2,4)
 + 30 droneCollected(D2,P5,4) <= 50
 packageWasCollectedAtRightPositionY1(P5,D2,5): + droneDY(D2,5)
 + 30 droneCollected(D2,P5,5) <= 50
 packageWasCollectedAtRightPositionY1(P5,D3,1): + droneDY(D3,1)
 + 30 droneCollected(D3,P5,1) <= 50
 packageWasCollectedAtRightPositionY1(P5,D3,2): + droneDY(D3,2)
 + 30 droneCollected(D3,P5,2) <= 50
 packageWasCollectedAtRightPositionY1(P5,D3,3): + droneDY(D3,3)
 + 30 droneCollected(D3,P5,3) <= 50
 packageWasCollectedAtRightPositionY1(P5,D3,4): + droneDY(D3,4)
 + 30 droneCollected(D3,P5,4) <= 50
 packageWasCollectedAtRightPositionY1(P5,D3,5): + droneDY(D3,5)
 + 30 droneCollected(D3,P5,5) <= 50
 packageWasCollectedAtRightPositionY1(P6,D1,1): + droneDY(D1,1)
 + 30 droneCollected(D1,P6,1) <= 40
 packageWasCollectedAtRightPositionY1(P6,D1,2): + droneDY(D1,2)
 + 30 droneCollected(D1,P6,2) <= 40
 packageWasCollectedAtRightPositionY1(P6,D1,3): + droneDY(D1,3)
 + 30 droneCollected(D1,P6,3) <= 40
 packageWasCollectedAtRightPositionY1(P6,D1,4): + droneDY(D1,4)
 + 30 droneCollected(D1,P6,4) <= 40
 packageWasCollectedAtRightPositionY1(P6,D1,5): + droneDY(D1,5)
 + 30 droneCollected(D1,P6,5) <= 40
 packageWasCollectedAtRightPositionY1(P6,D2,1): + droneDY(D2,1)
 + 30 droneCollected(D2,P6,1) <= 40
 packageWasCollectedAtRightPositionY1(P6,D2,2): + droneDY(D2,2)
 + 30 droneCollected(D2,P6,2) <= 40
 packageWasCollectedAtRightPositionY1(P6,D2,3): + droneDY(D2,3)
 + 30 droneCollected(D2,P6,3) <= 40
 packageWasCollectedAtRightPositionY1(P6,D2,4): + droneDY(D2,4)
 + 30 droneCollected(D2,P6,4) <= 40
 packageWasCollectedAtRightPositionY1(P6,D2,5): + droneDY(D2,5)
 + 30 droneCollected(D2,P6,5) <= 40
 packageWasCollectedAtRightPositionY1(P6,D3,1): + droneDY(D3,1)
 + 30 droneCollected(D3,P6,1) <= 40
 packageWasCollectedAtRightPositionY1(P6,D3,2): + droneDY(D3,2)
 + 30 droneCollected(D3,P6,2) <= 40
 packageWasCollectedAtRightPositionY1(P6,D3,3): + droneDY(D3,3)
 + 30 droneCollected(D3,P6,3) <= 40
 packageWasCollectedAtRightPositionY1(P6,D3,4): + droneDY(D3,4)
 + 30 droneCollected(D3,P6,4) <= 40
 packageWasCollectedAtRightPositionY1(P6,D3,5): + droneDY(D3,5)
 + 30 droneCollected(D3,P6,5) <= 40
 packageWasCollectedAtRightPositionY2(P1,D1,1): - droneDY(D1,1)
 + 30 droneCollected(D1,P1,1) <= 20
 packageWasCollectedAtRightPositionY2(P1,D1,2): - droneDY(D1,2)
 + 30 droneCollected(D1,P1,2) <= 20
 packageWasCollectedAtRightPositionY2(P1,D1,3): - droneDY(D1,3)
 + 30 droneCollected(D1,P1,3) <= 20
 packageWasCollectedAtRightPositionY2(P1,D1,4): - droneDY(D1,4)
 + 30 droneCollected(D1,P1,4) <= 20
 packageWasCollectedAtRightPositionY2(P1,D1,5): - droneDY(D1,5)
 + 30 droneCollected(D1,P1,5) <= 20
 packageWasCollectedAtRightPositionY2(P1,D2,1): - droneDY(D2,1)
 + 30 droneCollected(D2,P1,1) <= 20
 packageWasCollectedAtRightPositionY2(P1,D2,2): - droneDY(D2,2)
 + 30 droneCollected(D2,P1,2) <= 20
 packageWasCollectedAtRightPositionY2(P1,D2,3): - droneDY(D2,3)
 + 30 droneCollected(D2,P1,3) <= 20
 packageWasCollectedAtRightPositionY2(P1,D2,4): - droneDY(D2,4)
 + 30 droneCollected(D2,P1,4) <= 20
 packageWasCollectedAtRightPositionY2(P1,D2,5): - droneDY(D2,5)
 + 30 droneCollected(D2,P1,5) <= 20
 packageWasCollectedAtRightPositionY2(P1,D3,1): - droneDY(D3,1)
 + 30 droneCollected(D3,P1,1) <= 20
 packageWasCollectedAtRightPositionY2(P1,D3,2): - droneDY(D3,2)
 + 30 droneCollected(D3,P1,2) <= 20
 packageWasCollectedAtRightPositionY2(P1,D3,3): - droneDY(D3,3)
 + 30 droneCollected(D3,P1,3) <= 20
 packageWasCollectedAtRightPositionY2(P1,D3,4): - droneDY(D3,4)
 + 30 droneCollected(D3,P1,4) <= 20
 packageWasCollectedAtRightPositionY2(P1,D3,5): - droneDY(D3,5)
 + 30 droneCollected(D3,P1,5) <= 20
 packageWasCollectedAtRightPositionY2(P2,D1,1): - droneDY(D1,1)
 + 30 droneCollected(D1,P2,1) <= 25
 packageWasCollectedAtRightPositionY2(P2,D1,2): - droneDY(D1,2)
 + 30 droneCollected(D1,P2,2) <= 25
 packageWasCollectedAtRightPositionY2(P2,D1,3): - droneDY(D1,3)
 + 30 droneCollected(D1,P2,3) <= 25
 packageWasCollectedAtRightPositionY2(P2,D1,4): - droneDY(D1,4)
 + 30 droneCollected(D1,P2,4) <= 25
 packageWasCollectedAtRightPositionY2(P2,D1,5): - droneDY(D1,5)
 + 30 droneCollected(D1,P2,5) <= 25
 packageWasCollectedAtRightPositionY2(P2,D2,1): - droneDY(D2,1)
 + 30 droneCollected(D2,P2,1) <= 25
 packageWasCollectedAtRightPositionY2(P2,D2,2): - droneDY(D2,2)
 + 30 droneCollected(D2,P2,2) <= 25
 packageWasCollectedAtRightPositionY2(P2,D2,3): - droneDY(D2,3)
 + 30 droneCollected(D2,P2,3) <= 25
 packageWasCollectedAtRightPositionY2(P2,D2,4): - droneDY(D2,4)
 + 30 droneCollected(D2,P2,4) <= 25
 packageWasCollectedAtRightPositionY2(P2,D2,5): - droneDY(D2,5)
 + 30 droneCollected(D2,P2,5) <= 25
 packageWasCollectedAtRightPositionY2(P2,D3,1): - droneDY(D3,1)
 + 30 droneCollected(D3,P2,1) <= 25
 packageWasCollectedAtRightPositionY2(P2,D3,2): - droneDY(D3,2)
 + 30 droneCollected(D3,P2,2) <= 25
 packageWasCollectedAtRightPositionY2(P2,D3,3): - droneDY(D3,3)
 + 30 droneCollected(D3,P2,3) <= 25
 packageWasCollectedAtRightPositionY2(P2,D3,4): - droneDY(D3,4)
 + 30 droneCollected(D3,P2,4) <= 25
 packageWasCollectedAtRightPositionY2(P2,D3,5): - droneDY(D3,5)
 + 30 droneCollected(D3,P2,5) <= 25
 packageWasCollectedAtRightPositionY2(P3,D1,1): - droneDY(D1,1)
 + 30 droneCollected(D1,P3,1) <= 10
 packageWasCollectedAtRightPositionY2(P3,D1,2): - droneDY(D1,2)
 + 30 droneCollected(D1,P3,2) <= 10
 packageWasCollectedAtRightPositionY2(P3,D1,3): - droneDY(D1,3)
 + 30 droneCollected(D1,P3,3) <= 10
 packageWasCollectedAtRightPositionY2(P3,D1,4): - droneDY(D1,4)
 + 30 droneCollected(D1,P3,4) <= 10
 packageWasCollectedAtRightPositionY2(P3,D1,5): - droneDY(D1,5)
 + 30 droneCollected(D1,P3,5) <= 10
 packageWasCollectedAtRightPositionY2(P3,D2,1): - droneDY(D2,1)
 + 30 droneCollected(D2,P3,1) <= 10
 packageWasCollectedAtRightPositionY2(P3,D2,2): - droneDY(D2,2)
 + 30 droneCollected(D2,P3,2) <= 10
 packageWasCollectedAtRightPositionY2(P3,D2,3): - droneDY(D2,3)
 + 30 droneCollected(D2,P3,3) <= 10
 packageWasCollectedAtRightPositionY2(P3,D2,4): - droneDY(D2,4)
 + 30 droneCollected(D2,P3,4) <= 10
 packageWasCollectedAtRightPositionY2(P3,D2,5): - droneDY(D2,5)
 + 30 droneCollected(D2,P3,5) <= 10
 packageWasCollectedAtRightPositionY2(P3,D3,1): - droneDY(D3,1)
 + 30 droneCollected(D3,P3,1) <= 10
 packageWasCollectedAtRightPositionY2(P3,D3,2): - droneDY(D3,2)
 + 30 droneCollected(D3,P3,2) <= 10
 packageWasCollectedAtRightPositionY2(P3,D3,3): - droneDY(D3,3)
 + 30 droneCollected(D3,P3,3) <= 10
 packageWasCollectedAtRightPositionY2(P3,D3,4): - droneDY(D3,4)
 + 30 droneCollected(D3,P3,4) <= 10
 packageWasCollectedAtRightPositionY2(P3,D3,5): - droneDY(D3,5)
 + 30 droneCollected(D3,P3,5) <= 10
 packageWasCollectedAtRightPositionY2(P4,D1,1): - droneDY(D1,1)
 + 30 droneCollected(D1,P4,1) <= 20
 packageWasCollectedAtRightPositionY2(P4,D1,2): - droneDY(D1,2)
 + 30 droneCollected(D1,P4,2) <= 20
 packageWasCollectedAtRightPositionY2(P4,D1,3): - droneDY(D1,3)
 + 30 droneCollected(D1,P4,3) <= 20
 packageWasCollectedAtRightPositionY2(P4,D1,4): - droneDY(D1,4)
 + 30 droneCollected(D1,P4,4) <= 20
 packageWasCollectedAtRightPositionY2(P4,D1,5): - droneDY(D1,5)
 + 30 droneCollected(D1,P4,5) <= 20
 packageWasCollectedAtRightPositionY2(P4,D2,1): - droneDY(D2,1)
 + 30 droneCollected(D2,P4,1) <= 20
 packageWasCollectedAtRightPositionY2(P4,D2,2): - droneDY(D2,2)
 + 30 droneCollected(D2,P4,2) <= 20
 packageWasCollectedAtRightPositionY2(P4,D2,3): - droneDY(D2,3)
 + 30 droneCollected(D2,P4,3) <= 20
 packageWasCollectedAtRightPositionY2(P4,D2,4): - droneDY(D2,4)
 + 30 droneCollected(D2,P4,4) <= 20
 packageWasCollectedAtRightPositionY2(P4,D2,5): - droneDY(D2,5)
 + 30 droneCollected(D2,P4,5) <= 20
 packageWasCollectedAtRightPositionY2(P4,D3,1): - droneDY(D3,1)
 + 30 droneCollected(D3,P4,1) <= 20
 packageWasCollectedAtRightPositionY2(P4,D3,2): - droneDY(D3,2)
 + 30 droneCollected(D3,P4,2) <= 20
 packageWasCollectedAtRightPositionY2(P4,D3,3): - droneDY(D3,3)
 + 30 droneCollected(D3,P4,3) <= 20
 packageWasCollectedAtRightPositionY2(P4,D3,4): - droneDY(D3,4)
 + 30 droneCollected(D3,P4,4) <= 20
 packageWasCollectedAtRightPositionY2(P4,D3,5): - droneDY(D3,5)
 + 30 droneCollected(D3,P4,5) <= 20
 packageWasCollectedAtRightPositionY2(P5,D1,1): - droneDY(D1,1)
 + 30 droneCollected(D1,P5,1) <= 10
 packageWasCollectedAtRightPositionY2(P5,D1,2): - droneDY(D1,2)
 + 30 droneCollected(D1,P5,2) <= 10
 packageWasCollectedAtRightPositionY2(P5,D1,3): - droneDY(D1,3)
 + 30 droneCollected(D1,P5,3) <= 10
 packageWasCollectedAtRightPositionY2(P5,D1,4): - droneDY(D1,4)
 + 30 droneCollected(D1,P5,4) <= 10
 packageWasCollectedAtRightPositionY2(P5,D1,5): - droneDY(D1,5)
 + 30 droneCollected(D1,P5,5) <= 10
 packageWasCollectedAtRightPositionY2(P5,D2,1): - droneDY(D2,1)
 + 30 droneCollected(D2,P5,1) <= 10
 packageWasCollectedAtRightPositionY2(P5,D2,2): - droneDY(D2,2)
 + 30 droneCollected(D2,P5,2) <= 10
 packageWasCollectedAtRightPositionY2(P5,D2,3): - droneDY(D2,3)
 + 30 droneCollected(D2,P5,3) <= 10
 packageWasCollectedAtRightPositionY2(P5,D2,4): - droneDY(D2,4)
 + 30 droneCollected(D2,P5,4) <= 10
 packageWasCollectedAtRightPositionY2(P5,D2,5): - droneDY(D2,5)
 + 30 droneCollected(D2,P5,5) <= 10
 packageWasCollectedAtRightPositionY2(P5,D3,1): - droneDY(D3,1)
 + 30 droneCollected(D3,P5,1) <= 10
 packageWasCollectedAtRightPositionY2(P5,D3,2): - droneDY(D3,2)
 + 30 droneCollected(D3,P5,2) <= 10
 packageWasCollectedAtRightPositionY2(P5,D3,3): - droneDY(D3,3)
 + 30 droneCollected(D3,P5,3) <= 10
 packageWasCollectedAtRightPositionY2(P5,D3,4): - droneDY(D3,4)
 + 30 droneCollected(D3,P5,4) <= 10
 packageWasCollectedAtRightPositionY2(P5,D3,5): - droneDY(D3,5)
 + 30 droneCollected(D3,P5,5) <= 10
 packageWasCollectedAtRightPositionY2(P6,D1,1): - droneDY(D1,1)
 + 30 droneCollected(D1,P6,1) <= 20
 packageWasCollectedAtRightPositionY2(P6,D1,2): - droneDY(D1,2)
 + 30 droneCollected(D1,P6,2) <= 20
 packageWasCollectedAtRightPositionY2(P6,D1,3): - droneDY(D1,3)
 + 30 droneCollected(D1,P6,3) <= 20
 packageWasCollectedAtRightPositionY2(P6,D1,4): - droneDY(D1,4)
 + 30 droneCollected(D1,P6,4) <= 20
 packageWasCollectedAtRightPositionY2(P6,D1,5): - droneDY(D1,5)
 + 30 droneCollected(D1,P6,5) <= 20
 packageWasCollectedAtRightPositionY2(P6,D2,1): - droneDY(D2,1)
 + 30 droneCollected(D2,P6,1) <= 20
 packageWasCollectedAtRightPositionY2(P6,D2,2): - droneDY(D2,2)
 + 30 droneCollected(D2,P6,2) <= 20
 packageWasCollectedAtRightPositionY2(P6,D2,3): - droneDY(D2,3)
 + 30 droneCollected(D2,P6,3) <= 20
 packageWasCollectedAtRightPositionY2(P6,D2,4): - droneDY(D2,4)
 + 30 droneCollected(D2,P6,4) <= 20
 packageWasCollectedAtRightPositionY2(P6,D2,5): - droneDY(D2,5)
 + 30 droneCollected(D2,P6,5) <= 20
 packageWasCollectedAtRightPositionY2(P6,D3,1): - droneDY(D3,1)
 + 30 droneCollected(D3,P6,1) <= 20
 packageWasCollectedAtRightPositionY2(P6,D3,2): - droneDY(D3,2)
 + 30 droneCollected(D3,P6,2) <= 20
 packageWasCollectedAtRightPositionY2(P6,D3,3): - droneDY(D3,3)
 + 30 droneCollected(D3,P6,3) <= 20
 packageWasCollectedAtRightPositionY2(P6,D3,4): - droneDY(D3,4)
 + 30 droneCollected(D3,P6,4) <= 20
 packageWasCollectedAtRightPositionY2(P6,D3,5): - droneDY(D3,5)
 + 30 droneCollected(D3,P6,5) <= 20
 packageWasDeliveredAtRightPositionX1(P1,D1,1): + droneDX(D1,1)
 + 30 droneDelivered(D1,P1,1) <= 34
 packageWasDeliveredAtRightPositionX1(P1,D1,2): + droneDX(D1,2)
 + 30 droneDelivered(D1,P1,2) <= 34
 packageWasDeliveredAtRightPositionX1(P1,D1,3): + droneDX(D1,3)
 + 30 droneDelivered(D1,P1,3) <= 34
 packageWasDeliveredAtRightPositionX1(P1,D1,4): + droneDX(D1,4)
 + 30 droneDelivered(D1,P1,4) <= 34
 packageWasDeliveredAtRightPositionX1(P1,D1,5): + droneDX(D1,5)
 + 30 droneDelivered(D1,P1,5) <= 34
 packageWasDeliveredAtRightPositionX1(P1,D2,1): + droneDX(D2,1)
 + 30 droneDelivered(D2,P1,1) <= 34
 packageWasDeliveredAtRightPositionX1(P1,D2,2): + droneDX(D2,2)
 + 30 droneDelivered(D2,P1,2) <= 34
 packageWasDeliveredAtRightPositionX1(P1,D2,3): + droneDX(D2,3)
 + 30 droneDelivered(D2,P1,3) <= 34
 packageWasDeliveredAtRightPositionX1(P1,D2,4): + droneDX(D2,4)
 + 30 droneDelivered(D2,P1,4) <= 34
 packageWasDeliveredAtRightPositionX1(P1,D2,5): + droneDX(D2,5)
 + 30 droneDelivered(D2,P1,5) <= 34
 packageWasDeliveredAtRightPositionX1(P1,D3,1): + droneDX(D3,1)
 + 30 droneDelivered(D3,P1,1) <= 34
 packageWasDeliveredAtRightPositionX1(P1,D3,2): + droneDX(D3,2)
 + 30 droneDelivered(D3,P1,2) <= 34
 packageWasDeliveredAtRightPositionX1(P1,D3,3): + droneDX(D3,3)
 + 30 droneDelivered(D3,P1,3) <= 34
 packageWasDeliveredAtRightPositionX1(P1,D3,4): + droneDX(D3,4)
 + 30 droneDelivered(D3,P1,4) <= 34
 packageWasDeliveredAtRightPositionX1(P1,D3,5): + droneDX(D3,5)
 + 30 droneDelivered(D3,P1,5) <= 34
 packageWasDeliveredAtRightPositionX1(P2,D1,1): + droneDX(D1,1)
 + 30 droneDelivered(D1,P2,1) <= 40
 packageWasDeliveredAtRightPositionX1(P2,D1,2): + droneDX(D1,2)
 + 30 droneDelivered(D1,P2,2) <= 40
 packageWasDeliveredAtRightPositionX1(P2,D1,3): + droneDX(D1,3)
 + 30 droneDelivered(D1,P2,3) <= 40
 packageWasDeliveredAtRightPositionX1(P2,D1,4): + droneDX(D1,4)
 + 30 droneDelivered(D1,P2,4) <= 40
 packageWasDeliveredAtRightPositionX1(P2,D1,5): + droneDX(D1,5)
 + 30 droneDelivered(D1,P2,5) <= 40
 packageWasDeliveredAtRightPositionX1(P2,D2,1): + droneDX(D2,1)
 + 30 droneDelivered(D2,P2,1) <= 40
 packageWasDeliveredAtRightPositionX1(P2,D2,2): + droneDX(D2,2)
 + 30 droneDelivered(D2,P2,2) <= 40
 packageWasDeliveredAtRightPositionX1(P2,D2,3): + droneDX(D2,3)
 + 30 droneDelivered(D2,P2,3) <= 40
 packageWasDeliveredAtRightPositionX1(P2,D2,4): + droneDX(D2,4)
 + 30 droneDelivered(D2,P2,4) <= 40
 packageWasDeliveredAtRightPositionX1(P2,D2,5): + droneDX(D2,5)
 + 30 droneDelivered(D2,P2,5) <= 40
 packageWasDeliveredAtRightPositionX1(P2,D3,1): + droneDX(D3,1)
 + 30 droneDelivered(D3,P2,1) <= 40
 packageWasDeliveredAtRightPositionX1(P2,D3,2): + droneDX(D3,2)
 + 30 droneDelivered(D3,P2,2) <= 40
 packageWasDeliveredAtRightPositionX1(P2,D3,3): + droneDX(D3,3)
 + 30 droneDelivered(D3,P2,3) <= 40
 packageWasDeliveredAtRightPositionX1(P2,D3,4): + droneDX(D3,4)
 + 30 droneDelivered(D3,P2,4) <= 40
 packageWasDeliveredAtRightPositionX1(P2,D3,5): + droneDX(D3,5)
 + 30 droneDelivered(D3,P2,5) <= 40
 packageWasDeliveredAtRightPositionX1(P3,D1,1): + droneDX(D1,1)
 + 30 droneDelivered(D1,P3,1) <= 50
 packageWasDeliveredAtRightPositionX1(P3,D1,2): + droneDX(D1,2)
 + 30 droneDelivered(D1,P3,2) <= 50
 packageWasDeliveredAtRightPositionX1(P3,D1,3): + droneDX(D1,3)
 + 30 droneDelivered(D1,P3,3) <= 50
 packageWasDeliveredAtRightPositionX1(P3,D1,4): + droneDX(D1,4)
 + 30 droneDelivered(D1,P3,4) <= 50
 packageWasDeliveredAtRightPositionX1(P3,D1,5): + droneDX(D1,5)
 + 30 droneDelivered(D1,P3,5) <= 50
 packageWasDeliveredAtRightPositionX1(P3,D2,1): + droneDX(D2,1)
 + 30 droneDelivered(D2,P3,1) <= 50
 packageWasDeliveredAtRightPositionX1(P3,D2,2): + droneDX(D2,2)
 + 30 droneDelivered(D2,P3,2) <= 50
 packageWasDeliveredAtRightPositionX1(P3,D2,3): + droneDX(D2,3)
 + 30 droneDelivered(D2,P3,3) <= 50
 packageWasDeliveredAtRightPositionX1(P3,D2,4): + droneDX(D2,4)
 + 30 droneDelivered(D2,P3,4) <= 50
 packageWasDeliveredAtRightPositionX1(P3,D2,5): + droneDX(D2,5)
 + 30 droneDelivered(D2,P3,5) <= 50
 packageWasDeliveredAtRightPositionX1(P3,D3,1): + droneDX(D3,1)
 + 30 droneDelivered(D3,P3,1) <= 50
 packageWasDeliveredAtRightPositionX1(P3,D3,2): + droneDX(D3,2)
 + 30 droneDelivered(D3,P3,2) <= 50
 packageWasDeliveredAtRightPositionX1(P3,D3,3): + droneDX(D3,3)
 + 30 droneDelivered(D3,P3,3) <= 50
 packageWasDeliveredAtRightPositionX1(P3,D3,4): + droneDX(D3,4)
 + 30 droneDelivered(D3,P3,4) <= 50
 packageWasDeliveredAtRightPositionX1(P3,D3,5): + droneDX(D3,5)
 + 30 droneDelivered(D3,P3,5) <= 50
 packageWasDeliveredAtRightPositionX1(P4,D1,1): + droneDX(D1,1)
 + 30 droneDelivered(D1,P4,1) <= 35
 packageWasDeliveredAtRightPositionX1(P4,D1,2): + droneDX(D1,2)
 + 30 droneDelivered(D1,P4,2) <= 35
 packageWasDeliveredAtRightPositionX1(P4,D1,3): + droneDX(D1,3)
 + 30 droneDelivered(D1,P4,3) <= 35
 packageWasDeliveredAtRightPositionX1(P4,D1,4): + droneDX(D1,4)
 + 30 droneDelivered(D1,P4,4) <= 35
 packageWasDeliveredAtRightPositionX1(P4,D1,5): + droneDX(D1,5)
 + 30 droneDelivered(D1,P4,5) <= 35
 packageWasDeliveredAtRightPositionX1(P4,D2,1): + droneDX(D2,1)
 + 30 droneDelivered(D2,P4,1) <= 35
 packageWasDeliveredAtRightPositionX1(P4,D2,2): + droneDX(D2,2)
 + 30 droneDelivered(D2,P4,2) <= 35
 packageWasDeliveredAtRightPositionX1(P4,D2,3): + droneDX(D2,3)
 + 30 droneDelivered(D2,P4,3) <= 35
 packageWasDeliveredAtRightPositionX1(P4,D2,4): + droneDX(D2,4)
 + 30 droneDelivered(D2,P4,4) <= 35
 packageWasDeliveredAtRightPositionX1(P4,D2,5): + droneDX(D2,5)
 + 30 droneDelivered(D2,P4,5) <= 35
 packageWasDeliveredAtRightPositionX1(P4,D3,1): + droneDX(D3,1)
 + 30 droneDelivered(D3,P4,1) <= 35
 packageWasDeliveredAtRightPositionX1(P4,D3,2): + droneDX(D3,2)
 + 30 droneDelivered(D3,P4,2) <= 35
 packageWasDeliveredAtRightPositionX1(P4,D3,3): + droneDX(D3,3)
 + 30 droneDelivered(D3,P4,3) <= 35
 packageWasDeliveredAtRightPositionX1(P4,D3,4): + droneDX(D3,4)
 + 30 droneDelivered(D3,P4,4) <= 35
 packageWasDeliveredAtRightPositionX1(P4,D3,5): + droneDX(D3,5)
 + 30 droneDelivered(D3,P4,5) <= 35
 packageWasDeliveredAtRightPositionX1(P5,D1,1): + droneDX(D1,1)
 + 30 droneDelivered(D1,P5,1) <= 40
 packageWasDeliveredAtRightPositionX1(P5,D1,2): + droneDX(D1,2)
 + 30 droneDelivered(D1,P5,2) <= 40
 packageWasDeliveredAtRightPositionX1(P5,D1,3): + droneDX(D1,3)
 + 30 droneDelivered(D1,P5,3) <= 40
 packageWasDeliveredAtRightPositionX1(P5,D1,4): + droneDX(D1,4)
 + 30 droneDelivered(D1,P5,4) <= 40
 packageWasDeliveredAtRightPositionX1(P5,D1,5): + droneDX(D1,5)
 + 30 droneDelivered(D1,P5,5) <= 40
 packageWasDeliveredAtRightPositionX1(P5,D2,1): + droneDX(D2,1)
 + 30 droneDelivered(D2,P5,1) <= 40
 packageWasDeliveredAtRightPositionX1(P5,D2,2): + droneDX(D2,2)
 + 30 droneDelivered(D2,P5,2) <= 40
 packageWasDeliveredAtRightPositionX1(P5,D2,3): + droneDX(D2,3)
 + 30 droneDelivered(D2,P5,3) <= 40
 packageWasDeliveredAtRightPositionX1(P5,D2,4): + droneDX(D2,4)
 + 30 droneDelivered(D2,P5,4) <= 40
 packageWasDeliveredAtRightPositionX1(P5,D2,5): + droneDX(D2,5)
 + 30 droneDelivered(D2,P5,5) <= 40
 packageWasDeliveredAtRightPositionX1(P5,D3,1): + droneDX(D3,1)
 + 30 droneDelivered(D3,P5,1) <= 40
 packageWasDeliveredAtRightPositionX1(P5,D3,2): + droneDX(D3,2)
 + 30 droneDelivered(D3,P5,2) <= 40
 packageWasDeliveredAtRightPositionX1(P5,D3,3): + droneDX(D3,3)
 + 30 droneDelivered(D3,P5,3) <= 40
 packageWasDeliveredAtRightPositionX1(P5,D3,4): + droneDX(D3,4)
 + 30 droneDelivered(D3,P5,4) <= 40
 packageWasDeliveredAtRightPositionX1(P5,D3,5): + droneDX(D3,5)
 + 30 droneDelivered(D3,P5,5) <= 40
 packageWasDeliveredAtRightPositionX1(P6,D1,1): + droneDX(D1,1)
 + 30 droneDelivered(D1,P6,1) <= 30
 packageWasDeliveredAtRightPositionX1(P6,D1,2): + droneDX(D1,2)
 + 30 droneDelivered(D1,P6,2) <= 30
 packageWasDeliveredAtRightPositionX1(P6,D1,3): + droneDX(D1,3)
 + 30 droneDelivered(D1,P6,3) <= 30
 packageWasDeliveredAtRightPositionX1(P6,D1,4): + droneDX(D1,4)
 + 30 droneDelivered(D1,P6,4) <= 30
 packageWasDeliveredAtRightPositionX1(P6,D1,5): + droneDX(D1,5)
 + 30 droneDelivered(D1,P6,5) <= 30
 packageWasDeliveredAtRightPositionX1(P6,D2,1): + droneDX(D2,1)
 + 30 droneDelivered(D2,P6,1) <= 30
 packageWasDeliveredAtRightPositionX1(P6,D2,2): + droneDX(D2,2)
 + 30 droneDelivered(D2,P6,2) <= 30
 packageWasDeliveredAtRightPositionX1(P6,D2,3): + droneDX(D2,3)
 + 30 droneDelivered(D2,P6,3) <= 30
 packageWasDeliveredAtRightPositionX1(P6,D2,4): + droneDX(D2,4)
 + 30 droneDelivered(D2,P6,4) <= 30
 packageWasDeliveredAtRightPositionX1(P6,D2,5): + droneDX(D2,5)
 + 30 droneDelivered(D2,P6,5) <= 30
 packageWasDeliveredAtRightPositionX1(P6,D3,1): + droneDX(D3,1)
 + 30 droneDelivered(D3,P6,1) <= 30
 packageWasDeliveredAtRightPositionX1(P6,D3,2): + droneDX(D3,2)
 + 30 droneDelivered(D3,P6,2) <= 30
 packageWasDeliveredAtRightPositionX1(P6,D3,3): + droneDX(D3,3)
 + 30 droneDelivered(D3,P6,3) <= 30
 packageWasDeliveredAtRightPositionX1(P6,D3,4): + droneDX(D3,4)
 + 30 droneDelivered(D3,P6,4) <= 30
 packageWasDeliveredAtRightPositionX1(P6,D3,5): + droneDX(D3,5)
 + 30 droneDelivered(D3,P6,5) <= 30
 packageWasDeliveredAtRightPositionX2(P1,D1,1): - droneDX(D1,1)
 + 30 droneDelivered(D1,P1,1) <= 26
 packageWasDeliveredAtRightPositionX2(P1,D1,2): - droneDX(D1,2)
 + 30 droneDelivered(D1,P1,2) <= 26
 packageWasDeliveredAtRightPositionX2(P1,D1,3): - droneDX(D1,3)
 + 30 droneDelivered(D1,P1,3) <= 26
 packageWasDeliveredAtRightPositionX2(P1,D1,4): - droneDX(D1,4)
 + 30 droneDelivered(D1,P1,4) <= 26
 packageWasDeliveredAtRightPositionX2(P1,D1,5): - droneDX(D1,5)
 + 30 droneDelivered(D1,P1,5) <= 26
 packageWasDeliveredAtRightPositionX2(P1,D2,1): - droneDX(D2,1)
 + 30 droneDelivered(D2,P1,1) <= 26
 packageWasDeliveredAtRightPositionX2(P1,D2,2): - droneDX(D2,2)
 + 30 droneDelivered(D2,P1,2) <= 26
 packageWasDeliveredAtRightPositionX2(P1,D2,3): - droneDX(D2,3)
 + 30 droneDelivered(D2,P1,3) <= 26
 packageWasDeliveredAtRightPositionX2(P1,D2,4): - droneDX(D2,4)
 + 30 droneDelivered(D2,P1,4) <= 26
 packageWasDeliveredAtRightPositionX2(P1,D2,5): - droneDX(D2,5)
 + 30 droneDelivered(D2,P1,5) <= 26
 packageWasDeliveredAtRightPositionX2(P1,D3,1): - droneDX(D3,1)
 + 30 droneDelivered(D3,P1,1) <= 26
 packageWasDeliveredAtRightPositionX2(P1,D3,2): - droneDX(D3,2)
 + 30 droneDelivered(D3,P1,2) <= 26
 packageWasDeliveredAtRightPositionX2(P1,D3,3): - droneDX(D3,3)
 + 30 droneDelivered(D3,P1,3) <= 26
 packageWasDeliveredAtRightPositionX2(P1,D3,4): - droneDX(D3,4)
 + 30 droneDelivered(D3,P1,4) <= 26
 packageWasDeliveredAtRightPositionX2(P1,D3,5): - droneDX(D3,5)
 + 30 droneDelivered(D3,P1,5) <= 26
 packageWasDeliveredAtRightPositionX2(P2,D1,1): - droneDX(D1,1)
 + 30 droneDelivered(D1,P2,1) <= 20
 packageWasDeliveredAtRightPositionX2(P2,D1,2): - droneDX(D1,2)
 + 30 droneDelivered(D1,P2,2) <= 20
 packageWasDeliveredAtRightPositionX2(P2,D1,3): - droneDX(D1,3)
 + 30 droneDelivered(D1,P2,3) <= 20
 packageWasDeliveredAtRightPositionX2(P2,D1,4): - droneDX(D1,4)
 + 30 droneDelivered(D1,P2,4) <= 20
 packageWasDeliveredAtRightPositionX2(P2,D1,5): - droneDX(D1,5)
 + 30 droneDelivered(D1,P2,5) <= 20
 packageWasDeliveredAtRightPositionX2(P2,D2,1): - droneDX(D2,1)
 + 30 droneDelivered(D2,P2,1) <= 20
 packageWasDeliveredAtRightPositionX2(P2,D2,2): - droneDX(D2,2)
 + 30 droneDelivered(D2,P2,2) <= 20
 packageWasDeliveredAtRightPositionX2(P2,D2,3): - droneDX(D2,3)
 + 30 droneDelivered(D2,P2,3) <= 20
 packageWasDeliveredAtRightPositionX2(P2,D2,4): - droneDX(D2,4)
 + 30 droneDelivered(D2,P2,4) <= 20
 packageWasDeliveredAtRightPositionX2(P2,D2,5): - droneDX(D2,5)
 + 30 droneDelivered(D2,P2,5) <= 20
 packageWasDeliveredAtRightPositionX2(P2,D3,1): - droneDX(D3,1)
 + 30 droneDelivered(D3,P2,1) <= 20
 packageWasDeliveredAtRightPositionX2(P2,D3,2): - droneDX(D3,2)
 + 30 droneDelivered(D3,P2,2) <= 20
 packageWasDeliveredAtRightPositionX2(P2,D3,3): - droneDX(D3,3)
 + 30 droneDelivered(D3,P2,3) <= 20
 packageWasDeliveredAtRightPositionX2(P2,D3,4): - droneDX(D3,4)
 + 30 droneDelivered(D3,P2,4) <= 20
 packageWasDeliveredAtRightPositionX2(P2,D3,5): - droneDX(D3,5)
 + 30 droneDelivered(D3,P2,5) <= 20
 packageWasDeliveredAtRightPositionX2(P3,D1,1): - droneDX(D1,1)
 + 30 droneDelivered(D1,P3,1) <= 10
 packageWasDeliveredAtRightPositionX2(P3,D1,2): - droneDX(D1,2)
 + 30 droneDelivered(D1,P3,2) <= 10
 packageWasDeliveredAtRightPositionX2(P3,D1,3): - droneDX(D1,3)
 + 30 droneDelivered(D1,P3,3) <= 10
 packageWasDeliveredAtRightPositionX2(P3,D1,4): - droneDX(D1,4)
 + 30 droneDelivered(D1,P3,4) <= 10
 packageWasDeliveredAtRightPositionX2(P3,D1,5): - droneDX(D1,5)
 + 30 droneDelivered(D1,P3,5) <= 10
 packageWasDeliveredAtRightPositionX2(P3,D2,1): - droneDX(D2,1)
 + 30 droneDelivered(D2,P3,1) <= 10
 packageWasDeliveredAtRightPositionX2(P3,D2,2): - droneDX(D2,2)
 + 30 droneDelivered(D2,P3,2) <= 10
 packageWasDeliveredAtRightPositionX2(P3,D2,3): - droneDX(D2,3)
 + 30 droneDelivered(D2,P3,3) <= 10
 packageWasDeliveredAtRightPositionX2(P3,D2,4): - droneDX(D2,4)
 + 30 droneDelivered(D2,P3,4) <= 10
 packageWasDeliveredAtRightPositionX2(P3,D2,5): - droneDX(D2,5)
 + 30 droneDelivered(D2,P3,5) <= 10
 packageWasDeliveredAtRightPositionX2(P3,D3,1): - droneDX(D3,1)
 + 30 droneDelivered(D3,P3,1) <= 10
 packageWasDeliveredAtRightPositionX2(P3,D3,2): - droneDX(D3,2)
 + 30 droneDelivered(D3,P3,2) <= 10
 packageWasDeliveredAtRightPositionX2(P3,D3,3): - droneDX(D3,3)
 + 30 droneDelivered(D3,P3,3) <= 10
 packageWasDeliveredAtRightPositionX2(P3,D3,4): - droneDX(D3,4)
 + 30 droneDelivered(D3,P3,4) <= 10
 packageWasDeliveredAtRightPositionX2(P3,D3,5): - droneDX(D3,5)
 + 30 droneDelivered(D3,P3,5) <= 10
 packageWasDeliveredAtRightPositionX2(P4,D1,1): - droneDX(D1,1)
 + 30 droneDelivered(D1,P4,1) <= 25
 packageWasDeliveredAtRightPositionX2(P4,D1,2): - droneDX(D1,2)
 + 30 droneDelivered(D1,P4,2) <= 25
 packageWasDeliveredAtRightPositionX2(P4,D1,3): - droneDX(D1,3)
 + 30 droneDelivered(D1,P4,3) <= 25
 packageWasDeliveredAtRightPositionX2(P4,D1,4): - droneDX(D1,4)
 + 30 droneDelivered(D1,P4,4) <= 25
 packageWasDeliveredAtRightPositionX2(P4,D1,5): - droneDX(D1,5)
 + 30 droneDelivered(D1,P4,5) <= 25
 packageWasDeliveredAtRightPositionX2(P4,D2,1): - droneDX(D2,1)
 + 30 droneDelivered(D2,P4,1) <= 25
 packageWasDeliveredAtRightPositionX2(P4,D2,2): - droneDX(D2,2)
 + 30 droneDelivered(D2,P4,2) <= 25
 packageWasDeliveredAtRightPositionX2(P4,D2,3): - droneDX(D2,3)
 + 30 droneDelivered(D2,P4,3) <= 25
 packageWasDeliveredAtRightPositionX2(P4,D2,4): - droneDX(D2,4)
 + 30 droneDelivered(D2,P4,4) <= 25
 packageWasDeliveredAtRightPositionX2(P4,D2,5): - droneDX(D2,5)
 + 30 droneDelivered(D2,P4,5) <= 25
 packageWasDeliveredAtRightPositionX2(P4,D3,1): - droneDX(D3,1)
 + 30 droneDelivered(D3,P4,1) <= 25
 packageWasDeliveredAtRightPositionX2(P4,D3,2): - droneDX(D3,2)
 + 30 droneDelivered(D3,P4,2) <= 25
 packageWasDeliveredAtRightPositionX2(P4,D3,3): - droneDX(D3,3)
 + 30 droneDelivered(D3,P4,3) <= 25
 packageWasDeliveredAtRightPositionX2(P4,D3,4): - droneDX(D3,4)
 + 30 droneDelivered(D3,P4,4) <= 25
 packageWasDeliveredAtRightPositionX2(P4,D3,5): - droneDX(D3,5)
 + 30 droneDelivered(D3,P4,5) <= 25
 packageWasDeliveredAtRightPositionX2(P5,D1,1): - droneDX(D1,1)
 + 30 droneDelivered(D1,P5,1) <= 20
 packageWasDeliveredAtRightPositionX2(P5,D1,2): - droneDX(D1,2)
 + 30 droneDelivered(D1,P5,2) <= 20
 packageWasDeliveredAtRightPositionX2(P5,D1,3): - droneDX(D1,3)
 + 30 droneDelivered(D1,P5,3) <= 20
 packageWasDeliveredAtRightPositionX2(P5,D1,4): - droneDX(D1,4)
 + 30 droneDelivered(D1,P5,4) <= 20
 packageWasDeliveredAtRightPositionX2(P5,D1,5): - droneDX(D1,5)
 + 30 droneDelivered(D1,P5,5) <= 20
 packageWasDeliveredAtRightPositionX2(P5,D2,1): - droneDX(D2,1)
 + 30 droneDelivered(D2,P5,1) <= 20
 packageWasDeliveredAtRightPositionX2(P5,D2,2): - droneDX(D2,2)
 + 30 droneDelivered(D2,P5,2) <= 20
 packageWasDeliveredAtRightPositionX2(P5,D2,3): - droneDX(D2,3)
 + 30 droneDelivered(D2,P5,3) <= 20
 packageWasDeliveredAtRightPositionX2(P5,D2,4): - droneDX(D2,4)
 + 30 droneDelivered(D2,P5,4) <= 20
 packageWasDeliveredAtRightPositionX2(P5,D2,5): - droneDX(D2,5)
 + 30 droneDelivered(D2,P5,5) <= 20
 packageWasDeliveredAtRightPositionX2(P5,D3,1): - droneDX(D3,1)
 + 30 droneDelivered(D3,P5,1) <= 20
 packageWasDeliveredAtRightPositionX2(P5,D3,2): - droneDX(D3,2)
 + 30 droneDelivered(D3,P5,2) <= 20
 packageWasDeliveredAtRightPositionX2(P5,D3,3): - droneDX(D3,3)
 + 30 droneDelivered(D3,P5,3) <= 20
 packageWasDeliveredAtRightPositionX2(P5,D3,4): - droneDX(D3,4)
 + 30 droneDelivered(D3,P5,4) <= 20
 packageWasDeliveredAtRightPositionX2(P5,D3,5): - droneDX(D3,5)
 + 30 droneDelivered(D3,P5,5) <= 20
 packageWasDeliveredAtRightPositionX2(P6,D1,1): - droneDX(D1,1)
 + 30 droneDelivered(D1,P6,1) <= 30
 packageWasDeliveredAtRightPositionX2(P6,D1,2): - droneDX(D1,2)
 + 30 droneDelivered(D1,P6,2) <= 30
 packageWasDeliveredAtRightPositionX2(P6,D1,3): - droneDX(D1,3)
 + 30 droneDelivered(D1,P6,3) <= 30
 packageWasDeliveredAtRightPositionX2(P6,D1,4): - droneDX(D1,4)
 + 30 droneDelivered(D1,P6,4) <= 30
 packageWasDeliveredAtRightPositionX2(P6,D1,5): - droneDX(D1,5)
 + 30 droneDelivered(D1,P6,5) <= 30
 packageWasDeliveredAtRightPositionX2(P6,D2,1): - droneDX(D2,1)
 + 30 droneDelivered(D2,P6,1) <= 30
 packageWasDeliveredAtRightPositionX2(P6,D2,2): - droneDX(D2,2)
 + 30 droneDelivered(D2,P6,2) <= 30
 packageWasDeliveredAtRightPositionX2(P6,D2,3): - droneDX(D2,3)
 + 30 droneDelivered(D2,P6,3) <= 30
 packageWasDeliveredAtRightPositionX2(P6,D2,4): - droneDX(D2,4)
 + 30 droneDelivered(D2,P6,4) <= 30
 packageWasDeliveredAtRightPositionX2(P6,D2,5): - droneDX(D2,5)
 + 30 droneDelivered(D2,P6,5) <= 30
 packageWasDeliveredAtRightPositionX2(P6,D3,1): - droneDX(D3,1)
 + 30 droneDelivered(D3,P6,1) <= 30
 packageWasDeliveredAtRightPositionX2(P6,D3,2): - droneDX(D3,2)
 + 30 droneDelivered(D3,P6,2) <= 30
 packageWasDeliveredAtRightPositionX2(P6,D3,3): - droneDX(D3,3)
 + 30 droneDelivered(D3,P6,3) <= 30
 packageWasDeliveredAtRightPositionX2(P6,D3,4): - droneDX(D3,4)
 + 30 droneDelivered(D3,P6,4) <= 30
 packageWasDeliveredAtRightPositionX2(P6,D3,5): - droneDX(D3,5)
 + 30 droneDelivered(D3,P6,5) <= 30
 packageWasDeliveredAtRightPositionY1(P1,D1,1): + droneDY(D1,1)
 + 30 droneDelivered(D1,P1,1) <= 30
 packageWasDeliveredAtRightPositionY1(P1,D1,2): + droneDY(D1,2)
 + 30 droneDelivered(D1,P1,2) <= 30
 packageWasDeliveredAtRightPositionY1(P1,D1,3): + droneDY(D1,3)
 + 30 droneDelivered(D1,P1,3) <= 30
 packageWasDeliveredAtRightPositionY1(P1,D1,4): + droneDY(D1,4)
 + 30 droneDelivered(D1,P1,4) <= 30
 packageWasDeliveredAtRightPositionY1(P1,D1,5): + droneDY(D1,5)
 + 30 droneDelivered(D1,P1,5) <= 30
 packageWasDeliveredAtRightPositionY1(P1,D2,1): + droneDY(D2,1)
 + 30 droneDelivered(D2,P1,1) <= 30
 packageWasDeliveredAtRightPositionY1(P1,D2,2): + droneDY(D2,2)
 + 30 droneDelivered(D2,P1,2) <= 30
 packageWasDeliveredAtRightPositionY1(P1,D2,3): + droneDY(D2,3)
 + 30 droneDelivered(D2,P1,3) <= 30
 packageWasDeliveredAtRightPositionY1(P1,D2,4): + droneDY(D2,4)
 + 30 droneDelivered(D2,P1,4) <= 30
 packageWasDeliveredAtRightPositionY1(P1,D2,5): + droneDY(D2,5)
 + 30 droneDelivered(D2,P1,5) <= 30
 packageWasDeliveredAtRightPositionY1(P1,D3,1): + droneDY(D3,1)
 + 30 droneDelivered(D3,P1,1) <= 30
 packageWasDeliveredAtRightPositionY1(P1,D3,2): + droneDY(D3,2)
 + 30 droneDelivered(D3,P1,2) <= 30
 packageWasDeliveredAtRightPositionY1(P1,D3,3): + droneDY(D3,3)
 + 30 droneDelivered(D3,P1,3) <= 30
 packageWasDeliveredAtRightPositionY1(P1,D3,4): + droneDY(D3,4)
 + 30 droneDelivered(D3,P1,4) <= 30
 packageWasDeliveredAtRightPositionY1(P1,D3,5): + droneDY(D3,5)
 + 30 droneDelivered(D3,P1,5) <= 30
 packageWasDeliveredAtRightPositionY1(P2,D1,1): + droneDY(D1,1)
 + 30 droneDelivered(D1,P2,1) <= 30
 packageWasDeliveredAtRightPositionY1(P2,D1,2): + droneDY(D1,2)
 + 30 droneDelivered(D1,P2,2) <= 30
 packageWasDeliveredAtRightPositionY1(P2,D1,3): + droneDY(D1,3)
 + 30 droneDelivered(D1,P2,3) <= 30
 packageWasDeliveredAtRightPositionY1(P2,D1,4): + droneDY(D1,4)
 + 30 droneDelivered(D1,P2,4) <= 30
 packageWasDeliveredAtRightPositionY1(P2,D1,5): + droneDY(D1,5)
 + 30 droneDelivered(D1,P2,5) <= 30
 packageWasDeliveredAtRightPositionY1(P2,D2,1): + droneDY(D2,1)
 + 30 droneDelivered(D2,P2,1) <= 30
 packageWasDeliveredAtRightPositionY1(P2,D2,2): + droneDY(D2,2)
 + 30 droneDelivered(D2,P2,2) <= 30
 packageWasDeliveredAtRightPositionY1(P2,D2,3): + droneDY(D2,3)
 + 30 droneDelivered(D2,P2,3) <= 30
 packageWasDeliveredAtRightPositionY1(P2,D2,4): + droneDY(D2,4)
 + 30 droneDelivered(D2,P2,4) <= 30
 packageWasDeliveredAtRightPositionY1(P2,D2,5): + droneDY(D2,5)
 + 30 droneDelivered(D2,P2,5) <= 30
 packageWasDeliveredAtRightPositionY1(P2,D3,1): + droneDY(D3,1)
 + 30 droneDelivered(D3,P2,1) <= 30
 packageWasDeliveredAtRightPositionY1(P2,D3,2): + droneDY(D3,2)
 + 30 droneDelivered(D3,P2,2) <= 30
 packageWasDeliveredAtRightPositionY1(P2,D3,3): + droneDY(D3,3)
 + 30 droneDelivered(D3,P2,3) <= 30
 packageWasDeliveredAtRightPositionY1(P2,D3,4): + droneDY(D3,4)
 + 30 droneDelivered(D3,P2,4) <= 30
 packageWasDeliveredAtRightPositionY1(P2,D3,5): + droneDY(D3,5)
 + 30 droneDelivered(D3,P2,5) <= 30
 packageWasDeliveredAtRightPositionY1(P3,D1,1): + droneDY(D1,1)
 + 30 droneDelivered(D1,P3,1) <= 35
 packageWasDeliveredAtRightPositionY1(P3,D1,2): + droneDY(D1,2)
 + 30 droneDelivered(D1,P3,2) <= 35
 packageWasDeliveredAtRightPositionY1(P3,D1,3): + droneDY(D1,3)
 + 30 droneDelivered(D1,P3,3) <= 35
 packageWasDeliveredAtRightPositionY1(P3,D1,4): + droneDY(D1,4)
 + 30 droneDelivered(D1,P3,4) <= 35
 packageWasDeliveredAtRightPositionY1(P3,D1,5): + droneDY(D1,5)
 + 30 droneDelivered(D1,P3,5) <= 35
 packageWasDeliveredAtRightPositionY1(P3,D2,1): + droneDY(D2,1)
 + 30 droneDelivered(D2,P3,1) <= 35
 packageWasDeliveredAtRightPositionY1(P3,D2,2): + droneDY(D2,2)
 + 30 droneDelivered(D2,P3,2) <= 35
 packageWasDeliveredAtRightPositionY1(P3,D2,3): + droneDY(D2,3)
 + 30 droneDelivered(D2,P3,3) <= 35
 packageWasDeliveredAtRightPositionY1(P3,D2,4): + droneDY(D2,4)
 + 30 droneDelivered(D2,P3,4) <= 35
 packageWasDeliveredAtRightPositionY1(P3,D2,5): + droneDY(D2,5)
 + 30 droneDelivered(D2,P3,5) <= 35
 packageWasDeliveredAtRightPositionY1(P3,D3,1): + droneDY(D3,1)
 + 30 droneDelivered(D3,P3,1) <= 35
 packageWasDeliveredAtRightPositionY1(P3,D3,2): + droneDY(D3,2)
 + 30 droneDelivered(D3,P3,2) <= 35
 packageWasDeliveredAtRightPositionY1(P3,D3,3): + droneDY(D3,3)
 + 30 droneDelivered(D3,P3,3) <= 35
 packageWasDeliveredAtRightPositionY1(P3,D3,4): + droneDY(D3,4)
 + 30 droneDelivered(D3,P3,4) <= 35
 packageWasDeliveredAtRightPositionY1(P3,D3,5): + droneDY(D3,5)
 + 30 droneDelivered(D3,P3,5) <= 35
 packageWasDeliveredAtRightPositionY1(P4,D1,1): + droneDY(D1,1)
 + 30 droneDelivered(D1,P4,1) <= 40
 packageWasDeliveredAtRightPositionY1(P4,D1,2): + droneDY(D1,2)
 + 30 droneDelivered(D1,P4,2) <= 40
 packageWasDeliveredAtRightPositionY1(P4,D1,3): + droneDY(D1,3)
 + 30 droneDelivered(D1,P4,3) <= 40
 packageWasDeliveredAtRightPositionY1(P4,D1,4): + droneDY(D1,4)
 + 30 droneDelivered(D1,P4,4) <= 40
 packageWasDeliveredAtRightPositionY1(P4,D1,5): + droneDY(D1,5)
 + 30 droneDelivered(D1,P4,5) <= 40
 packageWasDeliveredAtRightPositionY1(P4,D2,1): + droneDY(D2,1)
 + 30 droneDelivered(D2,P4,1) <= 40
 packageWasDeliveredAtRightPositionY1(P4,D2,2): + droneDY(D2,2)
 + 30 droneDelivered(D2,P4,2) <= 40
 packageWasDeliveredAtRightPositionY1(P4,D2,3): + droneDY(D2,3)
 + 30 droneDelivered(D2,P4,3) <= 40
 packageWasDeliveredAtRightPositionY1(P4,D2,4): + droneDY(D2,4)
 + 30 droneDelivered(D2,P4,4) <= 40
 packageWasDeliveredAtRightPositionY1(P4,D2,5): + droneDY(D2,5)
 + 30 droneDelivered(D2,P4,5) <= 40
 packageWasDeliveredAtRightPositionY1(P4,D3,1): + droneDY(D3,1)
 + 30 droneDelivered(D3,P4,1) <= 40
 packageWasDeliveredAtRightPositionY1(P4,D3,2): + droneDY(D3,2)
 + 30 droneDelivered(D3,P4,2) <= 40
 packageWasDeliveredAtRightPositionY1(P4,D3,3): + droneDY(D3,3)
 + 30 droneDelivered(D3,P4,3) <= 40
 packageWasDeliveredAtRightPositionY1(P4,D3,4): + droneDY(D3,4)
 + 30 droneDelivered(D3,P4,4) <= 40
 packageWasDeliveredAtRightPositionY1(P4,D3,5): + droneDY(D3,5)
 + 30 droneDelivered(D3,P4,5) <= 40
 packageWasDeliveredAtRightPositionY1(P5,D1,1): + droneDY(D1,1)
 + 30 droneDelivered(D1,P5,1) <= 35
 packageWasDeliveredAtRightPositionY1(P5,D1,2): + droneDY(D1,2)
 + 30 droneDelivered(D1,P5,2) <= 35
 packageWasDeliveredAtRightPositionY1(P5,D1,3): + droneDY(D1,3)
 + 30 droneDelivered(D1,P5,3) <= 35
 packageWasDeliveredAtRightPositionY1(P5,D1,4): + droneDY(D1,4)
 + 30 droneDelivered(D1,P5,4) <= 35
 packageWasDeliveredAtRightPositionY1(P5,D1,5): + droneDY(D1,5)
 + 30 droneDelivered(D1,P5,5) <= 35
 packageWasDeliveredAtRightPositionY1(P5,D2,1): + droneDY(D2,1)
 + 30 droneDelivered(D2,P5,1) <= 35
 packageWasDeliveredAtRightPositionY1(P5,D2,2): + droneDY(D2,2)
 + 30 droneDelivered(D2,P5,2) <= 35
 packageWasDeliveredAtRightPositionY1(P5,D2,3): + droneDY(D2,3)
 + 30 droneDelivered(D2,P5,3) <= 35
 packageWasDeliveredAtRightPositionY1(P5,D2,4): + droneDY(D2,4)
 + 30 droneDelivered(D2,P5,4) <= 35
 packageWasDeliveredAtRightPositionY1(P5,D2,5): + droneDY(D2,5)
 + 30 droneDelivered(D2,P5,5) <= 35
 packageWasDeliveredAtRightPositionY1(P5,D3,1): + droneDY(D3,1)
 + 30 droneDelivered(D3,P5,1) <= 35
 packageWasDeliveredAtRightPositionY1(P5,D3,2): + droneDY(D3,2)
 + 30 droneDelivered(D3,P5,2) <= 35
 packageWasDeliveredAtRightPositionY1(P5,D3,3): + droneDY(D3,3)
 + 30 droneDelivered(D3,P5,3) <= 35
 packageWasDeliveredAtRightPositionY1(P5,D3,4): + droneDY(D3,4)
 + 30 droneDelivered(D3,P5,4) <= 35
 packageWasDeliveredAtRightPositionY1(P5,D3,5): + droneDY(D3,5)
 + 30 droneDelivered(D3,P5,5) <= 35
 packageWasDeliveredAtRightPositionY1(P6,D1,1): + droneDY(D1,1)
 + 30 droneDelivered(D1,P6,1) <= 30
 packageWasDeliveredAtRightPositionY1(P6,D1,2): + droneDY(D1,2)
 + 30 droneDelivered(D1,P6,2) <= 30
 packageWasDeliveredAtRightPositionY1(P6,D1,3): + droneDY(D1,3)
 + 30 droneDelivered(D1,P6,3) <= 30
 packageWasDeliveredAtRightPositionY1(P6,D1,4): + droneDY(D1,4)
 + 30 droneDelivered(D1,P6,4) <= 30
 packageWasDeliveredAtRightPositionY1(P6,D1,5): + droneDY(D1,5)
 + 30 droneDelivered(D1,P6,5) <= 30
 packageWasDeliveredAtRightPositionY1(P6,D2,1): + droneDY(D2,1)
 + 30 droneDelivered(D2,P6,1) <= 30
 packageWasDeliveredAtRightPositionY1(P6,D2,2): + droneDY(D2,2)
 + 30 droneDelivered(D2,P6,2) <= 30
 packageWasDeliveredAtRightPositionY1(P6,D2,3): + droneDY(D2,3)
 + 30 droneDelivered(D2,P6,3) <= 30
 packageWasDeliveredAtRightPositionY1(P6,D2,4): + droneDY(D2,4)
 + 30 droneDelivered(D2,P6,4) <= 30
 packageWasDeliveredAtRightPositionY1(P6,D2,5): + droneDY(D2,5)
 + 30 droneDelivered(D2,P6,5) <= 30
 packageWasDeliveredAtRightPositionY1(P6,D3,1): + droneDY(D3,1)
 + 30 droneDelivered(D3,P6,1) <= 30
 packageWasDeliveredAtRightPositionY1(P6,D3,2): + droneDY(D3,2)
 + 30 droneDelivered(D3,P6,2) <= 30
 packageWasDeliveredAtRightPositionY1(P6,D3,3): + droneDY(D3,3)
 + 30 droneDelivered(D3,P6,3) <= 30
 packageWasDeliveredAtRightPositionY1(P6,D3,4): + droneDY(D3,4)
 + 30 droneDelivered(D3,P6,4) <= 30
 packageWasDeliveredAtRightPositionY1(P6,D3,5): + droneDY(D3,5)
 + 30 droneDelivered(D3,P6,5) <= 30
 packageWasDeliveredAtRightPositionY2(P1,D1,1): - droneDY(D1,1)
 + 30 droneDelivered(D1,P1,1) <= 30
 packageWasDeliveredAtRightPositionY2(P1,D1,2): - droneDY(D1,2)
 + 30 droneDelivered(D1,P1,2) <= 30
 packageWasDeliveredAtRightPositionY2(P1,D1,3): - droneDY(D1,3)
 + 30 droneDelivered(D1,P1,3) <= 30
 packageWasDeliveredAtRightPositionY2(P1,D1,4): - droneDY(D1,4)
 + 30 droneDelivered(D1,P1,4) <= 30
 packageWasDeliveredAtRightPositionY2(P1,D1,5): - droneDY(D1,5)
 + 30 droneDelivered(D1,P1,5) <= 30
 packageWasDeliveredAtRightPositionY2(P1,D2,1): - droneDY(D2,1)
 + 30 droneDelivered(D2,P1,1) <= 30
 packageWasDeliveredAtRightPositionY2(P1,D2,2): - droneDY(D2,2)
 + 30 droneDelivered(D2,P1,2) <= 30
 packageWasDeliveredAtRightPositionY2(P1,D2,3): - droneDY(D2,3)
 + 30 droneDelivered(D2,P1,3) <= 30
 packageWasDeliveredAtRightPositionY2(P1,D2,4): - droneDY(D2,4)
 + 30 droneDelivered(D2,P1,4) <= 30
 packageWasDeliveredAtRightPositionY2(P1,D2,5): - droneDY(D2,5)
 + 30 droneDelivered(D2,P1,5) <= 30
 packageWasDeliveredAtRightPositionY2(P1,D3,1): - droneDY(D3,1)
 + 30 droneDelivered(D3,P1,1) <= 30
 packageWasDeliveredAtRightPositionY2(P1,D3,2): - droneDY(D3,2)
 + 30 droneDelivered(D3,P1,2) <= 30
 packageWasDeliveredAtRightPositionY2(P1,D3,3): - droneDY(D3,3)
 + 30 droneDelivered(D3,P1,3) <= 30
 packageWasDeliveredAtRightPositionY2(P1,D3,4): - droneDY(D3,4)
 + 30 droneDelivered(D3,P1,4) <= 30
 packageWasDeliveredAtRightPositionY2(P1,D3,5): - droneDY(D3,5)
 + 30 droneDelivered(D3,P1,5) <= 30
 packageWasDeliveredAtRightPositionY2(P2,D1,1): - droneDY(D1,1)
 + 30 droneDelivered(D1,P2,1) <= 30
 packageWasDeliveredAtRightPositionY2(P2,D1,2): - droneDY(D1,2)
 + 30 droneDelivered(D1,P2,2) <= 30
 packageWasDeliveredAtRightPositionY2(P2,D1,3): - droneDY(D1,3)
 + 30 droneDelivered(D1,P2,3) <= 30
 packageWasDeliveredAtRightPositionY2(P2,D1,4): - droneDY(D1,4)
 + 30 droneDelivered(D1,P2,4) <= 30
 packageWasDeliveredAtRightPositionY2(P2,D1,5): - droneDY(D1,5)
 + 30 droneDelivered(D1,P2,5) <= 30
 packageWasDeliveredAtRightPositionY2(P2,D2,1): - droneDY(D2,1)
 + 30 droneDelivered(D2,P2,1) <= 30
 packageWasDeliveredAtRightPositionY2(P2,D2,2): - droneDY(D2,2)
 + 30 droneDelivered(D2,P2,2) <= 30
 packageWasDeliveredAtRightPositionY2(P2,D2,3): - droneDY(D2,3)
 + 30 droneDelivered(D2,P2,3) <= 30
 packageWasDeliveredAtRightPositionY2(P2,D2,4): - droneDY(D2,4)
 + 30 droneDelivered(D2,P2,4) <= 30
 packageWasDeliveredAtRightPositionY2(P2,D2,5): - droneDY(D2,5)
 + 30 droneDelivered(D2,P2,5) <= 30
 packageWasDeliveredAtRightPositionY2(P2,D3,1): - droneDY(D3,1)
 + 30 droneDelivered(D3,P2,1) <= 30
 packageWasDeliveredAtRightPositionY2(P2,D3,2): - droneDY(D3,2)
 + 30 droneDelivered(D3,P2,2) <= 30
 packageWasDeliveredAtRightPositionY2(P2,D3,3): - droneDY(D3,3)
 + 30 droneDelivered(D3,P2,3) <= 30
 packageWasDeliveredAtRightPositionY2(P2,D3,4): - droneDY(D3,4)
 + 30 droneDelivered(D3,P2,4) <= 30
 packageWasDeliveredAtRightPositionY2(P2,D3,5): - droneDY(D3,5)
 + 30 droneDelivered(D3,P2,5) <= 30
 packageWasDeliveredAtRightPositionY2(P3,D1,1): - droneDY(D1,1)
 + 30 droneDelivered(D1,P3,1) <= 25
 packageWasDeliveredAtRightPositionY2(P3,D1,2): - droneDY(D1,2)
 + 30 droneDelivered(D1,P3,2) <= 25
 packageWasDeliveredAtRightPositionY2(P3,D1,3): - droneDY(D1,3)
 + 30 droneDelivered(D1,P3,3) <= 25
 packageWasDeliveredAtRightPositionY2(P3,D1,4): - droneDY(D1,4)
 + 30 droneDelivered(D1,P3,4) <= 25
 packageWasDeliveredAtRightPositionY2(P3,D1,5): - droneDY(D1,5)
 + 30 droneDelivered(D1,P3,5) <= 25
 packageWasDeliveredAtRightPositionY2(P3,D2,1): - droneDY(D2,1)
 + 30 droneDelivered(D2,P3,1) <= 25
 packageWasDeliveredAtRightPositionY2(P3,D2,2): - droneDY(D2,2)
 + 30 droneDelivered(D2,P3,2) <= 25
 packageWasDeliveredAtRightPositionY2(P3,D2,3): - droneDY(D2,3)
 + 30 droneDelivered(D2,P3,3) <= 25
 packageWasDeliveredAtRightPositionY2(P3,D2,4): - droneDY(D2,4)
 + 30 droneDelivered(D2,P3,4) <= 25
 packageWasDeliveredAtRightPositionY2(P3,D2,5): - droneDY(D2,5)
 + 30 droneDelivered(D2,P3,5) <= 25
 packageWasDeliveredAtRightPositionY2(P3,D3,1): - droneDY(D3,1)
 + 30 droneDelivered(D3,P3,1) <= 25
 packageWasDeliveredAtRightPositionY2(P3,D3,2): - droneDY(D3,2)
 + 30 droneDelivered(D3,P3,2) <= 25
 packageWasDeliveredAtRightPositionY2(P3,D3,3): - droneDY(D3,3)
 + 30 droneDelivered(D3,P3,3) <= 25
 packageWasDeliveredAtRightPositionY2(P3,D3,4): - droneDY(D3,4)
 + 30 droneDelivered(D3,P3,4) <= 25
 packageWasDeliveredAtRightPositionY2(P3,D3,5): - droneDY(D3,5)
 + 30 droneDelivered(D3,P3,5) <= 25
 packageWasDeliveredAtRightPositionY2(P4,D1,1): - droneDY(D1,1)
 + 30 droneDelivered(D1,P4,1) <= 20
 packageWasDeliveredAtRightPositionY2(P4,D1,2): - droneDY(D1,2)
 + 30 droneDelivered(D1,P4,2) <= 20
 packageWasDeliveredAtRightPositionY2(P4,D1,3): - droneDY(D1,3)
 + 30 droneDelivered(D1,P4,3) <= 20
 packageWasDeliveredAtRightPositionY2(P4,D1,4): - droneDY(D1,4)
 + 30 droneDelivered(D1,P4,4) <= 20
 packageWasDeliveredAtRightPositionY2(P4,D1,5): - droneDY(D1,5)
 + 30 droneDelivered(D1,P4,5) <= 20
 packageWasDeliveredAtRightPositionY2(P4,D2,1): - droneDY(D2,1)
 + 30 droneDelivered(D2,P4,1) <= 20
 packageWasDeliveredAtRightPositionY2(P4,D2,2): - droneDY(D2,2)
 + 30 droneDelivered(D2,P4,2) <= 20
 packageWasDeliveredAtRightPositionY2(P4,D2,3): - droneDY(D2,3)
 + 30 droneDelivered(D2,P4,3) <= 20
 packageWasDeliveredAtRightPositionY2(P4,D2,4): - droneDY(D2,4)
 + 30 droneDelivered(D2,P4,4) <= 20
 packageWasDeliveredAtRightPositionY2(P4,D2,5): - droneDY(D2,5)
 + 30 droneDelivered(D2,P4,5) <= 20
 packageWasDeliveredAtRightPositionY2(P4,D3,1): - droneDY(D3,1)
 + 30 droneDelivered(D3,P4,1) <= 20
 packageWasDeliveredAtRightPositionY2(P4,D3,2): - droneDY(D3,2)
 + 30 droneDelivered(D3,P4,2) <= 20
 packageWasDeliveredAtRightPositionY2(P4,D3,3): - droneDY(D3,3)
 + 30 droneDelivered(D3,P4,3) <= 20
 packageWasDeliveredAtRightPositionY2(P4,D3,4): - droneDY(D3,4)
 + 30 droneDelivered(D3,P4,4) <= 20
 packageWasDeliveredAtRightPositionY2(P4,D3,5): - droneDY(D3,5)
 + 30 droneDelivered(D3,P4,5) <= 20
 packageWasDeliveredAtRightPositionY2(P5,D1,1): - droneDY(D1,1)
 + 30 droneDelivered(D1,P5,1) <= 25
 packageWasDeliveredAtRightPositionY2(P5,D1,2): - droneDY(D1,2)
 + 30 droneDelivered(D1,P5,2) <= 25
 packageWasDeliveredAtRightPositionY2(P5,D1,3): - droneDY(D1,3)
 + 30 droneDelivered(D1,P5,3) <= 25
 packageWasDeliveredAtRightPositionY2(P5,D1,4): - droneDY(D1,4)
 + 30 droneDelivered(D1,P5,4) <= 25
 packageWasDeliveredAtRightPositionY2(P5,D1,5): - droneDY(D1,5)
 + 30 droneDelivered(D1,P5,5) <= 25
 packageWasDeliveredAtRightPositionY2(P5,D2,1): - droneDY(D2,1)
 + 30 droneDelivered(D2,P5,1) <= 25
 packageWasDeliveredAtRightPositionY2(P5,D2,2): - droneDY(D2,2)
 + 30 droneDelivered(D2,P5,2) <= 25
 packageWasDeliveredAtRightPositionY2(P5,D2,3): - droneDY(D2,3)
 + 30 droneDelivered(D2,P5,3) <= 25
 packageWasDeliveredAtRightPositionY2(P5,D2,4): - droneDY(D2,4)
 + 30 droneDelivered(D2,P5,4) <= 25
 packageWasDeliveredAtRightPositionY2(P5,D2,5): - droneDY(D2,5)
 + 30 droneDelivered(D2,P5,5) <= 25
 packageWasDeliveredAtRightPositionY2(P5,D3,1): - droneDY(D3,1)
 + 30 droneDelivered(D3,P5,1) <= 25
 packageWasDeliveredAtRightPositionY2(P5,D3,2): - droneDY(D3,2)
 + 30 droneDelivered(D3,P5,2) <= 25
 packageWasDeliveredAtRightPositionY2(P5,D3,3): - droneDY(D3,3)
 + 30 droneDelivered(D3,P5,3) <= 25
 packageWasDeliveredAtRightPositionY2(P5,D3,4): - droneDY(D3,4)
 + 30 droneDelivered(D3,P5,4) <= 25
 packageWasDeliveredAtRightPositionY2(P5,D3,5): - droneDY(D3,5)
 + 30 droneDelivered(D3,P5,5) <= 25
 packageWasDeliveredAtRightPositionY2(P6,D1,1): - droneDY(D1,1)
 + 30 droneDelivered(D1,P6,1) <= 30
 packageWasDeliveredAtRightPositionY2(P6,D1,2): - droneDY(D1,2)
 + 30 droneDelivered(D1,P6,2) <= 30
 packageWasDeliveredAtRightPositionY2(P6,D1,3): - droneDY(D1,3)
 + 30 droneDelivered(D1,P6,3) <= 30
 packageWasDeliveredAtRightPositionY2(P6,D1,4): - droneDY(D1,4)
 + 30 droneDelivered(D1,P6,4) <= 30
 packageWasDeliveredAtRightPositionY2(P6,D1,5): - droneDY(D1,5)
 + 30 droneDelivered(D1,P6,5) <= 30
 packageWasDeliveredAtRightPositionY2(P6,D2,1): - droneDY(D2,1)
 + 30 droneDelivered(D2,P6,1) <= 30
 packageWasDeliveredAtRightPositionY2(P6,D2,2): - droneDY(D2,2)
 + 30 droneDelivered(D2,P6,2) <= 30
 packageWasDeliveredAtRightPositionY2(P6,D2,3): - droneDY(D2,3)
 + 30 droneDelivered(D2,P6,3) <= 30
 packageWasDeliveredAtRightPositionY2(P6,D2,4): - droneDY(D2,4)
 + 30 droneDelivered(D2,P6,4) <= 30
 packageWasDeliveredAtRightPositionY2(P6,D2,5): - droneDY(D2,5)
 + 30 droneDelivered(D2,P6,5) <= 30
 packageWasDeliveredAtRightPositionY2(P6,D3,1): - droneDY(D3,1)
 + 30 droneDelivered(D3,P6,1) <= 30
 packageWasDeliveredAtRightPositionY2(P6,D3,2): - droneDY(D3,2)
 + 30 droneDelivered(D3,P6,2) <= 30
 packageWasDeliveredAtRightPositionY2(P6,D3,3): - droneDY(D3,3)
 + 30 droneDelivered(D3,P6,3) <= 30
 packageWasDeliveredAtRightPositionY2(P6,D3,4): - droneDY(D3,4)
 + 30 droneDelivered(D3,P6,4) <= 30
 packageWasDeliveredAtRightPositionY2(P6,D3,5): - droneDY(D3,5)
 + 30 droneDelivered(D3,P6,5) <= 30
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
 calcDronesSpeed(D1,2): + droneSpeed(D1,2) - 2 diffPosX(D1,2)
 - diffPosY(D1,2) - diffNegY(D1,2) = -0
 calcDronesSpeed(D1,3): + droneSpeed(D1,3) - 2 diffPosX(D1,3)
 - diffPosY(D1,3) - diffNegY(D1,3) = -0
 calcDronesSpeed(D1,4): + droneSpeed(D1,4) - 2 diffPosX(D1,4)
 - diffPosY(D1,4) - diffNegY(D1,4) = -0
 calcDronesSpeed(D1,5): + droneSpeed(D1,5) - 2 diffPosX(D1,5)
 - diffPosY(D1,5) - diffNegY(D1,5) = -0
 calcDronesSpeed(D2,2): + droneSpeed(D2,2) - 2 diffPosX(D2,2)
 - diffPosY(D2,2) - diffNegY(D2,2) = -0
 calcDronesSpeed(D2,3): + droneSpeed(D2,3) - 2 diffPosX(D2,3)
 - diffPosY(D2,3) - diffNegY(D2,3) = -0
 calcDronesSpeed(D2,4): + droneSpeed(D2,4) - 2 diffPosX(D2,4)
 - diffPosY(D2,4) - diffNegY(D2,4) = -0
 calcDronesSpeed(D2,5): + droneSpeed(D2,5) - 2 diffPosX(D2,5)
 - diffPosY(D2,5) - diffNegY(D2,5) = -0
 calcDronesSpeed(D3,2): + droneSpeed(D3,2) - 2 diffPosX(D3,2)
 - diffPosY(D3,2) - diffNegY(D3,2) = -0
 calcDronesSpeed(D3,3): + droneSpeed(D3,3) - 2 diffPosX(D3,3)
 - diffPosY(D3,3) - diffNegY(D3,3) = -0
 calcDronesSpeed(D3,4): + droneSpeed(D3,4) - 2 diffPosX(D3,4)
 - diffPosY(D3,4) - diffNegY(D3,4) = -0
 calcDronesSpeed(D3,5): + droneSpeed(D3,5) - 2 diffPosX(D3,5)
 - diffPosY(D3,5) - diffNegY(D3,5) = -0
 respectDroneMaxSpeed(D1,2): + droneSpeed(D1,2) <= 10
 respectDroneMaxSpeed(D1,3): + droneSpeed(D1,3) <= 10
 respectDroneMaxSpeed(D1,4): + droneSpeed(D1,4) <= 10
 respectDroneMaxSpeed(D1,5): + droneSpeed(D1,5) <= 10
 respectDroneMaxSpeed(D2,2): + droneSpeed(D2,2) <= 20
 respectDroneMaxSpeed(D2,3): + droneSpeed(D2,3) <= 20
 respectDroneMaxSpeed(D2,4): + droneSpeed(D2,4) <= 20
 respectDroneMaxSpeed(D2,5): + droneSpeed(D2,5) <= 20
 respectDroneMaxSpeed(D3,2): + droneSpeed(D3,2) <= 50
 respectDroneMaxSpeed(D3,3): + droneSpeed(D3,3) <= 50
 respectDroneMaxSpeed(D3,4): + droneSpeed(D3,4) <= 50
 respectDroneMaxSpeed(D3,5): + droneSpeed(D3,5) <= 50
 calcTotalDist: - 10 diffPosX(D1,2) - 10 diffPosX(D1,3)
 - 10 diffPosX(D1,4) - 10 diffPosX(D1,5) - 10 diffPosX(D2,2)
 - 10 diffPosX(D2,3) - 10 diffPosX(D2,4) - 10 diffPosX(D2,5)
 - 10 diffPosX(D3,2) - 10 diffPosX(D3,3) - 10 diffPosX(D3,4)
 - 10 diffPosX(D3,5) - 10 diffNegX(D1,2) - 10 diffNegX(D1,3)
 - 10 diffNegX(D1,4) - 10 diffNegX(D1,5) - 10 diffNegX(D2,2)
 - 10 diffNegX(D2,3) - 10 diffNegX(D2,4) - 10 diffNegX(D2,5)
 - 10 diffNegX(D3,2) - 10 diffNegX(D3,3) - 10 diffNegX(D3,4)
 - 10 diffNegX(D3,5) - 10 diffPosY(D1,2) - 10 diffPosY(D1,3)
 - 10 diffPosY(D1,4) - 10 diffPosY(D1,5) - 10 diffPosY(D2,2)
 - 10 diffPosY(D2,3) - 10 diffPosY(D2,4) - 10 diffPosY(D2,5)
 - 10 diffPosY(D3,2) - 10 diffPosY(D3,3) - 10 diffPosY(D3,4)
 - 10 diffPosY(D3,5) - 10 diffNegY(D1,2) - 10 diffNegY(D1,3)
 - 10 diffNegY(D1,4) - 10 diffNegY(D1,5) - 10 diffNegY(D2,2)
 - 10 diffNegY(D2,3) - 10 diffNegY(D2,4) - 10 diffNegY(D2,5)
 - 10 diffNegY(D3,2) - 10 diffNegY(D3,3) - 10 diffNegY(D3,4)
 - 10 diffNegY(D3,5) + totalDist = -0
 calcTotalTimePerDeliver: - timePerProductDelivered(P1)
 - timePerProductDelivered(P2) - timePerProductDelivered(P3)
 - timePerProductDelivered(P4) - timePerProductDelivered(P5)
 - timePerProductDelivered(P6) + timePerDeliver = -0
 calcNDrones: - droneIsUsed(D1) - droneIsUsed(D2) - droneIsUsed(D3)
 + nUsedDrones = -0
 calcDronesAVGSpeed: - droneSpeed(D1,2) - droneSpeed(D1,3)
 - droneSpeed(D1,4) - droneSpeed(D1,5) - droneSpeed(D2,2)
 - droneSpeed(D2,3) - droneSpeed(D2,4) - droneSpeed(D2,5)
 - droneSpeed(D3,2) - droneSpeed(D3,3) - droneSpeed(D3,4)
 - droneSpeed(D3,5) + dronesMaxSpeed >= -0

Bounds
 0 <= droneDX(D1,1) <= 30
 0 <= droneDX(D2,1) <= 30
 0 <= droneDX(D3,1) <= 30
 0 <= droneDX(D1,2) <= 30
 0 <= droneDX(D1,3) <= 30
 0 <= droneDX(D1,4) <= 30
 0 <= droneDX(D1,5) <= 30
 0 <= droneDX(D2,2) <= 30
 0 <= droneDX(D2,3) <= 30
 0 <= droneDX(D2,4) <= 30
 0 <= droneDX(D2,5) <= 30
 0 <= droneDX(D3,2) <= 30
 0 <= droneDX(D3,3) <= 30
 0 <= droneDX(D3,4) <= 30
 0 <= droneDX(D3,5) <= 30
 0 <= droneDY(D1,1) <= 30
 0 <= droneDY(D2,1) <= 30
 0 <= droneDY(D3,1) <= 30
 0 <= droneDY(D1,2) <= 30
 0 <= droneDY(D1,3) <= 30
 0 <= droneDY(D1,4) <= 30
 0 <= droneDY(D1,5) <= 30
 0 <= droneDY(D2,2) <= 30
 0 <= droneDY(D2,3) <= 30
 0 <= droneDY(D2,4) <= 30
 0 <= droneDY(D2,5) <= 30
 0 <= droneDY(D3,2) <= 30
 0 <= droneDY(D3,3) <= 30
 0 <= droneDY(D3,4) <= 30
 0 <= droneDY(D3,5) <= 30
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
 0 <= droneCollected(D1,P1,1) <= 1
 0 <= droneCollected(D1,P2,1) <= 1
 0 <= droneCollected(D1,P3,1) <= 1
 0 <= droneCollected(D1,P4,1) <= 1
 0 <= droneCollected(D1,P5,1) <= 1
 0 <= droneCollected(D1,P6,1) <= 1
 0 <= droneCollected(D2,P1,1) <= 1
 0 <= droneCollected(D2,P2,1) <= 1
 0 <= droneCollected(D2,P3,1) <= 1
 0 <= droneCollected(D2,P4,1) <= 1
 0 <= droneCollected(D2,P5,1) <= 1
 0 <= droneCollected(D2,P6,1) <= 1
 0 <= droneCollected(D3,P1,1) <= 1
 0 <= droneCollected(D3,P2,1) <= 1
 0 <= droneCollected(D3,P3,1) <= 1
 0 <= droneCollected(D3,P4,1) <= 1
 0 <= droneCollected(D3,P5,1) <= 1
 0 <= droneCollected(D3,P6,1) <= 1
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
 0 <= droneDelivered(D1,P1,1) <= 1
 0 <= droneDelivered(D1,P2,1) <= 1
 0 <= droneDelivered(D1,P3,1) <= 1
 0 <= droneDelivered(D1,P4,1) <= 1
 0 <= droneDelivered(D1,P5,1) <= 1
 0 <= droneDelivered(D1,P6,1) <= 1
 0 <= droneDelivered(D2,P1,1) <= 1
 0 <= droneDelivered(D2,P2,1) <= 1
 0 <= droneDelivered(D2,P3,1) <= 1
 0 <= droneDelivered(D2,P4,1) <= 1
 0 <= droneDelivered(D2,P5,1) <= 1
 0 <= droneDelivered(D2,P6,1) <= 1
 0 <= droneDelivered(D3,P1,1) <= 1
 0 <= droneDelivered(D3,P2,1) <= 1
 0 <= droneDelivered(D3,P3,1) <= 1
 0 <= droneDelivered(D3,P4,1) <= 1
 0 <= droneDelivered(D3,P5,1) <= 1
 0 <= droneDelivered(D3,P6,1) <= 1
 0 <= droneIsDelivering(D1,P1,1) <= 1
 0 <= droneIsDelivering(D1,P1,2) <= 1
 0 <= droneIsDelivering(D1,P1,3) <= 1
 0 <= droneIsDelivering(D1,P1,4) <= 1
 0 <= droneIsDelivering(D2,P1,1) <= 1
 0 <= droneIsDelivering(D2,P1,2) <= 1
 0 <= droneIsDelivering(D2,P1,3) <= 1
 0 <= droneIsDelivering(D2,P1,4) <= 1
 0 <= droneIsDelivering(D3,P1,1) <= 1
 0 <= droneIsDelivering(D3,P1,2) <= 1
 0 <= droneIsDelivering(D3,P1,3) <= 1
 0 <= droneIsDelivering(D3,P1,4) <= 1
 0 <= droneIsDelivering(D1,P2,1) <= 1
 0 <= droneIsDelivering(D1,P2,2) <= 1
 0 <= droneIsDelivering(D1,P2,3) <= 1
 0 <= droneIsDelivering(D1,P2,4) <= 1
 0 <= droneIsDelivering(D2,P2,1) <= 1
 0 <= droneIsDelivering(D2,P2,2) <= 1
 0 <= droneIsDelivering(D2,P2,3) <= 1
 0 <= droneIsDelivering(D2,P2,4) <= 1
 0 <= droneIsDelivering(D3,P2,1) <= 1
 0 <= droneIsDelivering(D3,P2,2) <= 1
 0 <= droneIsDelivering(D3,P2,3) <= 1
 0 <= droneIsDelivering(D3,P2,4) <= 1
 0 <= droneIsDelivering(D1,P3,1) <= 1
 0 <= droneIsDelivering(D1,P3,2) <= 1
 0 <= droneIsDelivering(D1,P3,3) <= 1
 0 <= droneIsDelivering(D1,P3,4) <= 1
 0 <= droneIsDelivering(D2,P3,1) <= 1
 0 <= droneIsDelivering(D2,P3,2) <= 1
 0 <= droneIsDelivering(D2,P3,3) <= 1
 0 <= droneIsDelivering(D2,P3,4) <= 1
 0 <= droneIsDelivering(D3,P3,1) <= 1
 0 <= droneIsDelivering(D3,P3,2) <= 1
 0 <= droneIsDelivering(D3,P3,3) <= 1
 0 <= droneIsDelivering(D3,P3,4) <= 1
 0 <= droneIsDelivering(D1,P4,1) <= 1
 0 <= droneIsDelivering(D1,P4,2) <= 1
 0 <= droneIsDelivering(D1,P4,3) <= 1
 0 <= droneIsDelivering(D1,P4,4) <= 1
 0 <= droneIsDelivering(D2,P4,1) <= 1
 0 <= droneIsDelivering(D2,P4,2) <= 1
 0 <= droneIsDelivering(D2,P4,3) <= 1
 0 <= droneIsDelivering(D2,P4,4) <= 1
 0 <= droneIsDelivering(D3,P4,1) <= 1
 0 <= droneIsDelivering(D3,P4,2) <= 1
 0 <= droneIsDelivering(D3,P4,3) <= 1
 0 <= droneIsDelivering(D3,P4,4) <= 1
 0 <= droneIsDelivering(D1,P5,1) <= 1
 0 <= droneIsDelivering(D1,P5,2) <= 1
 0 <= droneIsDelivering(D1,P5,3) <= 1
 0 <= droneIsDelivering(D1,P5,4) <= 1
 0 <= droneIsDelivering(D2,P5,1) <= 1
 0 <= droneIsDelivering(D2,P5,2) <= 1
 0 <= droneIsDelivering(D2,P5,3) <= 1
 0 <= droneIsDelivering(D2,P5,4) <= 1
 0 <= droneIsDelivering(D3,P5,1) <= 1
 0 <= droneIsDelivering(D3,P5,2) <= 1
 0 <= droneIsDelivering(D3,P5,3) <= 1
 0 <= droneIsDelivering(D3,P5,4) <= 1
 0 <= droneIsDelivering(D1,P6,1) <= 1
 0 <= droneIsDelivering(D1,P6,2) <= 1
 0 <= droneIsDelivering(D1,P6,3) <= 1
 0 <= droneIsDelivering(D1,P6,4) <= 1
 0 <= droneIsDelivering(D2,P6,1) <= 1
 0 <= droneIsDelivering(D2,P6,2) <= 1
 0 <= droneIsDelivering(D2,P6,3) <= 1
 0 <= droneIsDelivering(D2,P6,4) <= 1
 0 <= droneIsDelivering(D3,P6,1) <= 1
 0 <= droneIsDelivering(D3,P6,2) <= 1
 0 <= droneIsDelivering(D3,P6,3) <= 1
 0 <= droneIsDelivering(D3,P6,4) <= 1
 0 <= droneIsDelivering(D1,P1,5) <= 1
 0 <= droneIsDelivering(D2,P1,5) <= 1
 0 <= droneIsDelivering(D3,P1,5) <= 1
 0 <= droneIsDelivering(D1,P2,5) <= 1
 0 <= droneIsDelivering(D2,P2,5) <= 1
 0 <= droneIsDelivering(D3,P2,5) <= 1
 0 <= droneIsDelivering(D1,P3,5) <= 1
 0 <= droneIsDelivering(D2,P3,5) <= 1
 0 <= droneIsDelivering(D3,P3,5) <= 1
 0 <= droneIsDelivering(D1,P4,5) <= 1
 0 <= droneIsDelivering(D2,P4,5) <= 1
 0 <= droneIsDelivering(D3,P4,5) <= 1
 0 <= droneIsDelivering(D1,P5,5) <= 1
 0 <= droneIsDelivering(D2,P5,5) <= 1
 0 <= droneIsDelivering(D3,P5,5) <= 1
 0 <= droneIsDelivering(D1,P6,5) <= 1
 0 <= droneIsDelivering(D2,P6,5) <= 1
 0 <= droneIsDelivering(D3,P6,5) <= 1
 0 <= droneIsUsed(D1) <= 1
 0 <= droneIsUsed(D2) <= 1
 0 <= droneIsUsed(D3) <= 1

Generals
 droneDX(D1,1)
 droneDX(D2,1)
 droneDX(D3,1)
 droneDX(D1,2)
 droneDX(D1,3)
 droneDX(D1,4)
 droneDX(D1,5)
 droneDX(D2,2)
 droneDX(D2,3)
 droneDX(D2,4)
 droneDX(D2,5)
 droneDX(D3,2)
 droneDX(D3,3)
 droneDX(D3,4)
 droneDX(D3,5)
 droneDY(D1,1)
 droneDY(D2,1)
 droneDY(D3,1)
 droneDY(D1,2)
 droneDY(D1,3)
 droneDY(D1,4)
 droneDY(D1,5)
 droneDY(D2,2)
 droneDY(D2,3)
 droneDY(D2,4)
 droneDY(D2,5)
 droneDY(D3,2)
 droneDY(D3,3)
 droneDY(D3,4)
 droneDY(D3,5)
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
 droneCollected(D1,P1,1)
 droneCollected(D1,P2,1)
 droneCollected(D1,P3,1)
 droneCollected(D1,P4,1)
 droneCollected(D1,P5,1)
 droneCollected(D1,P6,1)
 droneCollected(D2,P1,1)
 droneCollected(D2,P2,1)
 droneCollected(D2,P3,1)
 droneCollected(D2,P4,1)
 droneCollected(D2,P5,1)
 droneCollected(D2,P6,1)
 droneCollected(D3,P1,1)
 droneCollected(D3,P2,1)
 droneCollected(D3,P3,1)
 droneCollected(D3,P4,1)
 droneCollected(D3,P5,1)
 droneCollected(D3,P6,1)
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
 droneDelivered(D1,P1,1)
 droneDelivered(D1,P2,1)
 droneDelivered(D1,P3,1)
 droneDelivered(D1,P4,1)
 droneDelivered(D1,P5,1)
 droneDelivered(D1,P6,1)
 droneDelivered(D2,P1,1)
 droneDelivered(D2,P2,1)
 droneDelivered(D2,P3,1)
 droneDelivered(D2,P4,1)
 droneDelivered(D2,P5,1)
 droneDelivered(D2,P6,1)
 droneDelivered(D3,P1,1)
 droneDelivered(D3,P2,1)
 droneDelivered(D3,P3,1)
 droneDelivered(D3,P4,1)
 droneDelivered(D3,P5,1)
 droneDelivered(D3,P6,1)
 droneIsDelivering(D1,P1,1)
 droneIsDelivering(D1,P1,2)
 droneIsDelivering(D1,P1,3)
 droneIsDelivering(D1,P1,4)
 droneIsDelivering(D2,P1,1)
 droneIsDelivering(D2,P1,2)
 droneIsDelivering(D2,P1,3)
 droneIsDelivering(D2,P1,4)
 droneIsDelivering(D3,P1,1)
 droneIsDelivering(D3,P1,2)
 droneIsDelivering(D3,P1,3)
 droneIsDelivering(D3,P1,4)
 droneIsDelivering(D1,P2,1)
 droneIsDelivering(D1,P2,2)
 droneIsDelivering(D1,P2,3)
 droneIsDelivering(D1,P2,4)
 droneIsDelivering(D2,P2,1)
 droneIsDelivering(D2,P2,2)
 droneIsDelivering(D2,P2,3)
 droneIsDelivering(D2,P2,4)
 droneIsDelivering(D3,P2,1)
 droneIsDelivering(D3,P2,2)
 droneIsDelivering(D3,P2,3)
 droneIsDelivering(D3,P2,4)
 droneIsDelivering(D1,P3,1)
 droneIsDelivering(D1,P3,2)
 droneIsDelivering(D1,P3,3)
 droneIsDelivering(D1,P3,4)
 droneIsDelivering(D2,P3,1)
 droneIsDelivering(D2,P3,2)
 droneIsDelivering(D2,P3,3)
 droneIsDelivering(D2,P3,4)
 droneIsDelivering(D3,P3,1)
 droneIsDelivering(D3,P3,2)
 droneIsDelivering(D3,P3,3)
 droneIsDelivering(D3,P3,4)
 droneIsDelivering(D1,P4,1)
 droneIsDelivering(D1,P4,2)
 droneIsDelivering(D1,P4,3)
 droneIsDelivering(D1,P4,4)
 droneIsDelivering(D2,P4,1)
 droneIsDelivering(D2,P4,2)
 droneIsDelivering(D2,P4,3)
 droneIsDelivering(D2,P4,4)
 droneIsDelivering(D3,P4,1)
 droneIsDelivering(D3,P4,2)
 droneIsDelivering(D3,P4,3)
 droneIsDelivering(D3,P4,4)
 droneIsDelivering(D1,P5,1)
 droneIsDelivering(D1,P5,2)
 droneIsDelivering(D1,P5,3)
 droneIsDelivering(D1,P5,4)
 droneIsDelivering(D2,P5,1)
 droneIsDelivering(D2,P5,2)
 droneIsDelivering(D2,P5,3)
 droneIsDelivering(D2,P5,4)
 droneIsDelivering(D3,P5,1)
 droneIsDelivering(D3,P5,2)
 droneIsDelivering(D3,P5,3)
 droneIsDelivering(D3,P5,4)
 droneIsDelivering(D1,P6,1)
 droneIsDelivering(D1,P6,2)
 droneIsDelivering(D1,P6,3)
 droneIsDelivering(D1,P6,4)
 droneIsDelivering(D2,P6,1)
 droneIsDelivering(D2,P6,2)
 droneIsDelivering(D2,P6,3)
 droneIsDelivering(D2,P6,4)
 droneIsDelivering(D3,P6,1)
 droneIsDelivering(D3,P6,2)
 droneIsDelivering(D3,P6,3)
 droneIsDelivering(D3,P6,4)
 droneIsDelivering(D1,P1,5)
 droneIsDelivering(D2,P1,5)
 droneIsDelivering(D3,P1,5)
 droneIsDelivering(D1,P2,5)
 droneIsDelivering(D2,P2,5)
 droneIsDelivering(D3,P2,5)
 droneIsDelivering(D1,P3,5)
 droneIsDelivering(D2,P3,5)
 droneIsDelivering(D3,P3,5)
 droneIsDelivering(D1,P4,5)
 droneIsDelivering(D2,P4,5)
 droneIsDelivering(D3,P4,5)
 droneIsDelivering(D1,P5,5)
 droneIsDelivering(D2,P5,5)
 droneIsDelivering(D3,P5,5)
 droneIsDelivering(D1,P6,5)
 droneIsDelivering(D2,P6,5)
 droneIsDelivering(D3,P6,5)
 droneIsUsed(D1)
 droneIsUsed(D2)
 droneIsUsed(D3)

End

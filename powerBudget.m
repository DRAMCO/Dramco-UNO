currentSleep = 8e-6
currentTransmit = 25.8e-3
currentProcessing = 13e-3

numberOfBatteries = 3
capacityAA = 2000
capacityBank = capacityAA*numberOfBatteries;

timeWake = 1022e-3+190e-3
timeTransmit = 1022e-3
timeSleep = 15*60
timeTotal = timeSleep + timeWake

dutySleep = timeSleep / timeTotal
dutyTransmit = timeTransmit / timeTotal
dutyWake = (timeWake - timeTransmit) / timeTotal

dutyCheck = dutySleep + dutyTransmit + dutyWake

avgCurrent = currentSleep*dutySleep + currentTransmit*dutyTransmit + currentProcessing*dutyWake

maPerHour = avgCurrent * 60 * 60

numberOfHours = capacityBank / maPerHour
numberOfYears = numberOfHours / (365*24) 


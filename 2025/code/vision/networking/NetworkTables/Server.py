import wpilib
from networktables import NetworkTables

NetworkTables.initialize()

table = NetworkTables.getTable("vision")

table.putNumber("x", 10)

print(table.getNumber("x", 0))
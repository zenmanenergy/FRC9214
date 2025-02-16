from networktables import NetworkTables


roborio_ip = "10.92.14.200"

NetworkTables.initialize(server=roborio_ip)

table = NetworkTables.getTable("vision")

while True:
    target_x = table.getNumber("target_x", 0)
    print(target_x)
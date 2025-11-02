from scapy.all import ARP, Ether, srp

# IP range to scan
ip_range = "192.168.1.1/24"

# Create ARP request
arp = ARP(pdst=ip_range)
ether = Ether(dst="ff:ff:ff:ff:ff:ff")
packet = ether/arp

# Send packet and receive responses
result = srp(packet, timeout=2, verbose=0)[0]

# List of Raspberry Pis
raspberry_pis = []

# Process each response
for sent, received in result:
	mac_prefix = received.hwsrc[:8].lower()
	if mac_prefix == "b8:27:eb" or mac_prefix == "dc:a6:32":
		raspberry_pis.append(received.psrc)
		print(f"Found Raspberry Pi at IP {received.psrc} with MAC {received.hwsrc}")

if not raspberry_pis:
	print("No Raspberry Pi devices found on the network.")

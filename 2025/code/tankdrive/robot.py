from navx import AHRS
import wpilib

class MyRobot(wpilib.TimedRobot):
	def robotInit(self):
		try:
			# Initialize the NavX sensor using SPI communication
			self.navx = AHRS.create_spi()
			if self.navx.isConnected():
				print("NavX Connected")
			else:
				print("NavX Connection Failed")
		except Exception as e:
			print(f"NavX Initialization Error: {str(e)}")
			self.navx = None

	def testPeriodic(self):
		if self.navx and self.navx.isConnected():
			# Retrieve the fused heading
			fused_heading = self.navx.getFusedHeading()

			# Check if the sensor is fully calibrated
			if self.navx.isMagnetometerCalibrated():
				calibration_status = "Calibrated"
			else:
				calibration_status = "NOT Calibrated"

			# Log the fused heading
			print(f"Fused Heading: {fused_heading:.2f} degrees (Calibration: {calibration_status})")
		else:
			print("NavX Disconnected")

if __name__ == "__main__":
	wpilib.run(MyRobot)

package team492.diagnostics.tests;

import frclib.FrcAHRSGyro;
import team492.diagnostics.DiagnosticsTest;

public class GyroNotConnectedTest extends DiagnosticsTest{
	
	private FrcAHRSGyro gyro;
	
	private boolean hasLostConnection = false;
	
	public GyroNotConnectedTest(String name, FrcAHRSGyro gyro) {
	    super(name);
		this.gyro = gyro;
	}

	@Override
	public void test() {
		if(!gyro.ahrs.isConnected()) {
			hasLostConnection = true;
		}
		
	}

	@Override
	public TestResult getResult() {
		return new TestResult(hasLostConnection, "gyro lost connection during the match");
	}
	

}
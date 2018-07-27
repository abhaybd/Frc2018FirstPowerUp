package team492;

import hallib.HalDashboard;
import trclib.TrcTankMotionProfile;
import frclib.FrcTankMotionProfileFollower;
import trclib.TrcPidController;
import trclib.TrcRobot;
import trclib.TrcUtil;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintStream;
import java.text.SimpleDateFormat;
import java.util.Date;

public class MotionProfileTest implements TrcRobot.RobotCommand
{
    private static final double kP = 1.194; //1.275;
    private static final double kI = 0.0;
    private static final double kD = 0.0899; //0.0956;
    private static final double kF = 0.8525; // TODO: Calculate this according to Phoenix docs

    private static final boolean WRITE_CSV = true;

    private String instanceName;
    private FrcTankMotionProfileFollower follower;
    private Robot robot;
    private PrintStream fileOut;
    private double startTime;

    public MotionProfileTest(String instanceName, Robot robot)
    {
        this.instanceName = instanceName;
        this.robot = robot;
        TrcPidController.PidCoefficients pidCoefficients = new TrcPidController.PidCoefficients(kP, kI, kD, kF);
        follower = new FrcTankMotionProfileFollower(instanceName + ".profileFollower", pidCoefficients,
            RobotInfo.ENCODER_Y_INCHES_PER_COUNT);
        follower.setLeftMotors(robot.leftFrontWheel, robot.leftRearWheel);
        follower.setRightMotors(robot.rightFrontWheel, robot.rightRearWheel);
        
        refreshData("Test/TargetPosLeft",0.0);
        refreshData("Test/ActualPosLeft",0.0);
        refreshData("Test/TargetVelLeft",0.0);
        refreshData("Test/ActualVelLeft",0.0);

        refreshData("Test/TargetPosRight",0.0);
        refreshData("Test/ActualPosRight",0.0);
        refreshData("Test/TargetVelRight",0.0);
        refreshData("Test/ActualVelRight",0.0);
    }
    
    private void refreshData(String name, double defaultValue)
	{
		HalDashboard.putNumber(name, HalDashboard.getNumber(name, defaultValue));		
	}

    public void start()
    {
        TrcTankMotionProfile profile = TrcTankMotionProfile
            .loadProfileFromCsv("/home/lvuser/loop_left_Jaci.csv", "/home/lvuser/loop_right_Jaci.csv");
        follower.start(profile);
        robot.globalTracer.traceInfo(instanceName + ".start", "Started following path!");

        if (WRITE_CSV)
        {
            try
            {
                startTime = TrcUtil.getCurrentTime();
                String timeStamp = new SimpleDateFormat("dd-MM-yy_HHmm").format(new Date());
                File dir = new File("/home/lvuser/MP_logs");
                if (dir.isDirectory() || dir.mkdir())
                {
                    fileOut = new PrintStream(new FileOutputStream(new File(dir, timeStamp + "_profilelog.csv")));
                    fileOut.println("Time," + "TargetPosLeft,ActualPosLeft,TargetVelLeft,ActualVelLeft,"
                        + "TargetPosRight,ActualPosRight,TargetVelRight,ActualVelRight");
                }
            }
            catch (IOException e)
            {
                robot.globalTracer.traceErr(instanceName + ".start", e.toString());
            }
        }
    }
    
    public void stop()
    {
    	follower.cancel();
    }

    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {
        boolean isActive = follower.isActive();

        double targetPosLeft = follower.leftTargetPosition();
        double actualPosLeft = follower.leftActualPosition();
        double targetVelLeft = follower.leftTargetVelocity();
        double actualVelLeft = follower.leftActualVelocity();

        double targetPosRight = follower.rightTargetPosition();
        double actualPosRight = follower.rightActualPosition();
        double targetVelRight = follower.rightTargetVelocity();
        double actualVelRight = follower.rightActualVelocity();

        HalDashboard.putNumber("Test/TargetPosLeft",targetPosLeft);
        HalDashboard.putNumber("Test/ActualPosLeft",actualPosLeft);
        HalDashboard.putNumber("Test/TargetVelLeft",targetVelLeft);
        HalDashboard.putNumber("Test/ActualVelLeft",actualVelLeft);

        HalDashboard.putNumber("Test/TargetPosRight",targetPosRight);
        HalDashboard.putNumber("Test/ActualPosRight",actualPosRight);
        HalDashboard.putNumber("Test/TargetVelRight",targetVelRight);
        HalDashboard.putNumber("Test/ActualVelRight",actualVelRight);

        String message = String.format(
            "MotionProfile: %s - Running: %b, Bottom Buffer: [%d,%d], Top Buffer: [%d,%d], Target Positions: [%.2f,%.2f], Target Velocities: [%.2f,%.2f]",
            follower.getInstanceName(), isActive, follower.leftBottomBufferCount(), follower.rightBottomBufferCount(),
            follower.leftTopBufferCount(), follower.rightTopBufferCount(), targetPosLeft, targetPosRight, targetVelLeft,
            targetVelRight);

        robot.dashboard.displayPrintf(1, message);
        robot.globalTracer.traceInfo(instanceName + ".cmdPeriodic", message);

        if (fileOut != null && isActive)
        {
            String line = String
                .format("%.2f," + "%.2f,%.2f,%.2f,%.2f," + "%.2f,%.2f,%.2f,%.2f", TrcUtil.getCurrentTime() - startTime,
                    targetPosLeft, actualPosLeft, targetVelLeft, actualVelLeft, targetPosRight, actualPosRight,
                    targetVelRight, actualVelRight);
            fileOut.println(line);
        }

        return !isActive;
    }
}
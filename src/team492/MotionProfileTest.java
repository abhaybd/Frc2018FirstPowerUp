package team492;

import trclib.TrcMotionProfile;
import frclib.FrcMotionProfileFollower;
import trclib.TrcPidController;
import trclib.TrcRobot;

public class MotionProfileTest implements TrcRobot.RobotCommand
{
    private static final double kP = 1.275;
    private static final double kI = 0.0;
    private static final double kD = 0.0956;
    private static final double kF = 0.8525; // TODO: Calculate this according to Phoenix docs

    private String instanceName;
    private TrcMotionProfile profile;
    private FrcMotionProfileFollower follower;
    private Robot robot;
    public MotionProfileTest(String instanceName, Robot robot)
    {
        this.instanceName = instanceName;
        this.robot = robot;
        profile = TrcMotionProfile.loadProfileFromCsv("/home/lvuser/test_left_Jaci.csv",
                "/home/lvuser/test_right_Jaci.csv");
        TrcPidController.PidCoefficients pidCoefficients = new TrcPidController.PidCoefficients(kP,kI,kD,kF);
        follower = new FrcMotionProfileFollower(instanceName + ".profileFollower",
                pidCoefficients, RobotInfo.ENCODER_Y_INCHES_PER_COUNT);
        follower.setLeftMotors(robot.leftFrontWheel,robot.leftRearWheel);
        follower.setRightMotors(robot.rightFrontWheel,robot.rightRearWheel);
    }

    public void start()
    {
        follower.start(profile);
        robot.globalTracer.traceInfo(instanceName + ".start","Starting following path!");
    }

    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {
        String message = String.format(
            "MotionProfile: %s - Running: %b, Bottom Buffer: [%d,%d], Top Buffer: [%d,%d], Target Positions: [%.2f,%.2f], Target Velocities: [%.2f,%.2f]",
            follower.getInstanceName(), follower.isActive(),
            follower.leftBottomBufferCount(), follower.rightBottomBufferCount(),
            follower.leftTopBufferCount(), follower.rightTopBufferCount(),
            follower.leftTargetPosition(), follower.rightTargetPosition(),
            follower.leftTargetVelocity(), follower.rightTargetVelocity());

        robot.dashboard.displayPrintf(1, message);
        robot.globalTracer.traceInfo(instanceName + ".cmdPeriodic", message);
        return !follower.isActive();
    }
}
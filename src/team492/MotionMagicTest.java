package team492;

import frclib.FrcMotionMagicController;
import trclib.TrcEvent;
import trclib.TrcPidController;
import trclib.TrcRobot;
import trclib.TrcUtil;

public class MotionMagicTest implements TrcRobot.RobotCommand
{
    private static final double kP = 0.5651851839;
    private static final double kI = 0.0;
    private static final double kD = 0.1695555552;
    private static final double kF = 1.131266385; // TODO: Calculate this according to Phoenix docs

    private static final double WORLD_UNITS_PER_TICK = RobotInfo.ENCODER_Y_INCHES_PER_COUNT;

    private static final double MAX_SPEED = 300;
    private static final double MAX_ACCEL = 300;

    private FrcMotionMagicController motionMagic;
    private Robot robot;
    private TrcEvent event;
    private double startTime;

    public MotionMagicTest(Robot robot)
    {
        this.robot = robot;
        TrcPidController.PidCoefficients pidCoefficients = new TrcPidController.PidCoefficients(kP, kI, kD, kF);
        this.motionMagic = new FrcMotionMagicController("MotionMagic", pidCoefficients, WORLD_UNITS_PER_TICK, MAX_SPEED,
            MAX_ACCEL, 1.0);
        motionMagic.setLeftMotors(robot.leftFrontWheel, robot.leftRearWheel);
        motionMagic.setRightMotors(robot.rightFrontWheel, robot.rightRearWheel);

        event = new TrcEvent("MotionMagicTest.TrcEvent");
    }

    public void start(double distance)
    {
        startTime = TrcUtil.getCurrentTime();
        robot.globalTracer.traceInfo("MotionMagicTest.start", "Started! Time: %.2f", startTime);
        motionMagic.drive(distance, event);
    }

    public void stop()
    {
        motionMagic.cancel();
    }

    @Override
    public boolean cmdPeriodic(double e)
    {
        if (event.isSignaled())
        {
            double elapsedTime = TrcUtil.getCurrentTime() - startTime;
            robot.dashboard.displayPrintf(1, "Motion Magic time: %.2f", elapsedTime);
            if (robot.globalTracer != null)
            {
                robot.globalTracer.traceInfo("cmdPeriodic", "Motion Magic time: %.2f", elapsedTime);
            }
            return true;
        }
        return false;
    }
}

package team492;

import frclib.FrcMotionMagicController;
import trclib.TrcDbgTrace;
import trclib.TrcEvent;
import trclib.TrcPidController;
import trclib.TrcRobot;
import trclib.TrcUtil;

public class MotionMagicTest implements TrcRobot.RobotCommand
{
    private static final double kP = 0.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kF = 0.0;

    private static final double WORLD_UNITS_PER_TICK = RobotInfo.ENCODER_Y_INCHES_PER_COUNT;

    private static final double MAX_SPEED = 120 * WORLD_UNITS_PER_TICK;
    private static final double MAX_ACCEL = 40 * WORLD_UNITS_PER_TICK;

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
        motionMagic.drive(distance, event);
    }

    public void stop()
    {
        motionMagic.cancel();
    }

    @Override
    public boolean cmdPeriodic(double e)
    {
        if(event.isSignaled())
        {
            double elapsedTime = TrcUtil.getCurrentTime() - startTime;
            TrcDbgTrace dbgTrace = TrcDbgTrace.getGlobalTracer();
            robot.dashboard.displayPrintf(1,"Motion Magic time: %.2f", elapsedTime);
            if(dbgTrace != null)
            {
                dbgTrace.traceInfo("cmdPeriodic", "Motion Magic time: %.2f", elapsedTime);
            }
            return true;
        }
        return false;
    }
}

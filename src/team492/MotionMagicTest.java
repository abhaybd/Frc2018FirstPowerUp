package team492;

import frclib.FrcMotionMagicController;
import hallib.HalDashboard;
import trclib.TrcEvent;
import trclib.TrcPidController;
import trclib.TrcRobot;
import trclib.TrcUtil;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintStream;
import java.text.SimpleDateFormat;
import java.util.Date;

public class MotionMagicTest implements TrcRobot.RobotCommand
{
    private static final double kP = 0.5651851839;
    private static final double kI = 0.0;
    private static final double kD = 0.1695555552;
    private static final double kF = 1.131266385; // TODO: Calculate this according to Phoenix docs

    private static final double WORLD_UNITS_PER_TICK = RobotInfo.ENCODER_Y_INCHES_PER_COUNT;

    private static final double MAX_SPEED = 300;
    private static final double MAX_ACCEL = 300;

    private static final boolean WRITE_CSV = true;

    private FrcMotionMagicController motionMagic;
    private Robot robot;
    private TrcEvent event;
    private double startTime;
    private PrintStream fileOut;

    public MotionMagicTest(Robot robot)
    {
        this.robot = robot;
        TrcPidController.PidCoefficients pidCoefficients = new TrcPidController.PidCoefficients(kP, kI, kD, kF);
        this.motionMagic = new FrcMotionMagicController("MotionMagic", WORLD_UNITS_PER_TICK, MAX_SPEED, MAX_ACCEL, 1.0);
        motionMagic.setPidCoefficients(pidCoefficients);
        motionMagic.setLeftMotors(robot.leftFrontWheel, robot.leftRearWheel);
        motionMagic.setRightMotors(robot.rightFrontWheel, robot.rightRearWheel);

        event = new TrcEvent("MotionMagicTest.TrcEvent");

        refreshData("Test/DriveDistance", 0.0);
    }

    private void refreshData(String name, double defaultValue)
    {
        HalDashboard.putNumber(name, HalDashboard.getNumber(name, defaultValue));
    }

    public void start(double distance)
    {
        startTime = TrcUtil.getCurrentTime();
        robot.globalTracer.traceInfo("MotionMagicTest.start", "Started! Time: %.2f", startTime);
        motionMagic.drive(distance, event);

        if (WRITE_CSV)
        {
            try
            {
                startTime = TrcUtil.getCurrentTime();
                String timeStamp = new SimpleDateFormat("dd-MM-yy_HHmm").format(new Date());
                File dir = new File("/home/lvuser/MotionMagic_logs");
                if (dir.isDirectory() || dir.mkdir())
                {
                    fileOut = new PrintStream(new FileOutputStream(new File(dir, timeStamp + "_profilelog.csv")));
                    fileOut.println("Time,AvgError,LPosition,RPosition,LVelocity,RVelocity");
                }
            }
            catch (IOException e)
            {
                robot.globalTracer.traceErr("MotionMagicTest.start", e.toString());
            }
        }
    }

    public void stop()
    {
        motionMagic.cancel();
    }

    @Override
    public boolean cmdPeriodic(double e)
    {
        double elapsedTime = TrcUtil.getCurrentTime() - startTime;
        if (event.isSignaled())
        {
            robot.dashboard.displayPrintf(1, "Motion Magic time: %.2f", elapsedTime);
            if (robot.globalTracer != null)
            {
                robot.globalTracer.traceInfo("cmdPeriodic", "Motion Magic time: %.2f", elapsedTime);
            }
            event.clear();
            return true;
        }

        if (motionMagic.isRunning())
        {
            fileOut.printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", elapsedTime, motionMagic.getError(),
                robot.leftFrontWheel.getPosition(), robot.rightFrontWheel.getPosition(),
                robot.leftFrontWheel.getSpeed(), robot.rightFrontWheel.getSpeed());
        }
        return false;
    }
}

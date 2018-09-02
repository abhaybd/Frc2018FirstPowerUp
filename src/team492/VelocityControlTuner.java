package team492;

import hallib.HalDashboard;
import trclib.TrcPidController;
import trclib.TrcRobot;
import trclib.TrcTaskMgr;
import trclib.TrcUtil;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.PrintStream;
import java.text.SimpleDateFormat;
import java.util.Date;

public class VelocityControlTuner implements TrcRobot.RobotCommand
{
    private enum State
    {
        START,
        ACCELERATE,
        HOLD_SPEED,
        DECELERATE,
        DONE
    }

    private static final boolean WRITE_CSV = true;

    // TODO: Tune these
    private static final double kP = 0.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kF = 0.0;

    private double startTime;
    private double lastTime;
    private double maxSpeed, maxAcceleration, driveTime;
    private double targetSpeed;
    private double maxRobotSpeed;
    private TrcTaskMgr.TaskObject taskObj;
    private State state;
    private Robot robot;
    private PrintStream fileOut;

    public VelocityControlTuner(String instanceName, Robot robot)
    {
        this.robot = robot;

        taskObj = TrcTaskMgr.getInstance().createTask(instanceName + ".velocityTask", this::velocityTask);

        TrcPidController.PidCoefficients pidCoefficients = new TrcPidController.PidCoefficients(kP, kI, kD, kF);

        // Convert from in/sec -> in/100ms
        robot.leftFrontWheel.enableVelocityMode(maxRobotSpeed * 0.1, pidCoefficients);
        robot.rightFrontWheel.enableVelocityMode(maxRobotSpeed * 0.1, pidCoefficients);
        robot.leftRearWheel.enableVelocityMode(maxRobotSpeed * 0.1, pidCoefficients);
        robot.rightRearWheel.enableVelocityMode(maxRobotSpeed * 0.1, pidCoefficients);

        refreshNumber("Test/MaxRobotSpeed");
        refreshNumber("Test/MaxSpeed");
        refreshNumber("Test/MaxAcceleration");
        refreshNumber("Test/DriveTime");

        maxRobotSpeed = HalDashboard.getNumber("Test/MaxRobotSpeed", 0.0);

        if(WRITE_CSV)
        {
            try
            {
                String timeStamp = new SimpleDateFormat("dd-MM-yy_HHmm").format(new Date());
                new File("/home/lvuser/SpeedLogs").mkdir();
                fileOut = new PrintStream(new FileOutputStream(
                    String.format("/home/lvuser/%s_speedlog.csv", timeStamp)));
                fileOut.println("TargetSpeed, ActualSpeed");
            }
            catch (FileNotFoundException e)
            {
                e.printStackTrace();
            }
        }
    }

    private void refreshNumber(String key)
    {
        HalDashboard.putNumber(key, HalDashboard.getNumber(key, 0.0));
    }

    public void start()
    {
        this.maxSpeed = HalDashboard.getNumber("Test/MaxSpeed", 0.0);
        this.maxAcceleration = HalDashboard.getNumber("Test/MaxAcceleration", 0.0);
        this.driveTime = HalDashboard.getNumber("Test/DriveTime", 0.0);

        setEnabled(true);
    }

    public void stop()
    {
        setEnabled(false);
    }

    private void setEnabled(boolean enabled)
    {
        targetSpeed = 0.0;

        if(enabled)
        {
            taskObj.registerTask(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
            state = State.START;
        }
        else
        {
            taskObj.unregisterTask(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
            state = State.DONE;
        }
    }

    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {
        if(fileOut != null)
        {
            double avgSpeed = TrcUtil.average(
                robot.leftFrontWheel.getSpeed(),
                robot.rightFrontWheel.getSpeed(),
                robot.leftRearWheel.getSpeed(),
                robot.rightRearWheel.getSpeed());
            fileOut.printf("%.3f,%.3f\n", targetSpeed, avgSpeed);
        }
        return false;
    }

    private void velocityTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        double currentTime = TrcUtil.getCurrentTime();
        double timeSinceLast = currentTime - lastTime;

        switch(state)
        {
            case START:
                targetSpeed = 0.0;
                startTime = TrcUtil.getCurrentTime();
                state = State.ACCELERATE;
                break;

            case ACCELERATE:
                targetSpeed += timeSinceLast * maxAcceleration;
                targetSpeed = Math.min(maxSpeed, targetSpeed);
                if(targetSpeed >= maxSpeed)
                {
                    state = State.HOLD_SPEED;
                }
                break;

            case HOLD_SPEED:
                targetSpeed = maxSpeed;
                if(currentTime - startTime >= driveTime)
                {
                    state = State.DECELERATE;
                }
                break;

            case DECELERATE:
                targetSpeed -= timeSinceLast * maxAcceleration;
                targetSpeed = Math.max(0, targetSpeed);
                if(targetSpeed <= 0.0)
                {
                    state = State.DONE;
                }
                break;

            case DONE:
                setEnabled(false);
                break;
        }

        lastTime = currentTime;

        double throttle = targetSpeed / maxRobotSpeed;
        robot.driveBase.tankDrive(throttle, throttle);
    }
}

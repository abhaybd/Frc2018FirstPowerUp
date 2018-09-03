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

    private double startTime;
    private double lastTime;
    private double maxSpeed, maxAcceleration, driveTime;
    private TrcTaskMgr.TaskObject taskObj;
    private State state;
    private Robot robot;
    private PrintStream fileOut;
    private int numDataPoints;
    private double totalAbsoluteError;
    private double totalSquareError;

    /**
     * The units on these MUST be in units/sec
     */
    private double targetSpeed;
    private double maxRobotSpeed;

    public VelocityControlTuner(String instanceName, Robot robot)
    {
        this.robot = robot;

        refreshNumber("Test/MaxRobotSpeed");
        refreshNumber("Test/MaxSpeed");
        refreshNumber("Test/MaxAcceleration");
        refreshNumber("Test/DriveTime");
        refreshNumber("Test/MAE"); // MAE is mean absolute error
        refreshNumber("Test/RMSE"); // RMSE is root mean square error

        taskObj = TrcTaskMgr.getInstance().createTask(instanceName + ".velocityTask", this::velocityTask);

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

    /**
     * Start following velocity. Test/MaxRobotSpeed in HalDashboard MUST be in encoder units per second.
     */
    public void start()
    {
        start(HalDashboard.getNumber("Test/MaxRobotSpeed", 0.0));
    }

    /**
     * Start following velocity.
     *
     * @param maxRobotSpeed The maximum robot speed in encoder units per second.
     */
    public void start(double maxRobotSpeed)
    {
        TrcPidController.PidCoefficients pidCoefficients = new TrcPidController.PidCoefficients(
            HalDashboard.getNumber("Test/kP", 0.0),
            HalDashboard.getNumber("Test/kI", 0.0),
            HalDashboard.getNumber("Test/kD", 0.0),
            HalDashboard.getNumber("Test/kF", 0.0));

        maxSpeed = HalDashboard.getNumber("Test/MaxSpeed", 0.0);
        maxAcceleration = HalDashboard.getNumber("Test/MaxAcceleration", 0.0);
        driveTime = HalDashboard.getNumber("Test/DriveTime", 0.0);
        this.maxRobotSpeed = maxRobotSpeed;

        // Convert from units/sec -> units/100ms
        robot.leftFrontWheel.enableVelocityMode(maxRobotSpeed * 0.1, pidCoefficients);
        robot.rightFrontWheel.enableVelocityMode(maxRobotSpeed * 0.1, pidCoefficients);
        robot.leftRearWheel.enableVelocityMode(maxRobotSpeed * 0.1, pidCoefficients);
        robot.rightRearWheel.enableVelocityMode(maxRobotSpeed * 0.1, pidCoefficients);

        totalAbsoluteError = 0;
        totalSquareError = 0;
        numDataPoints = 0;

        setEnabled(true);
    }

    public void stop()
    {
        robot.driveBase.stop();
        setEnabled(false);
        maxRobotSpeed = 0.0;
        targetSpeed = 0.0;
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
            double speed = robot.driveBase.getYSpeed();
            fileOut.printf("%.3f,%.3f\n", targetSpeed, speed);
        }

        HalDashboard.putNumber("Test/MAE", totalAbsoluteError / (double)numDataPoints);
        HalDashboard.putNumber("Test/RMSE", Math.sqrt(totalSquareError / (double)numDataPoints));
        return false;
    }

    private void velocityTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        double currentTime = TrcUtil.getCurrentTime();
        double timeSinceLast = currentTime - lastTime;

        if(maxRobotSpeed == 0.0)
        {
            state = State.DONE;
        }
        else if(currentTime - startTime >= driveTime)
        {
            state = State.DECELERATE;
        }

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
                stop();
                break;
        }

        lastTime = currentTime;

        double throttle = targetSpeed / maxRobotSpeed;
        robot.driveBase.tankDrive(throttle, throttle);

        double actualSpeed = robot.driveBase.getYSpeed();
        double error = targetSpeed - actualSpeed;
        totalSquareError += Math.pow(error, 2.0);
        totalAbsoluteError += Math.abs(error);
        numDataPoints++;
    }
}

package trclib;

import java.util.Arrays;
import java.util.List;

public class TrcSwerveDriveBase extends TrcDriveBase
{
    /**
     *  Programmed according to the this whitepaper: http://www.chiefdelphi.com/media/papers/download/3028
     */

    private static final int NUM_MOTORS = 4;

    private final String instanceName;
    private TrcSwerveModule lfModule, rfModule, lrModule, rrModule;
    private double wheelBaseWidth, wheelBaseLength, wheelBaseDiagonal;
    private double heading, turnSpeed, yPosition, ySpeed, xPosition, xSpeed;
    private double positionScale;
    private TrcGyro gyro;
    private MotorPowerMapper motorPowerMapper;
    private double lfStallStartTime, rfStallStartTime, lrStallStartTime, rrStallStartTime;

    public TrcSwerveDriveBase(String instanceName, double wheelBaseWidth, double wheelBaseLength, TrcGyro gyro,
        TrcSwerveModule lfModule, TrcSwerveModule rfModule, TrcSwerveModule lrModule, TrcSwerveModule rrModule)
    {
        this.instanceName = instanceName;

        this.wheelBaseWidth = wheelBaseWidth;
        this.wheelBaseLength = wheelBaseLength;
        this.wheelBaseDiagonal = magnitude(wheelBaseWidth, wheelBaseLength);

        this.gyro = gyro;

        this.lfModule = lfModule;
        this.rfModule = rfModule;
        this.lrModule = lrModule;
        this.rrModule = rrModule;

        this.positionScale = 1; // Init it to something

        motorPowerMapper = this::defaultMotorPowerMapper;

        TrcTaskMgr.TaskObject driveBaseTaskObj = TrcTaskMgr.getInstance().createTask(
            instanceName + ".driveBaseTask", this::driveBaseTask);
        driveBaseTaskObj.registerTask(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
        driveBaseTaskObj.registerTask(TrcTaskMgr.TaskType.STOP_TASK);
    }

    @Override
    public List<DriveMode> getSupportedDriveModes()
    {
        return Arrays.asList(DriveMode.CURVE_MODE, DriveMode.ARCADE_MODE, DriveMode.TANK_MODE, DriveMode.SWERVE_MODE);
    }

    @Override
    public int getNumMotors()
    {
        final String funcName = "getNumMotors";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%d", NUM_MOTORS);
        }

        return NUM_MOTORS; // Don't count the turn motors
    }

    @Override
    public void setMotorPowerMapper(MotorPowerMapper mapper)
    {
        motorPowerMapper = (mapper != null ? mapper : this::defaultMotorPowerMapper);
    }

    @Override
    public void setYPositionScale(double scale)
    {
        if (debugEnabled)
        {
            final String funcName = "setPositionScale";
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "scale=%f", scale);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        positionScale = scale; // Swerve doesn't have different x and y scales
    }

    @Override
    public void setXPositionScale(double scale)
    {
        if (debugEnabled)
        {
            final String funcName = "setPositionScale";
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "scale=%f", scale);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        positionScale = scale; // Swerve doesn't have different x and y scales
    }

    @Override
    public void resetPosition(boolean hardware)
    {
        final String funcName = "resetPosition";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "hardware=%b", hardware);
        }

        xPosition = 0;
        yPosition = 0;
        xSpeed = 0;
        ySpeed = 0;
        heading = 0;
        turnSpeed = 0;

        lfModule.resetPosition(hardware);
        rfModule.resetPosition(hardware);
        lrModule.resetPosition(hardware);
        rrModule.resetPosition(hardware);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }

    @Override
    public void stop()
    {
        final String funcName = "stop";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
        }

        stopModule(lfModule);
        stopModule(rfModule);
        stopModule(lrModule);
        stopModule(rrModule);

        resetStallTimer();

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }

    private void stopModule(TrcSwerveModule module)
    {
        module.setDrivePower(0.0);
        module.stopTrackingTurnAngle();
    }

    @Override
    public void reset()
    {
        final String funcName = "reset";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
        }

        lfModule.setDrivePower(0.0);
        rfModule.setDrivePower(0.0);
        lrModule.setDrivePower(0.0);
        rrModule.setDrivePower(0.0);

        lfModule.setAngle(0.0);
        rfModule.setAngle(0.0);
        lrModule.setAngle(0.0);
        rrModule.setAngle(0.0);

        resetPosition();
        resetStallTimer();

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }

    @Override
    public double getHeading()
    {
        if (debugEnabled)
        {
            final String funcName = "getHeading";
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", heading);
        }

        return heading;
    }

    @Override
    public double getTurnSpeed()
    {
        if (debugEnabled)
        {
            final String funcName = "getTurnSpeed";
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", turnSpeed);
        }

        return turnSpeed;
    }

    @Override
    public double getYPosition()
    {
        if (debugEnabled)
        {
            final String funcName = "getYPosition";
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", yPosition);
        }

        return yPosition;
    }

    @Override
    public double getXPosition()
    {
        if (debugEnabled)
        {
            final String funcName = "getXPosition";
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", xPosition);
        }

        return xPosition;
    }

    @Override
    public double getYSpeed()
    {
        if (debugEnabled)
        {
            final String funcName = "getYSpeed";
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", ySpeed);
        }

        return ySpeed;
    }

    @Override
    public double getXSpeed()
    {
        if (debugEnabled)
        {
            final String funcName = "getXSpeed";
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", xSpeed);
        }

        return xSpeed;
    }

    @Override
    public void setBrakeMode(boolean enabled)
    {
        if (debugEnabled)
        {
            final String funcName = "setBrakeMode";
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "enabled=%s", Boolean.toString(enabled));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        lfModule.setBrakeMode(enabled);
        rfModule.setBrakeMode(enabled);
        lrModule.setBrakeMode(enabled);
        rrModule.setBrakeMode(enabled);
    }

    @Override
    public void resetStallTimer()
    {
        if (debugEnabled)
        {
            final String funcName = "resetStallTimer";
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        lfStallStartTime = rfStallStartTime = lrStallStartTime = rrStallStartTime = TrcUtil.getCurrentTime();
    }

    @Override
    public boolean isStalled(MotorType motorType, double stallTime)
    {
        final String funcName = "isStalled";
        double time = TrcUtil.getCurrentTime();
        boolean stalled = false;

        if (debugEnabled)
        {
            dbgTrace
                .traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "motorType=%s,stallTime=%.3f", motorType, stallTime);
        }

        switch(motorType)
        {
            case LEFT_FRONT:
                stalled = time - lfStallStartTime >= stallTime;
                break;

            case RIGHT_FRONT:
                stalled = time - rfStallStartTime >= stallTime;
                break;

            case LEFT_REAR:
                stalled = time - lrStallStartTime >= stallTime;
                break;

            case RIGHT_REAR:
                stalled = time - rrStallStartTime >= stallTime;
                break;

            default:
                break;
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", Boolean.toString(stalled));
        }

        return stalled;
    }

    @Override
    public void tankDrive(double leftPower, double rightPower, boolean inverted)
    {
        final String funcName = "tankDrive";

        if (debugEnabled)
        {
            dbgTrace
                .traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "leftPower=%f,rightPower=%f,inverted=%s", leftPower,
                    rightPower, Boolean.toString(inverted));
        }

        lfModule.setAngle(0.0);
        rfModule.setAngle(0.0);
        lrModule.setAngle(0.0);
        rrModule.setAngle(0.0);

        leftPower = motorPowerMapper.translateMotorPower(leftPower,
            positionScale * average(lfModule.getDriveSpeed(), lrModule.getDriveSpeed()));
        rightPower = motorPowerMapper.translateMotorPower(rightPower,
            positionScale * average(rfModule.getDriveSpeed(), rrModule.getDriveSpeed()));

        if(inverted)
        {
            double temp = leftPower;
            leftPower = -rightPower;
            rightPower = -temp;
        }

        if (gyroAssistEnabled)
        {
            double diffPower = (leftPower - rightPower) / 2.0;
            double assistPower = TrcUtil
                .clipRange(gyroAssistKp * (diffPower - gyro.getZRotationRate().value / gyroMaxRotationRate));
            leftPower += assistPower;
            rightPower -= assistPower;
            double maxMag = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (maxMag > 1.0)
            {
                leftPower /= maxMag;
                rightPower /= maxMag;
            }
        }

        leftPower = TrcUtil.clipRange(leftPower, -maxOutput, maxOutput);
        rightPower = TrcUtil.clipRange(rightPower, -maxOutput, maxOutput);

        lfModule.setDrivePower(leftPower);
        lrModule.setDrivePower(leftPower);

        rfModule.setDrivePower(rightPower);
        rrModule.setDrivePower(rightPower);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }

    @Override
    public void swerveDrive_Cartesian(double x, double y, double rotation, boolean inverted, double gyroAngle)
    {
        final String funcName = "serveDrive_Cartesian";

        if (debugEnabled)
        {
            dbgTrace
                .traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "x=%f,y=%f,rotation=%f,inverted=%b,gyroAngle=%f",
                    x, y, rotation, inverted, gyroAngle);
        }

        x = TrcUtil.clipRange(x);
        y = TrcUtil.clipRange(y);
        rotation = TrcUtil.clipRange(rotation);

        if(inverted)
        {
            x = -x;
            y = -y;
        }

        if(gyroAngle != 0)
        {
            if(inverted)
            {
                dbgTrace.traceWarn(instanceName + ".swerveDrive_Cartesian",
                    "You should not be using inverted and field reference frame at the same time!");
            }

            double gyroRadians = Math.toRadians(gyroAngle);
            double temp = y * Math.cos(gyroRadians) + x * Math.sin(gyroRadians);
            x = -y * Math.sin(gyroRadians) + x * Math.cos(gyroRadians);
            y = temp;
        }

        double a = x - (rotation * wheelBaseLength/wheelBaseDiagonal);
        double b = x + (rotation * wheelBaseLength/wheelBaseDiagonal);
        double c = y - (rotation * wheelBaseWidth/wheelBaseDiagonal);
        double d = y + (rotation * wheelBaseWidth/wheelBaseDiagonal);

        // The whitepaper goes in order rf, lf, lr, rr. We like to do lf, rf, lr, rr.
        double lfAngle = Math.toDegrees(Math.atan2(b,d));
        double rfAngle = Math.toDegrees(Math.atan2(b,c));
        double lrAngle = Math.toDegrees(Math.atan2(a,d));
        double rrAngle = Math.toDegrees(Math.atan2(a,c));

        // The whitepaper goes in order rf, lf, lr, rr. We like to do lf, rf, lr, rr.
        double lfPower = magnitude(b, d);
        double rfPower = magnitude(b, c);
        double lrPower = magnitude(a, d);
        double rrPower = magnitude(a, c);

        double[] normalizedPowers = normalize(lfPower, rfPower, lrPower, rrPower);
        lfPower = TrcUtil.clipRange(normalizedPowers[0], -maxOutput, maxOutput);
        rfPower = TrcUtil.clipRange(normalizedPowers[1], -maxOutput, maxOutput);
        lrPower = TrcUtil.clipRange(normalizedPowers[2], -maxOutput, maxOutput);
        rrPower = TrcUtil.clipRange(normalizedPowers[3], -maxOutput, maxOutput);

        lfModule.setAngle(lfAngle);
        rfModule.setAngle(rfAngle);
        lrModule.setAngle(lrAngle);
        rrModule.setAngle(rrAngle);

        lfModule.setDrivePower(motorPowerMapper.translateMotorPower(lfPower, lfModule.getDriveSpeed() * positionScale));
        rfModule.setDrivePower(motorPowerMapper.translateMotorPower(rfPower, rfModule.getDriveSpeed() * positionScale));
        lrModule.setDrivePower(motorPowerMapper.translateMotorPower(lrPower, lrModule.getDriveSpeed() * positionScale));
        rrModule.setDrivePower(motorPowerMapper.translateMotorPower(rrPower, rrModule.getDriveSpeed() * positionScale));

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }

    private double magnitude(double... nums)
    {
        return Math.sqrt(Arrays.stream(nums).map(e -> e*e).sum());
    }

    private double[] normalize(double... nums)
    {
        double max = Arrays.stream(nums).max().orElse(0.0);
        return max > 1 ? Arrays.stream(nums).map(x -> x/max).toArray() : nums;
    }

    private double average(double... toAverage)
    {
        return Arrays.stream(toAverage).average().orElse(0.0);
    }

    private void resetModulePositions()
    {
        lfModule.resetPosition(false);
        rfModule.resetPosition(false);
        lrModule.resetPosition(false);
        rrModule.resetPosition(false);
    }

    private void driveBaseTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        if(taskType == TrcTaskMgr.TaskType.STOP_TASK)
        {
            stop();
        } else
        {
            heading = gyro.getZHeading().value;
            turnSpeed = gyro.getZRotationRate().value;

            double lfPos = lfModule.getPosition();
            double rfPos = rfModule.getPosition();
            double lrPos = lrModule.getPosition();
            double rrPos = rrModule.getPosition();

            double averageWheelAngle = average(lfModule.getAngle(), rfModule.getAngle(), lrModule.getAngle(), rrModule.getAngle());
            double averageWheelPosition = positionScale * average(lfPos, rfPos, lrPos, rrPos);
            double averageWheelSpeed = positionScale *
                average(lfModule.getDriveSpeed(), rfModule.getDriveSpeed(), lrModule.getDriveSpeed(), rrModule.getDriveSpeed());

            xSpeed = averageWheelSpeed * Math.cos(Math.toRadians(averageWheelAngle));
            ySpeed = averageWheelSpeed * Math.sin(Math.toRadians(averageWheelAngle));

            xPosition += averageWheelPosition * Math.cos(Math.toRadians(averageWheelAngle));
            yPosition += averageWheelPosition * Math.sin(Math.toRadians(averageWheelAngle));

            double time = TrcUtil.getCurrentTime();

            if(lfPos != 0 || lfModule.getDrivePower() == 0.0) lfStallStartTime = time;
            if(rfPos != 0 || rfModule.getDrivePower() == 0.0) rfStallStartTime = time;
            if(lrPos != 0 || lrModule.getDrivePower() == 0.0) lrStallStartTime = time;
            if(rrPos != 0 || rrModule.getDrivePower() == 0.0) rrStallStartTime = time;

            resetModulePositions();
        }
    }
}

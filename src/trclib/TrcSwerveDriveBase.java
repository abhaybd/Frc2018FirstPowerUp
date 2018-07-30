package trclib;

import java.util.Arrays;
import java.util.List;

public class TrcSwerveDriveBase implements TrcDriveBase
{
    /**
     *  Programmed according to the this whitepaper: http://www.chiefdelphi.com/media/papers/download/3028
     */

    private final String instanceName;
    private TrcSwerveModule lfModule, rfModule, lrModule, rrModule;
    private double wheelBaseWidth, wheelBaseLength, wheelBaseDiagonal;
    private double heading, turnSpeed, yPosition, ySpeed, xPosition, xSpeed; // TODO: Add a postperiodic task to set these values
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

        this.positionScale = 1;

        motorPowerMapper = this::defaultMotorPowerMapper;

        TrcTaskMgr.TaskObject driveBaseTaskObj = TrcTaskMgr.getInstance().createTask(
            instanceName + ".driveBaseTask", this::driveBaseTask);
        driveBaseTaskObj.registerTask(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
        driveBaseTaskObj.registerTask(TrcTaskMgr.TaskType.STOP_TASK);
    }

    @Override
    public List<DriveMode> getSupportedDriveModes()
    {
        return Arrays.asList(DriveMode.SWERVE_MODE);
    }

    @Override
    public void setMotorPowerMapper(MotorPowerMapper mapper)
    {
        motorPowerMapper = (mapper != null ? mapper : this::defaultMotorPowerMapper);
    }

    private double defaultMotorPowerMapper(double power, double speed)
    {
        return power;
    }

    @Override
    public void setYPositionScale(double scale)
    {
        positionScale = scale; // Swerve doesn't have different x and y scales
    }

    @Override
    public void setXPositionScale(double scale)
    {
        positionScale = scale; // Swerve doesn't have different x and y scales
    }

    @Override
    public void resetPosition(boolean hardware)
    {
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
    }

    @Override
    public void stop()
    {
        stopModule(lfModule);
        stopModule(rfModule);
        stopModule(lrModule);
        stopModule(rrModule);
    }

    private void stopModule(TrcSwerveModule module)
    {
        module.setDrivePower(0.0);
        module.stopTrackingTurnAngle();
    }

    @Override
    public void reset()
    {
        lfModule.setDrivePower(0.0);
        rfModule.setDrivePower(0.0);
        lrModule.setDrivePower(0.0);
        rrModule.setDrivePower(0.0);

        lfModule.setAngle(0.0);
        rfModule.setAngle(0.0);
        lrModule.setAngle(0.0);
        rrModule.setAngle(0.0);

        resetPosition();
    }

    @Override
    public double getHeading()
    {
        return heading;
    }

    @Override
    public double getTurnSpeed()
    {
        return turnSpeed;
    }

    @Override
    public double getYPosition()
    {
        return yPosition;
    }

    @Override
    public double getXPosition()
    {
        return xPosition;
    }

    @Override
    public double getYSpeed()
    {
        return ySpeed;
    }

    @Override
    public double getXSpeed()
    {
        return xSpeed;
    }

    @Override
    public void setBrakeMode(boolean enabled)
    {
        lfModule.setBrakeMode(enabled);
        rfModule.setBrakeMode(enabled);
        lrModule.setBrakeMode(enabled);
        rrModule.setBrakeMode(enabled);
    }

    @Override
    public void resetStallTimer()
    {
        lfStallStartTime = rfStallStartTime = lrStallStartTime = rrStallStartTime = TrcUtil.getCurrentTime();
    }

    @Override
    public boolean isStalled(MotorType motorType, double stallTime)
    {
        double time = TrcUtil.getCurrentTime();
        switch(motorType)
        {
            case LEFT_FRONT:
                return time - lfStallStartTime >= stallTime;

            case RIGHT_FRONT:
                return time - rfStallStartTime >= stallTime;

            case LEFT_REAR:
                return time - lrStallStartTime >= stallTime;

            case RIGHT_REAR:
                return time - rrStallStartTime >= stallTime;

            default:
                return false;
        }
    }

    @Override
    public void swerveDrive_Cartesian(double x, double y, double rotation, boolean inverted, double gyroAngle)
    {
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
                TrcDbgTrace.getGlobalTracer().traceWarn(instanceName + ".swerveDrive_Cartesian",
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
        lfPower = normalizedPowers[0];
        rfPower = normalizedPowers[1];
        lrPower = normalizedPowers[2];
        rrPower = normalizedPowers[3];

        lfModule.setAngle(lfAngle);
        rfModule.setAngle(rfAngle);
        lrModule.setAngle(lrAngle);
        rrModule.setAngle(rrAngle);

        lfModule.setDrivePower(motorPowerMapper.translateMotorPower(lfPower, lfModule.getDriveSpeed() * positionScale));
        rfModule.setDrivePower(motorPowerMapper.translateMotorPower(rfPower, rfModule.getDriveSpeed() * positionScale));
        lrModule.setDrivePower(motorPowerMapper.translateMotorPower(lrPower, lrModule.getDriveSpeed() * positionScale));
        rrModule.setDrivePower(motorPowerMapper.translateMotorPower(rrPower, rrModule.getDriveSpeed() * positionScale));
    }

    private double magnitude(double a, double b)
    {
        return Math.sqrt(a*a + b*b);
    }

    private double[] normalize(double... nums)
    {
        if(nums.length == 0)
        {
             return nums;
        }

        double max = Arrays.stream(nums).max().getAsDouble();
        if(max > 1)
        {
            return Arrays.stream(nums).map(x -> x/max).toArray();
        } else
        {
            return nums;
        }
    }

    private double average(double... nums)
    {
        double sum = 0;
        for(double d:nums)
        {
            sum += d;
        }
        return sum / Math.max(nums.length, 1); // Protect against length == 0
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

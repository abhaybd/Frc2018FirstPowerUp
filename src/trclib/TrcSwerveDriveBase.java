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
    private double heading, yPosition, ySpeed; // TODO: Add a postperiodic task to set these values
    public TrcSwerveDriveBase(String instanceName, double wheelBaseWidth, double wheelBaseLength,
        TrcSwerveModule lfModule, TrcSwerveModule rfModule, TrcSwerveModule lrModule, TrcSwerveModule rrModule)
    {
        this.instanceName = instanceName;

        this.wheelBaseWidth = wheelBaseWidth;
        this.wheelBaseLength = wheelBaseLength;
        this.wheelBaseDiagonal = magnitude(wheelBaseWidth, wheelBaseLength);

        this.lfModule = lfModule;
        this.rfModule = rfModule;
        this.lrModule = lrModule;
        this.rrModule = rrModule;
    }

    @Override
    public List<DriveMode> getSupportedDriveModes()
    {
        return Arrays.asList(DriveMode.SWERVE_MODE);
    }

    public void resetPosition()
    {
        heading = 0;
        yPosition = 0;
        ySpeed = 0;
        // TODO: hardware reset
        // TODO: Figure out how these values are going to be used/calculated
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
    public double getYPosition()
    {
        return yPosition;
    }

    @Override
    public double getYSpeed()
    {
        return ySpeed;
    }

    @Override
    public void swerveDrive_Cartesian(double x, double y, double rotation, boolean inverted, double gyroAngle)
    {
        x = TrcUtil.clipRange(x);
        y = TrcUtil.clipRange(y);
        rotation = TrcUtil.clipRange(rotation);

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

        lfModule.setDrivePower(inverted ? -lfPower : lfPower);
        rfModule.setDrivePower(inverted ? -rfPower : rfPower);
        lrModule.setDrivePower(inverted ? -lrPower : lrPower);
        rrModule.setDrivePower(inverted ? -rrPower : rrPower);
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
}

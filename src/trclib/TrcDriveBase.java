package trclib;

import java.util.Arrays;
import java.util.List;

public abstract class TrcDriveBase
{
    private static final String moduleName = "TrcDriveBase";

    protected static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final boolean useGlobalTracer = false;

    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;

    private static double DEF_SENSITIVITY = 0.5;
    private static double DEF_MAX_OUTPUT = 1.0;

    public enum DriveMode
    {
        CURVE_MODE,
        ARCADE_MODE,
        TANK_MODE,
        MECANUM_MODE,
        SWERVE_MODE
    }

    enum MotorType
    {
        LEFT_FRONT(0),
        RIGHT_FRONT(1),
        LEFT_REAR(2),
        RIGHT_REAR(3),
        LEFT_MID(4),
        RIGHT_MID(5);

        public final int value;

        MotorType(int value)
        {
            this.value = value;
        }
    }   //enum MotorType

    /**
     * This interface is provided by the caller to translate the motor power to actual motor power according to
     * the motor curve. This is useful to linearize the motor performance. This is very useful for many reasons.
     * It could allow the drive base to drive straight by translating wheel power to actual torque. It could also
     * allow us to implement our own ramp rate to limit acceleration and deceleration.
     */
    public interface MotorPowerMapper
    {
        /**
         * This method is called to translate the desired motor power to the actual motor power taking into
         * consideration of the motor torque curve with the current motor speed.
         *
         * @param power specifies the desired motor power.
         * @param speed specifies the current motor speed in the unit of encoder counts per second.
         * @return resulting motor power.
         */
        double translateMotorPower(double power, double speed);
    }   //interface MotorPowerMapper

    protected double sensitivity = DEF_SENSITIVITY;
    protected double maxOutput = DEF_MAX_OUTPUT;
    protected TrcDbgTrace dbgTrace;
    protected boolean gyroAssistEnabled = false;
    protected double gyroMaxRotationRate, gyroAssistKp;

    public TrcDriveBase()
    {
        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer ?
                TrcDbgTrace.getGlobalTracer() :
                new TrcDbgTrace(moduleName, tracingEnabled, traceLevel, msgLevel);
        }
    }

    /**
     * Default motor power mapper. Doesn't take speed into account at all.
     *
     * @param power Desired power. This will be returned.
     * @param speed Current speed. Ignored.
     * @return equal to <parameter>power</parameter>.
     */
    protected double defaultMotorPowerMapper(double power, double speed)
    {
        return power;
    }

    /**
     * This method returns the number of motors in the drive train.
     *
     * @return number of motors.
     */
    public abstract int getNumMotors();

    /**
     * This methods stops the drive base.
     */
    public abstract void stop();

    /**
     * This method resets the drive base position odometry. This includes the motor encoders, the gyro heading and
     * all the cached values.
     *
     * @param hardware specifies true for resetting hardware position, false for resetting software position.
     */
    public abstract void resetPosition(boolean hardware);

    /**
     * This method returns the gyro heading of the drive base in degrees.
     *
     * @return gyro heading.
     */
    public abstract double getHeading();

    /**
     * This method returns the drive base turn speed.
     *
     * @return turn speed.
     */
    public abstract double getTurnSpeed();

    /**
     * This method returns the Y position in scaled unit.
     *
     * @return Y position.
     */
    public abstract double getYPosition();

    /**
     * This method returns the drive base speed in the Y direction.
     *
     * @return Y speed.
     */
    public abstract double getYSpeed();

    /**
     * This method sets the Y position scale. The raw position from the encoder is in encoder counts. By setting the
     * scale factor, one could make getPosition to return unit in inches, for example.
     *
     * @param scale specifies the Y position scale.
     */
    public abstract void setYPositionScale(double scale);

    /**
     * This method enables/disables brake mode of the drive base.
     *
     * @param enabled specifies true to enable brake mode, false to disable it.
     */
    public abstract void setBrakeMode(boolean enabled);

    /**
     * This method checks whether this specific motor is stalled.
     *
     * @param motorType The motor to check.
     * @param stallTime How many seconds of stalling to count as stalling?
     * @return Whether this motor has stalled for <parameter>stallTime</parameter> seconds
     */
    public abstract boolean isStalled(MotorType motorType, double stallTime);

    /**
     * This method resets the stall timer.
     */
    public abstract void resetStallTimer();

    /**
     * This method enables gyro assist drive. Gyro assist will use the gyro sensor to apply corrective power to try to
     * maintain a consistent heading. If gyro assist is enabled, it will only be applied for drive modes that support it.
     *
     * @param gyroMaxRotationRate specifies the maximum rotation rate of the robot base reported by the gyro.
     * @param gyroAssistKp        specifies the gyro assist proportional constant.
     */
    public void enableGyroAssist(double gyroMaxRotationRate, double gyroAssistKp)
    {
        final String funcName = "enableGyroAssist";

        if (debugEnabled)
        {
            dbgTrace
                .traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "gyroMaxRate=%f,gyroAssistKp=%f", gyroMaxRotationRate,
                    gyroAssistKp);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        this.gyroMaxRotationRate = gyroMaxRotationRate;
        this.gyroAssistKp = gyroAssistKp;
        this.gyroAssistEnabled = true;
    }

    /**
     * This method disables gyro assist drive.
     */
    public void disableGyroAssist()
    {
        final String funcName = "enableGyroAssist";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        this.gyroMaxRotationRate = 0.0;
        this.gyroAssistKp = 1.0;
        this.gyroAssistEnabled = false;
    }

    /**
     * This method checks if all motors on the drive base have been stalled for at least the specified stallTime.
     *
     * @param stallTime specifies the stall time.
     * @return true if the drive base is stalled, false otherwise.
     */
    public boolean isStalled(double stallTime)
    {
        boolean stalled =
            isStalled(MotorType.LEFT_FRONT, stallTime) && isStalled(MotorType.RIGHT_FRONT, stallTime) && isStalled(
                MotorType.LEFT_REAR, stallTime) && isStalled(MotorType.RIGHT_REAR, stallTime);

        if (debugEnabled)
        {
            final String funcName = "isStalled";
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "stallTime=%.3f", stallTime);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", Boolean.toString(stalled));
        }

        return stalled;
    }

    /**
     * This method sets the maximum output value of the motor.
     *
     * @param maxOutput specifies the maximum output value.
     */
    public void setMaxOutput(double maxOutput)
    {
        this.maxOutput = maxOutput;
    }

    /**
     * This method returns the maximum output value of the motor.
     *
     * @return maximum output value.
     */
    public double getMaxOutput()
    {
        if (debugEnabled)
        {
            final String funcName = "setMaxOutput";
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "maxOutput=%f", maxOutput);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        return maxOutput;
    }

    /**
     * Get a list of all the supported drive modes.
     *
     * @return A list containing all the drive modes that this drive base supports.
     */
    public List<DriveMode> getSupportedDriveModes()
    {
        return Arrays.asList(DriveMode.CURVE_MODE, DriveMode.ARCADE_MODE, DriveMode.TANK_MODE);
    }

    /**
     * Does this drive base instance support this drive mode?
     *
     * @param driveMode DriveMode object to check.
     * @return Whether the drive mode is supported.
     */
    public boolean supportsDriveMode(DriveMode driveMode)
    {
        return getSupportedDriveModes().contains(driveMode);
    }

    /**
     * This method sets a motor power mapper. If null, it unsets the previously set mapper.
     *
     * @param motorPowerMapper specifies the motor power mapper. If null, clears the mapper.
     */
    public void setMotorPowerMapper(MotorPowerMapper motorPowerMapper)
    {
        throw new UnsupportedOperationException("Not supported!");
    }

    /**
     * This method sets the sensitivity for the drive() method.
     *
     * @param sensitivity specifies the sensitivity value.
     */
    public void setSensitivity(double sensitivity)
    {
        if (debugEnabled)
        {
            final String funcName = "setSensitivity";
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "sensitivity=%f", sensitivity);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        this.sensitivity = sensitivity;
    }

    /**
     * This method gets the sensitivity for the drive() method.
     *
     * @return sensitivity value.
     */
    public double getSensitivity()
    {
        return sensitivity;
    }

    /**
     * This method resets the drive base position odometry. This includes the motor encoders, the gyro heading and
     * all the cached values.
     */
    public void resetPosition()
    {
        resetPosition(false);
    }

    /**
     * This method returns the X position in scaled unit.
     *
     * @return X position.
     */
    public double getXPosition()
    {
        throw new UnsupportedOperationException("Not supported!");
    }

    /**
     * This method returns the drive base speed in the X direction.
     *
     * @return X speed.
     */
    public double getXSpeed()
    {
        throw new UnsupportedOperationException("Not supported!");
    }

    /**
     * This method sets the X position scale. The raw position from the encoder is in encoder counts. By setting the
     * scale factor, one could make getPosition to return unit in inches, for example.
     *
     * @param scale specifies the X position scale.
     */
    public void setXPositionScale(double scale)
    {
        throw new UnsupportedOperationException("Not supported!");
    }

    /**
     * Stop the drivebase and reset position and cached values.
     */
    public void reset()
    {
        stop();
        resetPosition();
    }

    /**
     * This method drives the motors at "magnitude" and "curve". Both magnitude and curve are -1.0 to +1.0 values,
     * where 0.0 represents stopped and not turning. curve less than 0 will turn left and curve greater than 0 will
     * turn right. The algorithm for steering provides a constant turn radius for any normal speed range, both
     * forward and backward. Increasing sensitivity causes sharper turns for fixed values of curve.
     *
     * @param magnitude specifies the speed setting for the outside wheel in a turn, forward or backwards, +1 to -1.
     * @param curve specifies the rate of turn, constant for different forward speeds. Set curve less than 0 for left
     *              turn or curve greater than 0 for right turn. Set curve = e^(-r/w) to get a turn radius r for
     *              wheel base w of your robot. Conversely, turn radius r = -ln(curve)*w for a given value of curve
     *              and wheel base w.
     * @param inverted specifies true to invert control (i.e. robot front becomes robot back).
     */
    public void drive(double magnitude, double curve, boolean inverted)
    {
        double leftOutput;
        double rightOutput;
        double sensitivity = getSensitivity();

        if (curve < 0.0)
        {
            double value = Math.log(-curve);
            double ratio = (value - sensitivity)/(value + sensitivity);
            if (ratio == 0.0)
            {
                ratio = 0.0000000001;
            }
            leftOutput = magnitude/ratio;
            rightOutput = magnitude;
        }
        else if (curve > 0.0)
        {
            double value = Math.log(curve);
            double ratio = (value - sensitivity)/(value + sensitivity);
            if (ratio == 0.0)
            {
                ratio = 0.0000000001;
            }
            leftOutput = magnitude;
            rightOutput = magnitude/ratio;
        }
        else
        {
            leftOutput = magnitude;
            rightOutput = magnitude;
        }

        tankDrive(leftOutput, rightOutput, inverted);
    }

    /**
     * This method drives the motors with the given magnitude and curve values.
     *
     * @param magnitude specifies the magnitude value.
     * @param curve specifies the curve value.
     */
    public void drive(double magnitude, double curve)
    {
        drive(magnitude, curve, false);
    }   //drive

    /**
     * This method implements tank drive where leftPower controls the left motors and right power controls the right
     * motors.
     *
     * @param leftPower specifies left power value.
     * @param rightPower specifies right power value.
     * @param inverted specifies true to invert control (i.e. robot front becomes robot back).
     */
    public abstract void tankDrive(double leftPower, double rightPower, boolean inverted);

    /**
     * This method implements tank drive where leftPower controls the left motors and right power controls the right
     * motors.
     *
     * @param leftPower specifies left power value.
     * @param rightPower specifies right power value.
     */
    public void tankDrive(double leftPower, double rightPower)
    {
        tankDrive(leftPower, rightPower, false);
    }   //tankDrive

    /**
     * This method implements arcade drive where drivePower controls how fast the robot goes in the y-axis and
     * turnPower controls how fast it will turn.
     *
     * @param drivePower specifies the drive power value.
     * @param turnPower specifies the turn power value.
     * @param inverted specifies true to invert control (i.e. robot front becomes robot back).
     */
    public void arcadeDrive(double drivePower, double turnPower, boolean inverted)
    {
        double leftPower;
        double rightPower;

        drivePower = TrcUtil.clipRange(drivePower);
        turnPower = TrcUtil.clipRange(turnPower);

        leftPower = drivePower + turnPower;
        rightPower = drivePower - turnPower;
        double maxMag = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (maxMag > 1.0)
        {
            leftPower /= maxMag;
            rightPower /= maxMag;
        }

        tankDrive(leftPower, rightPower, inverted);
    }

    /**
     * This method implements arcade drive where drivePower controls how fast the robot goes in the y-axis and
     * turnPower controls how fast it will turn.
     *
     * @param drivePower specifies the drive power value.
     * @param turnPower specifies the turn power value.
     */
    public void arcadeDrive(double drivePower, double turnPower)
    {
        arcadeDrive(drivePower, turnPower, false);
    }   //arcadeDrive

    /**
     * This method implements mecanum drive where x controls how fast the robot will go in the x direction, and y
     * controls how fast the robot will go in the y direction. Rotation controls how fast the robot rotates and
     * gyroAngle specifies the heading the robot should maintain.
     *
     * @param x specifies the x power.
     * @param y specifies the y power.
     * @param rotation specifies the rotating power.
     * @param inverted specifies true to invert control (i.e. robot front becomes robot back).
     * @param gyroAngle specifies the gyro angle to maintain.
     */
    public void mecanumDrive_Cartesian(double x, double y, double rotation, boolean inverted, double gyroAngle)
    {
        throw new UnsupportedOperationException("Not supported!");
    }

    /**
     * This method implements mecanum drive where x controls how fast the robot will go in the x direction, and y
     * controls how fast the robot will go in the y direction. Rotation controls how fast the robot rotates and
     * gyroAngle specifies the heading the robot should maintain.
     *
     * @param x specifies the x power.
     * @param y specifies the y power.
     * @param rotation specifies the rotating power.
     * @param inverted specifies true to invert control (i.e. robot front becomes robot back).
     */
    public void mecanumDrive_Cartesian(double x, double y, double rotation, boolean inverted)
    {
        mecanumDrive_Cartesian(x, y, rotation, inverted, 0.0);
    }   //mecanumDrive_Cartesian

    /**
     * This method implements mecanum drive where x controls how fast the robot will go in the x direction, and y
     * controls how fast the robot will go in the y direction. Rotation controls how fast the robot rotates.
     *
     * @param x specifies the x power.
     * @param y specifies the y power.
     * @param rotation specifies the rotating power.
     */
    public void mecanumDrive_Cartesian(double x, double y, double rotation)
    {
        mecanumDrive_Cartesian(x, y, rotation, false, 0.0);
    }   //mecanumDrive_Cartesian

    /**
     * This method implements mecanum drive where magnitude controls how fast the robot will go in the given direction
     * and how fast it will rotate.
     *
     * @param magnitude specifies the magnitude combining x and y axes.
     * @param direction specifies the direction in degrees.
     * @param rotation specifies the rotation power.
     * @param inverted specifies true to invert control (i.e. robot front becomes robot back).
     */
    public void mecanumDrive_Polar(double magnitude, double direction, double rotation, boolean inverted)
    {
        throw new UnsupportedOperationException("Not supported!");
    }

    /**
     * This method implements mecanum drive where magnitude controls how fast the robot will go in the given direction
     * and how fast it will rotate.
     *
     * @param magnitude specifies the magnitude combining x and y axes.
     * @param direction specifies the direction in degrees.
     * @param rotation specifies the rotation power.
     */
    public void mecanumDrive_Polar(double magnitude, double direction, double rotation)
    {
        mecanumDrive_Polar(magnitude, direction, rotation, false);
    }   //mecanumDrive_Polar

    /**
     * This method implements swerve drive where x controls how fast the robot will go in the x direction, and y
     * controls how fast the robot will go in the y direction. Rotation controls how fast the robot rotates and
     * gyroAngle is the current gyro reading (for driving with the field reference frame). If you want to drive with the
     * robot (local) reference frame, leave gyroAngle as 0.
     *
     * @param x specifies the x power.
     * @param y specifies the y power.
     * @param rotation specifies the rotating power.
     * @param inverted specifies true to invert control (i.e. robot front becomes robot back).
     * @param gyroAngle specifies the current gyro reading. Used to drive with field reference frame. Leave 0.0 for
     *                  robot reference frame.
     */
    public void swerveDrive_Cartesian(double x, double y, double rotation, boolean inverted, double gyroAngle)
    {
        throw new UnsupportedOperationException("Not supported!");
    }

    /**
     * This method implements swerve drive where x controls how fast the robot will go in the x direction, and y
     * controls how fast the robot will go in the y direction. Rotation controls how fast the robot rotates.
     *
     * @param x specifies the x power.
     * @param y specifies the y power.
     * @param rotation specifies the rotating power.
     * @param inverted specifies true to invert control (i.e. robot front becomes robot back).
     */
    public void swerveDrive_Cartesian(double x, double y, double rotation, boolean inverted)
    {
        swerveDrive_Cartesian(x, y, rotation, inverted, 0.0);
    }

    /**
     * This method implements swerve drive where x controls how fast the robot will go in the x direction, and y
     * controls how fast the robot will go in the y direction. Rotation controls how fast the robot rotates.
     *
     * @param x specifies the x power.
     * @param y specifies the y power.
     * @param rotation specifies the rotating power.
     */
    public void swerveDrive_Cartesian(double x, double y, double rotation)
    {
        swerveDrive_Cartesian(x, y, rotation, false);
    }

    /**
     * This method implements swerve drive where magnitude controls how fast the robot will go in the given direction
     * and how fast it will rotate.
     *
     * @param magnitude specifies the magnitude combining x and y axes.
     * @param direction specifies the direction in degrees.
     * @param rotation specifies the rotation power.
     * @param inverted specifies true to invert control (i.e. robot front becomes robot back).
     * @param gyroAngle specifies the current gyro reading. Used to drive with field reference frame. Leave 0.0 for
     *                  robot reference frame.
     */
    public void swerveDrive_Polar(double magnitude, double direction, double rotation, boolean inverted, double gyroAngle)
    {
        direction = Math.toRadians(direction);
        double x = Math.cos(direction) * magnitude;
        double y = Math.sin(direction) * magnitude;
        swerveDrive_Cartesian(x, y, rotation, inverted, gyroAngle);
    }

    /**
     * This method implements swerve drive where magnitude controls how fast the robot will go in the given direction
     * and how fast it will rotate.
     *
     * @param magnitude specifies the magnitude combining x and y axes.
     * @param direction specifies the direction in degrees.
     * @param rotation specifies the rotation power.
     * @param inverted specifies true to invert control (i.e. robot front becomes robot back).
     */
    public void swerveDrive_Polar(double magnitude, double direction, double rotation, boolean inverted)
    {
        swerveDrive_Polar(magnitude, direction, rotation, inverted, 0.0);
    }

    /**
     * This method implements swerve drive where magnitude controls how fast the robot will go in the given direction
     * and how fast it will rotate.
     *
     * @param magnitude specifies the magnitude combining x and y axes.
     * @param direction specifies the direction in degrees.
     * @param rotation specifies the rotation power.
     */
    public void swerveDrive_Polar(double magnitude, double direction, double rotation)
    {
        swerveDrive_Polar(magnitude, direction, rotation, false, 0.0);
    }
}

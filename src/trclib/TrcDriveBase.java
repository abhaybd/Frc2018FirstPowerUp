package trclib;

import java.util.List;

public interface TrcDriveBase
{
    enum DriveMode
    {
        CURVE_MODE,
        ARCADE_MODE,
        TANK_MODE,
        MECANUM_MODE,
        SWERVE_MODE
    }

    // TODO: Add more methods to make this useful (comparable to TrcCommonDriveBase)

    List<DriveMode> getSupportedDriveModes();

    void stop();

    void resetPosition();

    double getHeading();

    double getYPosition();

    double getYSpeed();

    default double getXPosition()
    {
        throw new UnsupportedOperationException("Not supported!");
    }

    default double getXSpeed()
    {
        throw new UnsupportedOperationException("Not supported!");
    }

    default void reset()
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
    default void drive(double magnitude, double curve, boolean inverted)
    {
        throw new UnsupportedOperationException("Not supported!");
    }

    /**
     * This method drives the motors with the given magnitude and curve values.
     *
     * @param magnitude specifies the magnitude value.
     * @param curve specifies the curve value.
     */
    default void drive(double magnitude, double curve)
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
    default void tankDrive(double leftPower, double rightPower, boolean inverted)
    {
        throw new UnsupportedOperationException("Not supported!");
    }

    /**
     * This method implements tank drive where leftPower controls the left motors and right power controls the right
     * motors.
     *
     * @param leftPower specifies left power value.
     * @param rightPower specifies right power value.
     */
    default void tankDrive(double leftPower, double rightPower)
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
    default void arcadeDrive(double drivePower, double turnPower, boolean inverted)
    {
        throw new UnsupportedOperationException("Not supported!");
    }

    /**
     * This method implements arcade drive where drivePower controls how fast the robot goes in the y-axis and
     * turnPower controls how fast it will turn.
     *
     * @param drivePower specifies the drive power value.
     * @param turnPower specifies the turn power value.
     */
    default void arcadeDrive(double drivePower, double turnPower)
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
    default void mecanumDrive_Cartesian(double x, double y, double rotation, boolean inverted, double gyroAngle)
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
    default void mecanumDrive_Cartesian(double x, double y, double rotation, boolean inverted)
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
    default void mecanumDrive_Cartesian(double x, double y, double rotation)
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
    default void mecanumDrive_Polar(double magnitude, double direction, double rotation, boolean inverted)
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
    default void mecanumDrive_Polar(double magnitude, double direction, double rotation)
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
    default void swerveDrive_Cartesian(double x, double y, double rotation, boolean inverted, double gyroAngle)
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
    default void swerveDrive_Cartesian(double x, double y, double rotation, boolean inverted)
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
    default void swerveDrive_Cartesian(double x, double y, double rotation)
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
    default void swerveDrive_Polar(double magnitude, double direction, double rotation, boolean inverted, double gyroAngle)
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
    default void swerveDrive_Polar(double magnitude, double direction, double rotation, boolean inverted)
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
    default void swerveDrive_Polar(double magnitude, double direction, double rotation)
    {
        swerveDrive_Polar(magnitude, direction, rotation, false, 0.0);
    }
}

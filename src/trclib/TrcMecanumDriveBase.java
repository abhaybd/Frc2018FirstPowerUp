/*
 * Copyright (c) 2018 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package trclib;

/**
 * This class implements a platform independent mecanum drive base. A mecanum drive base consists of 4 motor driven
 * wheels. It extends the TrcSimpleDriveBase class so it inherits all the SimpleDriveBase methods and features.
 */
public class TrcMecanumDriveBase extends TrcSimpleDriveBase
{
    private double k;
    private double maxXSpeed;
    private double maxYSpeed;
    private double maxAngularVelocity;
    private double maxMotorSpeed;
    private boolean kinematicDriveEnabled = false;

    /**
     * Constructor: Create an instance of the 4-wheel mecanum drive base.
     *
     * @param leftFrontMotor  specifies the left front motor of the drive base.
     * @param leftRearMotor   specifies the left rear motor of the drive base.
     * @param rightFrontMotor specifies the right front motor of the drive base.
     * @param rightRearMotor  specifies the right rear motor of the drive base.
     * @param gyro            specifies the gyro. If none, it can be set to null.
     */
    public TrcMecanumDriveBase(TrcMotorController leftFrontMotor, TrcMotorController leftRearMotor,
        TrcMotorController rightFrontMotor, TrcMotorController rightRearMotor, TrcGyro gyro)
    {
        super(leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor, gyro);
    }   //TrcMecanumDriveBase

    /**
     * Constructor: Create an instance of the 4-wheel mecanum drive base.
     *
     * @param leftFrontMotor  specifies the left front motor of the drive base.
     * @param leftRearMotor   specifies the left rear motor of the drive base.
     * @param rightFrontMotor specifies the right front motor of the drive base.
     * @param rightRearMotor  specifies the right rear motor of the drive base.
     */
    public TrcMecanumDriveBase(TrcMotorController leftFrontMotor, TrcMotorController leftRearMotor,
        TrcMotorController rightFrontMotor, TrcMotorController rightRearMotor)
    {
        super(leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor, null);
    }   //TrcMecanumDriveBase

    /**
     * This method checks if it supports holonomic drive.
     *
     * @return true if this drive base supports holonomic drive, false otherwise.
     */
    @Override
    public boolean supportsHolonomicDrive()
    {
        return true;
    }   //supportsHolonomicDrive

    /**
     * Starts using "smart kinematics", shown by a paper by Ether:
     * http://www.chiefdelphi.com/media/papers/download/2722
     * The units don't matter, as long as they are consistent.
     * wheelBase, trackWidth, maxXSpeed, maxYSpeed, and maxMotorSpeed should all have the same distance unit.
     * All time units should be consistent. The numerator of maxAngularSpeed should be in radians.
     *
     * @param wheelBase       Distance between the front and back wheels.
     * @param trackWidth      Distance between the left and right wheels.
     * @param maxXSpeed       Max speed of the robot in the x axis
     * @param maxYSpeed       Max speed of the robot in the y axis.
     * @param maxAngularSpeed Max rotation speed of the robot with the numerator as radians.
     * @param maxMotorSpeed   Max tangential speed of the drive motor
     */
    public void enableKinematicDrive(double wheelBase, double trackWidth, double maxXSpeed, double maxYSpeed,
        double maxAngularSpeed, double maxMotorSpeed)
    {
        this.k = TrcUtil.average(wheelBase, trackWidth);
        this.maxXSpeed = maxXSpeed;
        this.maxYSpeed = maxYSpeed;
        this.maxAngularVelocity = maxAngularSpeed;
        this.maxMotorSpeed = maxMotorSpeed;
        kinematicDriveEnabled = true;
    }

    public void disableKinematicDrive()
    {
        kinematicDriveEnabled = false;
    }

    /**
     * This method implements holonomic drive where x controls how fast the robot will go in the x direction, and y
     * controls how fast the robot will go in the y direction. Rotation controls how fast the robot rotates and
     * gyroAngle specifies the heading the robot should maintain.
     *
     * @param x         specifies the x power.
     * @param y         specifies the y power.
     * @param rotation  specifies the rotating power.
     * @param inverted  specifies true to invert control (i.e. robot front becomes robot back).
     * @param gyroAngle specifies the gyro angle to maintain.
     */
    @Override
    protected void holonomicDrive(double x, double y, double rotation, boolean inverted, double gyroAngle)
    {
        if (kinematicDriveEnabled)
        {
            smartKinematicsDrive(x, y, rotation, inverted, gyroAngle);
        }
        else
        {
            regularMecanumDrive(x, y, rotation, inverted, gyroAngle);
        }
    }   //holonomicDrive

    private void smartKinematicsDrive(double x, double y, double rotation, boolean inverted, double gyroAngle)
    {
        final String funcName = "holonomicDrive";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "x=%f,y=%f,rot=%f,inverted=%s,angle=%f", x, y,
                rotation, Boolean.toString(inverted), gyroAngle);
        }

        double[] transformedCommands = transformCommands(x, y, inverted, gyroAngle);
        x = transformedCommands[0];
        y = transformedCommands[1];

        x *= maxXSpeed;
        y *= maxYSpeed;
        rotation *= maxAngularVelocity;

        // The units are in dist/time
        // dist is the same distance unit used to define the max speeds and robot dimensions
        // time is the same time unit used to define the max speeds
        double lfSpeed = x + y + k * rotation;
        double rfSpeed = -x + y - k * rotation;
        double lrSpeed = -x + y + k * rotation;
        double rrSpeed = x + y - k * rotation;

        leftFrontMotor.set(lfSpeed / maxMotorSpeed);
        rightFrontMotor.set(rfSpeed / maxMotorSpeed);
        leftRearMotor.set(lrSpeed / maxMotorSpeed);
        rightRearMotor.set(rrSpeed / maxMotorSpeed);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }

    private void regularMecanumDrive(double x, double y, double rotation, boolean inverted, double gyroAngle)
    {
        final String funcName = "holonomicDrive";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "x=%f,y=%f,rot=%f,inverted=%s,angle=%f", x, y,
                rotation, Boolean.toString(inverted), gyroAngle);
        }

        x = TrcUtil.clipRange(x);
        y = TrcUtil.clipRange(y);
        rotation = TrcUtil.clipRange(rotation);

        double[] transformedCommands = transformCommands(x, y, inverted, gyroAngle);
        x = transformedCommands[0];
        y = transformedCommands[1];

        if (isGyroAssistEnabled())
        {
            rotation += getGyroAssistPower(rotation);
        }

        double wheelPowers[] = new double[4];
        wheelPowers[MotorType.LEFT_FRONT.value] = x + y + rotation;
        wheelPowers[MotorType.RIGHT_FRONT.value] = -x + y - rotation;
        wheelPowers[MotorType.LEFT_REAR.value] = -x + y + rotation;
        wheelPowers[MotorType.RIGHT_REAR.value] = x + y - rotation;
        TrcUtil.normalizeInPlace(wheelPowers);

        double wheelPower;

        wheelPower = motorPowerMapper
            .translateMotorPower(wheelPowers[MotorType.LEFT_FRONT.value], leftFrontMotor.getSpeed());
        leftFrontMotor.set(wheelPower);

        wheelPower = motorPowerMapper
            .translateMotorPower(wheelPowers[MotorType.RIGHT_FRONT.value], rightFrontMotor.getSpeed());
        rightFrontMotor.set(wheelPower);

        wheelPower = motorPowerMapper
            .translateMotorPower(wheelPowers[MotorType.LEFT_REAR.value], leftRearMotor.getSpeed());
        leftRearMotor.set(wheelPower);

        wheelPower = motorPowerMapper
            .translateMotorPower(wheelPowers[MotorType.RIGHT_REAR.value], rightRearMotor.getSpeed());
        rightRearMotor.set(wheelPower);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }

    /**
     * This method is called periodically to monitor the encoders to update the odometry data.
     */
    @Override
    protected void updateOdometry()
    {
        super.updateOdometry();

        updateXOdometry(TrcUtil.average(lfEnc, -rfEnc, -lrEnc, rrEnc),
            TrcUtil.average(lfSpeed, -rfSpeed, -lrSpeed, rrSpeed));

        if (kinematicDriveEnabled)
        {
            // According to the paper by Ether.
            updateRotationOdometry(TrcUtil.average(-lfEnc, rfEnc, -lrEnc, rrEnc) / k);
        }
    }   //updateOdometry

}   //class TrcMecanumDriveBase

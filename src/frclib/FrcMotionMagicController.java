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

package frclib;

import com.ctre.phoenix.motorcontrol.ControlMode;

import trclib.TrcDbgTrace;
import trclib.TrcEvent;
import trclib.TrcPidController;
import trclib.TrcRobot;
import trclib.TrcTaskMgr;
import trclib.TrcUtil;

/**
 * This class implements motion magic using the TalonSRX. This is mainly used for linear movement ONLY in the y axis.
 * It cannot be used in conjunction with x-axis movement or rotation. It uses the motion magic features of the TalonSRX
 * to run high resolution closed loop control using trapezoidal velocity profiles. This should result in more accurate
 * and more constrained motion than regular PID control. It is more constrained as you can set a real-world maximum
 * velocity and acceleration of the robot.
 */

public class FrcMotionMagicController
{
    private TrcPidController.PidCoefficients leftCoefficients = null, rightCoefficients = null;
    private FrcCANTalon leftMaster, rightMaster;
    private int pidSlot;
    private double worldUnitsPerTick;
    private TrcEvent onFinishedEvent;
    private TrcTaskMgr.TaskObject motionMagicTaskObj;
    private boolean running = false;
    private boolean cancelled = false;
    /**
     * Sensor units.
     */
    private double errorTolerance;
    /**
     * Sensor units.
     */
    private double targetPos;
    /**
     * Sensor units per 100ms.
     */
    private int maxVelocity;
    /**
     * Sensor units per 100ms^2
     */
    private int maxAcceleration;

    /**
     * Creates a motion magic controller with a default pid slot of 0.
     *
     * @param instanceName      The name of this instance.
     * @param worldUnitsPerTick The number of world units per encoder tick. This can be inches/tick, cm/tick, etc.
     *                          Whatever is used, MAKE SURE TO BE CONSISTENT.
     * @param maxVelocity       The maximum speed the robot should go during a move operation.
     *                          The robot may not reach this speed. This should be in world units per second.
     * @param maxAcceleration   The maximum acceleration of the robot during a move operation.
     *                          The robot may not reach this acceleration. This should be in world units per second per second.
     * @param errorTolerance    The tolerance of error, in world units. If the closed loop error is less than or equal to
     *                          the tolerance, the move operation will be finished.
     */
    public FrcMotionMagicController(String instanceName, double worldUnitsPerTick, double maxVelocity,
        double maxAcceleration, double errorTolerance)
    {
        this(instanceName, worldUnitsPerTick, maxVelocity, maxAcceleration, errorTolerance, 0);
    }

    /**
     * Creates a motion magic controller with a default pid slot of 0.
     *
     * @param instanceName      The name of this instance.
     * @param worldUnitsPerTick The number of world units per encoder tick. This can be inches/tick, cm/tick, etc.
     *                          Whatever is used, MAKE SURE TO BE CONSISTENT.
     * @param maxVelocity       The maximum speed the robot should go during a move operation.
     *                          The robot may not reach this speed. This should be in world units per second.
     * @param maxAcceleration   The maximum acceleration of the robot during a move operation.
     *                          The robot may not reach this acceleration. This should be in world units per second per second.
     * @param errorTolerance    The tolerance of error, in world units. If the closed loop error is less than or equal to
     *                          the tolerance, the move operation will be finished.
     * @param pidSlot           The pid slot to use for the TalonSRX. 0 is the main pid controller, 1 is the auxiliary.
     */
    public FrcMotionMagicController(String instanceName, double worldUnitsPerTick, double maxVelocity,
        double maxAcceleration, double errorTolerance, int pidSlot)
    {
        this.worldUnitsPerTick = worldUnitsPerTick;
        this.pidSlot = pidSlot;
        // Convert to encoder units
        this.errorTolerance = Math.abs(errorTolerance) / worldUnitsPerTick;
        // Scale velocity and acceleration to encoder units and time frame of 100ms
        this.maxVelocity = TrcUtil.round(0.1 * maxVelocity / worldUnitsPerTick);
        // For some reason CTRE is dumb, so acceleration is ticks/100ms/1sec. God I hate these people.
        this.maxAcceleration = TrcUtil.round(0.1 * maxAcceleration / worldUnitsPerTick);

        this.motionMagicTaskObj = TrcTaskMgr.getInstance().createTask(instanceName, this::motionMagicTask);
    }

    /**
     * Start motion magic move operation.
     *
     * @param targetPos The target position in world units to move to.
     */
    public void drive(double targetPos)
    {
        drive(targetPos, null);
    }

    /**
     * Start motion magic move operation.
     *
     * @param targetPos       The target position in world units to move to.
     * @param onFinishedEvent The event to signal when done.
     */
    public void drive(double targetPos, TrcEvent onFinishedEvent)
    {
        if (leftMaster == null || rightMaster == null)
        {
            throw new IllegalStateException("Cannot start before setting both left and right motors!");
        }

        if (leftCoefficients == null || rightCoefficients == null)
        {
            throw new IllegalStateException("Cannot start before setting the pid coefficients!");
        }

        // Convert target to encoder units
        targetPos /= worldUnitsPerTick;
        this.targetPos = targetPos;

        if (onFinishedEvent != null)
        {
            onFinishedEvent.clear();
        }
        this.onFinishedEvent = onFinishedEvent;

        configureTalon(leftMaster, leftCoefficients);
        configureTalon(rightMaster, rightCoefficients);

        TrcDbgTrace.getGlobalTracer().traceInfo("FrcMotionMagicController.drive", "driveDist: %.2f", targetPos);

        leftMaster.motor.set(ControlMode.MotionMagic, targetPos);
        rightMaster.motor.set(ControlMode.MotionMagic, targetPos);

        running = true;
        cancelled = false;
        setTaskEnabled(true);
    }

    /**
     * Sets the error tolerance. If the closed loop error is less than or equal to the tolerance,
     * the move operation will be finished.
     *
     * @param errorTolerance The new error tolerance, in world units.
     */
    public void setErrorTolerance(double errorTolerance)
    {
        this.errorTolerance = errorTolerance / worldUnitsPerTick;
    }

    /**
     * Cancel the current move operation.
     */
    public void cancel()
    {
        if (isRunning())
        {
            cancelled = true;
            if (onFinishedEvent != null)
            {
                onFinishedEvent.cancel();
            }
            stop();
        }
    }

    /**
     * Is there a current move operation in progress?
     *
     * @return True if the motion magic is running, false otherwise.
     */
    public boolean isRunning()
    {
        return running;
    }

    /**
     * Was the last move operation cancelled prematurely?
     *
     * @return True if the last move operation was cancelled, false otherwise.
     */
    public boolean isCancelled()
    {
        return cancelled;
    }

    /**
     * Get the target position in world units.
     *
     * @return The target position in world units.
     */
    public double getTargetPos()
    {
        return targetPos * worldUnitsPerTick;
    }

    /**
     * Get the closed-loop error, in world units.
     *
     * @return The closed loop error in world units.
     */
    public double getError()
    {
        return worldUnitsPerTick * getRawError();
    }

    /**
     * Set the PIDF coefficients. This will only apply for the next time start() is called. It will NOT affect a run
     * already in progress.
     *
     * @param pidCoefficients The PID coefficients to set.
     */
    public void setPidCoefficients(TrcPidController.PidCoefficients pidCoefficients)
    {
        setPidCoefficients(pidCoefficients, pidCoefficients);
    }

    public void setPidCoefficients(TrcPidController.PidCoefficients leftCoefficients,
        TrcPidController.PidCoefficients rightCoefficients)
    {
        this.leftCoefficients = leftCoefficients;
        this.rightCoefficients = rightCoefficients;
    }

    /**
     * Sets the motors on the left side of the drive train.
     *
     * @param leftMotors List of motors on the left side of the drive train. The first motor in the list will be used
     *                   as the master motor, and all others will be set as slaves.
     */
    public void setLeftMotors(FrcCANTalon... leftMotors)
    {
        if (leftMotors.length == 0)
        {
            throw new IllegalArgumentException("Cannot pass empty array of motors!");
        }

        this.leftMaster = leftMotors[0];

        if (leftMotors.length > 1)
        {
            for (int i = 1; i < leftMotors.length; i++)
            {
                leftMotors[i].motor.set(ControlMode.Follower, leftMaster.motor.getDeviceID());
            }
        }
    }

    /**
     * Sets the motors on the right side of the drive train.
     *
     * @param rightMotors List of motors on the right side of the drive train. The first motor in the list will be used
     *                    as the master motor, and all others will be set as slaves.
     */
    public void setRightMotors(FrcCANTalon... rightMotors)
    {
        if (rightMotors.length == 0)
        {
            throw new IllegalArgumentException("Cannot pass empty array of motors!");
        }

        this.rightMaster = rightMotors[0];

        if (rightMotors.length > 1)
        {
            for (int i = 1; i < rightMotors.length; i++)
            {
                rightMotors[i].motor.set(ControlMode.Follower, rightMaster.motor.getDeviceID());
            }
        }
    }

    private double getRawError()
    {
        double leftError = targetPos - leftMaster.motor.getSelectedSensorPosition(pidSlot);
        double rightError = targetPos - rightMaster.motor.getSelectedSensorPosition(pidSlot);
        TrcDbgTrace.getGlobalTracer()
            .traceInfo("FrcMotionMagicController.getRawError", "lError: %.2f, rError: %.2f", leftError, rightError);
        return TrcUtil.average(leftError, rightError);
    }

    private boolean isDone()
    {
        return running && Math.abs(getRawError()) <= errorTolerance;
    }

    private void stop()
    {
        setTaskEnabled(false);
        onFinishedEvent = null;
        running = false;
        targetPos = 0.0;
        // Stop the motors
        leftMaster.motor.neutralOutput();
        rightMaster.motor.neutralOutput();
    }

    private void configureTalon(FrcCANTalon talon, TrcPidController.PidCoefficients pidCoefficients)
    {
        talon.motor.config_kP(pidSlot, pidCoefficients.kP, 0);
        talon.motor.config_kI(pidSlot, pidCoefficients.kI, 0);
        talon.motor.config_kD(pidSlot, pidCoefficients.kD, 0);
        talon.motor.config_kF(pidSlot, pidCoefficients.kF, 0);
        talon.motor.config_IntegralZone(pidSlot, pidCoefficients.iZone, 0);

        talon.motor.configMotionCruiseVelocity(maxVelocity, 0);
        talon.motor.configMotionAcceleration(maxAcceleration, 0);

        talon.motor.setSelectedSensorPosition(0, 0, 10);
    }

    private void setTaskEnabled(boolean enabled)
    {
        if (enabled)
        {
            motionMagicTaskObj.registerTask(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
        }
        else
        {
            motionMagicTaskObj.unregisterTask(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
        }
    }

    private void motionMagicTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode mode)
    {
        if (isDone())
        {
            TrcDbgTrace.getGlobalTracer().traceInfo("FrcMotionMagicController.task", "Done!");
            if (onFinishedEvent != null)
            {
                onFinishedEvent.set(true);
            }
            stop();
        }
    }
}

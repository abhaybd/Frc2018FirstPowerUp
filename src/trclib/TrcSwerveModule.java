package trclib;

public class TrcSwerveModule
{
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final boolean useGlobalTracer = false;

    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;

    private TrcMotorController driveMotor, turnMotor;
    private TrcPidController turnPidCtrl;
    private double turnDegreesPerCount;
    private TrcTaskMgr.TaskObject turnTaskObj;
    private boolean trackingTurnAngle;
    private TrcWarpSpace warpSpace;
    private TrcDbgTrace dbgTrace;

    public TrcSwerveModule(String instanceName, TrcMotorController driveMotor, TrcMotorController turnMotor,
        double turnDegreesPerCount, TrcPidController.PidCoefficients turnPidCoefficients, double turnTolerance,
        double turnOutputLimit)
    {
        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;
        this.turnDegreesPerCount = turnDegreesPerCount;

        this.turnPidCtrl = new TrcPidController(instanceName + ".turnPidCtrl", turnPidCoefficients, turnTolerance,
            this::getAngle);
        this.turnPidCtrl.setAbsoluteSetPoint(true);
        this.turnPidCtrl.setTargetRange(0, 360);
        this.turnPidCtrl.setOutputLimit(turnOutputLimit);

        warpSpace = new TrcWarpSpace(instanceName + ".warpSpace", 0.0, 360.0);

        turnTaskObj = TrcTaskMgr.getInstance().createTask(instanceName + ".turnTask", this::turnTask);

        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer ?
                TrcDbgTrace.getGlobalTracer() :
                new TrcDbgTrace(instanceName, tracingEnabled, traceLevel, msgLevel);
        }
    }

    /**
     * Reset the encoder of the drive motor.
     *
     * @param hardware Whether or not to do a hardware or software reset.
     */
    public void resetPosition(boolean hardware)
    {
        driveMotor.resetPosition(hardware);
    }

    /**
     * Set the target angle for the turn motor. The turn motor will constantly try to keep this angle.
     *
     * @param angle The angle to set the turn motor to, in degrees, in the range [0,360).
     */
    public void setAngle(double angle)
    {
        angle = angle % 360;

        if (debugEnabled)
        {
            final String funcName = "setAngle";
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "angle=%f", angle);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        setEnabled(true);
        turnPidCtrl.setTarget(angle, warpSpace);
    }

    /**
     * The current angle of the turn motor. This is not necessarily the target angle.
     *
     * @return The angle of the turn motor, in degrees, in the range [0,360).
     */
    public double getAngle()
    {
        double angle = (turnMotor.getPosition() * turnDegreesPerCount) % 360.0;

        if (debugEnabled)
        {
            final String funcName = "getAngle";
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", angle);
        }

        return angle;
    }

    /**
     * Get the power being supplied to the turn motor.
     *
     * @return Power being supplied to the turn motor, in the range [-1,1].
     */
    public double getTurnPower()
    {
        double power = turnMotor.getPower();

        if (debugEnabled)
        {
            final String funcName = "getTurnPower";
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", power);
        }

        return power;
    }

    /**
     * Set the drive motor to drive with this much power.
     *
     * @param power Power to supply to drive motor, in the range [-1,1].
     */
    public void setDrivePower(double power)
    {
        if (debugEnabled)
        {
            final String funcName = "setDrivePower";
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "power=%f", power);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        driveMotor.setPower(power);
    }

    /**
     * Get the power being supplied to the drive motor.
     *
     * @return Power being supplied to the drive motor, in the range [-1,1].
     */
    public double getDrivePower()
    {
        double power = driveMotor.getPower();

        if (debugEnabled)
        {
            final String funcName = "getDrivePower";
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "=%f", power);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
        return power;
    }

    /**
     * The current speed of the drive motor.
     *
     * @return Speed of the drive motor in sensor units per second.
     */
    public double getDriveSpeed()
    {
        double speed = driveMotor.getSpeed();

        if (debugEnabled)
        {
            final String funcName = "getDriveSpeed";
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "=%f", speed);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        return speed;
    }

    /**
     * The current position of the drive motor.
     *
     * @return Position of the drive motor in sensor units.
     */
    public double getPosition()
    {
        double position = driveMotor.getPosition();

        if (debugEnabled)
        {
            final String funcName = "getPosition";
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "=%f", position);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        return position;
    }

    /**
     * Set the brake mode.
     *
     * @param enabled If true, the motors will brake when stopped. If false, they will coast instead.
     */
    public void setBrakeMode(boolean enabled)
    {
        if (debugEnabled)
        {
            final String funcName = "setBrakeMode";
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "brakeMode=%b", enabled);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        driveMotor.setBrakeModeEnabled(enabled);
    }

    /**
     * Stop the turn motor from following the target angle. Call <code>setAngle</code> again to re-enable.
     */
    public void stopTrackingTurnAngle()
    {
        if (debugEnabled)
        {
            final String funcName = "stopTrackingTurnAngle";
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        setEnabled(false);
    }

    /**
     * Whether or not the turn motor is following the target angle. This will be true if
     * <code>stopTrackingTurnAngle()</code> was called.
     *
     * @return True if the turn motor is following the target angle. False otherwise.
     */
    public boolean isTrackingTurnAngle()
    {
        if (debugEnabled)
        {
            final String funcName = "isTrackingTurnAngle";
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "=%b", trackingTurnAngle);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        return trackingTurnAngle;
    }

    /**
     * Enables or disables the turnTask. The turnTask is responsible for making sure the turn motor tracks the target angle.
     *
     * @param enabled Enable or disable the turnTask?
     */
    private void setEnabled(boolean enabled)
    {
        trackingTurnAngle = enabled;
        if (enabled)
        {
            turnTaskObj.registerTask(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
        }
        else
        {
            turnTaskObj.unregisterTask(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
        }
    }

    /**
     * It's the turn task. Pretty self explanatory. Also, you shouldn't be reading this because you don't need to.
     *
     * @param taskType What type of task? duh.
     * @param runMode  What mode is it being run in? duh.
     */
    private void turnTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        turnMotor.setPower(turnPidCtrl.isOnTarget() ? 0.0 : turnPidCtrl.getOutput());

        if (debugEnabled)
        {
            turnPidCtrl.printPidInfo(dbgTrace);
        }
    }
}

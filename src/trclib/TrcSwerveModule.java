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

    public TrcSwerveModule(String instanceName,
        TrcMotorController driveMotor, TrcMotorController turnMotor, double turnDegreesPerCount,
        TrcPidController.PidCoefficients turnPidCoefficients, double turnTolerance, double turnOutputLimit)
    {
        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;
        this.turnDegreesPerCount = turnDegreesPerCount;

        this.turnPidCtrl = new TrcPidController(instanceName + ".turnPidCtrl", turnPidCoefficients,
            turnTolerance, this::getAngle);
        this.turnPidCtrl.setAbsoluteSetPoint(true);
        this.turnPidCtrl.setTargetRange(0,360);
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

    public void resetPosition(boolean hardware)
    {
        driveMotor.resetPosition(hardware);
    }

    public void setAngle(double angle)
    {
        if (debugEnabled)
        {
            final String funcName = "setAngle";
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "angle=%f", angle);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        setEnabled(true);
        turnPidCtrl.setTarget(angle, warpSpace);
    }

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

    public boolean isTrackingTurnAngle()
    {
        if (debugEnabled)
        {
            final String funcName = "isTrackingTurnAngle";
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,"=%b", trackingTurnAngle);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        return trackingTurnAngle;
    }

    private void setEnabled(boolean enabled)
    {
        trackingTurnAngle = enabled;
        if(enabled)
        {
            turnTaskObj.registerTask(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
        } else
        {
            turnTaskObj.unregisterTask(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
        }
    }

    private void turnTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        turnMotor.setPower(turnPidCtrl.isOnTarget() ? 0.0 : turnPidCtrl.getOutput());

        if(debugEnabled)
        {
            turnPidCtrl.printPidInfo(dbgTrace);
        }
    }
}

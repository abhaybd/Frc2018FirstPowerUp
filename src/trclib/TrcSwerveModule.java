package trclib;

public class TrcSwerveModule
{
    private TrcMotorController driveMotor, turnMotor;
    private TrcPidController turnPidCtrl;
    private double turnDegreesPerCount;
    private TrcTaskMgr.TaskObject turnTaskObj;
    private boolean trackingTurnAngle;
    private TrcWarpSpace warpSpace;
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
    }

    public void resetPosition(boolean hardware)
    {
        driveMotor.resetPosition(hardware);
    }

    public void setAngle(double angle)
    {
        setEnabled(true);
        turnPidCtrl.setTarget(angle, warpSpace);
    }

    public double getAngle()
    {
        return (turnMotor.getPosition() * turnDegreesPerCount) % 360.0;
    }

    public double getTurnPower()
    {
        return turnMotor.getPower();
    }

    public void setDrivePower(double power)
    {
        driveMotor.setPower(power);
    }

    public double getDrivePower()
    {
        return driveMotor.getPower();
    }

    public double getDriveSpeed()
    {
        return driveMotor.getSpeed();
    }

    public double getPosition()
    {
        return driveMotor.getPosition();
    }

    public void setBrakeMode(boolean enabled)
    {
        driveMotor.setBrakeModeEnabled(enabled);
    }

    /**
     * Stop the turn motor from following the target angle. Call <code>setAngle</code> again to re-enable.
     */
    public void stopTrackingTurnAngle()
    {
        setEnabled(false);
    }

    public boolean isTrackingTurnAngle(){
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
    }
}

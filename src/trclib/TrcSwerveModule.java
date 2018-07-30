package trclib;

public class TrcSwerveModule
{
    private TrcMotorController driveMotor, turnMotor;
    private TrcPidController turnPidCtrl;
    private double turnDegreesPerCount;
    private TrcTaskMgr.TaskObject turnTaskObj;
    private boolean trackingTurnAngle;
    public TrcSwerveModule(String instanceName,
        TrcMotorController driveMotor, TrcMotorController turnMotor, TrcPidController turnPidCtrl, double turnDegreesPerCount)
    {
        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;
        this.turnDegreesPerCount = turnDegreesPerCount;
        this.turnPidCtrl = turnPidCtrl;

        turnTaskObj = TrcTaskMgr.getInstance().createTask(instanceName + ".turnTask", this::turnTask);
    }

    public void setAngle(double angle)
    {
        setEnabled(true);
        turnPidCtrl.setTarget(angle / turnDegreesPerCount);
    }

    public double getAngle()
    {
        return turnMotor.getPosition() * turnDegreesPerCount;
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

    public double getPosition()
    {
        return driveMotor.getPosition();
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

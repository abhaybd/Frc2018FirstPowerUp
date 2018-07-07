package trclib;

public class TrcSpeedDrive
{
    private String instanceName;
    private TrcDriveBase driveBase;
    private TrcTaskMgr.TaskObject speedTaskObj;
    private TrcPidController xSpeedController, ySpeedController, turnSpeedController;
    private boolean enabled, cancelled;
    public TrcSpeedDrive(String instanceName, TrcDriveBase driveBase, TrcPidController ySpeedController)
    {
        this(instanceName, driveBase, null, ySpeedController, null);
    }

    public TrcSpeedDrive(String instanceName, TrcDriveBase driveBase,
                         TrcPidController ySpeedController,
                         TrcPidController turnSpeedController)
    {
        this(instanceName, driveBase, null, ySpeedController, turnSpeedController);
    }

    public TrcSpeedDrive(String instanceName, TrcDriveBase driveBase,
                         TrcPidController xSpeedController,
                         TrcPidController ySpeedController,
                         TrcPidController turnSpeedController)
    {
        this.instanceName = instanceName;
        this.driveBase = driveBase;
        this.xSpeedController = xSpeedController;
        this.ySpeedController = ySpeedController;
        this.turnSpeedController = turnSpeedController;

        TrcTaskMgr taskMgr = TrcTaskMgr.getInstance();
        speedTaskObj = taskMgr.createTask(instanceName + ".speedTask", this::speedTask);
    }

    public boolean isActive()
    {
        return enabled;
    }

    public boolean isCancelled()
    {
        return cancelled;
    }

    public void cancel()
    {
        this.cancelled = true;
        stop();
    }

    public double xTargetSpeed()
    {
        return xSpeedController.getTarget();
    }

    public double yTargetSpeed()
    {
        return ySpeedController.getTarget();
    }

    public double turnTargetSpeed()
    {
        return turnSpeedController.getTarget();
    }

    public void setSpeed(double xSpeed, double ySpeed, double turnSpeed)
    {
        if(xSpeedController != null) xSpeedController.setTarget(xSpeed);
        if(ySpeedController != null) ySpeedController.setTarget(ySpeed);
        if(turnSpeedController != null) turnSpeedController.setTarget(turnSpeed);

        cancelled = false;
        setTaskEnabled(true);
    }

    private void stop()
    {
        setTaskEnabled(false);
        if(xSpeedController != null) xSpeedController.reset();
        if(ySpeedController != null) ySpeedController.reset();
        if(turnSpeedController != null) turnSpeedController.reset();
        driveBase.stop();
    }

    private void setTaskEnabled(boolean enabled)
    {
        this.enabled = enabled;
        if(enabled)
        {
            speedTaskObj.registerTask(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
        } else
        {
            speedTaskObj.unregisterTask(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
        }
    }

    private void speedTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        double xOutput = xSpeedController == null ? 0.0 : xSpeedController.getOutput();
        double yOutput = ySpeedController == null ? 0.0 : ySpeedController.getOutput();
        double turnOutput = turnSpeedController == null ? 0.0 : turnSpeedController.getOutput();

        if(xOutput != 0.0)
        {
            driveBase.mecanumDrive_Cartesian(xOutput, yOutput, turnOutput);
        } else
        {
            driveBase.arcadeDrive(yOutput, turnOutput);
        }
    }
}

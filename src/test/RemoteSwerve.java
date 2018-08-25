package test;

import trclib.*;

public class RemoteSwerve
{
    private final TrcGyro gyro;
    private final TrcSwerveModule lfModule;
    private final TrcSwerveModule rfModule;
    private final TrcSwerveModule lrModule;
    private final TrcSwerveModule rrModule;
    private final TrcSwerveDriveBase driveBase;
    private final TrcTaskMgr taskMgr;

    public RemoteSwerve()
    {
        double width = 39.0;
        double length = 39.0;

        double turnDegreesPerCount = 1.0;
        TrcPidController.PidCoefficients pidCoefficients = new TrcPidController.PidCoefficients(0.02);
        double turnTolerance = 1;
        double turnOutputLimit = 1;

        gyro = new MockGyro("Gyro");

        lfModule = new TrcSwerveModule("lfModule", new MockMotorController(3600), new MockMotorController(720),
            turnDegreesPerCount, pidCoefficients, turnTolerance, turnOutputLimit);
        rfModule = new TrcSwerveModule("rfModule", new MockMotorController(3600), new MockMotorController(720),
            turnDegreesPerCount, pidCoefficients, turnTolerance, turnOutputLimit);
        lrModule = new TrcSwerveModule("lrModule", new MockMotorController(3600), new MockMotorController(720),
            turnDegreesPerCount, pidCoefficients, turnTolerance, turnOutputLimit);
        rrModule = new TrcSwerveModule("rrModule", new MockMotorController(3600), new MockMotorController(720),
            turnDegreesPerCount, pidCoefficients, turnTolerance, turnOutputLimit);

        driveBase = new TrcSwerveDriveBase("SwerveDrive", width, length, lfModule, rfModule,
            lrModule, rrModule, gyro);

        taskMgr = TrcTaskMgr.getInstance();
    }

    public SwerveStatus getStatus(double x, double y, double turn, double gyroAngle)
    {
        taskMgr.executeTaskType(TrcTaskMgr.TaskType.PRECONTINUOUS_TASK, TrcRobot.RunMode.TELEOP_MODE);

        driveBase.holonomicDrive(x, y, turn, gyroAngle);

        SwerveStatus status = new SwerveStatus();
        status.lfPower = (float) lfModule.getDrivePower();
        status.rfPower = (float) rfModule.getDrivePower();
        status.lrPower = (float) lrModule.getDrivePower();
        status.rrPower = (float) rrModule.getDrivePower();

        status.lfAngle = (float) lfModule.getTargetAngle();
        status.rfAngle = (float) rfModule.getTargetAngle();
        status.lrAngle = (float) lrModule.getTargetAngle();
        status.rrAngle = (float) rrModule.getTargetAngle();

        taskMgr.executeTaskType(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK, TrcRobot.RunMode.TELEOP_MODE);

        return status;
    }

    private static class SwerveStatus
    {
        public float lfPower, lfAngle;
        public float rfPower, rfAngle;
        public float lrPower, lrAngle;
        public float rrPower, rrAngle;
    }
}
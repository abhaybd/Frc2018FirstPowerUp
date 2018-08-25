package test;

import trclib.TrcGyro;
import trclib.TrcPidController;
import trclib.TrcRobot;
import trclib.TrcSwerveDriveBase;
import trclib.TrcSwerveModule;
import trclib.TrcTaskMgr;

import java.awt.event.KeyEvent;

public class SwerveTest
{
    public static void main(String[] args)
    {
        double width = 48.0;
        double length = 48.0;

        double turnDegreesPerCount = 1.0;
        TrcPidController.PidCoefficients pidCoefficients = new TrcPidController.PidCoefficients(0.02);
        double turnTolerance = 1;
        double turnOutputLimit = 1;

        TrcGyro gyro = new MockGyro("Gyro");

        TrcSwerveModule lfModule = new TrcSwerveModule("lfModule",
            new MockMotorController(3600), new MockMotorController(720),
            turnDegreesPerCount, pidCoefficients, turnTolerance, turnOutputLimit);
        TrcSwerveModule rfModule = new TrcSwerveModule("rfModule",
            new MockMotorController(3600), new MockMotorController(720),
            turnDegreesPerCount, pidCoefficients, turnTolerance, turnOutputLimit);
        TrcSwerveModule lrModule = new TrcSwerveModule("lrModule",
            new MockMotorController(3600), new MockMotorController(720),
            turnDegreesPerCount, pidCoefficients, turnTolerance, turnOutputLimit);
        TrcSwerveModule rrModule = new TrcSwerveModule("rrModule",
            new MockMotorController(3600), new MockMotorController(720),
            turnDegreesPerCount, pidCoefficients, turnTolerance, turnOutputLimit);

        TrcSwerveDriveBase driveBase = new TrcSwerveDriveBase("SwerveDrive", width, length,
            lfModule, rfModule, lrModule, rrModule, gyro);

        KeyListener listener = KeyListener.getInstance();
        TrcTaskMgr taskMgr = TrcTaskMgr.getInstance();

        while(true)
        {
            double x = listener.isKeyPressed(KeyEvent.VK_D) ? 1.0 : (listener.isKeyPressed(KeyEvent.VK_A) ? -1.0 : 0.0);
            double y = listener.isKeyPressed(KeyEvent.VK_W) ? 1.0 : (listener.isKeyPressed(KeyEvent.VK_S) ? -1.0 : 0.0);
            double turn = listener.isKeyPressed(KeyEvent.VK_E) ? 1.0 : (listener.isKeyPressed(KeyEvent.VK_Q) ? -1.0 : 0.0);

            driveBase.holonomicDrive(x, y, turn);

            System.out.printf("\rx=% 4.1f,y=% 4.1f,turn=% 4.1f | lfPower=%.1f,rfPower=%.1f,lrPower=%.1f,rrPower=%.1f | "
                + "lfAngle=%05.1f,rfAngle=%05.1f,lrAngle=%05.1f,rrAngle=%05.1f",
                x, y, turn,
                lfModule.getDrivePower(), rfModule.getDrivePower(), lrModule.getDrivePower(), rrModule.getDrivePower(),
                lfModule.getAngle(), rfModule.getAngle(), lrModule.getAngle(), rrModule.getAngle());
            try
            {
                Thread.sleep(50);
            }
            catch (InterruptedException e)
            {
                e.printStackTrace();
            }

            taskMgr.executeTaskType(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK, TrcRobot.RunMode.TELEOP_MODE);
        }
    }
}

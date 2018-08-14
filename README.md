# Frc2018FirstPowerUp
This is the branch to develop swerve drive. The testing and simulation will happen [here.](https://github.com/coolioasjulio/Frc2018FirstPowerUp/tree/SwerveDrive-Testing)

To implement swerve drive, the TrcDriveBase has been completely restructured. Before, TrcDriveBase implemented everything, making it a hulking behemoth of a class. An absolute unit, that one. Simple collosal. That makes it pretty difficult to add more stuff to it, since it's huge and complicated.

What this does is change the old TrcDriveBase to TrcCommonDriveBase (it implements the common drive methods + mecanum) and moves a lot of shared functionality to the new TrcDriveBase. This cuts almost 400 lines of code off. The new TrcDriveBase is an abstract class with the shared functionality. Since curve drive and arcade drive are built on tank drive, those are implemented in TrcDriveBase. Additionally, all method overloads are also implemented. (the optional parameters for all drive modes) Also, support for all the other stuff like MotorPowerMapper, GyroAssist, etc. At minimum, a drive base class extending TrcDriveBase must implement TankDrive and a few other getter classes for the yPosition, heading, etc. Additionally, some stuff like GyroAssist and MotorPowerMapper will not be applied unless the drive methods explicitly use it.

Then, the new swerve functionality is added to TrcSwerveDriveBase.

Also, drivebases will implement supportedDriveModes(), which will tell users of that class what drive modes are supported by that implementation of TrcDriveBase. It's pretty nifty, if I may say so myself.

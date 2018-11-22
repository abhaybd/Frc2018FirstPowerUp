package team492;

import frclib.FrcTankMotionProfileFollower;
import hallib.HalDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.modifiers.TankModifier;
import trclib.TrcPidController;
import trclib.TrcRobot;
import trclib.TrcTankMotionProfile;
import trclib.TrcUtil;

public class MotionProfileBenchmark implements TrcRobot.RobotCommand
{
    // TODO: Retune these
    private static final double kP = 1.275;
    private static final double kI = 0.0;
    private static final double kD = 0.0956;
    private static final double kF = 0.8525; // calculated according to phoenix docs

    private String instanceName;
    private FrcTankMotionProfileFollower follower;
    private Robot robot;
    private Double startTime;

    public MotionProfileBenchmark(String instanceName, Robot robot)
    {
        this.instanceName = instanceName;
        this.robot = robot;
        TrcPidController.PidCoefficients pidCoefficients = new TrcPidController.PidCoefficients(kP, kI, kD, kF);
        follower = new FrcTankMotionProfileFollower(instanceName + ".profileFollower", pidCoefficients,
            RobotInfo.ENCODER_Y_INCHES_PER_COUNT);
        follower.setLeftMotors(robot.leftFrontWheel, robot.leftRearWheel);
        follower.setRightMotors(robot.rightFrontWheel, robot.rightRearWheel);

        refreshData("Test/MaxVelocity", 0.0);
        refreshData("Test/MaxAcceleration", 0.0);
        refreshData("Test/MaxJerk", 0.0);
        refreshData("Test/WheelBase", 0.0);
    }

    private void refreshData(String name, double defaultValue)
    {
        HalDashboard.putNumber(name, HalDashboard.getNumber(name, defaultValue));
    }

    public void start()
    {
        TrcTankMotionProfile profile = createProfile(robot.driveDistance);
        follower.start(profile);
        robot.globalTracer.traceInfo(instanceName + ".start", "Started following path!");
    }

    private TrcTankMotionProfile createProfile(double distance)
    {
        Waypoint[] waypoints = new Waypoint[] { new Waypoint(0, 0, 0), new Waypoint(0, distance, 0) };
        double maxVelocity = HalDashboard.getNumber("Test/MaxVelocity", 72);
        double maxAcceleration = HalDashboard.getNumber("Test/MaxAcceleration", 52);
        double maxJerk = HalDashboard.getNumber("Test/MaxAcceleration", 720);
        Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
            Trajectory.Config.SAMPLES_LOW, 0.05, maxVelocity, maxAcceleration, maxJerk);
        Trajectory middleTrajectory = Pathfinder.generate(waypoints, config);

        TankModifier modifier = new TankModifier(middleTrajectory);
        modifier.modify(HalDashboard.getNumber("Test/WheelBase", 40));

        Trajectory left = modifier.getLeftTrajectory();
        Trajectory right = modifier.getRightTrajectory();

        TrcTankMotionProfile.TrcMotionProfilePoint[] leftPoints = new TrcTankMotionProfile.TrcMotionProfilePoint[middleTrajectory
            .length()];
        TrcTankMotionProfile.TrcMotionProfilePoint[] rightPoints = new TrcTankMotionProfile.TrcMotionProfilePoint[middleTrajectory
            .length()];
        for (int i = 0; i < middleTrajectory.length(); i++)
        {
            Trajectory.Segment leftSeg = left.get(i);
            Trajectory.Segment rightSeg = right.get(i);

            leftPoints[i] = new TrcTankMotionProfile.TrcMotionProfilePoint(leftSeg.dt, leftSeg.x, leftSeg.y,
                leftSeg.position, leftSeg.velocity, leftSeg.acceleration, leftSeg.acceleration, leftSeg.jerk);
            rightPoints[i] = new TrcTankMotionProfile.TrcMotionProfilePoint(rightSeg.dt, rightSeg.x, rightSeg.y,
                rightSeg.position, rightSeg.velocity, rightSeg.acceleration, rightSeg.acceleration, rightSeg.jerk);
        }
        return new TrcTankMotionProfile(leftPoints, rightPoints);
    }

    public void stop()
    {
        follower.cancel();
    }

    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {
        boolean isActive = follower.isActive();

        if (!isActive)
        {
            return false;
        }

        if (startTime == null)
        {
            startTime = elapsedTime;
        }

        double currPos = TrcUtil.average(follower.leftActualPosition(), follower.rightActualPosition());
        double targetPos = TrcUtil.average(follower.leftTargetPosition(), follower.rightTargetPosition());

        String message = String
            .format("[%.3f] MotionProfile: %s, error: %.3f", elapsedTime - startTime, follower.getInstanceName(),
                targetPos - currPos);

        robot.dashboard.displayPrintf(1, message);
        robot.globalTracer.traceInfo(instanceName + ".cmdPeriodic", message);

        return true;
    }
}

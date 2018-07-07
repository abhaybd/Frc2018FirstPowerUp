package trclib;

import java.util.function.Supplier;

public class TrcTrapezoidPositionController
{
    private enum Phase
    {
        RAMP_UP,
        MAINTAIN_SPEED,
        RAMP_DOWN,
        DONE
    }

    private String instanceName;
    private double target;
    private long startTime;
    private double maxAcceleration, maxSpeed;
    private Supplier<Double> speedSupplier, positionSupplier;
    private TrcPidController speedController;
    private Phase currentPhase;
    private double speedTolerance;

    public TrcTrapezoidPositionController(String instanceName, double maxAcceleration, double maxSpeed,
                                          Supplier<Double> positionSupplier, Supplier<Double> speedSupplier,
                                          TrcPidController speedController, double speedTolerance)
    {
        this.instanceName = instanceName;
        this.speedController = speedController;
        this.maxAcceleration = maxAcceleration;
        this.maxSpeed = maxSpeed;
        this.speedSupplier = speedSupplier;
        this.positionSupplier = positionSupplier;
        this.speedTolerance = speedTolerance;
    }

    public void setTarget(double target)
    {
        this.target = target;
        this.startTime = TrcUtil.getCurrentTimeMillis();
        currentPhase = Phase.RAMP_UP;
    }

    public double getOutput()
    {
        double speed = speedSupplier.get();
        double position = positionSupplier.get();

        if(position >= target)
        {
            currentPhase = Phase.DONE;
        } else if(target - position <= stoppingDistance(speed))
        {
            startTime = TrcUtil.getCurrentTimeMillis();
            currentPhase = Phase.RAMP_DOWN;
        }

        long currentTime = TrcUtil.getCurrentTimeMillis();
        long timeDifference = currentTime - startTime;

        switch(currentPhase)
        {
            case RAMP_UP:
                speedController.setTarget(((double)timeDifference/1000.0) * maxAcceleration);
                if(speed >= maxSpeed)
                {
                    currentPhase = Phase.MAINTAIN_SPEED;
                }
                return speedController.getOutput();
            case MAINTAIN_SPEED:
                speedController.setTarget(maxSpeed);
                return speedController.getOutput();
            case RAMP_DOWN:
                speedController.setTarget(maxSpeed + ((double)timeDifference/1000.0 * -maxAcceleration));
                if(speed <= speedTolerance)
                {
                    currentPhase = Phase.DONE;
                }
                return speedController.getOutput();
            default:
            case DONE:
                return 0.0;
        }
    }

    /**
     * Calculate time required to fully stop the robot while deaccelerating at max acceleration
     * @param currentSpeed Current speed of robot
     * @return Time required to stop, in milliseconds
     */
    private long timeToStop(double currentSpeed)
    {
        return Math.round((currentSpeed / maxAcceleration) * 1000.0); // Convert from seconds to ms
    }

    /**
     * Calculate distance required to stop while deaccelerating as max acceleration
     * @param currentSpeed Current speed of robot
     * @return Distance required to stop
     */
    private double stoppingDistance(double currentSpeed)
    {
        double timeToStop = (double)timeToStop(currentSpeed) / 1000.0; // Convert from ms to seconds
        return (timeToStop * currentSpeed) / 2.0; // The integration of the speed-time graph as it slows down
    }
}

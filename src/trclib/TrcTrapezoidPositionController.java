/*
 * Copyright (c) 2018 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package trclib;

import java.util.function.Supplier;

public class TrcTrapezoidPositionController implements TrcController
{
    private enum Phase
    {
        START,
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
    private double coastSpeed; // Actual speed achieved at end of MAINTAIN_SPEED

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
        reset();
        this.target = target;
    }

    public void reset()
    {
        currentPhase = Phase.START;
        target = 0.0;
    }

    public double getTarget() {
        return target;
    }

    public double getError() {
        return target - positionSupplier.get();
    }

    public void printInfo(TrcDbgTrace tracer, double timestamp, TrcRobotBattery battery) {
        // TODO: Add some logging code
    }

    public boolean isOnTarget()
    {
        return currentPhase == Phase.DONE;
    }

    /**
     * Calculate power output. This should be called frequently, as the power output will change as the profile is followed.
     * @return power output
     */
    public double getOutput()
    {
        double speed = speedSupplier.get();
        double position = positionSupplier.get();

        if(position >= target)
        {
            currentPhase = Phase.DONE;
        } else if(currentPhase != Phase.RAMP_DOWN && currentPhase != Phase.DONE && target - position <= stoppingDistance(speed))
        {
            startTime = TrcUtil.getCurrentTimeMillis();
            currentPhase = Phase.RAMP_DOWN;
            coastSpeed = speed;
        }

        long timeDifference;

        switch(currentPhase)
        {
            case START:
                startTime = TrcUtil.getCurrentTimeMillis();
                currentPhase = Phase.RAMP_UP;
            case RAMP_UP:
                timeDifference = TrcUtil.getCurrentTimeMillis() - startTime;
                double targetSpeed = ((double)timeDifference/1000.0) * maxAcceleration;
                speedController.setTarget(targetSpeed);
                if(targetSpeed >= maxSpeed)
                {
                    currentPhase = Phase.MAINTAIN_SPEED;
                }
                return speedController.getOutput();
            case MAINTAIN_SPEED:
                speedController.setTarget(maxSpeed);
                return speedController.getOutput();
            case RAMP_DOWN:
                timeDifference = TrcUtil.getCurrentTimeMillis() - startTime;
                speedController.setTarget(coastSpeed + ((double)timeDifference/1000.0 * -maxAcceleration));
                if(speed <= speedTolerance)
                {
                    currentPhase = Phase.DONE;
                }
                return speedController.getOutput();
            default:
            case DONE:
                speedController.setTarget(0.0);
                return speedController.getOutput();
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

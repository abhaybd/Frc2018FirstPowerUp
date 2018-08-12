package test;

import trclib.TrcMotorController;
import trclib.TrcSensor;
import trclib.TrcUtil;

public class MockMotorController implements TrcMotorController
{
    final private TrcSensor.SensorData<Double> position;
    private double power;
    private Thread monitorThread;
    private double speed;

    /**
     *
     * @param topSpeed Top speed, in degrees per second
     */
    public MockMotorController(final double topSpeed)
    {
        position = new TrcSensor.SensorData<>(TrcUtil.getCurrentTime(),0.0);

        monitorThread = new Thread(() -> {
            while(!Thread.interrupted())
            {
                double currTime = TrcUtil.getCurrentTime();
                double positionTime = position.timestamp;
                if(currTime == positionTime) continue;
                double positionChange = (power * topSpeed) * (currTime - positionTime);
                double newPosition = position.value + positionChange;
                speed = positionChange / (currTime - positionTime);
                synchronized (position)
                {
                    position.timestamp = currTime;
                    position.value = newPosition;
                }
                try
                {
                    Thread.sleep(50);
                }
                catch (InterruptedException e)
                {
                    break;
                }
            }
        });
        monitorThread.setDaemon(true);
        monitorThread.start();
    }

    public void close()
    {
        monitorThread.interrupt();
    }

    @Override
    public double getPosition()
    {
        return position.value;
    }

    @Override
    public double getPower()
    {
        return power;
    }

    @Override
    public double getSpeed()
    {
        return speed;
    }

    @Override
    public void resetPosition(boolean hardware)
    {
        synchronized (position)
        {
            position.timestamp = TrcUtil.getCurrentTime();
            position.value = 0.0;
        }
    }

    @Override
    public void setPower(double power)
    {
        this.power = power;
    }

    @Override
    public boolean getInverted()
    {
        return false;
    }

    @Override
    public boolean isLowerLimitSwitchActive()
    {
        return false;
    }

    @Override
    public boolean isUpperLimitSwitchActive()
    {
        return false;
    }

    @Override
    public void setBrakeModeEnabled(boolean enabled)
    {

    }

    @Override
    public void setInverted(boolean inverted)
    {

    }

    @Override
    public void setPositionSensorInverted(boolean inverted)
    {

    }

    @Override
    public void setSoftLimitEnabled(boolean lowerLimitEnabled, boolean upperLimitEnabled)
    {

    }

    @Override
    public void setSoftLowerLimit(double position)
    {

    }

    @Override
    public void setSoftUpperLimit(double position)
    {

    }
}

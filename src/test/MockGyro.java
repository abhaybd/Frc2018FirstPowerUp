package test;

import trclib.TrcGyro;
import trclib.TrcUtil;

public class MockGyro extends TrcGyro
{
    public MockGyro(String instanceName)
    {
        super(instanceName, 1, TrcGyro.GYRO_HAS_Z_AXIS);
    }

    @Override
    public SensorData<Double> getRawXData(DataType dataType)
    {
        return null;
    }

    @Override
    public SensorData<Double> getRawYData(DataType dataType)
    {
        return null;
    }

    @Override
    public SensorData<Double> getRawZData(DataType dataType)
    {
        return new SensorData<>(TrcUtil.getCurrentTime(), 0.0);
    }
}

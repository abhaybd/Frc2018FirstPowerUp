package trclib;

public interface TrcController
{
    void setTarget(double target);

    boolean isOnTarget();

    void reset();

    double getTarget();

    double getOutput();

    double getError();

    void printInfo(TrcDbgTrace tracer, double timestamp, TrcRobotBattery battery);

    default void printInfo()
    {
        printInfo(null, 0.0, null);
    }

    default void printInfo(TrcDbgTrace tracer)
    {
        printInfo(tracer, 0.0, null);
    }

    default void printInfo(TrcDbgTrace tracer, double timestamp)
    {
        printInfo(tracer, timestamp, null);
    }
}

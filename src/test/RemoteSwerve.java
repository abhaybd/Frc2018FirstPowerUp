package test;

import com.google.gson.Gson;
import com.google.gson.reflect.TypeToken;
import trclib.*;

import java.io.*;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class RemoteSwerve
{
    public static void main(String[] args) throws IOException
    {
        Map<Class<?>,Class<?>> unboxMap = new HashMap<>();
        unboxMap.put(Double.class, double.class);
        unboxMap.put(Integer.class, int.class);
        unboxMap.put(Float.class, float.class);
        unboxMap.put(Long.class, long.class);
        unboxMap.put(Boolean.class, boolean.class);
        unboxMap.put(Character.class, char.class);
        unboxMap.put(Byte.class, byte.class);
        unboxMap.put(Short.class, short.class);

        double width = 39.0;
        double length = 39.0;

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

        TrcSwerveDriveBase driveBase = new TrcSwerveDriveBase("SwerveDrive", width, length, gyro,
            lfModule, rfModule, lrModule, rrModule);

        TrcTaskMgr taskMgr = TrcTaskMgr.getInstance();
        ServerSocket serverSocket = new ServerSocket(4444);

        while(true)
        {
            System.out.println("Waiting for connection...");
            Socket socket = serverSocket.accept();
            System.out.println("Received connection from " + socket.getInetAddress().toString());
            BufferedReader in = new BufferedReader(new InputStreamReader(socket.getInputStream()));
            PrintStream out = new PrintStream(socket.getOutputStream());

            Gson gson = new Gson();

            while(true)
            {
                String line = in.readLine();
                if(line == null)
                {
                    socket.close();
                    break;
                }
                RPCRequest request = gson.fromJson(line, new TypeToken<RPCRequest>(){}.getType());
                System.out.println("Received request: " + line);
                try
                {
                    Class<?>[] classes = request.args.stream().map(e -> unboxMap.get(e.getClass())).toArray(Class<?>[]::new);
                    Method method = driveBase.getClass().getMethod(request.methodName, classes);
                    System.out.printf("Invoking method %s.%s(%s)\n", "driveBase", request.methodName, request.args.toString());
                    method.invoke(driveBase, request.args.toArray());

                    SwerveStatus status = new SwerveStatus();
                    status.lfPower = (float) lfModule.getDrivePower();
                    status.rfPower = (float) rfModule.getDrivePower();
                    status.lrPower = (float) lrModule.getDrivePower();
                    status.rrPower = (float) rrModule.getDrivePower();

                    status.lfAngle = (float) lfModule.getTargetAngle();
                    status.rfAngle = (float) rfModule.getTargetAngle();
                    status.lrAngle = (float) lrModule.getTargetAngle();
                    status.rrAngle = (float) rrModule.getTargetAngle();

                    String response = gson.toJson(status);
                    System.out.println("Sending response: " + response);
                    out.println(response);
                    out.flush();
                }
                catch (NoSuchMethodException | IllegalAccessException | InvocationTargetException e)
                {
                    e.printStackTrace();
                }

                taskMgr.executeTaskType(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK, TrcRobot.RunMode.TELEOP_MODE);
            }
        }
    }

    private static class RPCRequest
    {
        private long id;
        private String methodName;
        private List<Object> args;
    }

    private static class SwerveStatus
    {
        public float lfPower, lfAngle;
        public float rfPower, rfAngle;
        public float lrPower, lrAngle;
        public float rrPower, rrAngle;
    }
}
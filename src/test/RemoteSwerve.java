package test;

import com.google.gson.Gson;
import com.google.gson.reflect.TypeToken;
import trclib.*;

import java.io.*;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.List;

public class RemoteSwerve
{
    public static void main(String[] args) throws IOException
    {
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

        System.out.println("Waiting for connection...");
        ServerSocket serverSocket = new ServerSocket(4444);
        Socket socket = serverSocket.accept();
        System.out.println("Received connection from " + socket.getInetAddress().toString());
        BufferedReader in = new BufferedReader(new InputStreamReader(socket.getInputStream()));
        PrintStream out = new PrintStream(socket.getOutputStream());

        Gson gson = new Gson();

        while(true)
        {
            String line = in.readLine();
            RPCRequest request = gson.fromJson(line, new TypeToken<RPCRequest>(){}.getType());
            System.out.println("Received request: " + line);
            try
            {

                Method method = driveBase.getClass().getMethod(request.methodName, double.class, double.class, double.class);
                System.out.printf("Invoking method %s.%s(%s)\n", "driveBase", request.methodName, request.args.toString());
                method.invoke(driveBase, request.args.toArray());

                SwerveStatus status = new SwerveStatus();
                status.lfPower = (float) lfModule.getDrivePower();
                status.rfPower = (float) rfModule.getDrivePower();
                status.lrPower = (float) lrModule.getDrivePower();
                status.rrPower = (float) rrModule.getDrivePower();

                status.lfAngle = (float) lfModule.getAngle();
                status.rfAngle = (float) rfModule.getAngle();
                status.lrAngle = (float) lrModule.getAngle();
                status.rrAngle = (float) rrModule.getAngle();

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
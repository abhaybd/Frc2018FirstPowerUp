package test;

import com.google.gson.Gson;
import com.google.gson.reflect.TypeToken;

import java.io.*;
import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.stream.Collectors;

public class RPC
{

    public static void main(String[] args)
    {
        RPC.getInstance();
    }

    private static final int DEFAULT_PORT = 4444;
    private static RPC instance;

    public static RPC getInstance()
    {
        return getInstance(DEFAULT_PORT);
    }

    public static RPC getInstance(int port)
    {
        if(instance == null)
        {
            instance = new RPC(port);
        }
        return instance;
    }

    private final ServerSocket serverSocket;
    private Map<Class<?>,Class<?>> unboxMap;
    private Thread thread, requestHandlerThread;
    private Map<String,Object> variables;
    private BlockingQueue<RPCResponse> commQueue;

    private RPC(int port)
    {
        try
        {
            serverSocket = new ServerSocket(port);

            variables = new HashMap<>();
            commQueue = new LinkedBlockingQueue<>();

            unboxMap = new HashMap<>();
            unboxMap.put(Double.class, double.class);
            unboxMap.put(Integer.class, int.class);
            unboxMap.put(Float.class, float.class);
            unboxMap.put(Long.class, long.class);
            unboxMap.put(Boolean.class, boolean.class);
            unboxMap.put(Character.class, char.class);
            unboxMap.put(Byte.class, byte.class);
            unboxMap.put(Short.class, short.class);

            thread = new Thread(this::rpcThread);
            thread.setDaemon(true);
            thread.start();
        }
        catch (IOException e)
        {
            throw new RuntimeException(e);
        }
    }

    public void close()
    {
        thread.interrupt();
        requestHandlerThread.interrupt();
    }

    private void rpcThread()
    {
        try
        {
            Socket socket = serverSocket.accept();
            BufferedReader in = new BufferedReader(new InputStreamReader(socket.getInputStream()));
            PrintStream out = new PrintStream(socket.getOutputStream());

            launchRequestHandlerThread(in, out);
        }
        catch (IOException e)
        {
            e.printStackTrace();
        }
    }

    private void launchRequestHandlerThread(final BufferedReader in, final PrintStream out)
    {
        requestHandlerThread = new Thread(() ->
        {
            try
            {
                Gson gson = new Gson();
                while(!Thread.interrupted())
                {
                    try
                    {
                        String line = in.readLine();
                        RPCRequest request = gson.fromJson(line, new TypeToken<RPCRequest>(){}.getType());
                        Class<?>[] argClasses = request.getUnboxedClasses().toArray(new Class<?>[0]);
                        if(request.instantiate)
                        {
                            Class<?> clazz = Class.forName(request.className);
                            Constructor<?> constructor = clazz.getConstructor(argClasses);
                            Object object = constructor.newInstance(request.args.toArray());
                            variables.put(request.objectName, object);
                        } else
                        {
                            Object object = variables.get(request.objectName);
                            if(object == null && !request.objectName.equals("static")) continue;
                            Method method = object.getClass().getMethod(request.methodName, argClasses);
                            Object result = method.invoke(object, request.args.toArray());
                            if(result != null)
                            {
                                RPCResponse response = new RPCResponse();
                                response.id = request.id;
                                response.value = result;
                                String jsonResponse = gson.toJson(response);
                                out.println(jsonResponse);
                                out.flush();
                            }
                        }
                    }
                    catch(ClassNotFoundException | NullPointerException | IllegalAccessException
                        | InvocationTargetException | NoSuchMethodException | InstantiationException e)
                    {
                        e.printStackTrace();
                    }
                }
            } catch (IOException e)
            {
                e.printStackTrace();
            }
        });
        requestHandlerThread.setDaemon(true);
        requestHandlerThread.start();
    }

    private class RPCRequest
    {
        private long id;
        private boolean instantiate;
        private String className;
        private String objectName;
        private String methodName;
        private List<Object> args;

        public List<Class<?>> getBoxedClasses()
        {
            return args.stream().map(Object::getClass).collect(Collectors.toList());
        }

        public List<Class<?>> getUnboxedClasses()
        {
            return args.stream().map(e -> unboxMap.get(e.getClass())).collect(Collectors.toList());
        }
    }

    private static class RPCResponse
    {
        private long id;
        private Object value;
    }
}

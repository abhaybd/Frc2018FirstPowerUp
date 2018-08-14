package test;

import com.google.gson.Gson;
import com.google.gson.reflect.TypeToken;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.InterruptedIOException;
import java.io.PrintStream;
import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

public class RPC
{
    public static void main(String[] args)
    {
        RPC.getInstance().start();
    }

    private static final int DEFAULT_PORT = 4444;
    private static RPC instance;

    /**
     * Get the instance of the RPC server. If an instance doesn't exist, create one and bind it to the default port.
     *
     * @return The RPC server instance
     */
    public static RPC getInstance()
    {
        return getInstance(DEFAULT_PORT);
    }

    /**
     * Get the instance of the RPC server. If an instance doesn't exist, create one and bind it to the specified port.
     *
     * @param port Port to bind the RPC server if it doesn't exist.
     * @return The RPC server instance
     */
    public static RPC getInstance(int port)
    {
        if(instance == null)
        {
            instance = new RPC(port);
        }
        return instance;
    }

    private ServerSocket serverSocket = null;
    private Map<Class<?>,Class<?>> unboxMap;
    private Thread connectionHandlerThread;
    private List<Thread> requestHandlerThreads = new ArrayList<>();
    private int port;

    private RPC(int port)
    {
        this.port = port;

        Map<Class<?>,Class<?>> unboxMap = new HashMap<>();
        unboxMap.put(Double.class, double.class);
        unboxMap.put(Integer.class, int.class);
        unboxMap.put(Float.class, float.class);
        unboxMap.put(Long.class, long.class);
        unboxMap.put(Boolean.class, boolean.class);
        unboxMap.put(Character.class, char.class);
        unboxMap.put(Byte.class, byte.class);
        unboxMap.put(Short.class, short.class);
        this.unboxMap = Collections.unmodifiableMap(unboxMap);
    }

    /**
     * Is the RPC server currently running?
     *
     * @return True if the RPC server is running, false otherwise.
     */
    public boolean isActive()
    {
        return serverSocket != null;
    }

    /**
     * Initialize the RPC server if it is not already running. If it is, do nothing.
     */
    public void start()
    {
        if(serverSocket != null) return;
        try
        {
            serverSocket = new ServerSocket(port);

            connectionHandlerThread = new Thread(this::rpcThread);
            connectionHandlerThread.setDaemon(false);
            connectionHandlerThread.start();
        }
        catch (IOException e)
        {
            e.printStackTrace();
            throw new RuntimeException(e);
        }
    }

    /**
     * Close the RPC server socket, interrupt all the threads, and wait for the threads to end.
     * This method does not return until all the threads have stopped.
     */
    public void close()
    {
        if(serverSocket != null)
        {
            try
            {
                serverSocket.close();
            }
            catch (IOException e)
            {
                e.printStackTrace();
            }
        }

        connectionHandlerThread.interrupt();
        for(Thread t : requestHandlerThreads)
        {
            t.interrupt();
        }

        for(Thread t : requestHandlerThreads)
        {
            try
            {
                t.join();
            }
            catch (InterruptedException e)
            {
                e.printStackTrace();
            }
        }
    }

    private void rpcThread()
    {
        while(!Thread.interrupted())
        {
            try
            {
                System.out.println("Waiting for connection...");
                Socket socket = serverSocket.accept();
                System.out.println("Received connection from " + socket.getInetAddress().toString());
                BufferedReader in = new BufferedReader(new InputStreamReader(socket.getInputStream()));
                PrintStream out = new PrintStream(socket.getOutputStream());

                launchRequestHandlerThread(in, out);
            }
            catch(InterruptedIOException e)
            {
                break;
            }
            catch (IOException e)
            {
                e.printStackTrace();
            }
        }
    }

    private void launchRequestHandlerThread(final BufferedReader in, final PrintStream out)
    {
        Thread t = new Thread(() ->
        {
            try
            {
                Gson gson = new Gson();
                Map<String,Object> variables = new HashMap<>();
                while(!Thread.interrupted())
                {
                    String line = in.readLine();
                    if(line.length() == 0) continue;
                    System.out.println("Received request: " + line);
                    RPCRequest request = gson.fromJson(line, new TypeToken<RPCRequest>(){}.getType());
                    Class<?>[] argClasses = request.getUnboxedClasses(unboxMap).toArray(new Class<?>[0]);
                    boolean success = true;
                    if(request.instantiate)
                    {
                        try
                        {
                            Class<?> clazz = Class.forName(request.className);
                            Constructor<?> constructor = clazz.getConstructor(argClasses);
                            Object object = constructor.newInstance(request.args.toArray());
                            variables.put(request.objectName, object);

                        } catch(ClassNotFoundException | NoSuchMethodException | InvocationTargetException |
                            IllegalAccessException | InstantiationException e)
                        {
                            e.printStackTrace();
                            success = false;
                        }
                        RPCResponse response = new RPCResponse();
                        response.id = request.id;
                        response.value = success;

                        String jsonResponse = gson.toJson(response);
                        System.out.println("Sending response: " + jsonResponse);
                        out.println(jsonResponse);
                        out.flush();
                    } else
                    {
                        Object object = variables.get(request.objectName);
                        if(object == null && !request.objectName.equals("static")) continue;

                        Object result = null;
                        try
                        {
                            Class<?> clazz = object == null ? Class.forName(request.className) : object.getClass();
                            Method method = clazz.getMethod(request.methodName, argClasses);
                            result = method.invoke(object, request.args.toArray());
                        } catch(NullPointerException | NoSuchMethodException |
                            IllegalAccessException | InvocationTargetException |
                            ClassNotFoundException e)
                        {
                            e.printStackTrace();
                        }

                        RPCResponse response = new RPCResponse();
                        response.id = request.id;
                        response.value = result;

                        String jsonResponse = gson.toJson(response);
                        System.out.println("Sending response: " + jsonResponse);
                        out.println(jsonResponse);
                        out.flush();
                    }
                }
            } catch (IOException e)
            {
                e.printStackTrace();
            }
        });
        t.setDaemon(true);
        t.start();
        requestHandlerThreads.add(t);
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

        public List<Class<?>> getUnboxedClasses(Map<Class<?>,Class<?>> unboxMap)
        {
            List<Class<?>> unboxedClasses = new ArrayList<>();
            for(Object o : args)
            {
                Class<?> clazz = o.getClass();
                Class<?> unboxedClazz = unboxMap.getOrDefault(clazz, clazz);
                unboxedClasses.add(unboxedClazz);
            }
            return unboxedClasses;
        }
    }

    private static class RPCResponse
    {
        private long id;
        private Object value;
    }
}

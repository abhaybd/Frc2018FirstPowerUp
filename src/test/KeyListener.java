package test;

import javax.swing.*;
import java.awt.KeyboardFocusManager;
import java.awt.event.KeyEvent;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

public class KeyListener
{
    private static KeyListener instance;

    public static KeyListener getInstance()
    {
        if(instance == null)
        {
            instance = new KeyListener();
        }
        return instance;
    }

    private Map<Integer,Boolean> pressedMap = new ConcurrentHashMap<>();
    private JFrame frame;

    private KeyListener()
    {
        frame = new JFrame("Key listener");
        frame.addKeyListener(new java.awt.event.KeyListener()
        {
            @Override
            public void keyTyped(KeyEvent e)
            {}

            @Override
            public void keyPressed(KeyEvent e)
            {
                pressedMap.put(e.getKeyCode(), true);
            }

            @Override
            public void keyReleased(KeyEvent e)
            {
                pressedMap.put(e.getKeyCode(), false);
            }
        });
        frame.setSize(400,400);
        frame.setVisible(true);
    }

    public boolean isKeyPressed(int keyCode)
    {
        return pressedMap.getOrDefault(keyCode, false);
    }
}

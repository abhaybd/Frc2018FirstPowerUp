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
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
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

import trclib.TrcDbgTrace.TraceLevel;

/**
 * This class implements a generic warp space. A warp space is a linear space with two points: a low point and a
 * high point. The space is warped such that point A and point B represent the same physical position in space
 * even though the numeric distance between the two points is far away. A typical example of warp space is
 * demonstrated by a compass. A compass has a low point of 0-degree and a high point of 360-degree but these two
 * points are at the same physical position (NORTH).
 */
public class TrcWarpSpace
{
    private static final String moduleName = "TrcWarpSpace";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final boolean useGlobalTracer = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    private final String instanceName;
    private final double warpSpaceRange;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param warpSpaceLowPoint specifies the low point of the warp range.
     * @param warpSpaceHighPoint specifies the high point of the warp range.
     */
    public TrcWarpSpace(final String instanceName, double warpSpaceLowPoint, double warpSpaceHighPoint)
    {
        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer?
                TrcDbgTrace.getGlobalTracer():
                new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        if (warpSpaceHighPoint > warpSpaceLowPoint)
        {
            this.instanceName = instanceName;
            this.warpSpaceRange = warpSpaceHighPoint - warpSpaceLowPoint;
        }
        else
        {
            throw new IllegalArgumentException("LowPoint must be less than HighPoint.");
        }
    }   //TrcWarpSpace

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    public String toString()
    {
        return instanceName;
    }   //toString

    /**
     * This method returns the optimized target position. In the compass example, if one is currently headed NORTH
     * (0-degree) and wants to turn to WEST (270-degree), one may turn all the way 270 degrees clockwise and landed
     * pointing WEST. But one could also turn towards the warp point (0-degree), passed it and landed WEST by turning
     * counter clockwise.
     *
     * @param targetPos specifies the target position.
     * @param currentPos specifies the current position.
     * @param range specifies the warp space range.
     * @return optimized target position resulted in shorter traveling distance.
     */
    public static double optimizedTarget(double targetPos, double currentPos, double range)
    {
        double distance = (targetPos - currentPos) % range;
        double absDistance = Math.abs(distance);
        double optimizedDistance = (absDistance > range/2.0)? -(range - absDistance): distance;
        double optimizedTargetPos = optimizedDistance + currentPos;

        return optimizedTargetPos;
    }   //optimizedTarget

    /**
     * This method returns the optimized target position. In the compass example, if one is currently headed NORTH
     * (0-degree) and wants to turn to WEST (270-degree), one may turn all the way 270 degrees clockwise and landed
     * pointing WEST. But one could also turn towards the warp point (0-degree), passed it and landed WEST by turning
     * counter clockwise.
     *
     * @param targetPos specifies the target position.
     * @param currentPos specifies the current position.
     * @return optimized target position resulted in shorter traveling distance.
     */
    public double optimizedTarget(double targetPos, double currentPos)
    {
        final String funcName = "optimizedTarget";
        double optimizedTargetPos = optimizedTarget(targetPos, currentPos, warpSpaceRange);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TraceLevel.API, "target=%f,current=%f", targetPos, currentPos);
            dbgTrace.traceExit(funcName, TraceLevel.API, "=%f", optimizedTargetPos);
        }

        return optimizedTargetPos;
    }   //optimizedTarget

}   //class TrcWarpSpace

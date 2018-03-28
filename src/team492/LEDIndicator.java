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

package team492;

import frclib.FrcRevBlinkin;
import trclib.TrcRevBlinkin.LEDPattern;

public class LEDIndicator
{
    //
    // LED strip pattern constants.
    //
    private static final LEDPattern LED_OFF                     = LEDPattern.SolidBlack;
    private static final LEDPattern LED_DIAGNOSTIC_NORMAL       = LEDPattern.FixedLightChaseBlue;
    private static final LEDPattern LED_DIAGNOSTIC_ERROR        = LEDPattern.FixedLightChaseRed;
    private static final LEDPattern LED_CUBE_IN_VIEW            = LEDPattern.SolidBlue;
    private static final LEDPattern LED_CUBE_ALIGNED            = LEDPattern.SolidViolet;
    private static final LEDPattern LED_CUBE_IN_POSSESSION      = LEDPattern.SolidGreen;
//  private static final LEDPattern LED_CUBE_IN_PROXIMITY       = LEDPattern.SolidYellow;
//  private static final LEDPattern LED_GYRO_ASSIST_OFF         = LEDPattern.SolidBlack;
//  private static final LEDPattern LED_GYRO_ASSIST_ON          = LEDPattern.SolidRed;
    private static final LEDPattern[] patternPriorities =
    {
            LED_DIAGNOSTIC_NORMAL,
            LED_DIAGNOSTIC_ERROR,
            LED_CUBE_IN_VIEW,
            LED_CUBE_ALIGNED,
            LED_CUBE_IN_POSSESSION
    };

    private FrcRevBlinkin ledStrip;

    public LEDIndicator()
    {
        ledStrip = new FrcRevBlinkin("LEDStrip", RobotInfo.PWM_REV_BLINKIN);
        ledStrip.setPattern(LED_OFF);
    }

    public void indicateHasCube()
    {
        ledStrip.setPatternWithPriority(LED_CUBE_IN_POSSESSION, patternPriorities);
    }

    public void indicateHasNoCube()
    {
        turnOffPattern(LED_CUBE_IN_POSSESSION);
    }

    public void indicateSeesCube()
    {
        // The cube is in view on the Pixy but we only indicate this if we don't already have the cube.
        ledStrip.setPatternWithPriority(LED_CUBE_IN_VIEW, patternPriorities);
    }

    public void indicateSeesNoCube()
    {
        turnOffPattern(LED_CUBE_IN_VIEW);
    }

    public void indicateAlignedToCube()
    {
        // Pixy found the cube right at center. Only indicate this if we don't already have the cube.
        ledStrip.setPatternWithPriority(LED_CUBE_ALIGNED, patternPriorities);
    }

    public void indicateNotAlignedToCube()
    {
        turnOffPattern(LED_CUBE_ALIGNED);
    }

    public void indicateDiagnosticError()
    {
        // Diagnostic error is low priority. Only indicate this when nobody is active. So it is the lowest
        // on the priority list.
        ledStrip.setPattern(LED_DIAGNOSTIC_ERROR);
   }

    public void indicateNoDiagnosticError()
    {
        ledStrip.setPattern(LED_DIAGNOSTIC_NORMAL);
    }

    public void setPattern(LEDPattern pattern)
    {
        if (ledStrip.findPatternPriority(pattern, patternPriorities) != -1)
        {
            throw new IllegalArgumentException("Pattern is already reserved in the priority list.");
        }
        else
        {
            ledStrip.setPatternWithPriority(pattern, patternPriorities);
        }
    }

    private void turnOffPattern(LEDPattern pattern)
    {
        if (ledStrip.getPattern() == pattern)
        {
            ledStrip.setPattern(LED_OFF);
        }
    }
}
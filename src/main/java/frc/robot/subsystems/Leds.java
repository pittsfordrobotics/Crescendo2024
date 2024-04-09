// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Leds extends SubsystemBase {
    // public enum LedColor {
    // pink, black, white, blue, green, purple
    // }
    public enum LedMode {
        solid, blink, blinkDual, fade, wrap, bounce
    }

    public enum LedSpeed {
        slow, mid, fast
    }

    Color MainColor = Color.kDeepPink;
    Color SecondaryColor = Color.kAqua;
    LedMode ledMode = LedMode.solid;
    LedSpeed ledSpeed = LedSpeed.slow;
    AddressableLED ledstrip;
    AddressableLEDBuffer ledBuffer;
    SendableChooser<Color> mainColorChooser = new SendableChooser<Color>();
    SendableChooser<Color> secondaryColorChooser = new SendableChooser<Color>();
    SendableChooser<LedMode> modeChooser = new SendableChooser<LedMode>();
    SendableChooser<LedSpeed> speedChooser = new SendableChooser<LedSpeed>();

    /** Creates a new Leds. */
    public Leds() {
        ledstrip = new AddressableLED(4);
        ledBuffer = new AddressableLEDBuffer(9);
        ledBuffer.setLED(0, Color.kDeepPink);
        ledstrip.setLength(ledBuffer.getLength());
        ledstrip.setData(ledBuffer);
        ledstrip.start();

        for (LedMode mode : LedMode.values()) {
            modeChooser.addOption(mode.toString(), mode);
        }
        for (LedSpeed speed : LedSpeed.values()) {
            speedChooser.addOption(speed.toString(), speed);
        }
        for (Color color : List.of(Color.kBlack, Color.kWhite, Color.kRed, Color.kGreen, Color.kBlue, Color.kYellow,
                Color.kPurple, Color.kOrange, Color.kAqua, Color.kDeepPink)) {
            mainColorChooser.addOption(color.toString(), color);
            secondaryColorChooser.addOption(color.toString(), color);
        }
        Shuffleboard.getTab("LEDS").add("Color", mainColorChooser);
        Shuffleboard.getTab("LEDS").add("Secondary Color", secondaryColorChooser);
        Shuffleboard.getTab("LEDS").add("Mode", modeChooser);
        Shuffleboard.getTab("LEDS").add("Speed", speedChooser);

        mainColorChooser.onChange(color -> {
            MainColor = color;
        });
        secondaryColorChooser.onChange(coloor -> {
            SecondaryColor = coloor;
        });
        modeChooser.onChange(mode -> {
            ledMode = mode;
        });
        speedChooser.onChange(speed -> {
            ledSpeed = speed;
        });
    }

    @Override
    public void periodic() {
        int i = 0;
        int q = 0;
        switch (ledSpeed) {
            case slow:
                q = 1;
                break;
            case mid:
                q = 2;
                break;
            case fast:
                q = 4;
                break;
        }
        switch (ledMode) {
            case solid:
                setFullLengthofBuffer(ledBuffer, MainColor);
                break;
            case blink:
                switch (ledSpeed) {
                    case slow:
                        if (i % 2 == 0) {
                            setFullLengthofBuffer(ledBuffer, MainColor);
                        } else {
                            setFullLengthofBuffer(ledBuffer, Color.kBlack);
                        }
                        break;
                    case mid:
                        if (i % 3 == 0) {
                            setFullLengthofBuffer(ledBuffer, MainColor);
                        } else {
                            setFullLengthofBuffer(ledBuffer, Color.kBlack);
                        }
                        break;
                    case fast:
                        if (i % 4 == 0) {
                            setFullLengthofBuffer(ledBuffer, MainColor);
                        } else {
                            setFullLengthofBuffer(ledBuffer, Color.kBlack);
                        }
                        break;
                }
                break;
            case blinkDual:
                switch (ledSpeed) {
                    case slow:
                        if (i % 2 == 0) {
                            setFullLengthofBuffer(ledBuffer, MainColor);
                        } else {
                            setFullLengthofBuffer(ledBuffer, SecondaryColor);
                        }
                        break;
                    case mid:
                        if (i % 3 == 0) {
                            setFullLengthofBuffer(ledBuffer, MainColor);
                        } else {
                            setFullLengthofBuffer(ledBuffer, SecondaryColor);
                        }
                        break;
                    case fast:
                        if (i % 4 == 0) {
                            setFullLengthofBuffer(ledBuffer, MainColor);
                        } else {
                            setFullLengthofBuffer(ledBuffer, SecondaryColor);
                        }
                        break;
                }
                break;
            case fade:
                q = q * 50;
                // defines the brightness of the color based on q should go up from 0 to 1 and
                // back to 0 in one iteration of length q
                double brightness = MathUtil.clamp((double) (Math.abs((i % (q)) - (q * 0.5)) / q), 0, 1);
                setFullLengthofBuffer(ledBuffer, changeColorBrightness(MainColor, brightness));
                break;
            case wrap:
                q = q * ledBuffer.getLength() * 5;
                // Sets certain led to color based on q
                for (var j = 0; j < ledBuffer.getLength(); j++) {
                    if (j == (i % q) / ledBuffer.getLength()) {
                        ledBuffer.setLED(j, MainColor);
                    } else {
                        ledBuffer.setLED(j, SecondaryColor);
                    }
                }
                break;
            case bounce:
                q = q * ledBuffer.getLength() * 5;
                // Sets certain led to color based on q
                for (var j = 0; j < ledBuffer.getLength(); j++) {
                    if (j == Math.abs((i % q) - q / 2) / ledBuffer.getLength()) {
                        ledBuffer.setLED(j, MainColor);
                    } else {
                        ledBuffer.setLED(j, SecondaryColor);
                    }
                }
                break;
        }
        ledstrip.setData(ledBuffer);
        i = i + 1;
        // This method will be called once per scheduler run
    }

    // sets speed
    private void setLedSpeed(LedSpeed speed) {
        ledSpeed = speed;
    }

    // sets mode
    private void setLedMode(LedMode mode) {
        ledMode = mode;
    }

    // sets color
    private void setLedColor(Color color) {
        MainColor = color;
    }

    // sets secondary color
    private void setSecondaryColor(Color color) {
        SecondaryColor = color;
    }

    private void setLedAll(Color mainColor, Color secondaryColor, LedMode mode, LedSpeed speed) {
        setLedSpeed(speed);
        setLedMode(mode);
        setLedColor(mainColor);
        setSecondaryColor(secondaryColor);
    }

    // sets the led strip to a solid color
    public Command setLed(Color mainColor, Color secondaryColor, LedMode mode, LedSpeed speed) {
        return this.runOnce(() -> {
            setLedAll(mainColor, secondaryColor, mode, speed);
        });
    }

    // When the robot is ready to shoot
    public Command setLedHasGoodSpeakerShot() {
        return this.runOnce(() -> {
            setLedAll(Color.kGreen, Color.kBlack, LedMode.solid, LedSpeed.slow);
        });
    }

    // When the robot is intaking
    public Command setLedHasGoodSpeakerIntake() {
        return this.runOnce(() -> {
            setLedAll(Color.kBlue, Color.kBlack, LedMode.solid, LedSpeed.slow);
        });
    }

    // When the robot is in stored position
    public Command setLedHasGoodSpeakerStored() {
        return this.runOnce(() -> {
            setLedAll(Color.kYellow, Color.kBlack, LedMode.solid, LedSpeed.slow);
        });
    }

    private void setFullLengthofBuffer(AddressableLEDBuffer buffer, Color color) {
        for (var i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, color);
        }
    }

    // takes in a color and double and rerurns a color with the brightness relative
    // to the double
    private Color changeColorBrightness(Color color, double brightness) {
        return new Color(color.red * brightness, color.green * brightness, color.blue * brightness);
    }
}

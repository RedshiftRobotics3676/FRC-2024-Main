
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.Subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_buffer;
  private final int pwmPort = 0;
  private final int ledLength = 104; //220

  private static int counter = 0;

  SendableChooser<String> chooser;

  /** Creates a new LEDs. */
  public LEDs() {
    m_led = new AddressableLED(pwmPort);
    m_led.setLength(ledLength);
    m_buffer = new AddressableLEDBuffer(ledLength);

    m_led.start();

    chooser = new SendableChooser<String>();
    chooser.setDefaultOption("Pulse Alliance Color", "Pulse Alliance Color");
    chooser.addOption("Wave Alliance Color", "Wave Alliance Color");
    chooser.addOption("Rainbow", "Rainbow");

    SmartDashboard.putData(chooser);
  }

  public void updateLEDs() {
    m_led.setData(m_buffer);
  }

  public void setSolidRGB(int r, int b, int g) {
    for (int i = 0; i < ledLength; i++) {
      m_buffer.setRGB(i, r, g, b);
    }
    m_led.setData(m_buffer);
  }

  public void setSolidHSV(int h, int s, int v) {
    for (int i = 0; i < ledLength; i++) {
        m_buffer.setHSV(i, h, s, v);
    }
    m_led.setData(m_buffer);
}

public void showIndex(int i) {
  setSolidRGB(0, 0, 0);
  m_buffer.setRGB(99, 255, 255, 255);
  m_led.setData(m_buffer);
}

  static int m_rainbowFirstPixelHue = 0;
  public void rainbow() {
    // For every pixel
    for (var i = 0; i < ledLength; i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / ledLength)) % 180;
      // Set the value
      m_buffer.setHSV(i, hue, 255, 100);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 1;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;

    m_led.setData(m_buffer);
  }

  /** @param hue hue of main color from 0-180 */
  public void waveHue(int hue) {
    // Move the position in the direction
    m_position -= 1;
    
    int phase = 0;
    for (int i = 0; i < ledLength; i++) {
        // Calculate brightness using a sine wave pattern
        int brightness = (int) (255 - ((0.2*Math.sin(i + phase + m_position) + 1) / 2 * 255));
        // Set the color and brightness of the LED
        m_buffer.setHSV(i, hue, 255, brightness); // Change the hue value to change the color
    }
    // Increment the phase
    phase = (phase + 1) % ledLength;
    // Set the LED data
    m_led.setData(m_buffer);
  }

  public void waveAllianceColor() {
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
        if (ally.get() == Alliance.Red) {
            waveHue(0);
        }
        if (ally.get() == Alliance.Blue) {
          waveHue(120);
        }
    }
    else {
      waveHue(70);
    }
  }


  public void solidAllianceColor() {
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
        if (ally.get() == Alliance.Red) {
            setSolidHSV(0, 255, 100);
        }
        if (ally.get() == Alliance.Blue) {
          setSolidHSV(120, 255, 100);
        }
    }
    else {
      setSolidHSV(70, 255, 100);
    }
  }

  public void pulseAllianceColor() {
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
        if (ally.get() == Alliance.Red) {
            pulseColorHSV2(0, 255, 255);
        }
        if (ally.get() == Alliance.Blue) {
          pulseColorHSV2(120, 255, 255);
        }
    }
    else {
      pulseColorHSV2(30, 255, 255);
    }
  }

  
  
  public void runFromChooser() {
    counter++;
    if (chooser.getSelected() == "Rainbow") {
      if (counter >= 15) {
        counter = 0;
        rainbow();
      }
    }
    else if (chooser.getSelected() == "Pulse Alliance Color") {
      if (counter >= 15) {
        counter = 0;
        pulseAllianceColor();
      }
    }
    else if (chooser.getSelected() == "Wave Alliance Color") {
      if (counter >= 30) {
        counter = 0;
        waveAllianceColor();
      }
    }
    else if (counter >= 15) {
      counter = 0;
      pulseAllianceColor();
    }
    
    // if (counter >= 15) {
    //   counter = 0;

    //   switch (chooser.getSelected()) {
    //     case "Rainbow" :
    //       rainbow();
    //       break;
    //     case "Pulse Alliance Color" :
    //       pulseAllianceColor();
    //       break;
    //     case "Wave Alliance Color" :
    //       waveAllianceColor();
    //       break;
    //     default :
    //       pulseAllianceColor();
    //       break;
    //   }
    // }
  }


  private int direction = 1; // 1 for forward, -1 for backward
  private int position = 0; // Start at the first LED

  // private static int ledCallCounter = 0; //keeps track of how many times method has been called so you can use it to time it with robotperiodic
  static boolean runFlag = true;

  /**
   * DOESN'T WORK
   */
  public void pulseColorHSV1(int h, int s, int v) {
    // if (!runFlag) {
    //   runFlag = true; // Toggle the flag when the method is skipped
    //   return;
    // }
  
    // runFlag = false; // Toggle the flag when the method is run

    // Move the position in the direction
    position += direction;

    // If the position reaches the end of the strip, reverse the direction
    if (position == ledLength - 1 || position == 0) {
      direction *= -1;
    }

    // if (position == ledLength - 2 || position == 1) {
    //   direction *= -1;
    // }

     // Calculate the brightness of the furthest part of the patch
     int patchSize = 5; // Size of the color patch
     int distance = patchSize;
     int minBrightness = v - (distance * 50); // Decrease brightness by 10 for each LED away from the center
     if (minBrightness < 1) {
         minBrightness = 1;
     }

    // Set all LEDs to white/gray
    setSolidHSV(0, 0, minBrightness);

    // Set the LEDs in the color patch to the specified color
    for (int i = -patchSize; i <= patchSize; i++) {
      // Calculate the distance from the center of the patch
      distance = Math.abs(i);

      // Calculate the brightness based on the distance from the center
      int brightness = v - (distance * 50); // Decrease brightness by 20 for each LED away from the center
      if (brightness < 1) {
        brightness = 1;
      }

      // Set the color of the LED, making sure not to go outside the strip
      int index = position + i;
      if (index >= 0 && index < ledLength) {
        m_buffer.setHSV(index, h, s, brightness);
      }
    }

    // Update the LEDs
    m_led.setData(m_buffer);
  }


  int patchSize1 = 8;
  int brightness1 = 10;
  int distance1 = patchSize1;
  int whiteBrightness = 10; //10

  int m_position = 0;
  int m_direction = 1;

  public void pulseColorHSV2(int h, int s, int v) {
    // Move the position in the direction
    m_position += m_direction;

    // If the position reaches the end of the strip, reverse the direction
    if (m_position == m_buffer.getLength() - 1 || m_position == 0) {
        m_direction *= -1;
    }

    
    // int minBrightness = (v - (distance1 * 30)); // Decrease brightness by 10 for each LED away from the center
    // if (minBrightness < 1) {
    //      minBrightness = 1;
    //  }

    // int brightness1 = v - ((v - whiteBrightness) * distance1 / patchSize1);
    // if (brightness1 < whiteBrightness) {
    //   brightness1 = whiteBrightness;
    // }


    // Set all LEDs to white
    for (var i = 0; i < m_buffer.getLength(); i++) {
        m_buffer.setHSV(i, 0, 0, whiteBrightness);
        // m_buffer.setRGB(i, 15, 15 , 10);
    }
    
    for (int i = -patchSize1; i <= patchSize1; i++) {
      // Calculate the distance from the center of the patch
      distance1 = Math.abs(i);

      // Calculate the brightness based on the distance from the center
      // int brightness1 = v - (distance1 * 30); // Decrease brightness by 20 for each LED away from the center
      // if (brightness1 < 1) {
      //   brightness1 = 1;
      // }

      int brightness1 = v - ((v - whiteBrightness) * distance1 / patchSize1);
      if (brightness1 < whiteBrightness) {
        brightness1 = whiteBrightness;
      }

      // Set the color of the LED, making sure not to go outside the strip
      int index = m_position + i;
      if (index >= 0 && index < ledLength) {
        m_buffer.setHSV(index, h, s, brightness1);
      }
    }

    // Set the current LED to the specified color
    // m_buffer.setHSV(m_position, h, s, v);

    // Update the LEDs
    m_led.setData(m_buffer);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

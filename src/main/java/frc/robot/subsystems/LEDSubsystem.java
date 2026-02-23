package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Map;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class LEDSubsystem extends SubsystemBase {
  private static final int kPort = 9;
  private static final int kLength = 150;

  private final int brightness = 40;
  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_buffer;

  // Patterns
  private final LEDPattern blank = LEDPattern.solid(Color.kBlack);
  private final LEDPattern restpattern = LEDPattern.solid(Color.kPurple).breathe(Seconds.of(5)).atBrightness(Percent.of(brightness));
  private final LEDPattern passpattern = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kPeru, Color.kRed).atBrightness(Percent.of(brightness)).scrollAtAbsoluteSpeed(FeetPerSecond.of(5), Feet.of(8.2/150));
  private final LEDPattern intakepattern = LEDPattern.steps(Map.of(0, Color.kBlack, 0.25, Color.kYellow, 0.5, Color.kYellow, 0.75, Color.kBlack)).atBrightness(Percent.of(brightness)).scrollAtAbsoluteSpeed(FeetPerSecond.of(5), Feet.of(8.2/150));
  public LEDSubsystem() {
    m_led = new AddressableLED(kPort);
    m_buffer = new AddressableLEDBuffer(kLength);
    m_led.setLength(kLength);
    m_led.start();

    // Set the default command to turn the strip off, otherwise the last colors written by
    // the last command to run will continue to be displayed.
    // Note: Other default patterns could be used instead!
    setDefaultCommand(runPattern("none"));
  }

  @Override
  public void periodic() {
  
    // Periodically send the latest LED color data to the LED strip for it to display
    m_led.setData(m_buffer);
    
  }

  /**
   * Creates a command that runs a pattern on the entire LED strip.
   *
   * @param pattern the LED pattern to run
   */
  public Command runPattern(String pattern) {
    if (pattern == "SHOOT") {
      return run(() -> {LEDPattern.steps(Map.of(0, Color.kDarkGreen, 0.5, Color.kWhite)).atBrightness(Percent.of(brightness)).scrollAtAbsoluteSpeed(FeetPerSecond.of(Shooter.shootspeed / 30), Feet.of(8.2/150)).applyTo(m_buffer);});
    }
    else if (pattern == "REST") {
      return run(() -> restpattern.applyTo(m_buffer));
    }
    else if (pattern == "PASS") {
      return run(() -> passpattern.applyTo(m_buffer));
    }
    else if (pattern == "INTAKE") {
      return run(() -> intakepattern.applyTo(m_buffer));
    }
    else if (pattern == "CLIMB") {
      return run(() -> {
        LEDPattern.steps(Map.of(0, Color.kWhite, 0.5, Color.kBlue))
    .atBrightness(Percent.of(brightness))
    .scrollAtAbsoluteSpeed(FeetPerSecond.of(RobotContainer.ClimbPercent * 5), Feet.of(8.2 / 150))
    .applyTo(m_buffer);  });
    }
    else {
      return run(() -> blank.applyTo(m_buffer));
    }
  }

  public void off() {
    blank.applyTo(m_buffer);
  }
}
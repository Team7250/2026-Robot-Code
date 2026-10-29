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

public class LEDSubsystem extends SubsystemBase {
  private static final int kPort = 9;
  private static final int kLength = 235;
  private static final double FeetLength = 16.4;

  private final int brightness = 20;
  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_buffer;

  // Patterns
  private final LEDPattern blank = LEDPattern.solid(Color.kBlack);
  private final LEDPattern restpattern = LEDPattern.solid(Color.kBlue).breathe(Seconds.of(5))
      .atBrightness(Percent.of(brightness));
  private final LEDPattern passpattern = LEDPattern
      .gradient(LEDPattern.GradientType.kContinuous, Color.kPeru, Color.kRed).atBrightness(Percent.of(brightness))
      .scrollAtAbsoluteSpeed(FeetPerSecond.of(5), Feet.of(FeetLength / 150));
  private final LEDPattern intakepattern = LEDPattern
      .steps(Map.of(0, Color.kBlack, 0.25, Color.kYellow, 0.5, Color.kYellow, 0.75, Color.kBlack))
      .atBrightness(Percent.of(brightness)).scrollAtAbsoluteSpeed(FeetPerSecond.of(5), Feet.of(FeetLength / 150));
  private final LEDPattern autopattern = LEDPattern.rainbow(255, brightness * 2)
      .scrollAtAbsoluteSpeed(FeetPerSecond.of(50), Feet.of(FeetLength / 150));

  private LEDPattern m_currentPattern = restpattern;

  public LEDSubsystem() {
    m_led = new AddressableLED(kPort);
    m_buffer = new AddressableLEDBuffer(kLength);
    m_led.setLength(kLength);
    m_led.start();

    // Set the default command to turn the strip off, otherwise the last colors
    // written by
    // the last command to run will continue to be displayed.
    // Note: Other default patterns could be used instead!
    // setDefaultCommand(runPattern("REST"));
  }

  private int ledTick = 0;

  @Override
  public void periodic() {
    if (ledTick++ % 3 == 0) {
      m_currentPattern.applyTo(m_buffer);
      m_led.setData(m_buffer);
    }
  }

  /**
   * Creates a command that runs a pattern on the entire LED strip.
   *
   * @param pattern the LED pattern to run
   */
  /*
   * public Command runPattern(String pattern) {
   * System.out.println(pattern);
   * 
   * if (pattern == "SHOOT") {
   * return runOnce(() -> {LEDPattern.steps(Map.of(0, Color.kDarkGreen, 0.5,
   * Color.kWhite)).atBrightness(Percent.of(brightness)).scrollAtAbsoluteSpeed(
   * FeetPerSecond.of(Shooter.shootspeed / 30),
   * Feet.of(FeetLength/150)).applyTo(m_buffer);});
   * }
   * else if (pattern.equals("REST")) {
   * return runOnce(() -> restpattern.applyTo(m_buffer));
   * }
   * else if (pattern.equals("PASS")) {
   * return runOnce(() -> passpattern.applyTo(m_buffer));
   * }
   * else if (pattern.equals("INTAKE")) {
   * return runOnce(() -> intakepattern.applyTo(m_buffer));
   * }
   * /*else if (pattern == "CLIMB") {
   * return run(() -> {
   * LEDPattern.steps(Map.of(0, Color.kWhite, 0.5, Color.kBlue))
   * .atBrightness(Percent.of(brightness))
   * .scrollAtAbsoluteSpeed(FeetPerSecond.of(RobotContainer.ClimbPercent * 5),
   * Feet.of(8.2 / 150))
   * .applyTo(m_buffer); });
   * }
   * else if(pattern.equals("AUTO")){
   * return runOnce(() -> {
   * autopattern.applyTo(m_buffer);
   * });
   * }
   * else {
   * return runOnce(() -> blank.applyTo(m_buffer));
   * }
   * }
   */

  public Command runPattern(String pattern) {
    return runOnce(() -> {
      if (pattern.equals("AUTO"))
        m_currentPattern = autopattern;
      else if (pattern.equals("REST"))
        m_currentPattern = restpattern;
      else if (pattern.equals("PASS"))
        m_currentPattern = passpattern;
      else if (pattern.equals("INTAKE"))
        m_currentPattern = intakepattern;
      else if (pattern.equals("SHOOT")) {
        if (Shooter.aimed)
          m_currentPattern = LEDPattern.steps(Map.of(0, Color.kDarkGreen, 0.5, Color.kWhite))
              .atBrightness(Percent.of(brightness))
              .scrollAtAbsoluteSpeed(FeetPerSecond.of(Shooter.shootspeed), Feet.of(FeetLength / 150));
      } else {
        m_currentPattern = LEDPattern.steps(Map.of(0, Color.kRed, 0.5, Color.kBlack))
            .atBrightness(Percent.of(brightness))
            .scrollAtAbsoluteSpeed(FeetPerSecond.of(Shooter.shootspeed), Feet.of(FeetLength / 150));
      }
    });
  }

  public void updatePattern(String pattern) {
      if (pattern.equals("AUTO"))
        m_currentPattern = autopattern;
      else if (pattern.equals("REST"))
        m_currentPattern = restpattern;
      else if (pattern.equals("PASS"))
        m_currentPattern = passpattern;
      else if (pattern.equals("INTAKE"))
        m_currentPattern = intakepattern;
      else if (pattern.equals("GOOD")) {
          m_currentPattern = LEDPattern.steps(Map.of(0, Color.kDarkGreen, 0.5, Color.kWhite))
              .atBrightness(Percent.of(brightness))
              .scrollAtAbsoluteSpeed(FeetPerSecond.of(Shooter.shootspeed), Feet.of(FeetLength / 150));
      } else if(pattern.equals("BAD")) {
        m_currentPattern = LEDPattern.steps(Map.of(0, Color.kRed, 0.5, Color.kBlack))
            .atBrightness(Percent.of(brightness))
            .scrollAtAbsoluteSpeed(FeetPerSecond.of(Shooter.shootspeed), Feet.of(FeetLength / 150));
      }
    }
  

  public void on() {
    restpattern.applyTo(m_buffer);
  }

  public void off() {
    blank.applyTo(m_buffer);
  }
}
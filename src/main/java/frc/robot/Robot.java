// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.StrictFollower;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.Storage;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class Robot extends TimedRobot {

    /*
     *      CAN MAP
     * SWERVES
     * FRONT-LEFT:
     *  D: 0
     *  R: 1
     *  E: 2
     * FRONT-RIGHT:
     *  D: 3
     *  R: 4
     *  E: 5
     * BACK-LEFT: 
     *  D: 6
     *  R: 7
     *  E: 8
     * BACK-RIGHT:
     *  D: 9
     *  R: 10
     *  E: 11
     *  Pigeon: 12
     * CLIMB:
     *  1: 13
     *  2: 14
     * SHOOTER:
     *  1: 15
     *  2: 16
     * INTAKE:
     *  RaiseLower1: 17
     *  RaiseLower2: 18
     *  Rotate: 19
     * INFEEDER
     *  feeder: 20
     */

    //CANIVORE
    public static CANBus CANivore = new CANBus("drivetrain");

    // Pigeon
    public Pigeon2 gyro = new Pigeon2(12);

    // Climb
    public static TalonFX climb1 = new TalonFX(13);
    public static TalonFX climb2 = new TalonFX(14);

    // Shooter
    public static TalonFX shooter1 = new TalonFX(15);
    public static TalonFX shooter2 = new TalonFX(16);

    // Intake
    public static TalonFX RaiseLower1 = new TalonFX(17);
    public static TalonFX RaiseLower2 = new TalonFX(18);
    public static SparkMax RotateIntake = new SparkMax(19, MotorType.kBrushless);

    // Storage
    public static TalonFX storage1 = new TalonFX(17);

    // Infeeder
    public static TalonFX feeder = new TalonFX(20);

    // Controller Declarations
    public static Joystick joystick = new Joystick(0); // Patrick

    // Joystick Button/Axis Declarations
    public static JoystickButton orientSwitch = new JoystickButton(joystick, 2);
    public static double speedSlider = joystick.getRawAxis(3);

    // Xbox Controller Button/Axis Declarations (MOVED TO TELEOPINIT)

    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    // debounce
    private boolean cmdDebounce = false;

    AddressableLED m_led = new AddressableLED(9);
    AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(10);

    //LED Patterns
    LEDPattern redpattern = LEDPattern.solid(Color.kRed);
    LEDPattern gradpattern = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kTomato, Color.kAqua);

    

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();
    
    public Robot() {
        m_robotContainer = new RobotContainer();

        //LED
        m_ledBuffer = new AddressableLEDBuffer(60);
        m_led.setLength(m_ledBuffer.getLength());

        m_led.setData(m_ledBuffer);
        m_led.start();

        //Motors

        configureMotors(climb1, 80, 160, 1600);
        configureMotors(climb2, 80, 160, 1600);

        configureMotors(shooter1, 80, 160, 1600);
        configureMotors(shooter2, 80, 160, 1600);

        configureMotors(RaiseLower1, 80, 160, 1600);
        configureMotors(RaiseLower2, 80, 160, 1600);
        RaiseLower2.setControl(new StrictFollower(17));
        SparkMaxConfig config = new SparkMaxConfig();
        RotateIntake.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        
        
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();

        //LED Pattern apply
        redpattern.applyTo(m_ledBuffer);
        m_led.setData(m_ledBuffer);
        
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    public void testFunction(){
        System.out.println("Hi");
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }

        // joystick button command bindings 
        if (cmdDebounce == false) {

            
            
            cmdDebounce = true;
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {

    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {
        
    }

    public void configureMotors(TalonFX motor, int v, int a, int j) {
        TalonFXConfiguration TalonFXConfigs = new TalonFXConfiguration();
        TalonFXConfigs.Slot0
            .withKS(0.25) // Add 0.25V output to overcome static friction
            .withKV(.12)  // Velocity target of 1 rps results in 0.12V output
            .withKA(.01)  // Acceleration of 1 rps requires 0.01V output
            .withKP(4.8)  // Position error of 2.5 rots results in 12V output
            .withKI(0)    // No output for integrated error
            .withKD(0.1); // Velocity error of 1 rps results in 0.1V output
        
        // MotionMagicConfigs mmc = TalonFXConfigs.MotionMagic;
        // mmc.MotionMagicCruiseVelocity = 80; // Cruise velocity of 80 rps
        // mmc.MotionMagicAcceleration = 160; // Acceleration of 160 rps (.5 seconds)
        // mmc.MotionMagicJerk = 1600; // Jerk of 1600 rps/s/s (.1 seconds)
        
        TalonFXConfigs.MotionMagic
            .withMotionMagicCruiseVelocity(v)
            .withMotionMagicAcceleration(a)
            .withMotionMagicJerk(j);
        // Default is 80-160-1600. Adjust as needed.

        motor.getConfigurator().apply(TalonFXConfigs);
    }
}

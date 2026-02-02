// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Shooter;

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
     */

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
    public static TalonFX RotateIntake = new TalonFX(19);

    // Controller Declarations
    public static Joystick joystick = new Joystick(0); // Patrick
    public static CommandXboxController xbox = new CommandXboxController(1); // Alex

    // Joystick Button/Axis Declarations
    public static JoystickButton orientSwitch = new JoystickButton(joystick, 2);
    public static double speedSlider = joystick.getRawAxis(3);

    // Xbox Controller Button/Axis Declarations (MOVED TO TELEOPINIT)

    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    // debounce
    private boolean cmdDebounce = false;

    

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();
    
    public Robot() {
        m_robotContainer = new RobotContainer();

        

        configureMotors(climb1, 80, 160, 1600);
        configureMotors(climb2, 80, 160, 1600);

        configureMotors(shooter1, 80, 160, 1600);
        configureMotors(shooter2, 80, 160, 1600);

        configureMotors(RaiseLower1, 80, 160, 1600);
        configureMotors(RaiseLower2, 80, 160, 1600);
        configureMotors(RotateIntake, 80, 160, 1600);

        
        
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run(); 

        
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
            xbox.a().onTrue(Commands.print("A button pressed").andThen(Commands.waitSeconds(1)).andThen(Commands.print("5 seconds passed")));
            xbox.b().onTrue(Commands.print("B button pressed"));
            xbox.y().onTrue(Commands.print("Y button pressed"));
            xbox.x().onTrue(Commands.print("X button pressed"));
            xbox.rightTrigger().onTrue(Commands.print("RT button pressed"));
            xbox.leftTrigger().onTrue(Commands.print("LT button pressed"));
            xbox.rightBumper().onTrue(Commands.print("RB button pressed"));
            xbox.leftBumper().onTrue(Commands.print("LB button pressed"));
            xbox.povDown().onTrue(Commands.print("POV down button pressed"));
            xbox.povUp().onTrue(Commands.print("POV up button pressed"));
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
    public void simulationPeriodic() {}

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

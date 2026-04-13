// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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
     *  RaiseLower1: 17 (Right Arm)
     *  RaiseLower2: 18 (Left Arm)
     *  Rotate: 19
     * INFEEDER\
     *  feeder: 20
     *  indexer: 21
     */

    //CANIVORE
    public static CANBus CANivore = new CANBus("drivetrain");

    // Pigeon
    public Pigeon2 gyro = new Pigeon2(12);

    // Climb
    //public static TalonFX climb1 = new TalonFX(13);
    //public static TalonFX climb2 = new TalonFX(14);

    // Shooter
    public static TalonFX shooter1 = new TalonFX(15);
    public static TalonFX shooter2 = new TalonFX(16);

    // Intake
    public static TalonFX RaiseLower1 = new TalonFX(17);
    public static TalonFX RaiseLower2 = new TalonFX(18);
    public static TalonFX RotateIntake = new TalonFX(19);

    // Infeeder
    public static TalonFX feeder = new TalonFX(20);
    public static TalonFX indexer = new TalonFX(21);

    // Controller Declarations
    public static Joystick joystick = new Joystick(0); // Patrick

    // Joystick Button/Axis Declarations
    public static JoystickButton orientSwitch = new JoystickButton(joystick, 2);
    public static double speedSlider = joystick.getRawAxis(3);

    // Xbox Controller Button/Axis Declarations (MOVED TO TELEOPINIT)

    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    public static int index = 0;
   

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();
    
    public Robot() {
        m_robotContainer = new RobotContainer();
        //LimelightHelpers.setupPortForwardingUSB(0);


        //s = .25
        //v = .12
        //a = .01
        //p = 4.8
        //i = 0
        //d = .1
        //Motors

        //configureMotors(climb1, 80, 160, 1600, false, true, 40,0,0,0,0,0,0);
        //configureMotors(climb2, 80, 160, 1600, false, true, 40,0,0,0,0,0,0);

        configureMotors(shooter1, 0, 1000, 5000, true, false, 40, false, 0.02, 0.0092, 0.0, 0.05, 0.001, 0.0);
        configureMotors(shooter2, 0, 1000, 5000, false, false, 40, false, 0.015, 0.0092, 0.0, 0.05, 0.005, 0.0);

        configureMotors(RaiseLower1, 60, 150, 300, false, true, 30, true, .2, .12, .01, 1, 0, 0.15);
        configureMotors(RaiseLower2, 60, 150, 300, false, true, 30, true, .2, .12, .01, 1, 0, 0.15);
        configureMotors(RotateIntake, 0, 1000, 6000, false, false, 40, false, 0.0525, .00877, 0.0001, 0.04, 0, 0);

        configureMotors(feeder, 0, 1000, 6000, false, false, 30, false, 0.013, 0.00865, 0.0001, 0.04, 0, 0);
        configureMotors(indexer, 0, 1000, 6000, true, false, 40, false, 0.02, 0.008573, 0.000, 0.04, 0, 0);
        
    }

    @Override
    public void robotPeriodic() {
        //m_timeAndJoystickReplay.update();
        index += 1;
        CommandScheduler.getInstance().run();
        if (index % 5 == 0) {
            SmartDashboard.putNumber("allianceShiftTime", Math.ceil(RobotContainer.allianceShiftTime*10)/10);
            SmartDashboard.putBoolean("allianceShiftState", RobotContainer.isHubActive());
        }
        //SmartDashboard.putString("Drivetrain Command", 
        //RobotContainer.drivetrain.getCurrentCommand() != null ? 
        //RobotContainer.drivetrain.getCurrentCommand().getName() : "none");
        //SmartDashboard.putNumber("Heartbeat", Timer.getFPGATimestamp());
    }

    @Override
    public void disabledInit() {
        //RobotContainer.ledSubsystem.off();
        }

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

        RobotContainer.shooter.stopShooter();
        
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
        RobotContainer.shooter.stopShooter();
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

    public void configureMotors(TalonFX motor, double v, double a, double j, boolean inverted, boolean brakeMode, double currentLimit, boolean isArmMotor, double ks, double kv, double ka, double kp, double ki, double kd) {
        TalonFXConfiguration TalonFXConfigs = new TalonFXConfiguration();
        TalonFXConfigs.Slot0
            .withKS(ks) // Add 0.25V output to overcome static friction
            .withKV(kv)  // Velocity target of 1 rps results in 0.12V output
            .withKA(ka)  // Acceleration of 1 rps requires 0.01V output
            .withKP(kp)  // Position error of 2.5 rots results in 12V output
            .withKI(ki)    // No output for integrated error
            .withKD(kd); // Velocity error of 1 rps results in 0.1V output

        if(isArmMotor)
        {
            TalonFXConfigs.Slot0.withKG(0.5);
            TalonFXConfigs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        }
        
        //s = .25
        //v = .12
        //a = .01
        //p = 4.8
        //i = 0
        //d = .1

        // MotionMagicConfigs mmc = TalonFXConfigs.MotionMagic;
        // mmc.MotionMagicCruiseVelocity = 80; // Cruise velocity of 80 rps
        // mmc.MotionMagicAcceleration = 160; // Acceleration of 160 rps (.5 seconds)
        // mmc.MotionMagicJerk = 1600; // Jerk of 1600 rps/s/s (.1 seconds)
        
        TalonFXConfigs.MotionMagic
            .withMotionMagicCruiseVelocity(v)
            .withMotionMagicAcceleration(a)
            .withMotionMagicJerk(j);

        MotorOutputConfigs mconfigs = TalonFXConfigs.MotorOutput;

        if(inverted)
        {
            mconfigs.Inverted = InvertedValue.Clockwise_Positive;
        }
        else
        {
            mconfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        }
        
        if(brakeMode)
        {
            mconfigs.NeutralMode = NeutralModeValue.Brake;
        }
        else
        {
            mconfigs.NeutralMode = NeutralModeValue.Coast;
        }

        CurrentLimitsConfigs currentLimitsConfigs = TalonFXConfigs.CurrentLimits;
        currentLimitsConfigs.StatorCurrentLimit = currentLimit;
        currentLimitsConfigs.StatorCurrentLimitEnable = true;

        

        motor.getConfigurator().apply(TalonFXConfigs);
        motor.setPosition(0);
    }
}

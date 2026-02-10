// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.booltoggle;
import frc.robot.subsystems.infeeder;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    //magnitude changed to spin at a slower rate
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);


    // Joysticks and buttons
    public Joystick joystick = Robot.joystick;
    public JoystickButton orientSwitch = Robot.orientSwitch;
    JoystickButton resetFieldCentricHeadingButton = new JoystickButton(joystick, 4);
    JoystickButton brakeButton = new JoystickButton(joystick, 2);
    public static CommandXboxController xbox = new CommandXboxController(1); // Alex
    boolean fieldOrient = true;


    //Subsystems
    public static Intake intake = new Intake();
    public static Shooter shooter = new Shooter();
    public static Climb climb = new Climb();
    public static StateMachine stateMachine = new StateMachine();
    public static Storage storage = new Storage();
    public static booltoggle btoggle = new booltoggle();
    public static infeeder infeed = new infeeder();

    public static final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        configureBindings();
        drivetrain.configurePathPlanner();
        SmartDashboard.putData(Telemetry.m_field);
        PathPlannerLogging.setLogTargetPoseCallback(Telemetry::setTargetPose);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            
            drivetrain.applyRequest(() ->{
                double reduction = (-Robot.joystick.getRawAxis(3) + 1) / 2;
                return drive.withVelocityX(-joystick.getRawAxis(1) * MaxSpeed * reduction) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getRawAxis(0) * MaxSpeed * reduction) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRawAxis(2) * MaxAngularRate * reduction); // Drive counterclockwise with negative X (left)
            })
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        //brakes
        //brakeButton.whileTrue(drivetrain.applyRequest(() -> brake));

        /* STATE MACHINE */

            /* PARAMETER LABELS */
            //Intake: rotate (false = off, true = on), raiselower(DOWN, UP, UNCHANGED)
            //Storage: state (ON, OFF)
            //Shooter: mode (HIGH, LOW, or OFF)
            //StateMachine: stateId (int)
            //Climb: io (UP, DOWN)

            // NOTHING
            xbox.a().onTrue(Commands.print("A button pressed").andThen(Commands.waitSeconds(1)).andThen(Commands.print("5 seconds passed")));
            
            //REST
            xbox.b().onTrue(
                intake.intake(false, "UP")
                .alongWith(storage.storage("OFF"))
                .alongWith(shooter.shoot("OFF"))
                .alongWith(infeed.feed(0))
                .alongWith(stateMachine.SetState(0))
            );
            
            //TRANSITION
            xbox.y().onTrue(
                intake.intake(false, "UNCHANGED")
                .alongWith(storage.storage("ON"))
                .alongWith(shooter.shoot("OFF"))
                .alongWith(infeed.feed(0))
                .alongWith(stateMachine.SetState(2))
            );
            
            //INTAKING
            xbox.x().onTrue(
                intake.intake(true, "DOWN")
                .alongWith(storage.storage("ON"))
                .alongWith(shooter.shoot("OFF"))
                .alongWith(infeed.feed(0))
                .alongWith(stateMachine.SetState(1))
            );
            
            //SHOOTING
            xbox.rightTrigger().onTrue(
                intake.intake(false, "UNCHANGED")
                .alongWith(storage.storage("ON"))
                .alongWith(shooter.shoot("HIGH"))
                .alongWith(infeed.feed(1))
                .alongWith(stateMachine.SetState(1))
            );
            
            //PASSING
            xbox.leftTrigger().onTrue(
                intake.intake(false, "UNCHANGED")
                .alongWith(storage.storage("ON"))
                .alongWith(shooter.shoot("LOW"))
                .alongWith(infeed.feed(0))
                .alongWith(stateMachine.SetState(4))
            );
            
            //NOTHING
            xbox.rightBumper().onTrue(Commands.print("RB button pressed"));
            
            //HOPPER MOVEMENT LOCK (manual override)
            xbox.leftBumper().onTrue(Commands.print("LB button pressed"));
            
            //CLIMB DOWN
            xbox.povDown().onTrue(
                intake.intake(false, "UNCHANGED")
                .alongWith(storage.storage("OFF"))
                .alongWith(shooter.shoot("OFF"))
                .alongWith(infeed.feed(0))
                .alongWith(stateMachine.SetState(5))
                .alongWith(climb.climb("DOWN"))
            );
            
            //CLIMB UP
            xbox.povUp().onTrue(
                intake.intake(false, "UNCHANGED")
                .alongWith(storage.storage("OFF"))
                .alongWith(shooter.shoot("OFF"))
                .alongWith(infeed.feed(0))
                .alongWith(stateMachine.SetState(5))
                .alongWith(climb.climb("UP"))
            );

            orientSwitch.onTrue(
                btoggle.toggleBoolean()
            );

        

        /* 
        Commenting this out because I have no clue what it does, it seems like
        while you're holding a button it just points your wheels instead
        of driving? There was no commented explanation.

        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));
        */

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // This is for a system id routine, if we do sysid to calculate PID values. This could be super useful.
        /*
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        */

        // Reset the field-centric heading on left bumper press.
        resetFieldCentricHeadingButton.onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return AutoBuilder.buildAuto("New Auto");
    }
}

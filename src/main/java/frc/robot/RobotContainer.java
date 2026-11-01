// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.Set;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.counter;
import frc.robot.subsystems.infeeder;
import frc.robot.subsystems.LEDSubsystem;

public class RobotContainer {
        public static double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts
                                                                                                  // desired
                                                                                                  // top speed
        // magnitude changed to spin at a slower rate
        public static double MaxAngularRate = RotationsPerSecond.of(1.5).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                                // second max angular
                                                                                                // velocity

        /* Setting up bindings for necessary control of the swerve drive platform */
        public static SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors
        public static SwerveRequest.RobotCentric rcdrive = new SwerveRequest.RobotCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors

        public static double allianceShiftTime = 0;

        @SuppressWarnings("unused")
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        @SuppressWarnings("unused")
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

        private final Telemetry logger = new Telemetry(MaxSpeed);

        // Joysticks and buttons
        public static CommandXboxController xbox = new CommandXboxController(1); // Alex
        public static CommandXboxController xboxdrive = new CommandXboxController(0); // Patrick
        public static Trigger orientSwitch = xboxdrive.x();
        public static Trigger resetFieldCentricHeadingButton = xboxdrive.y();
        boolean fieldOrient = true;

        // Subsystems
        public static Intake intake = new Intake();
        public static Shooter shooter = new Shooter();
        public static StateMachine stateMachine = new StateMachine();
        public static infeeder infeed = new infeeder();
        public static LEDSubsystem ledSubsystem = new LEDSubsystem();
        public static counter counterSubsystem = new counter();

        public final SendableChooser<Command> chooser;

        // Triggers
        public static Trigger manualOverride = xbox.start();
        // public static Trigger intakeDown = new Trigger(() ->
        // Robot.RotateIntake.getPosition().getValueAsDouble() >= Intake.LOW);

        public static final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

        public RobotContainer() {
                configureBindings();
                configurePathPlannerEventTriggers();
                drivetrain.configurePathPlanner();
                // SmartDashboard.putData(Telemetry.m_field);
                PathPlannerLogging.setLogTargetPoseCallback(Telemetry::setTargetPose);
                chooser = new SendableChooser<>();

                chooser.setDefaultOption("None", Commands.none());
                chooser.addOption("LeftMid2x", AutoBuilder.buildAuto("AimTestLeftMid2x"));
                // chooser.addOption("bsejfsdLeftMid2x",
                // AutoBuilder.buildAuto("imTestLeftMid2x"));

                chooser.addOption("RightMid2x", AutoBuilder.buildAuto("ShooterRightMid2x"));
                // chooser.addOption("TESTING (DO NOT USE) RightMid2x",
                // AutoBuilder.buildAuto("TestingShooterRightMid2x"));
                // chooser.addOption("RightHumanPlayer",
                // AutoBuilder.buildAuto("RightHumanPlayerAuto"));

                chooser.addOption("CenterToHumanPlayer", AutoBuilder.buildAuto("CenterHP"));
                chooser.addOption("CenterHPDepot", AutoBuilder.buildAuto("CenterHPDepot"));
                // chooser.addOption("CenterJustShoot", AutoBuilder.buildAuto("CenterAuto"));
                // chooser.addOption("TEST", AutoBuilder.buildAuto("NewAuto"));

                SmartDashboard.putData("Auto", chooser);

                var followerControl = new Follower(Robot.RaiseLower1.getDeviceID(), MotorAlignmentValue.Opposed);
                Robot.RaiseLower2.setControl(followerControl);

                // SmartDashboard.putData(Telemetry.m_field);

                ledSubsystem.on();

    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> {
                    // double reduction = (-joystick.getRawAxis(3) + 1) / 2;
                    // TODO: setting reduction permanently to one since there isnt a slider on xbox controller. need to ask to see what we can do during the meeting
                    double reduction = 0.6;
                    // SmartDashboard.putNumber("Speed %", reduction);
                    reduction = MathUtil.clamp(reduction, .25, 1);

                    if (fieldOrient) {
                        return drive
                                .withVelocityX(-xboxdrive.getLeftY() * MaxSpeed * reduction)
                                .withVelocityY(-xboxdrive.getLeftX() * MaxSpeed * reduction)
                                .withRotationalRate(-xboxdrive.getRightX() * MaxAngularRate * reduction)
                                .withDeadband(.6)
                                .withRotationalDeadband(1);
                    } else {
                        return rcdrive
                                .withVelocityX(-xboxdrive.getLeftY() * MaxSpeed * reduction)
                                .withVelocityY(-xboxdrive.getLeftX() * MaxSpeed * reduction)
                                .withRotationalRate(-xboxdrive.getRightX() * MaxAngularRate * reduction);
                    }

                }));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        // SysID
        /*
         * xbox.y().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
         * xbox.x().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
         * xbox.a().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
         * xbox.b().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
         */

        // brakes
        // brakeButton.whileTrue(drivetrain.applyRequest(() -> brake));

        /* STATE MACHINE */

        /* PARAMETER LABELS */
        // Intake: rotate (false = off, true = on), raiselower(DOWN, UP, UNCHANGED)
        // Storage: state (ON, OFF)
        // Shooter: mode (HIGH, LOW, or OFF)
        // StateMachine: stateId (int)
        // Climb: io (UP, DOWN)

        // REST
        xbox.b().and(manualOverride.negate()).onTrue(
                intake.intake(false, "UP")
                        .alongWith(shooter.shoot("OFF"))
                        .alongWith(stateMachine.SetState(0))
                        .alongWith(ledSubsystem.runPattern("REST")));

        // TRANSITION
        xbox.y().and(manualOverride.negate()).onTrue(
                intake.intake(false, "UNCHANGED")
                        .alongWith(shooter.shoot("OFF"))
                        .alongWith(stateMachine.SetState(2))
                        .alongWith(ledSubsystem.runPattern("REST")));

        // INTAKING
        xbox.x().and(manualOverride.negate()).onTrue(
                intake.intake(true, "DOWN")
                        .alongWith(shooter.shoot("OFF"))
                        .alongWith(stateMachine.SetState(1))
                        .alongWith(ledSubsystem.runPattern("INTAKE")));

        // SHOOTING
        xbox.rightTrigger().and(manualOverride.negate()).whileTrue(
                shooter.shoot("HIGH")
                        .alongWith(stateMachine.SetState(3))
                        .alongWith(ledSubsystem.runPattern("SHOOT")));

        xbox.rightTrigger().and(manualOverride.negate()).onFalse(
                shooter.shoot("OFF")
                        .alongWith(stateMachine.SetState(3))
                        .alongWith(ledSubsystem.runPattern("SHOOT")));

        // PASSING
        xbox.leftTrigger().and(manualOverride.negate()).whileTrue(
                shooter.shoot("LOW")
                        .alongWith(stateMachine.SetState(4)));
        xbox.leftTrigger().and(manualOverride.negate()).onFalse(
                shooter.shoot("OFF")
                        .alongWith(stateMachine.SetState(4))
                        .alongWith(ledSubsystem.runPattern("PASS")));

        // INTAKE WHILE SHOOT
        xbox.rightBumper().and(manualOverride.negate()).whileTrue(
                intake.intake(true, "DOWN")
                        .alongWith(shooter.shoot("HIGH"))
                        .alongWith(stateMachine.SetState(7))
                        .alongWith(ledSubsystem.runPattern("SHOOT")));
        xbox.rightBumper().and(manualOverride.negate()).onFalse(
                intake.intake(false, "DOWN")
                        .alongWith(shooter.shoot("OFF"))
                        .alongWith(stateMachine.SetState(7))
                        .alongWith(ledSubsystem.runPattern("SHOOT")));
        // INTAKE WHILE PASS
        xbox.leftBumper().and(manualOverride.negate()).whileTrue(
                intake.intake(true, "DOWN")
                        .alongWith(shooter.shoot("LOW"))
                        .alongWith(stateMachine.SetState(4))
                        .alongWith(ledSubsystem.runPattern("PASS")));
        xbox.leftBumper().and(manualOverride.negate()).onFalse(
                intake.intake(false, "DOWN")
                        .alongWith(shooter.shoot("OFF"))
                        .alongWith(stateMachine.SetState(4))
                        .alongWith(ledSubsystem.runPattern("PASS")));

        // FLOOR PASS
        xbox.povDown().and(manualOverride.negate()).onTrue(
                intake.testRotate(-100));
        xbox.povDown().and(manualOverride.negate()).onFalse(
                intake.stopRoller());

        // AGITATE
        xbox.a().and(manualOverride.negate()).onTrue(intake.testBoth(60, 9.25));
        xbox.a().and(manualOverride.negate()).onFalse(intake.intake(false, "DOWN"));

        // intakeDown.onTrue(intake.testIntakePower(0));
        /*
         * //CLIMB DOWN
         * xbox.povDown().and(manualOverride.negate()).onTrue(
         * intake.intake(false, "UNCHANGED")
         * .alongWith(shooter.shoot("OFF"))
         * .alongWith(stateMachine.SetState(5))
         * .alongWith(ledSubsystem.runPattern("CLIMB"))
         * );
         * 
         * //CLIMB UP
         * xbox.povUp().and(manualOverride.negate()).onTrue(
         * intake.intake(false, "UNCHANGED")
         * .alongWith(shooter.shoot("OFF"))
         * .alongWith(stateMachine.SetState(5))
         * .alongWith(ledSubsystem.runPattern("CLIMB"))
         * );
         */
        // ORIENT SWITCH
        orientSwitch.onTrue(
                Commands.runOnce(() -> {
                    fieldOrient = !fieldOrient;
                    System.out.println("Field Oriented: " + fieldOrient);
                }));

        // MANUAL OVERRIDE CONTROLS
        // INTAKE SPINNER
        xbox.povRight().and(manualOverride).onTrue(
                intake.testRotate(10));
        xbox.povLeft().and(manualOverride).onTrue(
                intake.testRotate(-10));
        xbox.povLeft().and(manualOverride).onFalse(
                intake.stopRoller());
        xbox.povRight().and(manualOverride).onFalse(
                intake.stopRoller());
        // INTAKE ARM ADJUST
        xbox.povUp().and(manualOverride).onTrue(
                intake.testIntakePower(-0.05));
        xbox.povDown().and(manualOverride).onTrue(
                intake.testIntakePower(0.05));
        xbox.povUp().and(manualOverride).onFalse(
                intake.testIntakePower(0));
        xbox.povDown().and(manualOverride).onFalse(
                intake.testIntakePower(0));
        // RESET ZERO
        xbox.a().and(manualOverride).onTrue(
                intake.Rezero("UP"));
        xbox.leftBumper().and(manualOverride).onTrue(
                intake.Rezero("DOWN"));
        // INDEXER AND FEEDER
        xbox.x().and(manualOverride).onTrue(
                infeed.testfeed(-10).alongWith(shooter.testShooters(22.4)));
        xbox.b().and(manualOverride).onTrue(
                infeed.testfeed(10).alongWith(shooter.testShooters(-22.4)));
        xbox.b().and(manualOverride).onFalse(
                infeed.testfeed(0).alongWith(shooter.stopShooterAuto()));
        xbox.x().and(manualOverride).onFalse(
                infeed.testfeed(0).alongWith(shooter.stopShooterAuto()));

        /*
         * //CLIMB
         * xbox.povUp().and(manualOverride).onTrue(
         * climb.testClimbVoltage(1)
         * );
         * xbox.povDown().and(manualOverride).onTrue(
         * climb.testClimbVoltage(-1)
         * );
         */

        /*
         * Commenting this out because I have no clue what it does, it seems like
         * while you're holding a button it just points your wheels instead
         * of driving? There was no commented explanation.
         * 
         * joystick.b().whileTrue(drivetrain.applyRequest(() ->
         * point.withModuleDirection(new Rotation2d(-joystick.getLeftY(),
         * -joystick.getLeftX()))
         * ));
         */

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // This is for a system id routine, if we do sysid to calculate PID values. This
        // could be super useful.
        /*
         * joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction
         * .kForward));
         * joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction
         * .kReverse));
         * joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(
         * Direction.kForward));
         * joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(
         * Direction.kReverse));
         */

        // Reset the field-centric heading on left bumper press.
        resetFieldCentricHeadingButton.onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // drivetrain.registerTelemetry(logger::telemeterize);

        // drivetrain.resetPose(new Pose2d(new Translation2d(16.137, 7.690), new
        // Rotation2d(Math.toRadians(180))));
    }

    public void configurePathPlannerEventTriggers() {
        NamedCommands.registerCommand("intakedown", intake.testBoth(100, 8.75));
        NamedCommands.registerCommand("intake", intake.intake(true, "DOWN"));
        NamedCommands.registerCommand("stopintake", intake.intake(false, "DOWN"));

        NamedCommands.registerCommand("shoot", shooter.shootAuto());
        NamedCommands.registerCommand("stopshoot", shooter.stopShooterAuto());

        NamedCommands.registerCommand("feed", infeed.feedAuto(true));
        NamedCommands.registerCommand("stopfeed", infeed.feedAuto(false));
        NamedCommands.registerCommand("reversefeed", infeed.testindexer(-5));

        NamedCommands.registerCommand("ActivateLEDs", ledSubsystem.runPattern("AUTO"));
        // NamedCommands.registerCommand("climb", climb.climb("UP"));
        // new EventTrigger("climb").onTrue(Commands.print("Climbing"));
    }

    public static boolean isHubActive() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        // If we have no alliance, we cannot be enabled, therefore no hub.
        if (alliance.isEmpty()) {
            return false;
        }
        // Hub is always enabled in autonomous.
        if (DriverStation.isAutonomousEnabled()) {
            return true;
        }
        // At this point, if we're not teleop enabled, there is no hub.
        if (!DriverStation.isTeleopEnabled()) {
            return false;
        }
        return false;
}


        public Command getAutonomousCommand() {
                return chooser.getSelected();
        }
}

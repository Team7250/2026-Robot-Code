package frc.robot.subsystems;

import java.util.Optional;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class Shooter extends SubsystemBase {
    
    private final double HIGHPOWER = 60; // Rotations per second
    private final double LOWPOWER = 30;  // Rotations per second

    // Pre-allocate the swerve request
    private final FieldCentricFacingAngle driveAtAngle = 
        new SwerveRequest.FieldCentricFacingAngle();

    public Shooter() {}

    private final edu.wpi.first.math.controller.PIDController headingPID = 
        new edu.wpi.first.math.controller.PIDController(5.0, 0, 0.1); 

    public Command shoot(String mode) {
        if(mode.equals("HIGH"))
        {
            return Commands.deferredProxy(() -> {
                // Hub Positions
                Optional<Alliance> alliance = DriverStation.getAlliance();
                final Translation2d hubPosition = (alliance.isPresent() && alliance.get() == Alliance.Red)
                    ? new Translation2d(11.911, 4.305)
                    : new Translation2d(4.627, 4.305);

                // Configure the PID for circular wrapping (so it doesn't spin 359 degrees to go left 1 degree)
                headingPID.enableContinuousInput(-Math.PI, Math.PI);

                // The Aiming Command
                Command aimAndDrive = Commands.run(() -> {
                    Pose2d currentPose = RobotContainer.drivetrain.getState().Pose;
                    
                    // MATH: Calculate angle to hub
                    Rotation2d targetAngle = hubPosition.minus(currentPose.getTranslation()).getAngle();

                    // Calculate rotational velocity
                    // We use current rotation and target angle (both in radians)
                    double rotationOutput = headingPID.calculate(
                        currentPose.getRotation().getRadians(), 
                        targetAngle.getRadians()
                    );

                    // Use the standard FieldCentric request
                    RobotContainer.drivetrain.setControl(
                        new SwerveRequest.FieldCentric()
                            .withVelocityX(-Robot.joystick.getRawAxis(1) * 1.0)
                            .withVelocityY(-Robot.joystick.getRawAxis(0) * 1.0)
                            .withRotationalRate(rotationOutput) // Apply our PID result
                            .withDeadband(0.1)
                    );
                }, RobotContainer.drivetrain);

                // Shooter Motor Command
                Command spinUp = runOnce(() -> {
                    double rps = mode.equals("HIGH") ? HIGHPOWER : (mode.equals("LOW") ? LOWPOWER : 0);
                    Robot.shooter1.setControl(new MotionMagicVelocityVoltage(rps));
                    Robot.shooter2.setControl(new MotionMagicVelocityVoltage(rps));
                });

                return spinUp.andThen(aimAndDrive).finallyDo((interrupted) -> stopShooter());
            });
        }
        else if (mode.equals("LOW"))
        {
            return Commands.print("PASSING MODE");
        }
        else
        {
            return Commands.print("SHOOTER OFF");
        }
        
    }

    public void stopShooter() {
        Robot.shooter1.setControl(new MotionMagicVelocityVoltage(0));
        Robot.shooter2.setControl(new MotionMagicVelocityVoltage(0));
    }
}
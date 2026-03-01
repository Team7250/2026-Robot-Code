package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class Shooter extends SubsystemBase {

    //create a map for smooth velocity scaling
    private static InterpolatingDoubleTreeMap shooterVelocitiesMap = new InterpolatingDoubleTreeMap();
    public static double shootspeed;

    public Shooter() {
        //we add our keys in the format of distance (meters), velocity (rotations per second)
        shooterVelocitiesMap.put(1.9304, 45.0);
        shooterVelocitiesMap.put(2.3876, 48.0);
        shooterVelocitiesMap.put(3.6576, 55.0);
        
        
    }
    //we'll have to tune this when robot is ready
    private final edu.wpi.first.math.controller.PIDController headingPID = 
        new edu.wpi.first.math.controller.PIDController(5.0, 0, 0.1); 

    //this is for calculating the vluaes that we need to put into the tree
    public Command testShooters(double rps)
    {
        return runOnce(() -> {
            Robot.shooter1.setControl(new MotionMagicVelocityDutyCycle(rps));
            Robot.shooter2.setControl(new MotionMagicVelocityDutyCycle(rps));
        });
    }

    //the actual shoot method
    public Command shoot(String mode) {

        //HIGH means auto aim while driving
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
                    Rotation2d targetAngle = hubPosition.minus(currentPose.getTranslation()).getAngle();//.plus(Rotation2d.fromDegrees(180));

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
                    

                    double rps = getShooterRate();
                    Robot.shooter1.setControl(new MotionMagicVelocityDutyCycle(rps));
                    Robot.shooter2.setControl(new MotionMagicVelocityDutyCycle(rps));

                    if(Robot.shooter1.getClosedLoopError(true).getValue() < 1)
                    {
                        infeeder.feed(true);
                    }
                    shootspeed = rps;

                }, RobotContainer.drivetrain);

                // Shooter Motor Command
                Command spinUp = runOnce(() -> {
                    double rps = getShooterRate();
                    Robot.shooter1.setControl(new MotionMagicVelocityDutyCycle(rps));
                    Robot.shooter2.setControl(new MotionMagicVelocityDutyCycle(rps));
                });

                return spinUp.andThen(aimAndDrive).finallyDo((interrupted) -> {
                    stopShooter();
                });
            });
        }

        //LOW means passing
        else if (mode.equals("LOW"))
        {
            return runOnce( () -> {
                double rps = 10;
                Robot.shooter1.setControl(new MotionMagicVelocityDutyCycle(rps));
                Robot.shooter2.setControl(new MotionMagicVelocityDutyCycle(rps));
            });
        }

        //AUTO is for shooting during auto, aiming will be done by pathplanner
        else if(mode.equals("AUTO"))
        {
            return runOnce( () -> {
                double rps = getShooterRate();
                Robot.shooter1.setControl(new MotionMagicVelocityDutyCycle(rps));
                Robot.shooter2.setControl(new MotionMagicVelocityDutyCycle(rps));
            });
        }
        else
        {
            return runOnce(() -> {
                stopShooter();
            });
        }
        
    }

    //calls the map to get the estimated shooter rate for current position
    public static double getShooterRate()
    {
        Pose2d currPose = RobotContainer.drivetrain.getState().Pose;
        Optional<Alliance> alliance = DriverStation.getAlliance();
        final Translation2d hubPosition = (alliance.isPresent() && alliance.get() == Alliance.Red)
            ? new Translation2d(11.911, 4.305)
            : new Translation2d(4.627, 4.305);
        
        double distance = currPose.getTranslation().getDistance(hubPosition);
        //we can just get from shooter map, it will automatically determine the value, even if it's not one we put in the table. It's very precise.
        return shooterVelocitiesMap.get(distance);
    }

    public void stopShooter() {
        Robot.shooter1.set(0);
        Robot.shooter2.set(0);
        infeeder.feed(false);
    }

    private final VoltageOut voltReq = new VoltageOut(0.0);
    SysIdRoutine routine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            Volts.of(4),
            null,
            (state) -> SignalLogger.writeString("state", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            (volts) -> Robot.shooter1.setControl(voltReq.withOutput(volts.in(Volts))),
            null,
            this
        )   
    );

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }
}
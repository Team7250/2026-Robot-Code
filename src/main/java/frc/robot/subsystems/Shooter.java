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
        shooterVelocitiesMap.put(2.032, 46.0);
        shooterVelocitiesMap.put(2.3876, 48.0);
        shooterVelocitiesMap.put(2.7432, 50.0);
        shooterVelocitiesMap.put(3.2512, 53.0);
        shooterVelocitiesMap.put(3.3274, 53.5);
        shooterVelocitiesMap.put(3.7338, 56.0);
        shooterVelocitiesMap.put(3.9624, 60.0);
        shooterVelocitiesMap.put(4.6482, 61.0);
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
                    Rotation2d targetAngle = getPredictedTargetAngle(hubPosition);//hubPosition.minus(currentPose.getTranslation()).getAngle();//.plus(Rotation2d.fromDegrees(180));

                    // Calculate rotational velocity
                    // We use current rotation and target angle (both in radians)
                    double rotationOutput = headingPID.calculate(
                        currentPose.getRotation().getRadians(), 
                        targetAngle.getRadians()
                    );
                    
                    // Use the standard FieldCentric request
                    RobotContainer.drivetrain.setControl(
                        new SwerveRequest.FieldCentric()
                            .withVelocityX(-Robot.joystick.getRawAxis(1) * .5)
                            .withVelocityY(-Robot.joystick.getRawAxis(0) * .5)
                            .withRotationalRate(rotationOutput) // Apply our PID result
                            .withDeadband(0.1)
                    );
                    

                    double rps = getShooterRate();
                    Robot.shooter1.setControl(new MotionMagicVelocityDutyCycle(rps));
                    Robot.shooter2.setControl(new MotionMagicVelocityDutyCycle(rps));

                    if(Math.abs(Robot.shooter1.getClosedLoopError(true).getValue()) < 5.0 && Robot.shooter1.getVelocity(true).getValueAsDouble() > 25.0)
                    {
                        infeeder.feed(true);
                    }
                    else
                    {
                        infeeder.feed(false);
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
                double rps = 60;
                Robot.shooter1.setControl(new MotionMagicVelocityDutyCycle(rps));
                Robot.shooter2.setControl(new MotionMagicVelocityDutyCycle(rps));
                infeeder.feed(true);
            });
        }
        else
        {
            return runOnce(() -> {
                stopShooter();
            });
        }
        
    }

    public Command shootAuto()
    {
        return runOnce( () -> {
            double rps = getShooterRate();
            Robot.shooter1.setControl(new MotionMagicVelocityDutyCycle(rps));
            Robot.shooter2.setControl(new MotionMagicVelocityDutyCycle(rps));
            //infeeder.feed(true);
        });
    }

    public Command stopShooterAuto()
    {
        return runOnce( () -> {
            Robot.shooter1.set(0);
            Robot.shooter2.set(0);
            //infeeder.feed(true);
        });
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

    public Rotation2d getPredictedTargetAngle(Translation2d hubPosition) {
        var state = RobotContainer.drivetrain.getState();   
        Translation2d currentTranslation = state.Pose.getTranslation();
        
        double distance = currentTranslation.getDistance(hubPosition);
        
        double timeOfFlight = distance / 5.0; 
        double systemLatency = 0.16; 
        double totalLookAhead = timeOfFlight + systemLatency;

        double vx = state.Speeds.vxMetersPerSecond;
        double vy = state.Speeds.vyMetersPerSecond;

        Translation2d predictedTranslation = new Translation2d(
            currentTranslation.getX() - (vx * totalLookAhead),
            currentTranslation.getY() - (vy * totalLookAhead)
        );

        return hubPosition.minus(predictedTranslation).getAngle();
    }

    public static double getPredictedShooterRate() {
        var state = RobotContainer.drivetrain.getState();
        Pose2d currPose = state.Pose;
        Translation2d robotVelocity = new Translation2d(
            state.Speeds.vxMetersPerSecond, 
            state.Speeds.vyMetersPerSecond
        );

        Optional<Alliance> alliance = DriverStation.getAlliance();
        final Translation2d hubPosition = (alliance.isPresent() && alliance.get() == Alliance.Red)
            ? new Translation2d(11.911, 4.305)
            : new Translation2d(4.627, 4.305);

        // 1. Calculate current distance and initial ToF estimate
        double currentDistance = currPose.getTranslation().getDistance(hubPosition);
        double timeOfFlight = currentDistance / 5; // Assume ~13m/s average exit velocity
        
        // 2. Predict distance at the moment of impact
        Translation2d predictedTranslation = currPose.getTranslation().plus(
            robotVelocity.times(timeOfFlight + 0.08) // Velocity * (ToF + Latency)
        );
        double predictedDistance = predictedTranslation.getDistance(hubPosition);

        // 3. Get base RPS from your map using the PREDICTED distance
        double baseRps = shooterVelocitiesMap.get(predictedDistance);

        // 4. Compensate for "Added Velocity" from robot movement
        // We project robot velocity onto the vector pointing to the hub
        Translation2d unitVectorToHub = hubPosition.minus(currPose.getTranslation()).div(currentDistance);
        
        // Dot product: how much of our 2D velocity is directed AT the hub?
        double radialVelocity = (robotVelocity.getX() * unitVectorToHub.getX()) + 
                                (robotVelocity.getY() * unitVectorToHub.getY());

        // Convert radial velocity (m/s) to an RPS offset. 
        // This is a "fudge factor" you'll need to tune. 
        // Usually, 1 m/s of robot speed equals ~1.5 to 2.5 RPS adjustment.
        double velocityAdjustmentFactor = 2.0; 
        double finalRps = baseRps - (radialVelocity * velocityAdjustmentFactor);

        return finalRps;
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
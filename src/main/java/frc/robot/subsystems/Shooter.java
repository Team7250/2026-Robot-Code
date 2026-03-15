package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;
import java.util.Optional;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class Shooter extends SubsystemBase {

    private static InterpolatingDoubleTreeMap shooterVelocitiesMap = new InterpolatingDoubleTreeMap();
    public static double shootspeed;

    // --- TUNABLES  ---
    private final double BASE_EXIT_VELO = 11.0; // lower = stronger
    private final double HIGH_SPEED_EXIT_VELO = 2.0; 
    private final double BASE_RADIAL_FUDGE = 2.5; // higher = stronger
    private final double HIGH_SPEED_RADIAL_FUDGE = 15.0; 
    private final double FEEDER_DELAY = 0.08; 

    // --- CALIBRATION ---
    // This ONLY applies when movingRight is true.
    private final double RIGHT_SIDE_FUDGE_DEG = 5.0; 
    // Multiplier for leading the shot specifically when moving toward driver's right.
    private final double RIGHT_SIDE_BIAS_SCALAR = 1.35; 

    private double targetRPS = 0;
    private Rotation2d targetHeading = new Rotation2d();
    private double currentMeasuredLatency = 0.02; 
    public double reduction = 1.0; // RE-ADDED

    private final edu.wpi.first.math.controller.PIDController headingPID = 
        new edu.wpi.first.math.controller.PIDController(6.0, 0, 0.2); 

    public Shooter() {
        shooterVelocitiesMap.put(2.0, 48.0);
        shooterVelocitiesMap.put(3.0, 52.0);
        shooterVelocitiesMap.put(4.0, 59.0);
        shooterVelocitiesMap.put(5.0, 63.0);

        headingPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void periodic() {
        var state = RobotContainer.drivetrain.getState();
        Pose2d currentPose = state.Pose;

        // 1. Sync Latency
        currentMeasuredLatency = com.ctre.phoenix6.Utils.getCurrentTimeSeconds() - state.Timestamp;
        currentMeasuredLatency = Math.max(0, Math.min(currentMeasuredLatency, 0.1));

        // 2. Speed-Based Constant Scaling
        double robotSpeed = Math.sqrt(Math.pow(state.Speeds.vxMetersPerSecond, 2) + Math.pow(state.Speeds.vyMetersPerSecond, 2));
        double scalar = MathUtil.clamp(robotSpeed / RobotContainer.MaxSpeed, 0, 1);
        
        double dynamicExitVelo = MathUtil.interpolate(BASE_EXIT_VELO, HIGH_SPEED_EXIT_VELO, scalar);
        double dynamicRadialFudge = MathUtil.interpolate(BASE_RADIAL_FUDGE, HIGH_SPEED_RADIAL_FUDGE, scalar);

        // 3. Convert Robot Velocity to Field Coordinates
        Translation2d robotVelField = new Translation2d(state.Speeds.vxMetersPerSecond, state.Speeds.vyMetersPerSecond)
            .rotateBy(currentPose.getRotation());

        // 4. Alliance & Hub Parameters
        Optional<Alliance> alliance = DriverStation.getAlliance();
        boolean isRed = alliance.isPresent() && alliance.get() == Alliance.Red;
        final Translation2d hubPos = isRed ? new Translation2d(11.911, 4.305) : new Translation2d(4.627, 4.305);

        // 5. Predictive Position Lookahead
        double lookahead = currentMeasuredLatency + FEEDER_DELAY;
        Translation2d predictedPose = currentPose.getTranslation().plus(robotVelField.times(lookahead));
        
        Translation2d toHubVec = hubPos.minus(predictedPose);
        double dist = toHubVec.getNorm();

        // 6. Directional Lead Bias Logic
        double vx = robotVelField.getX();
        double vy = robotVelField.getY();
        
        // Check if moving to driver's right (Red: +Y, Blue: -Y)
        boolean movingRight = isRed ? (vy > 0.1) : (vy < -0.1);
        boolean movingLeft = isRed ? (vy < -0.1) : (vy > 0.1);
        
        double currentFudge = 0;
        if (movingRight && isRed) {
            vy *= RIGHT_SIDE_BIAS_SCALAR;
            currentFudge = RIGHT_SIDE_FUDGE_DEG; // Only apply fudge to the right
        }
        else if(movingLeft && !isRed)
        {
            vy *= RIGHT_SIDE_BIAS_SCALAR;
            currentFudge = -RIGHT_SIDE_FUDGE_DEG - 2;
        }
        
        Translation2d compensatedVel = new Translation2d(vx, vy);

        // 7. Core Vector Math: V_shot = V_target_to_hub - V_robot_motion
        Translation2d unitToHub = toHubVec.div(dist);
        Translation2d shotVeloVec = unitToHub.times(dynamicExitVelo).minus(compensatedVel);
        
        // Apply target heading with the conditional fudge
        targetHeading = shotVeloVec.getAngle().plus(Rotation2d.fromDegrees(currentFudge));

        // 8. Radial RPS Compensation (Dot Product)
        double baseRps = shooterVelocitiesMap.get(dist);
        double radialVel = (robotVelField.getX() * unitToHub.getX()) + (robotVelField.getY() * unitToHub.getY());
        targetRPS = baseRps - (radialVel * dynamicRadialFudge);

        reduction = MathUtil.clamp((-Robot.joystick.getRawAxis(3) + 1) / 2, 0.2, 1.0);

        Robot.shooter1.setControl(new MotionMagicVelocityDutyCycle(targetRPS));
        Robot.shooter2.setControl(new MotionMagicVelocityDutyCycle(targetRPS));
        

        // Dashboard Telemetry
        /*SmartDashboard.putNumber("Shooter/Target RPS", targetRPS);
        SmartDashboard.putNumber("Shooter/Target Heading", targetHeading.getDegrees());
        SmartDashboard.putBoolean("Shooter/Is Moving Right", movingRight);
        SmartDashboard.putNumber("Shooter/Current Fudge", currentFudge);
        SmartDashboard.putNumber("Shooter/Reduction", reduction);
        SmartDashboard.putNumber("Speed %", reduction);*/
    }

    public Command shoot(String mode) {
        if(mode.equals("HIGH")) {
            return Commands.run(() -> {
                double currentRot = RobotContainer.drivetrain.getState().Pose.getRotation().getRadians();
                double rotOutput = headingPID.calculate(currentRot, targetHeading.getRadians());
                
                // Uses the reduction variable for drivetrain speed
                RobotContainer.drivetrain.setControl(new SwerveRequest.FieldCentric()
                    .withVelocityX(-Robot.joystick.getRawAxis(1) * RobotContainer.MaxSpeed * reduction * 0.5)
                    .withVelocityY(-Robot.joystick.getRawAxis(0) * RobotContainer.MaxSpeed * reduction * 0.5)
                    .withRotationalRate(rotOutput)
                    .withDeadband(0.1));
                
                

                if(Math.abs(currentRot) - Math.abs(targetHeading.getRadians()) < 1)  {
                    infeeder.feed(true);
                } else {
                    infeeder.feed(false);
                }
                shootspeed = targetRPS;
            }, this, RobotContainer.drivetrain).finallyDo(i -> stopShooter());
        }
        else if (mode.equals("LOW")) {
            return runOnce(() -> {
                //Robot.shooter1.setControl(new MotionMagicVelocityDutyCycle(60));
                //Robot.shooter2.setControl(new MotionMagicVelocityDutyCycle(60));
                infeeder.feed(true);
            });
        }
        return runOnce(this::stopShooter);
    }

    public void stopShooter() {
        Robot.shooter1.set(0); 
        Robot.shooter2.set(0); 
        infeeder.feed(false);
    }

    public Command testShooters(double rps) {
        return runOnce(() -> {
            Robot.shooter1.setControl(new MotionMagicVelocityDutyCycle(rps));
            Robot.shooter2.setControl(new MotionMagicVelocityDutyCycle(rps));
        });
    }

    public Command shootAuto() {
        return runOnce(() -> {
            Robot.shooter1.setControl(new MotionMagicVelocityDutyCycle(targetRPS));
            Robot.shooter2.setControl(new MotionMagicVelocityDutyCycle(targetRPS));
        });
    }

    public Command stopShooterAuto() {
        return runOnce(() -> {
            Robot.shooter1.set(0);
            Robot.shooter2.set(0);
        });
    }

    private final VoltageOut vReq = new VoltageOut(0.0);
    SysIdRoutine routine = new SysIdRoutine(
        new SysIdRoutine.Config(null, Volts.of(4), null, (s) -> SignalLogger.writeString("state", s.toString())),
        new SysIdRoutine.Mechanism((v) -> Robot.shooter1.setControl(vReq.withOutput(v.in(Volts))), null, this)
    );

    public Command sysIdQuasistatic(SysIdRoutine.Direction d) { return routine.quasistatic(d); }
    public Command sysIdDynamic(SysIdRoutine.Direction d) { return routine.dynamic(d); }
}
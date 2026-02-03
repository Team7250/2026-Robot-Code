package frc.robot.subsystems;

import java.util.Optional;

import javax.lang.model.util.ElementScanner14;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class Shooter extends SubsystemBase {
    
    double HIGHPOWER = 60; // Rotations per second
    double LOWPOWER = 30; // Rotations per second


    Translation2d hubPosition;
    Pose2d currentPose;

    //fill in table with the necessary shooter RPM scaled by 0.5m distance from hub each time.
    double table[];


    String mode = "";
    public Shooter(String mode) {
        this.mode = mode;
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if(alliance.isPresent())
        {
            if(alliance.get() == Alliance.Red)
            {
                hubPosition = new Translation2d(11.911, 4.305);
            }
            if(alliance.get() == Alliance.Blue)
            {
                hubPosition = new Translation2d(4.627, 4.305);
            }
        }
    }

    public Command shoot() {
        return runOnce(
            () -> {
                final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(0);
                m_request.Acceleration = 500;

                if(mode == "HIGH")
                {
                    System.out.println("Attempting to allign to Hub");
                    /*
                    double rps;
                    double distanceFromHub;
                    currentPose = RobotContainer.drivetrain.getState().Pose;
                    distanceFromHub = currentPose.getTranslation().getDistance(hubPosition);
                    distanceFromHub = Math.round(distanceFromHub * 2.0) / 2.0; // rounds it to the nearest 0.5
                    distanceFromHub = (distanceFromHub * 2) - 1;
                    rps = table[(int)distanceFromHub];*/
                    currentPose = RobotContainer.drivetrain.getState().Pose;

                    Translation2d translationToTarget = currentPose.getTranslation().minus(hubPosition);
                    Rotation2d angleToTarget = new Rotation2d(translationToTarget.getX(), translationToTarget.getY());
                    Pose2d targetPose = new Pose2d(currentPose.getTranslation(),angleToTarget);

                    AutoBuilder.pathfindToPose(targetPose, new PathConstraints(3.0, 1.5, 360, 720));

                    
                    //Robot.shooter1.setControl(m_request.withVelocity(rps));
                    //Robot.shooter2.setControl(m_request.withVelocity(rps));

                    Robot.shooter1.setControl(m_request.withVelocity(HIGHPOWER));
                    Robot.shooter2.setControl(m_request.withVelocity(HIGHPOWER));
                }
                else if(mode == "LOW")
                {
                    Robot.shooter1.setControl(m_request.withVelocity(LOWPOWER));
                    Robot.shooter2.setControl(m_request.withVelocity(LOWPOWER));
                }
                else
                {
                    Robot.shooter1.setControl(m_request.withVelocity(0));
                    Robot.shooter2.setControl(m_request.withVelocity(0));
                }
            }
        );
    }
}

package frc.robot.subsystems;

import javax.lang.model.util.ElementScanner14;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Shooter extends SubsystemBase {
    
    double HIGHPOWER = 60; // Rotations per second
    double LOWPOWER = 30; // Rotations per second


    String mode = "";
    public Shooter(String mode) {
        this.mode = mode;
    }

    public Command shoot() {
        return runOnce(
            () -> {
                final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(0);
                m_request.Acceleration = 500;

                if(mode == "HIGH")
                {
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

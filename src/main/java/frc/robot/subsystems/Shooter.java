package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Shooter extends SubsystemBase {
    double force = 0;
    public Shooter(double force) {
        this.force = force;
    }

    public Command shoot() {
        return runOnce(
            () -> {
                final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(0);

                m_request.Acceleration = 500;
                Robot.shooter1.setControl(m_request.withVelocity(force));
                Robot.shooter2.setControl(m_request.withVelocity(force));
            }
        );
    }
}

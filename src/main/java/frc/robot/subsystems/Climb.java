package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Climb extends SubsystemBase {
    
    int UPPOSITION = 50;
    int DOWNPOSITION = 0;
    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
    
    
    String io;
    public Climb() {
    }

    //test functions for climb, one for positional and one for voltage
    public Command testClimb(double position)
    {
        return runOnce(() -> {
            Robot.climb1.setControl(m_request.withPosition(position));
            Robot.climb2.setControl(m_request.withPosition(position));
        });
    }
    public Command testClimbVoltage(double voltage)
    {
        return runOnce(() -> {
            Robot.climb1.set(voltage);
            Robot.climb2.set(voltage);
        });
    }

    public Command climb(String io) {
        return runOnce(
            () -> {
                
                switch (io) {
                    case "up":
                        Robot.climb1.setControl(m_request.withPosition(UPPOSITION));
                        Robot.climb2.setControl(m_request.withPosition(UPPOSITION));
                    case "down":
                        Robot.climb1.setControl(m_request.withPosition(DOWNPOSITION));
                        Robot.climb2.setControl(m_request.withPosition(DOWNPOSITION));
                    default:
                        Robot.climb1.set(0);
                        Robot.climb2.set(0);
                }
            }
        );
    }
}
package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class Agitator extends SubsystemBase {
    
    private final Servo actuator;
    private final Servo actuator2;

    final double HOME = 0.025;
    final double OUT = 0.25;
    final double EXTENSIONTIME = 500;
    final double OFFSETFROMOTHER = 0.05;
    
    String io;
    public Agitator() {
        actuator = new Servo(2);
        actuator2 = new Servo(1);
        actuator.setBoundsMicroseconds(2000, 1501, 1500, 1499, 1000);
        actuator2.setBoundsMicroseconds(2000, 1501, 1500, 1499, 1000);
    }
    int i = 0;
    @Override
    public void periodic()
    {
        if(i % EXTENSIONTIME < EXTENSIONTIME / 2)
        {
            setPositionLocal(HOME);
        }
        else
        {
            setPositionLocal(OUT);
        }
        i++;
    }
    
    public Command setPosition(double position)
    {
        return runOnce(() -> {
            actuator.set(position);
            actuator2.set(position);
        });
    }

    public void setPositionLocal(double position)
    {
        actuator.set(position + OFFSETFROMOTHER);
        actuator2.set(position);
    }
    /*
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
                    case "UP":
                        Robot.climb1.setControl(m_request.withPosition(UPPOSITION));
                        Robot.climb2.setControl(m_request.withPosition(UPPOSITION));
                        RobotContainer.ClimbPercent = 1;
                        break;
                    case "DOWN":
                        Robot.climb1.setControl(m_request.withPosition(DOWNPOSITION));
                        Robot.climb2.setControl(m_request.withPosition(DOWNPOSITION));
                        RobotContainer.ClimbPercent = -1;
                        break;
                    default:
                        Robot.climb1.set(0);
                        Robot.climb2.set(0);
                        RobotContainer.ClimbPercent = 0;
                        break;
                }
                System.out.println(RobotContainer.ClimbPercent);
            }
        );
    }*/
}
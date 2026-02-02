package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Climb extends SubsystemBase {
    double p1, p2;
    public Climb(double p1, double p2) {
        this.p1 = p1;
        this.p2 = p2;
    }

    public Command climb(String io) {
        return runOnce(
            () -> {
                switch (io) {
                    case "up":
                        Robot.climb1.set(0.25);
                        Robot.climb2.set(-0.25);
                    case "down":
                        Robot.climb1.set(-0.25);
                        Robot.climb2.set(0.25);
                    default:
                        Robot.climb1.set(0);
                        Robot.climb2.set(0);
                }
            }
        );
    }
}
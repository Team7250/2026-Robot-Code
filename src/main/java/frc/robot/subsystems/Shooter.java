package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Shooter extends SubsystemBase {
    public Shooter() {}

    public Command shoot(double force) {
        return runOnce(
            () -> {
                Robot.shooter1.set(force);
                Robot.shooter2.set(force);
                System.out.println("cool printer");
            }
        );
    }
}

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    boolean rotate = false;
    boolean raiselower = false;
    public Intake(boolean rotate, boolean raiselower) {
        this.rotate = rotate;
        this.raiselower = raiselower;
    }

    public Command intake() {
        return runOnce(() -> {
            
        });
    }
}

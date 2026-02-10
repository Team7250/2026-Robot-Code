package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class infeeder extends SubsystemBase{


    public Command feed(double powervolts) {
        return runOnce(
          () -> {
            Robot.feeder.set(powervolts);
          }  
        );
    }
}

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class infeeder extends SubsystemBase{

    //testcommands, we need to map buttons to them
    public Command testfeed(double powervolts) {
        return runOnce(() -> {
            System.out.println("testfeed");
            Robot.feeder.set(powervolts);
        });
    }
    public Command testindexer(double powervolts) {
        return runOnce(() -> {
            System.out.println("testindexer");
            Robot.indexer.set(powervolts);
        });
    }
    public Command feed(double powervolts) {
        return runOnce(() -> {
            System.out.println("FEEDER COMMAND RUNNING");
            Robot.feeder.set(powervolts);
            Robot.indexer.set(-powervolts);
          }  
        );
    }
}

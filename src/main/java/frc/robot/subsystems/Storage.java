package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Storage extends SubsystemBase{
    String state;
    public Storage(String state){
        this.state = state;
    }

    public Command storage()
    {
        return runOnce(() ->
        {
            if(state == "ON")
            {
                Robot.storage.set(.25);
            }
            else
            {
                Robot.storage.set(0);
            }
        }
        );
    }
}

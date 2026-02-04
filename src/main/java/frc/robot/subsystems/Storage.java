package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Storage extends SubsystemBase{
    String state;
    public Storage(){
        
    }

    public Command storage(String state)
    {
        return runOnce(() ->
        {
            if(state == "ON")
            {
                Robot.storage1.set(.25);
            }
            else
            {
                Robot.storage1.set(0);
            }
        }
        );
    }
}

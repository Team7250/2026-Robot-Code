package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class booltoggle extends SubsystemBase{
    boolean fieldOrient = true;
    public Command toggleBoolean() {
        return runOnce(() -> {
                fieldOrient = !fieldOrient;
            }
        ).alongWith(Commands.print(String.valueOf(fieldOrient)));
    } 
}

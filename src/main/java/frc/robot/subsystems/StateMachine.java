package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StateMachine extends SubsystemBase {

    private int stateId;
    public StateMachine()
    {
        
    }

    public int getStateId()
    {
        return stateId;
    }

    public Command SetState(int stateId) {
        return runOnce(() -> {
            switch (stateId) {
                case 0:
                    break;
                case 1:
                    break;
                case 2:
                    break;
                case 3:
                    break;
                case 4:
                    break;
                case 5:
                    break;
                case 6:
                    break;
                case 7:
                break;
                default:
            }
        });
    }
}

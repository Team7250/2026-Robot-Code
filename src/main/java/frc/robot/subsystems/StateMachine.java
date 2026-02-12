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
                    System.out.println("REST");
                    break;
                case 1:
                    System.out.println("INTAKE");
                    break;
                case 2:
                    System.out.println("TRANSITION");
                    break;
                case 3:
                    System.out.println("SHOOT");
                    break;
                case 4:
                    System.out.println("PASS");
                    break;
                case 5:
                    System.out.println("CLIMB");
                    break;
                case 6:
                    System.out.println("MANUAL");
                    break;
                case 7:
                    System.out.println("SHOOT AND INTAKE");
                break;
                default:
                    System.out.println("INVALID STATE ID!");
            }
        });
    }
}

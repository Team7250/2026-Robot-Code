package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StateMachine extends SubsystemBase {
    private int stateId;
    public StateMachine(int stateId)
    {
        this.stateId = stateId;
    }

    public int getStateId()
    {
        return stateId;
    }

    public Command SetState() {
        return runOnce(() -> {
            switch (stateId) {
                case 0:
                    System.out.println("REST");
                case 1:
                    System.out.println("INTAKE");
                case 2:
                    System.out.println("TRANSITION");
                case 3:
                    System.out.println("SHOOT");
                case 4:
                    System.out.println("PASS");
                case 5:
                    System.out.println("CLIMB");
                case 6:
                    System.out.println("MANUAL");
                default:
                    System.out.println("INVALID STATE ID!");
            }
        });
    }
}

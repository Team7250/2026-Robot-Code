package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Intake extends SubsystemBase {

    double LOW = 30;
    double HIGH = 0;


    boolean rotate = false;
    String raiselower;
    public Intake() {
    }

    public Command intake(boolean rotate, String raiselower) {
        return runOnce(() -> {
            //use motion magic to control the position
            final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

            //if raiselower == true then it's going down
            if(raiselower == "DOWN")
            {
                Robot.RaiseLower1.setControl(m_request.withPosition(LOW));
                Robot.RaiseLower2.setControl(m_request.withPosition(-LOW));
            }
            else if (raiselower == "UP")
            {
                Robot.RaiseLower1.setControl(m_request.withPosition(HIGH));
                Robot.RaiseLower2.setControl(m_request.withPosition(-HIGH));
            }

            //we can turn off the roller immediately
            if(!rotate)
            {
                Robot.RotateIntake.set(0);
            }

            //spin the intake sequentially after the arm goes down
        }).andThen(() -> {
            if(rotate)
            {
                Robot.RotateIntake.set(.6);
            }
        });
    }
}

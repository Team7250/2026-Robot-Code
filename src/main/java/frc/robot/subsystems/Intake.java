package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Intake extends SubsystemBase {

    public static double LOW = 15.25;
    double HIGH = 0;
     //use motion magic to control the position
    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
    final MotionMagicVelocityDutyCycle velocity = new MotionMagicVelocityDutyCycle(0);

    boolean rotate = false;
    String raiselower;
    public Intake() {
    }

    //this command is for testing intake
    public Command testIntake(double position)
    {
        return runOnce(() -> {
            Robot.RaiseLower1.setControl(m_request.withPosition(position));
        });
    }

    //this command is for testing intake
    public Command testIntakePower(double power)
    {
        return runOnce(() -> {
            Robot.RaiseLower1.set(power);
        });
    }

    public Command testRotate(double rps)
    {
        return runOnce(() -> {
            if (rps != 0) {
               Robot.RotateIntake.setControl(velocity.withVelocity(rps));
            }
            else {
                Robot.RotateIntake.set(0); 
            }
        });
    }

    public Command stopRoller()
    {
        return runOnce(() -> {
            Robot.RotateIntake.set(0);
        });
    }

    public Command Rezero() {
        return runOnce(() -> {
                Robot.RaiseLower1.setPosition(0);
                Robot.RaiseLower2.setPosition(0);
            }
        );
    }
    public Command testBoth(double rps, double position)
    {
        return runOnce(() -> {
            if (rps != 0) {
                Robot.RaiseLower1.setControl(m_request.withPosition(position));
                Robot.RotateIntake.setControl(velocity.withVelocity(rps));
            }
            else {
                Robot.RotateIntake.set(0); 
            }
        });
    }

    //actual intaking command
        public Command intake(boolean rotate, String raiselower) {
            return runOnce(() -> {

            
           

            //raiselower is a string now, DOWN = down position
            if(raiselower == "DOWN")
            {
                Robot.RaiseLower1.setControl(m_request.withPosition(LOW));
            }
            //UP = upper position
            else if (raiselower == "UP")
            {
                Robot.RaiseLower1.setControl(m_request.withPosition(HIGH));
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
                Robot.RotateIntake.setControl(velocity.withVelocity(125));
            }
        });
    }
}

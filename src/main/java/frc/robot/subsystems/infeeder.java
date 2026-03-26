package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class infeeder extends SubsystemBase{

    static VoltageOut vIndexer = new VoltageOut(Volts.of(-8));
    static VoltageOut vFeeder = new VoltageOut(Volts.of(-9));
    final static MotionMagicVelocityDutyCycle velocity = new MotionMagicVelocityDutyCycle(0);
    
        //testcommands, we need to map buttons to them
        public Command testfeed(double rps) {
            return runOnce(() -> {
                Robot.feeder.setControl(velocity.withVelocity(rps));
                Robot.indexer.setControl(velocity.withVelocity(rps));
            });
        }
        public Command testindexer(double rps) {
            return runOnce(() -> {
                Robot.indexer.setControl(velocity.withVelocity(rps));
            });
        }
        public static void feed(boolean on) {
            if(on)
            {
                
                Robot.feeder.setControl(velocity.withVelocity(6));
                Robot.indexer.setControl(velocity.withVelocity(5));
        }
        else
        {
            Robot.feeder.set(0);
            Robot.indexer.set(0);
        }
    }  
    public Command feedAuto(boolean on) {
        return runOnce(() -> {
            if(on)
            {
                
                Robot.feeder.setControl(velocity.withVelocity(6));
                Robot.indexer.setControl(velocity.withVelocity(5));
            }
            else
            {
                Robot.feeder.set(0);
                Robot.indexer.set(0);
            }
        });
    }  
}

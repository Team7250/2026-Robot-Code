package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class infeeder extends SubsystemBase{

    static VoltageOut vIndexer = new VoltageOut(Volts.of(-5));
    static VoltageOut vFeeder = new VoltageOut(Volts.of(-4));

    //testcommands, we need to map buttons to them
    public Command testfeed(double powervolts) {
        return runOnce(() -> {
            Robot.feeder.set(powervolts);
        });
    }
    public Command testindexer(double powervolts) {
        return runOnce(() -> {
            Robot.indexer.set(powervolts);
        });
    }
    public static void feed(boolean on) {
        if(on && (Math.abs(Robot.shooter1.getVelocity().getValueAsDouble() - Shooter.getShooterRate()) < 1))
        {
            
            Robot.feeder.setControl(vIndexer);
            Robot.indexer.setControl(vFeeder);
        }
        else
        {
            System.out.println(Robot.shooter1.getVelocity().getValueAsDouble());
            Robot.feeder.set(0);
            Robot.indexer.set(0);
        }
    }  
}

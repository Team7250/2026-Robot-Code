package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class counter extends SubsystemBase {
    CANrange leftcounter = new CANrange(22);
    CANrange rightcounter = new CANrange(21);
    boolean leftbuffer = leftcounter.getIsDetected().getValue();
    boolean rightbuffer = rightcounter.getIsDetected().getValue();
    public int leftcount = 0;
    public int rightcount = 0;
    public int count = 0;

    @Override
    public void periodic() {
        counting();
    }

    public void counting() {
        if (!rightcounter.getIsDetected().getValue() == rightbuffer) {
            if (rightcounter.getIsDetected().getValue() == true) {
                rightcount += 1;
            }
            ;
            rightbuffer = rightcounter.getIsDetected().getValue();
        }
        ;
        if (!leftcounter.getIsDetected().getValue() == leftbuffer) {
            if (leftcounter.getIsDetected().getValue() == true) {
                leftcount += 1;
            }
            ;
            leftbuffer = leftcounter.getIsDetected().getValue();
        }
        ;
        count = leftcount + rightcount;
        SmartDashboard.putNumber("Left Count", leftcount);
        SmartDashboard.putNumber("Right Count", rightcount);
        SmartDashboard.putNumber("Count", count);
    }
}

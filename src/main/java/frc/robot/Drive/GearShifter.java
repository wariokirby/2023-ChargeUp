package frc.robot.Drive;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class GearShifter {
    private Solenoid shift;

    public GearShifter(int channel) {
        shift = new Solenoid(PneumaticsModuleType.REVPH, channel);
        
        //+++ I think I saw you have a double solenoid for the shifting.  This is a different class called DoubleSolenoid and it needs 2 channels 
        //and runs kForward and kReverse in the setters
    }

    public void setLowGear() {
        // actuate
        shift.set(true);
    }

    public void setHighGear() {
        // un-actuate
        shift.set(false);
    }
}
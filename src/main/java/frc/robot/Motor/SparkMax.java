package frc.robot.Motor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//+++ Why did you write a wrapper class for the SparkMax? Everything is already in the SparkMax library, specifically the setInverted method that seems to be the only reason for this
public class SparkMax {
    private CANSparkMax maxspark;
    private RelativeEncoder encoder;
    private boolean isReversed;

    public SparkMax(int canID, boolean isReversed) {
        this.maxspark = new CANSparkMax(canID, MotorType.kBrushless);
        this.encoder = maxspark.getEncoder();
        this.isReversed = isReversed;
    }

    public void setVoltage(double volts) {
        maxspark.setVoltage((isReversed) ? -volts : volts);
        /**
         * This is a ternerary
         * Equivalent to
         * 
         * if (isReversed)
         * maxspark.setVoltage(-volts);
         * else
         * maxspark.setVoltage(volts);
         * 
         * If you don't understand and need to make a change, you can uncomment this
         * code
         */
    }

    public double getPosition() {
        return (isReversed) ? -encoder.getPosition() : encoder.getPosition();
    }

    public void stop() {
        maxspark.stopMotor();
    }

    public void resetEncoder() {
        encoder.setPosition(0);
    }
}

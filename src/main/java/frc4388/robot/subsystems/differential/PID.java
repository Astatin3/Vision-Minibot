package frc4388.robot.subsystems.differential;

import frc4388.utility.structs.Gains;

public class PID {
    protected Gains  gains;
    private   double output    = 0;


    /** Creates a new PelvicInflammatoryDisease. */
    public PID(Gains gains) {
        this.gains = gains;
    }

    private double prevError, cumError = 0;
    
    // Called every time the scheduler runs while the command is scheduled.
    public double update(double error) {
        cumError += error * .02; // 20 ms
        double delta = error - prevError;

        output = error * gains.kP;
        output += cumError * gains.kI;
        output += delta * gains.kD;
        output += gains.kF;

        return output;
    }
}
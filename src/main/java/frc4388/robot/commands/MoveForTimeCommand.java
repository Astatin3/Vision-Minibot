package frc4388.robot.commands;

import java.time.Instant;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc4388.robot.subsystems.differential.DiffDrive;
import frc4388.utility.structs.Drivebase;

// Command to repeat a joystick movement for a specific time.
public class MoveForTimeCommand extends Command {
    private final DiffDrive drivebase;
    private final Translation2d leftStick;
    private final Translation2d rightStick;
    private final long duration;

    private Instant startTime;

    public MoveForTimeCommand(
        DiffDrive drivebase, 
        Translation2d leftStick, 
        Translation2d rightStick, 
        long millis) {
        
        addRequirements(drivebase);

        this.drivebase = drivebase;
        this.leftStick = leftStick;
        this.rightStick = rightStick;
        this.duration = millis;
    }

    @Override
    public void initialize() {
        startTime = Instant.now();
    }

    @Override
    public void execute() {
        drivebase.arcadeDrive(leftStick, rightStick);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(startTime.toEpochMilli() - Instant.now().toEpochMilli()) > duration;
    }
}
package org.firstinspires.ftc.teamcode.subsystems.drive.driveCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.drive.driveSubsystem;


import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {

    private final driveSubsystem driveSubsystem;
    private final DoubleSupplier driveSupplier;
    private final DoubleSupplier strafeSupplier;
    private final DoubleSupplier turnSupplier;
    private static double wall_distance;
    private static double DRIVE_GAIN = 0.25;
    private static double WALL_DISTANCE_THRESHOLD = 12;

    public DefaultDriveCommand(driveSubsystem subsystem, DoubleSupplier driveInput, DoubleSupplier strafeInput, DoubleSupplier turnInput, double WALL_DISTANCE) {
        driveSubsystem = subsystem;
        driveSupplier = driveInput;
        strafeSupplier = strafeInput;
        turnSupplier = turnInput;
        wall_distance = WALL_DISTANCE;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (wall_distance > WALL_DISTANCE_THRESHOLD){
            driveSubsystem.setDrivePower(driveSupplier.getAsDouble(), strafeSupplier.getAsDouble(), turnSupplier.getAsDouble());
        }else{
            driveSubsystem.setDrivePower(driveSupplier.getAsDouble(), strafeSupplier.getAsDouble()  * DRIVE_GAIN, turnSupplier.getAsDouble());
        }
    }

}
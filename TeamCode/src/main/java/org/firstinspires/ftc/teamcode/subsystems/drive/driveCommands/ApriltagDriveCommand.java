package org.firstinspires.ftc.teamcode.subsystems.drive.driveCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.drive.driveSubsystem;

import java.util.function.DoubleSupplier;

public class ApriltagDriveCommand extends CommandBase {

    private final driveSubsystem driveSubsystem;
    private final double driveSupplier;
    private final double strafeSupplier;
    private final double turnSupplier;

    public ApriltagDriveCommand(driveSubsystem subsystem, double driveInput, double strafeInput, double turnInput) {
        driveSubsystem = subsystem;
        driveSupplier = driveInput;
        strafeSupplier = strafeInput;
        turnSupplier = turnInput;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
            driveSubsystem.setDrivePower(driveSupplier, strafeSupplier, turnSupplier);
    }

}
package org.firstinspires.ftc.teamcode.subsystems.Climb.Climb_Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Climb.Climb_Servos;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake_Elbow;

public class MoveClimbServoCommand extends CommandBase {
        private final Climb_Servos climbSubsystem;
        private final Climb_Servos.ClimbSubsystemState targetState;

    public MoveClimbServoCommand(Climb_Servos subsystem, Climb_Servos.ClimbSubsystemState inputState){
        climbSubsystem = subsystem;
        targetState = inputState;
        addRequirements(climbSubsystem);
    }

    @Override
        public void initialize() {
            //set the Servo position to the target - we only have to set the servo position one time
            climbSubsystem.setState(targetState);
        }

        public void execute() {
            //nothing to do in the loop
        }

        @Override
        public boolean isFinished() {
            //always return true because the command simply sets the servo and we have no way of telling when the servo has finished moving
            return true;
        }

        @Override
        public void end(boolean interrupted) {
        }
    }


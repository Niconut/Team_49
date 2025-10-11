package org.firstinspires.ftc.teamcode.subsystems.scoring.scoring_commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.scoring.Scoring_Pusher;

public class ActuateScoringPusherCommand extends CommandBase {
        private final Scoring_Pusher scoringPusherSubsystem;
        private final Scoring_Pusher.ScoringPusherState targetState;

    public ActuateScoringPusherCommand(Scoring_Pusher subsystem, Scoring_Pusher.ScoringPusherState inputState){
        scoringPusherSubsystem = subsystem;
        targetState = inputState;
        addRequirements(scoringPusherSubsystem);
    }

    @Override
        public void initialize() {
            //set the Servo position to the target - we only have to set the servo position one time
            scoringPusherSubsystem.setState(targetState);
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


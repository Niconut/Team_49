package org.firstinspires.ftc.teamcode.subsystems.scoring.scoring_commands;

import com.arcrobotics.ftclib.command.CommandBase;


import org.firstinspires.ftc.teamcode.subsystems.scoring.Scoring_Shooter;

public class SpinScoringShooterCommand extends CommandBase {
        private final Scoring_Shooter scoringShooterSubsystem;
        private final Scoring_Shooter.ScoringShooterState targetState;

    public SpinScoringShooterCommand(Scoring_Shooter subsystem, Scoring_Shooter.ScoringShooterState inputState){
        scoringShooterSubsystem = subsystem;
        targetState = inputState;
        addRequirements(scoringShooterSubsystem);
    }

    @Override
        public void initialize() {
            //set the Servo position to the target - we only have to set the servo position one time
            scoringShooterSubsystem.setState(targetState);
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


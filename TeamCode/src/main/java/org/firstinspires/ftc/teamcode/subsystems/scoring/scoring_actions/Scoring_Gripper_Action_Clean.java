package org.firstinspires.ftc.teamcode.subsystems.scoring.scoring_actions;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.scoring.Scoring_Gripper;

public class Scoring_Gripper_Action_Clean implements Action{
    Scoring_Gripper scoringGripper = new Scoring_Gripper(hardwareMap);
    private final Scoring_Gripper.ScoringGripperState targetState;

    public Scoring_Gripper_Action_Clean(Scoring_Gripper.ScoringGripperState inputState){
        targetState = inputState;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        scoringGripper.setState(targetState);
        return false;
    }
}

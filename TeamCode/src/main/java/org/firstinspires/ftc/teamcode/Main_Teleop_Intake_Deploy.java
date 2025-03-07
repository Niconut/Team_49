/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.subsystems.scoring.Scoring_Arm.ScoringArmState;
import static org.firstinspires.ftc.teamcode.subsystems.scoring.Scoring_Gripper.ScoringGripperState.AUTO_SCORE;
import static org.firstinspires.ftc.teamcode.subsystems.scoring.Scoring_Gripper.ScoringGripperState.CLOSED;
import static org.firstinspires.ftc.teamcode.subsystems.scoring.Scoring_Gripper.ScoringGripperState.INIT;
import static org.firstinspires.ftc.teamcode.subsystems.scoring.Scoring_Gripper.ScoringGripperState.OPEN;
import static org.firstinspires.ftc.teamcode.subsystems.scoring.Scoring_Gripper.ScoringGripperState.SYSCHECK;
import static org.firstinspires.ftc.teamcode.subsystems.scoring.Scoring_Slide.ScoringSlideState.CLIMB_DONE;
import static org.firstinspires.ftc.teamcode.subsystems.scoring.Scoring_Slide.ScoringSlideState.CLIMB_PREP;
import static org.firstinspires.ftc.teamcode.subsystems.scoring.Scoring_Slide.ScoringSlideState.DIRECT_SCORE;
import static org.firstinspires.ftc.teamcode.subsystems.scoring.Scoring_Slide.ScoringSlideState.HAND_OFF;
import static org.firstinspires.ftc.teamcode.subsystems.scoring.Scoring_Slide.ScoringSlideState.HAND_OFF_PREP;
import static org.firstinspires.ftc.teamcode.subsystems.scoring.Scoring_Slide.ScoringSlideState.HIGH_BASKET_SCORE;
import static org.firstinspires.ftc.teamcode.subsystems.scoring.Scoring_Slide.ScoringSlideState.HIGH_CHAMBER_SCORE_PREP;
import static org.firstinspires.ftc.teamcode.subsystems.scoring.Scoring_Slide.ScoringSlideState.LOW_BASKET_SCORE;
import static org.firstinspires.ftc.teamcode.subsystems.scoring.Scoring_Slide.ScoringSlideState.LOW_BASKET_SCORE_PREP;
import static org.firstinspires.ftc.teamcode.subsystems.scoring.Scoring_Slide.ScoringSlideState.TELEOP_START;
import static org.firstinspires.ftc.teamcode.subsystems.scoring.Scoring_Slide.ScoringSlideState.WALL_PICKUP;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Sensors.Light_Indicator;
import org.firstinspires.ftc.teamcode.subsystems.drive.driveCommands.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.driveCommands.SlowModeCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.driveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.gyro.gyroSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake_Elbow;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake_Gripper;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake_Shoulder;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake_Slide;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake_Wrist;
import org.firstinspires.ftc.teamcode.subsystems.intake.intake_commands.ActuateIntakeGripperCommand;
import org.firstinspires.ftc.teamcode.subsystems.intake.intake_commands.MoveIntakeElbowCommand;
import org.firstinspires.ftc.teamcode.subsystems.intake.intake_commands.MoveIntakeShoulderCommand;
import org.firstinspires.ftc.teamcode.subsystems.intake.intake_commands.MoveIntakeSlideCommand;
import org.firstinspires.ftc.teamcode.subsystems.intake.intake_commands.MoveIntakeWristCommand;
import org.firstinspires.ftc.teamcode.subsystems.scoring.Scoring_Arm;
import org.firstinspires.ftc.teamcode.subsystems.scoring.Scoring_Gripper;
import org.firstinspires.ftc.teamcode.subsystems.scoring.Scoring_Slide;
import org.firstinspires.ftc.teamcode.subsystems.scoring.scoring_commands.ActuateScoringGripperCommand;
import org.firstinspires.ftc.teamcode.subsystems.scoring.scoring_commands.MoveScoringArmCommand;

//@Disabled
@TeleOp(name="DEPLOY_INTAKE", group="AA_DriveCode")
public class Main_Teleop_Intake_Deploy extends LinearOpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private static int gametime = 120;
    //Scoring_Gripper scoringGripper = null;

    private static double DRIVE_COMMAND = 1;
    private static double STRAFE_COMMAND = 1;
    private static double ROT_COMMAND = 0.5;

    private static double DRIVE_SCALE = 1;
    private static double STRAFE_SCALE = 1;
    private static double ROT_SCALE = 0.5;

    private static double WRIST_TARGET_POSITION = 0.5;
    private static double SHOULDER_TARGET_POSITION = 0.5;
    private static double SLIDE_TARGGET_POSITION = 0.5;
    private static double ELBOW_TARGET_POSITION = 0.5;
    private static double SCORING_ARM_TARGET_POSITION = 0.5;
    private static int SCORING_SLIDE_SETPOINT = 0;

    private static double DRIVE_NORMAL_SCALE = 1;
    private static double STRAFE_NORMAL_SCALE = 1;
    private static double ROT_NORMAL_SCALE = 0.5;

    private static double DRIVE_SLOW_SCALE = 0.3;
    private static double STRAFE_SLOW_SCALE = 0.3;
    private static double ROT_SLOW_SCALE = 0.3;

    private static double WRIST_MOVE_INCREMENTS = 0.0225;
    private static double WRIST_MOVE_THRESHOLD = 0.05;

    private static double SHOULDER_MOVE_INCREMENTS = 0.0075;
    private static double SHOULDER_MOVE_THRESHOLD = 0.05;
    private static double SHOULDER_MOVE_LIMIT = 0.765;

    private static double SLIDE_MOVE_INCREMENTS = 0.025;
    private static double SLIDE_MOVE_THRESHOLD = 0.05;

    private static double ELBOW_MOVE_INCREMENTS = 0.005;
    private static double ELBOW_MOVE_THRESHOLD = 0.25;

    private static double DRIVE_THRESHOLD = 0.5;

    public static ElapsedTime teleopTimer;

    public Command defaultDriveCommand;
    public Command slowModeCommand;

    driveSubsystem drive = null;
    gyroSubsystem gyro = null;

    Light_Indicator lightIndicator = null;

    Scoring_Gripper scoringGripper = null;
    Scoring_Arm scoringArm = null;
    Scoring_Slide scoringSlide = null;

    Intake_Gripper intakeGripper = null;
    Intake_Wrist intakeWrist = null;
    Intake_Elbow intakeElbow = null;
    Intake_Shoulder intakeShoulder = null;
    Intake_Slide intakeSlide = null;


    @Override public void runOpMode()
    {
        drive = new driveSubsystem(hardwareMap,new Pose2d(0,0,0));
        //gyro = new gyroSubsystem(hardwareMap, "imu");

        lightIndicator = new Light_Indicator(hardwareMap);

        scoringGripper = new Scoring_Gripper(hardwareMap);
        scoringArm = new Scoring_Arm(hardwareMap);
        scoringSlide = new Scoring_Slide(hardwareMap);

        intakeGripper = new Intake_Gripper(hardwareMap);
        intakeWrist = new Intake_Wrist(hardwareMap);
        intakeElbow = new Intake_Elbow(hardwareMap);
        intakeShoulder = new Intake_Shoulder(hardwareMap);
        intakeSlide = new Intake_Slide(hardwareMap);

        teleopTimer = new ElapsedTime();

        PIDController scoringSlidePID = new PIDController(0.005, 0, 0);
        scoringSlidePID.setTolerance(10,10);

        GamepadEx driver = new GamepadEx(gamepad1);
        GamepadEx operator = new GamepadEx(gamepad2);

        MakeCommands(driver);

        CommandScheduler.getInstance().reset();

        /* ******** GROUP ALL DRIVER CONTROLS HERE ******** */
        /*
        left stick      --> drive/strafe
        right stick     --> rotate
        left bumper     --> scoring gripper OPEN
        right bumper    --> normal driving speed when pressed
        left trigger    --> rotate scoring arm
        right trigger   --> rotate scoring arm
        dpad up         --> high basket scoring
        dpad down       --> ground pickup
        dpad left       --> NOT USED
        dpad right      --> NOT USED
        a               --> wall pickup prep
        b               --> wall pickup then transition to high chamber score prep
        y               --> score high chamber
        x               --> not used
        */

        Button wallPickupPrepButton = new GamepadButton(driver, GamepadKeys.Button.A);
        Button highChamberScoreButton = new GamepadButton(driver, GamepadKeys.Button.Y);
        Button highChamberScorePrepButton = new GamepadButton(driver, GamepadKeys.Button.X);
        Button highChamberDirectScoreButton = new GamepadButton(driver, GamepadKeys.Button.B);

        Button driveSpeedButton = new GamepadButton(driver, GamepadKeys.Button.RIGHT_BUMPER);
        Button wallPickupButton = new GamepadButton(driver, GamepadKeys.Button.LEFT_BUMPER);
        Button openScoringGripperButton = new GamepadButton(driver, GamepadKeys.Button.RIGHT_STICK_BUTTON);

        Button highClimbPrep = new GamepadButton(driver, GamepadKeys.Button.DPAD_UP);
        Button highClimbDone = new GamepadButton(driver, GamepadKeys.Button.DPAD_DOWN);

        Button backButton = new GamepadButton(driver, GamepadKeys.Button.BACK);
        Button startButton = new GamepadButton(driver, GamepadKeys.Button.START);

        TriggerReader driverLeftTrigger = new TriggerReader(driver, GamepadKeys.Trigger.LEFT_TRIGGER);
        TriggerReader driverRightTrigger = new TriggerReader(driver, GamepadKeys.Trigger.RIGHT_TRIGGER);

        /* ******** GROUP ALL OPERATOR CONTROLS HERE ******** */
        /*
        left stick x    --> move shoulder
        left stick y    --> move slide
        R3              --> intake gripper OPEN
        left bumper     --> drop sample
        right bumper    --> NOT USED
        left trigger    --> change wrist orientation - clockwise
        right trigger   --> change wrist orientation - counter clockwise
        dpad up         --> NOT USED
        dpad down       --> NOT USED
        dpad left       --> NOT USED
        dpad right      --> NOT USED
        a               --> pickup prep
        b               --> pickup sample
        y               --> NOT USED
        x               --> stow arm
        */

        Button pickUpPrepButton = new GamepadButton(operator, GamepadKeys.Button.A);
        Button dropSampleButton = new GamepadButton(operator, GamepadKeys.Button.B);
        Button stowArmButton = new GamepadButton(operator, GamepadKeys.Button.X);
        Button frontDropButton = new GamepadButton(operator, GamepadKeys.Button.Y);

        Button pickupButton = new GamepadButton(operator, GamepadKeys.Button.RIGHT_BUMPER);
        Button handOffButton = new GamepadButton(operator, GamepadKeys.Button.LEFT_BUMPER);

        Button highBasketScoreButton = new GamepadButton(operator, GamepadKeys.Button.DPAD_UP);
        Button lowBasketScoreButton = new GamepadButton(operator, GamepadKeys.Button.DPAD_DOWN);

        Button openIntakeGripperButton = new GamepadButton(operator, GamepadKeys.Button.RIGHT_STICK_BUTTON);

        Button syscheckButton = new GamepadButton(operator, GamepadKeys.Button.START);
        Button resetPIDButton = new GamepadButton(operator, GamepadKeys.Button.BACK);

        TriggerReader operatorLeftTrigger = new TriggerReader(operator, GamepadKeys.Trigger.LEFT_TRIGGER);
        TriggerReader operatorRightTrigger = new TriggerReader(operator, GamepadKeys.Trigger.RIGHT_TRIGGER);

        waitForStart();

        // start teleop with safe subsystem states
        intakeGripper.setState(Intake_Gripper.IntakeGripperState.OPEN);
        intakeWrist.setState(Intake_Wrist.IntakeWristState.INIT);
        intakeElbow.setState(Intake_Elbow.IntakeElbowState.PICKUP_PREP);
        intakeShoulder.setState(Intake_Shoulder.IntakeShoulderState.PICKUP_PREP);
        intakeSlide.setState(Intake_Slide.IntakeSlideState.INIT);

        scoringGripper.setState(INIT);
        scoringArm.setState(ScoringArmState.HIGH_CHAMBER_SCORE);
        SCORING_SLIDE_SETPOINT = scoringSlide.setState(TELEOP_START);

        CommandScheduler.getInstance().setDefaultCommand(drive, slowModeCommand);
        CommandScheduler.getInstance().schedule();

        teleopTimer.reset();

        while (opModeIsActive())
        {
            MakeCommands(driver);
            CommandScheduler.getInstance().run();
            lightIndication(teleopTimer.seconds());

            /* ******** GROUP ALL DRIVER CONTROLS HERE ******** */
            driveSpeedButton
                .whenHeld(defaultDriveCommand)
                .whenReleased(slowModeCommand);

            openScoringGripperButton.whenPressed(
                new SequentialCommandGroup(
                    new ActuateScoringGripperCommand(scoringGripper, OPEN)
                )
            );

            wallPickupButton
                .whenHeld(
                    new SequentialCommandGroup(
                        new InstantCommand(() -> {SCORING_SLIDE_SETPOINT = scoringSlide.setState(WALL_PICKUP);}),
                        new ActuateScoringGripperCommand(scoringGripper,OPEN),
                        new MoveScoringArmCommand(scoringArm, ScoringArmState.WALL_PICKUP)
                    )
                )
                .whenReleased(
                    new SequentialCommandGroup(
                        new ActuateScoringGripperCommand(scoringGripper,CLOSED),
                        new WaitCommand(200),
                        new InstantCommand(() -> {SCORING_SLIDE_SETPOINT = scoringSlide.setState(HIGH_CHAMBER_SCORE_PREP);}),
                        new MoveScoringArmCommand(scoringArm, ScoringArmState.WALL_PICKUP_RAISE)
                    )
                );

            highChamberScorePrepButton.whenPressed(
                new SequentialCommandGroup(
                    new MoveScoringArmCommand(scoringArm, ScoringArmState.HIGH_CHAMBER_SCORE_PREP),
                    new WaitCommand(300),
                    new InstantCommand(() -> {SCORING_SLIDE_SETPOINT = scoringSlide.setState(HIGH_CHAMBER_SCORE_PREP);}),
                    new WaitCommand(200),
                    new ActuateScoringGripperCommand(scoringGripper, AUTO_SCORE),
                    new WaitCommand(50),
                    new ActuateScoringGripperCommand(scoringGripper, CLOSED),
                    new WaitCommand(50),
                    new ActuateScoringGripperCommand(scoringGripper, AUTO_SCORE),
                    new WaitCommand(50),
                    new ActuateScoringGripperCommand(scoringGripper, CLOSED),
                    new WaitCommand(50),
                    new ActuateScoringGripperCommand(scoringGripper, AUTO_SCORE),
                    new WaitCommand(50),
                    new ActuateScoringGripperCommand(scoringGripper, CLOSED)
                )
            );

            highChamberScoreButton.whenPressed(
                new SequentialCommandGroup(
                    new InstantCommand(() -> {SCORING_SLIDE_SETPOINT = scoringSlide.setState(Scoring_Slide.ScoringSlideState.HIGH_CHAMBER_SCORE);}),
                    new MoveScoringArmCommand(scoringArm, ScoringArmState.HIGH_CHAMBER_SCORE)
                )
            );

            highChamberDirectScoreButton.whenPressed(
                new SequentialCommandGroup(
                    new MoveScoringArmCommand(scoringArm, ScoringArmState.DIRECT_SCORE),
                    new InstantCommand(() -> {SCORING_SLIDE_SETPOINT = scoringSlide.setState(DIRECT_SCORE);})
                )
            );

            highClimbPrep.whenPressed(
                new ParallelCommandGroup(
                    new InstantCommand(() -> {scoringSlidePID.setPID(0.005, 0 , 0);}),
                    new InstantCommand(() -> {SCORING_SLIDE_SETPOINT = scoringSlide.setState(CLIMB_PREP);}),
                    new MoveScoringArmCommand(scoringArm, ScoringArmState.CLIMB_PREP)
                )
            );

            // NOTE : Scoring slide will oscillate it not in the submersible!!!
            highClimbDone.whenPressed(
                new ParallelCommandGroup(
                    new InstantCommand(() -> {scoringSlidePID.setPID(0.02, 0, 0);}),
                    new InstantCommand(() -> {SCORING_SLIDE_SETPOINT = scoringSlide.setState(CLIMB_DONE);}),
                    new MoveScoringArmCommand(scoringArm, ScoringArmState.CLIMB_PREP)
                )
            );
            /* ************************************************** */

            /* ******** GROUP ALL OPERATOR CONTROLS HERE ******** */
            syscheckButton.whenPressed(
                new SequentialCommandGroup(
                    new MoveScoringArmCommand(scoringArm, ScoringArmState.SYSCHECK),
                    new ActuateScoringGripperCommand(scoringGripper, SYSCHECK),
                    new ActuateIntakeGripperCommand(intakeGripper, Intake_Gripper.IntakeGripperState.SYSCHECK),
                    new MoveIntakeWristCommand(intakeWrist, Intake_Wrist.IntakeWristState.SYSCHECK),
                    new MoveIntakeElbowCommand(intakeElbow, Intake_Elbow.IntakeElbowState.SYSCHECK),
                    new MoveIntakeShoulderCommand(intakeShoulder, Intake_Shoulder.IntakeShoulderState.SYSCHECK),
                    new MoveIntakeSlideCommand(intakeSlide, Intake_Slide.IntakeSlideState.SYSCHECK)
                )
            );

            resetPIDButton.whenPressed(
                new ParallelCommandGroup(
                    new InstantCommand(() -> {scoringSlidePID.setPID(0.005, 0 , 0);})
                )
            );

            pickUpPrepButton.whenPressed(
                new SequentialCommandGroup(
                    new MoveIntakeShoulderCommand(intakeShoulder, Intake_Shoulder.IntakeShoulderState.PICKUP_PREP),
                    new WaitCommand(250),
                    new MoveIntakeElbowCommand(intakeElbow, Intake_Elbow.IntakeElbowState.PICKUP_PREP),
                    new ActuateIntakeGripperCommand(intakeGripper, Intake_Gripper.IntakeGripperState.OPEN)
                ));
            
            pickupButton
                .whenHeld(
                    new SequentialCommandGroup(
                        new ActuateIntakeGripperCommand(intakeGripper, Intake_Gripper.IntakeGripperState.OPEN),
                        new MoveIntakeElbowCommand(intakeElbow, Intake_Elbow.IntakeElbowState.HOVER)
                    )
                )
                .whenReleased(
                    new SequentialCommandGroup(
                        new MoveIntakeElbowCommand(intakeElbow, Intake_Elbow.IntakeElbowState.PICKUP),
                        new WaitCommand(25),
                        new ActuateIntakeGripperCommand(intakeGripper, Intake_Gripper.IntakeGripperState.CLOSE),
                        new WaitCommand(150),
                        new MoveIntakeElbowCommand(intakeElbow, Intake_Elbow.IntakeElbowState.PICKUP_DONE)
                    )
                );

            openIntakeGripperButton.whenPressed(
                new SequentialCommandGroup(
                    new MoveIntakeElbowCommand(intakeElbow, Intake_Elbow.IntakeElbowState.PICKUP_PREP),
                    new ActuateIntakeGripperCommand(intakeGripper, Intake_Gripper.IntakeGripperState.OPEN)
                )
            );

            stowArmButton.whenPressed(
                new SequentialCommandGroup(
                        new MoveIntakeElbowCommand(intakeElbow, Intake_Elbow.IntakeElbowState.STOW),
                        new MoveIntakeSlideCommand(intakeSlide, Intake_Slide.IntakeSlideState.STOW),
                        new WaitCommand(200),
                        new MoveIntakeWristCommand(intakeWrist, Intake_Wrist.IntakeWristState.STOW),
                        new MoveIntakeShoulderCommand(intakeShoulder, Intake_Shoulder.IntakeShoulderState.STOW),
                        new ActuateIntakeGripperCommand(intakeGripper, Intake_Gripper.IntakeGripperState.CLOSE)
                )
            );

            dropSampleButton
                .whenPressed(
                    new SequentialCommandGroup(
                        new MoveIntakeElbowCommand(intakeElbow, Intake_Elbow.IntakeElbowState.INIT),
                        new MoveIntakeWristCommand(intakeWrist, Intake_Wrist.IntakeWristState.DROP),
                        new MoveIntakeShoulderCommand(intakeShoulder, Intake_Shoulder.IntakeShoulderState.DROP),
                        new WaitCommand(200),
                        new MoveIntakeElbowCommand(intakeElbow, Intake_Elbow.IntakeElbowState.DROP),
                        new MoveIntakeSlideCommand(intakeSlide, Intake_Slide.IntakeSlideState.DROP)
                    )
                )
                .whenReleased(
                    new SequentialCommandGroup(
                        new ActuateIntakeGripperCommand(intakeGripper, Intake_Gripper.IntakeGripperState.OPEN),
                        new WaitCommand(300),
                        new MoveIntakeElbowCommand(intakeElbow, Intake_Elbow.IntakeElbowState.STOW),
                        new MoveIntakeShoulderCommand(intakeShoulder, Intake_Shoulder.IntakeShoulderState.STOW),
                        new MoveIntakeSlideCommand(intakeSlide, Intake_Slide.IntakeSlideState.STOW),
                        new MoveIntakeWristCommand(intakeWrist, Intake_Wrist.IntakeWristState.INIT)
                    )
                );

            frontDropButton
                    .whenPressed(
                            new SequentialCommandGroup(
                                    new ActuateIntakeGripperCommand(intakeGripper, Intake_Gripper.IntakeGripperState.CLOSE),
                                    //new MoveIntakeElbowCommand(intakeElbow, Intake_Elbow.IntakeElbowState.INIT),
                                    //new MoveIntakeWristCommand(intakeWrist, Intake_Wrist.IntakeWristState.PICKUP_PREP),
                                    //new WaitCommand(200),
                                    new MoveIntakeSlideCommand(intakeSlide, Intake_Slide.IntakeSlideState.FRONT_DROP),
                                    new MoveIntakeShoulderCommand(intakeShoulder, Intake_Shoulder.IntakeShoulderState.FRONT_DROP),
                                    new MoveIntakeElbowCommand(intakeElbow, Intake_Elbow.IntakeElbowState.FRONT_DROP)
                            )
                    )
                    .whenReleased(
                            new SequentialCommandGroup(
                                    new ActuateIntakeGripperCommand(intakeGripper, Intake_Gripper.IntakeGripperState.OPEN),
                                    new WaitCommand(200),
                                    new MoveIntakeSlideCommand(intakeSlide, Intake_Slide.IntakeSlideState.STOW),
                                    new MoveIntakeElbowCommand(intakeElbow, Intake_Elbow.IntakeElbowState.STOW),
                                    new MoveIntakeShoulderCommand(intakeShoulder, Intake_Shoulder.IntakeShoulderState.STOW),
                                    new MoveIntakeWristCommand(intakeWrist, Intake_Wrist.IntakeWristState.INIT)
                            )
                    );

            handOffButton
                .whenHeld(
                    new SequentialCommandGroup(
                        new ParallelCommandGroup(
                            new ActuateIntakeGripperCommand(intakeGripper, Intake_Gripper.IntakeGripperState.CLOSE),
                            new MoveIntakeShoulderCommand(intakeShoulder, Intake_Shoulder.IntakeShoulderState.HAND_OFF),
                            new MoveIntakeWristCommand(intakeWrist, Intake_Wrist.IntakeWristState.HANDOFF),
                            new MoveIntakeElbowCommand(intakeElbow, Intake_Elbow.IntakeElbowState.MID),
                            new MoveIntakeSlideCommand(intakeSlide, Intake_Slide.IntakeSlideState.HAND_OFF_PREP),
                            new InstantCommand(() -> {SCORING_SLIDE_SETPOINT = scoringSlide.setState(HAND_OFF_PREP);}),
                            new ActuateScoringGripperCommand(scoringGripper, OPEN),
                            new MoveScoringArmCommand(scoringArm, ScoringArmState.HANDOFF_PREP_EARLY)
                        ),
                        new WaitCommand(200),
                        new MoveIntakeSlideCommand(intakeSlide, Intake_Slide.IntakeSlideState.HAND_OFF),
                        new MoveIntakeElbowCommand(intakeElbow, Intake_Elbow.IntakeElbowState.HAND_OFF),
                        new WaitCommand(25),
                        new ActuateIntakeGripperCommand(intakeGripper, Intake_Gripper.IntakeGripperState.PARTIAL_CLOSE),
                        new WaitCommand(75),
                        new ActuateIntakeGripperCommand(intakeGripper, Intake_Gripper.IntakeGripperState.CLOSE),
                        new WaitCommand(200),
                        new MoveScoringArmCommand(scoringArm, ScoringArmState.HANDOFF_PREP)
                    )
                )
                .whenReleased(
                    new SequentialCommandGroup(
                        new InstantCommand(() -> {SCORING_SLIDE_SETPOINT = scoringSlide.setState(HAND_OFF);}),
                        new MoveScoringArmCommand(scoringArm, ScoringArmState.HANDOFF),
                        new WaitCommand(200),
                        new ActuateScoringGripperCommand(scoringGripper, CLOSED),
                        new WaitCommand(150),
                        new ActuateIntakeGripperCommand(intakeGripper, Intake_Gripper.IntakeGripperState.OPEN),
                        new WaitCommand(100),
                        new InstantCommand(() -> {SCORING_SLIDE_SETPOINT = scoringSlide.setState(LOW_BASKET_SCORE_PREP);}),
                        new MoveScoringArmCommand(scoringArm, ScoringArmState.LOW_BASKET_SCORE_PREP),
                        new WaitCommand(100),
                        new MoveIntakeShoulderCommand(intakeShoulder, Intake_Shoulder.IntakeShoulderState.STOW),
                        new MoveIntakeSlideCommand(intakeSlide, Intake_Slide.IntakeSlideState.STOW)
                    )
                );

            highBasketScoreButton.whenPressed(
                new ParallelCommandGroup(
                    new ActuateScoringGripperCommand(scoringGripper, CLOSED),
                    new InstantCommand(() -> {SCORING_SLIDE_SETPOINT = scoringSlide.setState(HIGH_BASKET_SCORE);}),
                    new MoveScoringArmCommand(scoringArm, ScoringArmState.HIGH_BASKET_SCORE)
                )
            );

            lowBasketScoreButton.whenPressed(
                new ParallelCommandGroup(
                    new ActuateScoringGripperCommand(scoringGripper, CLOSED),
                    new InstantCommand(() -> {SCORING_SLIDE_SETPOINT = scoringSlide.setState(LOW_BASKET_SCORE);}),
                    new MoveScoringArmCommand(scoringArm, ScoringArmState.LOW_BASKET_SCORE)
                )
            );

            /* calculate new wrist position */
            //wrist_move = (operator.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) - operator.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));
            double wrist_move = (operator.gamepad.right_trigger - operator.gamepad.left_trigger);
            if (Math.abs(wrist_move) > WRIST_MOVE_THRESHOLD) {
                WRIST_TARGET_POSITION = intakeWrist.getCurrentPosition() - (wrist_move * Math.abs(wrist_move) * WRIST_MOVE_INCREMENTS);
                intakeWrist.setPosition(WRIST_TARGET_POSITION);
            }

            /* calculate new shoulder position */
            //shoulder_move = (operator.getRightX());
            double shoulder_move = (operator.gamepad.right_stick_x);
            if (Math.abs(shoulder_move) > SHOULDER_MOVE_THRESHOLD) {
                SHOULDER_TARGET_POSITION = intakeShoulder.getCurrentPosition() + (shoulder_move * Math.abs(shoulder_move) * SHOULDER_MOVE_INCREMENTS);
                SHOULDER_TARGET_POSITION = (SHOULDER_TARGET_POSITION > SHOULDER_MOVE_LIMIT )? SHOULDER_MOVE_LIMIT : SHOULDER_TARGET_POSITION;
                intakeShoulder.setPosition(SHOULDER_TARGET_POSITION);
            }

            /* calculate new slide position */
            //slide_move = (operator.getLeftY());
            double slide_move = (operator.gamepad.left_stick_y);
            if (Math.abs(slide_move) > SLIDE_MOVE_THRESHOLD) {
                SLIDE_TARGGET_POSITION = intakeSlide.getCurrentPositionLeft() + (slide_move * Math.abs(slide_move) * SLIDE_MOVE_INCREMENTS);
                intakeSlide.setPosition(SLIDE_TARGGET_POSITION);
            }

            if (gamepad1.left_trigger > 0.5 && gamepad1.right_trigger > 0.5) {
                SCORING_SLIDE_SETPOINT = SCORING_SLIDE_SETPOINT + 25;
            }

            if (gamepad1.start && gamepad1.back){
                scoringSlide.stopAndResetEncoder();
                SCORING_SLIDE_SETPOINT = 0;
            }

            /* move scoring slide to new setpoint */
            scoringSlidePID.setSetPoint(SCORING_SLIDE_SETPOINT);
            double power = scoringSlidePID.calculate(scoringSlide.getLeftCurrentPosition());
            scoringSlide.setPower(power);

            /* display data in driver hub */
            telemetry.addData("intakeGripper: ", intakeGripper.getCurrentPosition());
            telemetry.addData("intakeWrist: ", intakeWrist.getCurrentPosition());
            telemetry.addData("intakeElbow: ", intakeElbow.getCurrentPosition());
            telemetry.addData("intakeShoulder: ", intakeShoulder.getCurrentPosition());
            telemetry.addData("intakeSlide: ", intakeSlide.getCurrentPositionLeft());

            telemetry.addData("scoringGripper: ", scoringGripper.getCurrentPosition());
            telemetry.addData("scoringArm1: ", scoringArm.getSoringArm1position());
            telemetry.addData("scoringArm2: ", scoringArm.getSoringArm2position());
            telemetry.addData("scoringSlide: ", scoringSlide.getLeftCurrentPosition());

            telemetry.addData("GameTime: ", 120 - teleopTimer.seconds());
            telemetry.addData("Runtime", runtime.seconds());

            telemetry.update();
        }
    }
    private void MakeCommands(GamepadEx gamepad) {
        defaultDriveCommand = new DefaultDriveCommand(drive,
                gamepad::getLeftX,
                gamepad::getLeftY,
                gamepad::getRightX
        );

        slowModeCommand = new SlowModeCommand(drive,
                gamepad::getLeftX,
                gamepad::getLeftY,
                gamepad::getRightX
        );
    }

    public void lightIndication(double timer){
        double remainingTime = 120 - timer;
        if (remainingTime > 70){
            lightIndicator.setState(Light_Indicator.LightIndicatorState.BLUE);
        }else if(remainingTime > 69) {
            lightIndicator.setState(Light_Indicator.LightIndicatorState.PURPLE);
        }else if(remainingTime > 68) {
            lightIndicator.setState(Light_Indicator.LightIndicatorState.WHITE);
        }else if(remainingTime > 67) {
            lightIndicator.setState(Light_Indicator.LightIndicatorState.PURPLE);
        }else if(remainingTime > 66) {
            lightIndicator.setState(Light_Indicator.LightIndicatorState.WHITE);
        }else if(remainingTime > 65) {
            lightIndicator.setState(Light_Indicator.LightIndicatorState.PURPLE);
        }else if(remainingTime > 64) {
            lightIndicator.setState(Light_Indicator.LightIndicatorState.WHITE);
        }else if(remainingTime > 63) {
            lightIndicator.setState(Light_Indicator.LightIndicatorState.GREEN);
        }else if(remainingTime > 62) {
            lightIndicator.setState(Light_Indicator.LightIndicatorState.WHITE);
        }else if(remainingTime > 61) {
            lightIndicator.setState(Light_Indicator.LightIndicatorState.GREEN);
        }else if(remainingTime > 60) {
            lightIndicator.setState(Light_Indicator.LightIndicatorState.WHITE);
        }  else if (remainingTime > 35){
            lightIndicator.setState(Light_Indicator.LightIndicatorState.GREEN);
        }else if (remainingTime > 34){
            lightIndicator.setState(Light_Indicator.LightIndicatorState.WHITE);
        }else if (remainingTime > 33){
            lightIndicator.setState(Light_Indicator.LightIndicatorState.GREEN);
        }else if (remainingTime > 32){
            lightIndicator.setState(Light_Indicator.LightIndicatorState.WHITE);
        }else if (remainingTime > 31){
            lightIndicator.setState(Light_Indicator.LightIndicatorState.YELLOW);
        } else if(remainingTime > 30) {
            lightIndicator.setState(Light_Indicator.LightIndicatorState.WHITE);
        } else if (remainingTime > 10){
            lightIndicator.setState(Light_Indicator.LightIndicatorState.YELLOW);
        } else if (remainingTime > 9){
            lightIndicator.setState(Light_Indicator.LightIndicatorState.RED);
        } else if (remainingTime > 8){
            lightIndicator.setState(Light_Indicator.LightIndicatorState.YELLOW);
        } else if (remainingTime > 7){
            lightIndicator.setState(Light_Indicator.LightIndicatorState.RED);
        } else if (remainingTime > 6){
            lightIndicator.setState(Light_Indicator.LightIndicatorState.YELLOW);
        } else if (remainingTime > 5){
            lightIndicator.setState(Light_Indicator.LightIndicatorState.RED);
        }else if (remainingTime > 4){
            lightIndicator.setState(Light_Indicator.LightIndicatorState.YELLOW);
        }  else {
            lightIndicator.setState(Light_Indicator.LightIndicatorState.RED);
        }
    }
}

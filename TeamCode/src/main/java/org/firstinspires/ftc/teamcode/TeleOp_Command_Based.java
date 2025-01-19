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
import static org.firstinspires.ftc.teamcode.subsystems.scoring.Scoring_Gripper.ScoringGripperState.CLOSED;
import static org.firstinspires.ftc.teamcode.subsystems.scoring.Scoring_Gripper.ScoringGripperState.OPEN;
import static org.firstinspires.ftc.teamcode.subsystems.scoring.Scoring_Slide.ScoringSlideState.CLIMB_DONE;
import static org.firstinspires.ftc.teamcode.subsystems.scoring.Scoring_Slide.ScoringSlideState.CLIMB_PREP;
import static org.firstinspires.ftc.teamcode.subsystems.scoring.Scoring_Slide.ScoringSlideState.GROUND_PICKUP;
import static org.firstinspires.ftc.teamcode.subsystems.scoring.Scoring_Slide.ScoringSlideState.HAND_OFF;
import static org.firstinspires.ftc.teamcode.subsystems.scoring.Scoring_Slide.ScoringSlideState.HIGH_BASKET_SCORE;
import static org.firstinspires.ftc.teamcode.subsystems.scoring.Scoring_Slide.ScoringSlideState.HIGH_CHAMBER_SCORE_PREP;
import static org.firstinspires.ftc.teamcode.subsystems.scoring.Scoring_Slide.ScoringSlideState.HOME;
import static org.firstinspires.ftc.teamcode.subsystems.scoring.Scoring_Slide.ScoringSlideState.WALL_PICKUP;
import static org.firstinspires.ftc.teamcode.subsystems.scoring.Scoring_Slide.ScoringSlideState.WALL_PICKUP_DONE;
import static org.firstinspires.ftc.teamcode.subsystems.scoring.Scoring_Slide.ScoringSlideState.WALL_PICKUP_PREP;

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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.drive.driveCommands.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.driveCommands.SlowModeCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.driveSubsystem;
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
import org.firstinspires.ftc.teamcode.subsystems.gyro.gyroSubsystem;

//@Disabled
@TeleOp(name="TeleOp_Command_Based", group="AA_DriveCode")
public class TeleOp_Command_Based extends LinearOpMode
{
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

    private static double WRIST_MOVE_INCREMENTS = 0.01;
    private static double WRIST_MOVE_THRESHOLD = 0.25;

    private static double SHOULDER_MOVE_INCREMENTS = 0.005;
    private static double SHOULDER_MOVE_THRESHOLD = 0.25;

    private static double SLIDE_MOVE_INCREMENTS = 0.01;
    private static double SLIDE_MOVE_THRESHOLD = 0.25;

    private static double ELBOW_MOVE_INCREMENTS = 0.005;
    private static double ELBOW_MOVE_THRESHOLD = 0.25;

    private static double DRIVE_THRESHOLD = 0.5;

    public Command defaultDriveCommand;
    public Command slowModeCommand;

    driveSubsystem drive = null;
    gyroSubsystem gyro = null;
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

        scoringGripper = new Scoring_Gripper(hardwareMap);
        scoringArm = new Scoring_Arm(hardwareMap);
        scoringSlide = new Scoring_Slide(hardwareMap);

        intakeGripper = new Intake_Gripper(hardwareMap);
        intakeWrist = new Intake_Wrist(hardwareMap);
        intakeElbow = new Intake_Elbow(hardwareMap);
        intakeShoulder = new Intake_Shoulder(hardwareMap);
        intakeSlide = new Intake_Slide(hardwareMap);


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
        //Button wallPickupButton = new GamepadButton(driver, GamepadKeys.Button.B);
        Button highChamberScoreButton = new GamepadButton(driver, GamepadKeys.Button.Y);
        Button highChamberScorePrepButton = new GamepadButton(driver, GamepadKeys.Button.X);

        //Button openScoringGripperButton = new GamepadButton(driver, GamepadKeys.Button.LEFT_BUMPER);
        //Button driveSpeedButton = new GamepadButton(driver, GamepadKeys.Button.RIGHT_BUMPER);

        Button driveSpeedButton = new GamepadButton(driver, GamepadKeys.Button.RIGHT_BUMPER);
        Button wallPickupButton = new GamepadButton(driver, GamepadKeys.Button.LEFT_BUMPER);
        Button openScoringGripperButton = new GamepadButton(driver, GamepadKeys.Button.RIGHT_STICK_BUTTON);

        Button groundPickUpButton = new GamepadButton(driver, GamepadKeys.Button.DPAD_DOWN);
        Button highBasketScoreButton = new GamepadButton(driver, GamepadKeys.Button.DPAD_UP);
        Button highClimbPrep = new GamepadButton(driver, GamepadKeys.Button.DPAD_LEFT);
        Button highClimbDone = new GamepadButton(driver, GamepadKeys.Button.DPAD_RIGHT);
        Button handOffDriver = new GamepadButton(driver, GamepadKeys.Button.BACK);


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
        Button pickupButton = new GamepadButton(operator, GamepadKeys.Button.RIGHT_BUMPER);
        //Button pickupButton = new GamepadButton(operator, GamepadKeys.Button.B);
        Button stowArmButton = new GamepadButton(operator, GamepadKeys.Button.X);
        Button openIntakeGripperButton = new GamepadButton(operator, GamepadKeys.Button.RIGHT_STICK_BUTTON);
        //Button dropSampleButton = new GamepadButton(operator, GamepadKeys.Button.LEFT_BUMPER);
        Button dropSampleButton = new GamepadButton(operator, GamepadKeys.Button.DPAD_LEFT);
        Button handOffOperator = new GamepadButton(operator, GamepadKeys.Button.BACK);
        Button resetAll = new GamepadButton(operator, GamepadKeys.Button.START);
        waitForStart();
        // start teleop with safe subsystem states
        intakeGripper.setState(Intake_Gripper.IntakeGripperState.INIT);
        intakeWrist.setState(Intake_Wrist.IntakeWristState.INIT);
        intakeElbow.setState(Intake_Elbow.IntakeElbowState.INIT);
        intakeShoulder.setState(Intake_Shoulder.IntakeShoulderState.INIT);
        intakeSlide.setState(Intake_Slide.IntakeSlideState.INIT);

        scoringGripper.setState(OPEN);
        scoringArm.setState(ScoringArmState.INIT);
        SCORING_SLIDE_SETPOINT = scoringSlide.setState(HOME);

        CommandScheduler.getInstance().setDefaultCommand(drive, slowModeCommand);
        CommandScheduler.getInstance().schedule();

        while (opModeIsActive())
        {
            MakeCommands(driver);
            CommandScheduler.getInstance().run();

            /* ******** GROUP ALL DRIVER CONTROLS HERE ******** */
            openScoringGripperButton.whenPressed(
                new SequentialCommandGroup(
                    new ActuateScoringGripperCommand(scoringGripper, OPEN)
                )
            );

            wallPickupPrepButton.whenPressed(
                new SequentialCommandGroup(
                    new ActuateScoringGripperCommand(scoringGripper,OPEN),
                    new WaitCommand(50),
                    new InstantCommand(() -> {SCORING_SLIDE_SETPOINT = scoringSlide.setState(WALL_PICKUP_DONE);}),
                    new WaitCommand(200),
                    new MoveScoringArmCommand(scoringArm, ScoringArmState.WALL_PICKUP),
                    new WaitCommand(300),
                    new InstantCommand(() -> {SCORING_SLIDE_SETPOINT = scoringSlide.setState(WALL_PICKUP_PREP);})
                )
            );

            wallPickupButton
                .whenHeld(
                    new SequentialCommandGroup(
                        new ActuateScoringGripperCommand(scoringGripper,OPEN),
                        new MoveScoringArmCommand(scoringArm, ScoringArmState.WALL_PICKUP),
                        new InstantCommand(() -> {SCORING_SLIDE_SETPOINT = scoringSlide.setState(WALL_PICKUP);})
                    )
                )
                .whenReleased(
                    new SequentialCommandGroup(
                        new ActuateScoringGripperCommand(scoringGripper,CLOSED),
                        new WaitCommand(200),
                        new InstantCommand(() -> {SCORING_SLIDE_SETPOINT = scoringSlide.setState(WALL_PICKUP_DONE);})
                    )
                );

            highChamberScorePrepButton.whenPressed(
                new SequentialCommandGroup(
                    new MoveScoringArmCommand(scoringArm, ScoringArmState.HIGH_CHAMBER_SCORE_PREP),
                    new WaitCommand(300),
                    new InstantCommand(() -> {SCORING_SLIDE_SETPOINT = scoringSlide.setState(HIGH_CHAMBER_SCORE_PREP);})
                )
            );

            highChamberScoreButton.whenPressed(
                new SequentialCommandGroup(
                    new InstantCommand(() -> {SCORING_SLIDE_SETPOINT = scoringSlide.setState(Scoring_Slide.ScoringSlideState.HIGH_CHAMBER_SCORE);}),
                    new MoveScoringArmCommand(scoringArm, ScoringArmState.HIGH_CHAMBER_SCORE)
                )
            );

            /*groundPickUpButton.whenPressed(
                new SequentialCommandGroup(
                    new ActuateScoringGripperCommand(scoringGripper, OPEN),
                    new InstantCommand(() -> {SCORING_SLIDE_SETPOINT = scoringSlide.setState(WALL_PICKUP_DONE);}),
                    new WaitCommand(200),
                    new MoveScoringArmCommand(scoringArm, ScoringArmState.GROUND_PICKUP),
                    new WaitCommand(200),
                    new InstantCommand(() -> {SCORING_SLIDE_SETPOINT = scoringSlide.setState(GROUND_PICKUP);})
                )
            );*/

            highBasketScoreButton.whenPressed(
                new SequentialCommandGroup(
                    new ActuateScoringGripperCommand(scoringGripper, CLOSED),
                    new WaitCommand(200),
                    new InstantCommand(() -> {SCORING_SLIDE_SETPOINT = scoringSlide.setState(HIGH_BASKET_SCORE);}),
                    new MoveScoringArmCommand(scoringArm, ScoringArmState.HIGH_BASKET_SCORE)
                )
            );

            highClimbPrep.whenPressed(
                new SequentialCommandGroup(
                    new InstantCommand(() -> {scoringSlidePID.setPID(0.02, 0 , 0);}),
                    new InstantCommand(() -> {SCORING_SLIDE_SETPOINT = scoringSlide.setState(CLIMB_PREP);}),
                    new MoveScoringArmCommand(scoringArm, ScoringArmState.CLIMB_PREP)
                )
            );

            highClimbDone.whenPressed(
                new SequentialCommandGroup(
                    new InstantCommand(() -> {SCORING_SLIDE_SETPOINT = scoringSlide.setState(CLIMB_DONE);}),
                    new MoveScoringArmCommand(scoringArm, ScoringArmState.CLIMB_PREP)
                )
            );
            resetAll.whenPressed(
                    new SequentialCommandGroup(
                            new ActuateIntakeGripperCommand(intakeGripper, Intake_Gripper.IntakeGripperState.MID),
                            new MoveIntakeWristCommand(intakeWrist, Intake_Wrist.IntakeWristState.MID),
                            new MoveIntakeElbowCommand(intakeElbow, Intake_Elbow.IntakeElbowState.PICKUP),
                            new MoveIntakeShoulderCommand(intakeShoulder, Intake_Shoulder.IntakeShoulderState.PICKUP_PREP),
                            new MoveIntakeSlideCommand(intakeSlide, Intake_Slide.IntakeSlideState.STOW)
                    )
            );
            /* ************************************************** */

            /* ******** GROUP ALL OPERATOR CONTROLS HERE ******** */
            pickUpPrepButton.whenPressed(
                new SequentialCommandGroup(
                    new MoveIntakeElbowCommand(intakeElbow, Intake_Elbow.IntakeElbowState.PICKUP_DONE),
                    new WaitCommand(50),
                    new MoveIntakeShoulderCommand(intakeShoulder, Intake_Shoulder.IntakeShoulderState.PICKUP_PREP),
                    new WaitCommand(100),
                    new MoveIntakeElbowCommand(intakeElbow, Intake_Elbow.IntakeElbowState.PICKUP_PREP),
                    new ActuateIntakeGripperCommand(intakeGripper, Intake_Gripper.IntakeGripperState.OPEN)
                ));
            
            pickupButton.whenHeld(
                new SequentialCommandGroup(
                    new MoveIntakeElbowCommand(intakeElbow, Intake_Elbow.IntakeElbowState.HOVER),
                    new ActuateIntakeGripperCommand(intakeGripper, Intake_Gripper.IntakeGripperState.OPEN)
                )
            );

            pickupButton.whenReleased(
                new SequentialCommandGroup(
                    new MoveIntakeElbowCommand(intakeElbow, Intake_Elbow.IntakeElbowState.PICKUP),
                    new WaitCommand(50),
                    new ActuateIntakeGripperCommand(intakeGripper, Intake_Gripper.IntakeGripperState.CLOSE),
                    new WaitCommand(100),
                    new MoveIntakeElbowCommand(intakeElbow, Intake_Elbow.IntakeElbowState.PICKUP_DONE)
                ));

            openIntakeGripperButton.whenPressed(
                new SequentialCommandGroup(
                    new ActuateIntakeGripperCommand(intakeGripper, Intake_Gripper.IntakeGripperState.OPEN),
                    new MoveIntakeElbowCommand(intakeElbow, Intake_Elbow.IntakeElbowState.PICKUP_PREP)
                ));

            stowArmButton.whenPressed(
                new SequentialCommandGroup(
                    new ActuateIntakeGripperCommand(intakeGripper, Intake_Gripper.IntakeGripperState.CLOSE),
                    new MoveIntakeWristCommand(intakeWrist, Intake_Wrist.IntakeWristState.STOW),
                    new MoveIntakeElbowCommand(intakeElbow, Intake_Elbow.IntakeElbowState.STOW),
                    new WaitCommand(200),
                    new MoveIntakeShoulderCommand(intakeShoulder, Intake_Shoulder.IntakeShoulderState.STOW),
                    //new WaitCommand(100),
                    new MoveIntakeSlideCommand(intakeSlide, Intake_Slide.IntakeSlideState.STOW)
                ));

            dropSampleButton.whenPressed(
                new SequentialCommandGroup(
                    new MoveIntakeShoulderCommand(intakeShoulder, Intake_Shoulder.IntakeShoulderState.DROP),
                    //new WaitCommand(100),
                    new MoveIntakeElbowCommand(intakeElbow, Intake_Elbow.IntakeElbowState.DROP),
                    //new WaitCommand(100),
                    new MoveIntakeWristCommand(intakeWrist, Intake_Wrist.IntakeWristState.DROP),
                    new WaitCommand(200),
                    new ActuateIntakeGripperCommand(intakeGripper, Intake_Gripper.IntakeGripperState.OPEN),
                    new WaitCommand(150),
                    new MoveIntakeShoulderCommand(intakeShoulder, Intake_Shoulder.IntakeShoulderState.STOW),
                    new MoveIntakeSlideCommand(intakeSlide, Intake_Slide.IntakeSlideState.STOW),
                    new MoveIntakeElbowCommand(intakeElbow, Intake_Elbow.IntakeElbowState.STOW),
                    new MoveIntakeWristCommand(intakeWrist, Intake_Wrist.IntakeWristState.STOW)
                ));

            handOffOperator.whenPressed(
            //    .and(handOffDriver.whenPressed(
                    new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            SCORING_SLIDE_SETPOINT = scoringSlide.setState(WALL_PICKUP_DONE);
                        }),
                        new MoveScoringArmCommand(scoringArm, ScoringArmState.HIGH_BASKET_SCORE),
                        new WaitCommand(500),
                        new ActuateIntakeGripperCommand(intakeGripper, Intake_Gripper.IntakeGripperState.HAND_OFF_PREP),
                        new InstantCommand(() -> {
                            SCORING_SLIDE_SETPOINT = scoringSlide.setState(HAND_OFF);
                        }),
                        new MoveIntakeSlideCommand(intakeSlide, Intake_Slide.IntakeSlideState.HAND_OFF_PREP),
                        new WaitCommand(500),
                        new ParallelCommandGroup(
                            new ActuateScoringGripperCommand(scoringGripper, OPEN),

                            new MoveIntakeElbowCommand(intakeElbow, Intake_Elbow.IntakeElbowState.HAND_OFF),
                            new MoveIntakeWristCommand(intakeWrist, Intake_Wrist.IntakeWristState.INIT),
                            new MoveIntakeShoulderCommand(intakeShoulder, Intake_Shoulder.IntakeShoulderState.HAND_OFF)
                            ),
                        new WaitCommand(1000),
                        new ParallelCommandGroup(
                            new MoveScoringArmCommand(scoringArm, ScoringArmState.HANDOFF),
                            new MoveIntakeSlideCommand(intakeSlide, Intake_Slide.IntakeSlideState.HAND_OFF)
                            ),
                        new WaitCommand(500),
                        new ActuateScoringGripperCommand(scoringGripper, CLOSED),
                        new WaitCommand(50),
                        new ActuateIntakeGripperCommand(intakeGripper, Intake_Gripper.IntakeGripperState.OPEN),
                        new WaitCommand(200),
                        new MoveIntakeSlideCommand(intakeSlide, Intake_Slide.IntakeSlideState.HAND_OFF_PREP),
                        new WaitCommand(300),
                        new ParallelCommandGroup(
                            new MoveIntakeElbowCommand(intakeElbow, Intake_Elbow.IntakeElbowState.STOW),
                            new MoveIntakeShoulderCommand(intakeShoulder, Intake_Shoulder.IntakeShoulderState.STOW)
                            //new MoveIntakeSlideCommand(intakeSlide, Intake_Slide.IntakeSlideState.STOW)
                        )
                    )
            //    )
            );

            /*
            handOffDriver
                .toggleWhenPressed(new InstantCommand(() -> {
                    drive.setFieldOriented(true);
                    telemetry.addLine("Field Centric Driving");
                }), new InstantCommand(() -> {
                    drive.setFieldOriented(false);
                    telemetry.addLine("Robot Centric Driving");
                }));

            */

            driveSpeedButton
                    .whenHeld(defaultDriveCommand)
                    .whenReleased(slowModeCommand);

            /* calculate new wrist position */
            double wrist_move = (operator.gamepad.left_trigger - operator.gamepad.right_trigger);
            if (Math.abs(wrist_move) > WRIST_MOVE_THRESHOLD) {
                WRIST_TARGET_POSITION = intakeWrist.getCurrentPosition() + (wrist_move * WRIST_MOVE_INCREMENTS);
                intakeWrist.setPosition(WRIST_TARGET_POSITION);
            }

            /* calculate new shoulder position */
            double shoulder_move = (-operator.gamepad.right_stick_x);
            if (Math.abs(shoulder_move) > SHOULDER_MOVE_THRESHOLD) {
                SHOULDER_TARGET_POSITION = intakeShoulder.getCurrentPosition() + (shoulder_move * SHOULDER_MOVE_INCREMENTS);
                intakeShoulder.setPosition(SHOULDER_TARGET_POSITION);
            }

            /* calculate new slide position */
            double slide_move = (-operator.gamepad.left_stick_y);
            if (Math.abs(slide_move) > SLIDE_MOVE_THRESHOLD) {
                SLIDE_TARGGET_POSITION = intakeSlide.getCurrentPosition() + (slide_move * SLIDE_MOVE_INCREMENTS);
                intakeSlide.setPosition(SLIDE_TARGGET_POSITION);
            }

            if (gamepad1.left_trigger > 0.5 && gamepad1.right_trigger > 0.5) {
                SCORING_SLIDE_SETPOINT = SCORING_SLIDE_SETPOINT + 25;
            }

            if (gamepad1.start && gamepad1.back){
                scoringSlide.stopAndResetEncoder();
                SCORING_SLIDE_SETPOINT = 0;
            }
            /* calculate new elbow position */
           /* double elbow_move = (-operator.gamepad.right_stick_y);
            if (Math.abs(elbow_move) > ELBOW_MOVE_THRESHOLD) {
                ELBOW_TARGET_POSITION = intakeElbow.getCurrentPosition() + (elbow_move * ELBOW_MOVE_INCREMENTS);
                intakeElbow.setPosition(ELBOW_TARGET_POSITION);
            }*/

            /* move scoring slide to new setpoint */
            scoringSlidePID.setSetPoint(SCORING_SLIDE_SETPOINT);
            double power = scoringSlidePID.calculate(scoringSlide.getLeftCurrentPosition());
            scoringSlide.setPower(power);

            telemetry.addData("intakeGripper: ", intakeGripper.getCurrentPosition());
            telemetry.addData("intakeWrist: ", intakeWrist.getCurrentPosition());
            telemetry.addData("intakeElbow: ", intakeElbow.getCurrentPosition());
            telemetry.addData("intakeShoulder: ", intakeShoulder.getCurrentPosition());
            telemetry.addData("intakeSlide: ", intakeSlide.getCurrentPosition());

            telemetry.addData("scoringGripper: ", scoringGripper.getCurrentPosition());
            telemetry.addData("scoringArm1: ", scoringArm.getSoringArm1position());
            telemetry.addData("scoringArm2: ", scoringArm.getSoringArm2position());
            telemetry.addData("scoringSlide: ", scoringSlide.getLeftCurrentPosition());

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
}

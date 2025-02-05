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
import static org.firstinspires.ftc.teamcode.subsystems.scoring.Scoring_Gripper.ScoringGripperState.INIT;
import static org.firstinspires.ftc.teamcode.subsystems.scoring.Scoring_Gripper.ScoringGripperState.MID;
import static org.firstinspires.ftc.teamcode.subsystems.scoring.Scoring_Gripper.ScoringGripperState.OPEN;
import static org.firstinspires.ftc.teamcode.subsystems.scoring.Scoring_Slide.ScoringSlideState.CLIMB_DONE;
import static org.firstinspires.ftc.teamcode.subsystems.scoring.Scoring_Slide.ScoringSlideState.CLIMB_PREP;
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
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ScheduleCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.apache.commons.math3.random.Well44497a;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.subsystems.Sensors.Light_Indicator;
import org.firstinspires.ftc.teamcode.subsystems.Sensors.Sensor_Distance;
import org.firstinspires.ftc.teamcode.subsystems.drive.driveCommands.ApriltagDriveCommand;
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
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

//@Disabled
@TeleOp(name="Main", group="AA_DriveCode")
public class Main_Teleop extends LinearOpMode
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

    private static double WRIST_MOVE_INCREMENTS = 0.015;
    private static double WRIST_MOVE_THRESHOLD = 0.05;

    private static double SHOULDER_MOVE_INCREMENTS = 0.005;
    private static double SHOULDER_MOVE_THRESHOLD = 0.05;
    private static double SHOULDER_MOVE_LIMIT = 0.695;

    private static double SLIDE_MOVE_INCREMENTS = 0.025;
    private static double SLIDE_MOVE_THRESHOLD = 0.05;

    private static double ELBOW_MOVE_INCREMENTS = 0.005;
    private static double ELBOW_MOVE_THRESHOLD = 0.25;

    private static double DRIVE_THRESHOLD = 0.5;

    private static double WALL_DISTANCE = 0;
    private static double WALL_DISTANCE_THRESHOLD = 12;

    public static ElapsedTime teleopTimer;
    public static ElapsedTime loopTimer;

    public Command defaultDriveCommand;
    public Command slowModeCommand;
    public Command aprilTagCommand;

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

    Sensor_Distance sensorDistance = null;

    final double DESIRED_DISTANCE = 28.0;

    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the strafing speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    private DcMotor leftFrontDrive   = null;  //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive  = null;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive    = null;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive   = null;  //  Used to control the right back drive wheel

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;


    @Override public void runOpMode()
    {
        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  drive_apriltag  = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe_apriltag = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn_apriltag   = 0;        // Desired turning power/speed (-1 to +1)

        // Initialize the Apriltag Detection process
        initAprilTag();

        if (USE_WEBCAM){
            setManualExposure(6, 250);}

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

        sensorDistance = new Sensor_Distance (hardwareMap);

        teleopTimer = new ElapsedTime();
        loopTimer = new ElapsedTime();

        PIDController scoringSlidePID = new PIDController(0.005, 0, 0);
        scoringSlidePID.setTolerance(10,10);

        GamepadEx driver = new GamepadEx(gamepad1);
        GamepadEx operator = new GamepadEx(gamepad2);

        MakeCommands(driver, WALL_DISTANCE, drive_apriltag, turn_apriltag, strafe_apriltag);

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
        Button aprilTagDriveButton = new GamepadButton(driver, GamepadKeys.Button.B);
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
        Button syscheckButton = new GamepadButton(operator, GamepadKeys.Button.START);
        waitForStart();

        // start teleop with safe subsystem states
        intakeGripper.setState(Intake_Gripper.IntakeGripperState.INIT);
        intakeWrist.setState(Intake_Wrist.IntakeWristState.INIT);
        intakeElbow.setState(Intake_Elbow.IntakeElbowState.INIT);
        intakeShoulder.setState(Intake_Shoulder.IntakeShoulderState.INIT);
        intakeSlide.setState(Intake_Slide.IntakeSlideState.INIT);

        scoringGripper.setState(CLOSED);
        scoringArm.setState(ScoringArmState.HIGH_CHAMBER_SCORE_PREP);
        //SCORING_SLIDE_SETPOINT = scoringSlide.setState(WALL_PICKUP_PREP);

        CommandScheduler.getInstance().setDefaultCommand(drive, slowModeCommand);
        CommandScheduler.getInstance().schedule();

        teleopTimer.reset();

        while (opModeIsActive())
        {
            targetFound = false;
            desiredTag  = null;

            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        break;  // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }

            if (targetFound) {
                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double  headingError    = desiredTag.ftcPose.bearing;
                double  yawError        = desiredTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive_apriltag  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn_apriltag   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                strafe_apriltag = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
            }

            MakeCommands(driver, WALL_DISTANCE, drive_apriltag, turn_apriltag, strafe_apriltag);
            CommandScheduler.getInstance().run();
            lightIndication(teleopTimer.seconds());
            loopTimer.reset();

            /* ******** GROUP ALL DRIVER CONTROLS HERE ******** */
            openScoringGripperButton.whenPressed(
                new SequentialCommandGroup(
                    new ActuateScoringGripperCommand(scoringGripper, OPEN)
                )
            );

            wallPickupPrepButton.whenPressed(
                new ParallelCommandGroup(
                    new ActuateScoringGripperCommand(scoringGripper,OPEN),
                    new MoveScoringArmCommand(scoringArm, ScoringArmState.WALL_PICKUP),
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
                        new WaitCommand(100),
                        new InstantCommand(() -> {SCORING_SLIDE_SETPOINT = scoringSlide.setState(WALL_PICKUP_DONE);})
                    )
                );

            highChamberScorePrepButton.whenPressed(
                new SequentialCommandGroup(
                    new MoveScoringArmCommand(scoringArm, ScoringArmState.HIGH_CHAMBER_SCORE_PREP),
                    //new WaitCommand(300),
                    new InstantCommand(() -> {SCORING_SLIDE_SETPOINT = scoringSlide.setState(HIGH_CHAMBER_SCORE_PREP);})
                )
            );

            highChamberScoreButton.whenPressed(
                new SequentialCommandGroup(
                    new InstantCommand(() -> {SCORING_SLIDE_SETPOINT = scoringSlide.setState(Scoring_Slide.ScoringSlideState.HIGH_CHAMBER_SCORE);}),
                    new MoveScoringArmCommand(scoringArm, ScoringArmState.HIGH_CHAMBER_SCORE)
                )
            );

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

            syscheckButton.whenPressed(
                new SequentialCommandGroup(
                    new MoveScoringArmCommand(scoringArm, ScoringArmState.MID),
                    new ActuateIntakeGripperCommand(intakeGripper, Intake_Gripper.IntakeGripperState.MID),
                    new MoveIntakeWristCommand(intakeWrist, Intake_Wrist.IntakeWristState.MID),
                    new MoveIntakeElbowCommand(intakeElbow, Intake_Elbow.IntakeElbowState.PICKUP),
                    new MoveIntakeShoulderCommand(intakeShoulder, Intake_Shoulder.IntakeShoulderState.PICKUP_PREP),
                    new MoveIntakeSlideCommand(intakeSlide, Intake_Slide.IntakeSlideState.SYSCHECK)
                )
            );
            /* ************************************************** */

            /* ******** GROUP ALL OPERATOR CONTROLS HERE ******** */
            pickUpPrepButton.whenPressed(
                new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        new MoveIntakeShoulderCommand(intakeShoulder, Intake_Shoulder.IntakeShoulderState.PICKUP_PREP),
                        new MoveIntakeElbowCommand(intakeElbow, Intake_Elbow.IntakeElbowState.STOW)
                    ),
                    new WaitCommand(75),
                    new MoveIntakeElbowCommand(intakeElbow, Intake_Elbow.IntakeElbowState.PICKUP_PREP),
                    new ActuateIntakeGripperCommand(intakeGripper, Intake_Gripper.IntakeGripperState.OPEN)
                ));
            
            pickupButton.whenHeld(
                new SequentialCommandGroup(
                    new ActuateIntakeGripperCommand(intakeGripper, Intake_Gripper.IntakeGripperState.OPEN),
                    new WaitCommand(50),
                    new MoveIntakeElbowCommand(intakeElbow, Intake_Elbow.IntakeElbowState.HOVER)
                )
            );

            pickupButton.whenReleased(
                new SequentialCommandGroup(
                    new MoveIntakeElbowCommand(intakeElbow, Intake_Elbow.IntakeElbowState.PICKUP),
                    new WaitCommand(50),
                    new ActuateIntakeGripperCommand(intakeGripper, Intake_Gripper.IntakeGripperState.CLOSE),
                    new WaitCommand(50),
                    new MoveIntakeElbowCommand(intakeElbow, Intake_Elbow.IntakeElbowState.PICKUP_DONE)
                ));

            openIntakeGripperButton.whenPressed(
                new SequentialCommandGroup(
                    new ActuateIntakeGripperCommand(intakeGripper, Intake_Gripper.IntakeGripperState.OPEN),
                    new MoveIntakeElbowCommand(intakeElbow, Intake_Elbow.IntakeElbowState.PICKUP_PREP)
                ));

            stowArmButton.whenPressed(
                new ParallelCommandGroup(
                    new ActuateIntakeGripperCommand(intakeGripper, Intake_Gripper.IntakeGripperState.CLOSE),
                    //new MoveIntakeWristCommand(intakeWrist, Intake_Wrist.IntakeWristState.STOW),
                    new MoveIntakeSlideCommand(intakeSlide, Intake_Slide.IntakeSlideState.STOW),
                    new MoveIntakeElbowCommand(intakeElbow, Intake_Elbow.IntakeElbowState.STOW),
                    //new WaitCommand(250),
                    new MoveIntakeShoulderCommand(intakeShoulder, Intake_Shoulder.IntakeShoulderState.STOW)
                    //new WaitCommand(100),

                ));

            dropSampleButton.whenPressed(
                new ParallelCommandGroup(
                    new ActuateIntakeGripperCommand(intakeGripper, Intake_Gripper.IntakeGripperState.CLOSE),
                    new MoveIntakeWristCommand(intakeWrist, Intake_Wrist.IntakeWristState.DROP),
                    new MoveIntakeElbowCommand(intakeElbow, Intake_Elbow.IntakeElbowState.DROP),
                    new MoveIntakeShoulderCommand(intakeShoulder, Intake_Shoulder.IntakeShoulderState.DROP),
                    new MoveIntakeSlideCommand(intakeSlide, Intake_Slide.IntakeSlideState.DROP)
            ));
            dropSampleButton.whenReleased(
                new SequentialCommandGroup(
                    new ActuateIntakeGripperCommand(intakeGripper, Intake_Gripper.IntakeGripperState.OPEN),
                    new WaitCommand(200),
                    new ParallelCommandGroup(
                        new MoveIntakeElbowCommand(intakeElbow, Intake_Elbow.IntakeElbowState.STOW),
                        new MoveIntakeShoulderCommand(intakeShoulder, Intake_Shoulder.IntakeShoulderState.STOW),
                        new MoveIntakeSlideCommand(intakeSlide, Intake_Slide.IntakeSlideState.STOW),
                        new MoveIntakeWristCommand(intakeWrist, Intake_Wrist.IntakeWristState.INIT)
                    )
                ));

            WALL_DISTANCE = sensorDistance.getRange();

            aprilTagDriveButton
                .whenHeld(aprilTagCommand);

            driveSpeedButton
                .whenHeld(defaultDriveCommand)
                .whenReleased(slowModeCommand);

            /* calculate new wrist position */
            double wrist_move = (operator.gamepad.right_trigger - operator.gamepad.left_trigger);
            if (Math.abs(wrist_move) > WRIST_MOVE_THRESHOLD) {
                WRIST_TARGET_POSITION = intakeWrist.getCurrentPosition() + (wrist_move * Math.abs(wrist_move) * WRIST_MOVE_INCREMENTS);
                intakeWrist.setPosition(WRIST_TARGET_POSITION);
            }

            /* calculate new shoulder position */
            double shoulder_move = (operator.gamepad.right_stick_x);
            if (Math.abs(shoulder_move) > SHOULDER_MOVE_THRESHOLD) {
                SHOULDER_TARGET_POSITION = intakeShoulder.getCurrentPosition() + (shoulder_move * Math.abs(shoulder_move) * SHOULDER_MOVE_INCREMENTS);
                SHOULDER_TARGET_POSITION = (SHOULDER_TARGET_POSITION > SHOULDER_MOVE_LIMIT )? SHOULDER_MOVE_LIMIT : SHOULDER_TARGET_POSITION;
                intakeShoulder.setPosition(SHOULDER_TARGET_POSITION);
            }

            /* calculate new slide position */
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
            telemetry.addData("intakeSlide: ", intakeSlide.getCurrentPositionLeft());

            telemetry.addData("scoringGripper: ", scoringGripper.getCurrentPosition());
            telemetry.addData("scoringArm1: ", scoringArm.getSoringArm1position());
            telemetry.addData("scoringArm2: ", scoringArm.getSoringArm2position());
            telemetry.addData("scoringSlide: ", scoringSlide.getLeftCurrentPosition());

            telemetry.addData("GameTime: ", 120 - teleopTimer.seconds());
            telemetry.addData("Loop Timer: ", loopTimer.milliseconds());
            telemetry.addData("Runtime", runtime.seconds());

            telemetry.addData("Wall Distance", WALL_DISTANCE);

            telemetry.update();
        }
    }
    private void MakeCommands(GamepadEx gamepad, double wall_distance, double drive_apriltag, double turn_apriltag, double strafe_apriltag) {
        defaultDriveCommand = new DefaultDriveCommand(drive,
                gamepad::getLeftX,
                gamepad::getLeftY,
                gamepad::getRightX,
                wall_distance
        );

        slowModeCommand = new SlowModeCommand(drive,
                gamepad::getLeftX,
                gamepad::getLeftY,
                gamepad::getRightX
        );

        aprilTagCommand = new ApriltagDriveCommand(drive,
                drive_apriltag,
                turn_apriltag,
                strafe_apriltag
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

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // e.g. Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
}

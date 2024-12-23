/* Copyright (c) 2021 FIRST. All rights reserved.
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





import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import org.firstinspires.ftc.teamcode.subsystems.intake.Intake_Gripper;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake_Wrist;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake_Elbow;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake_Shoulder;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake_Slide;

import org.firstinspires.ftc.teamcode.subsystems.scoring.Scoring_Gripper;
import org.firstinspires.ftc.teamcode.subsystems.scoring.Scoring_Arm;
import org.firstinspires.ftc.teamcode.subsystems.scoring.Scoring_Slide;

import org.firstinspires.ftc.teamcode.subsystems.scoring.Scoring_Gripper.ScoringGripperState;
import org.firstinspires.ftc.teamcode.subsystems.scoring.Scoring_Arm.ScoringArmState;
import org.firstinspires.ftc.teamcode.subsystems.scoring.Scoring_Slide.ScoringSlideState;


/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Teleop", group="A_DriveCode")
public class Teleop extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();

    private static Intake_Gripper intakeGripper = null;
    private static Intake_Wrist intakeWrist = null;
    private static Intake_Elbow intakeElbow = null;
    private static Intake_Shoulder intakeShoulder = null;
    private static Intake_Slide intakeSlide = null;

    private static Scoring_Gripper scoringGripper = null;
    private static Scoring_Arm scoringArm = null;
    private static Scoring_Slide scoringSlide = null;

    private static double DriveScale = 1;
    private static double StrafeScale = 1;
    private static double RotScale = 0.5;

    private static double DriveNormalScale = 1;
    private static double StrafeNormalScale = 1;
    private static double RotNormalScale = 0.5;

    private static double DriveSlowScale = 0.3;
    private static double StrafeSlowScale = 0.3;
    private static double RotSlowScale = 0.3;

    private static double wristMoveIncrements = 0.025;
    private static double wristMoveThreshold = 0.25;
    private static double wristNewPosition = 0;

    private static double shoulderMoveIncrements = 0.01;
    private static double shoulderMoveThreshold = 0.25;
    private static double shoulderNewPosition = 0;

    private static double slideMoveIncrements = 0.01;
    private static double slideMoveThreshold = 0.25;
    private static double slideNewPosition = 0;

    private static double scoringArm2position = 0;


    /*
    private static int newViperPosition = 0;

    public static double viper_target_position = 0;
    public static double viperSlidePositionStart = -25;
    public static double viperSlidePositionPickUp = -950;
    public static double viperSlidePositionHighBasket = -4300;
    public static double viperSlidePositionHighRung = -1300;
    public static double viperSlidePositionHighRungScore = -1560;
    public static double viperSlidePositionHighClimb = -2900;


    private static double slideCurrentPosition = 0;
    private static double shoulderCurrentPosition = 0;
    private static double driveSlowScale = 0.5;
    private static double driveAngSlowScale = 0.25;

    private static double DriveAngScale = 1;
    private static double viper_Power = 0;
    private static double viper_Current_Position = 0;
    private static double wrist_move = 0;
    private static double intake_spin = 0;
    private static double intake_roller_position = 0;
    private static double armPower = 0;
    private static double wristOrientation = 0;
    private static double viper_Manual_Position = 0;
    private boolean intake_Switch = false;
    private boolean intake_Status = false;
    private static int wristpos = 1;
    private static double wristLeft = 0.65;
    private static double wristRight = 0.35;
    private static double wristInit = 0.5;
    private static double intakeGripperInit = 0.55;
    private static double intakeGripperOpen = 0.55;
    private static double intakeGripperClose = 0.325;
    private static double gripperClosed = 0.325;
    private static double gripperOpen = 0.6;
    private static double slideInit = 0.25;
    private static double slideMin = 0.25;
    private static double slideMax = 0.75;
    private static double elbowInit = 0.47;
    private static double elbowStow = 0.47;
    private static double elbowDrop = 0.54;
    private static double elbowPickUpPrep = 0.555;
    private static double elbowPickUp = 0.57;
    private static double shoulderInit = 0.15;
    private static double shoulderStow = 0.15;
    private static double shoulderPickup = 0.675;
    private static double shoulderDrop = 0.2;
    */
    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        intakeGripper = new Intake_Gripper(hardwareMap);
        intakeWrist = new Intake_Wrist(hardwareMap);
        intakeElbow = new Intake_Elbow(hardwareMap);
        intakeShoulder = new Intake_Shoulder(hardwareMap);
        intakeSlide = new Intake_Slide(hardwareMap);

        scoringGripper = new Scoring_Gripper(hardwareMap);
        scoringArm = new Scoring_Arm(hardwareMap);
        scoringSlide = new Scoring_Slide(hardwareMap);

        PIDController viper_SlidePID = new PIDController(0.01, 0, 0);
        viper_SlidePID.setTolerance(10,10);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        while (opModeInInit()){
            double slidepos = scoringSlide.getCurrentPosition();
            telemetry.addData("intakeGripper: ", intakeGripper.getCurrentPosition());
            telemetry.addData("intakeWrist: ", intakeWrist.getCurrentPosition());
            telemetry.addData("intakeElbow: ", intakeElbow.getCurrentPosition());
            telemetry.addData("intakeShoulder: ", intakeShoulder.getCurrentPosition());
            telemetry.addData("intakeSlide: ", intakeSlide.getCurrentPosition());

            telemetry.addData("scoringGripper: ", scoringGripper.getCurrentPosition());
            telemetry.addData("scoringArm1: ", "(%.2f)", scoringArm.getSoringArm1position());
            telemetry.addData("scoringArm2: ", "(%.2f)", scoringArm.getSoringArm2position());
            telemetry.addData("scoringSlide: ", slidepos);

            telemetry.update();
        }

        waitForStart();
        runtime.reset();
        int setpoint = 0;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            /* ******** GROUP ALL DRIVER CONTROLS HERE ******** */
            /*
            left stick      --> drive/strafe
            right stick     --> rotate
            left bumper     --> scoring gripper OPEN
            right bumper    --> normal driving speed when pressed
            left trigger    --> NOT USED
            right trigger   --> NOT USED
            dpad up         --> high basket scoring
            dpad down       --> ground pickup
            dpad left       --> NOT USED
            dpad right      --> NOT USED
            a               --> wall pickup prep
            b               --> wall pickup then transition to high chamber score prep
            y               --> score high chamber
            x               --> not used
            */

            if (gamepad1.right_bumper){
                DriveScale = DriveNormalScale;
                StrafeScale = StrafeNormalScale;
                RotScale = RotNormalScale;

            }else{
                DriveScale = DriveSlowScale;
                StrafeScale = StrafeSlowScale;
                RotScale = RotSlowScale;
            }

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y) * DriveScale,
                            -gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x) * StrafeScale,
                            -gamepad1.right_stick_x * Math.abs(gamepad1.right_stick_x) * RotScale
                    )
            );
            drive.update();

            //if (gamepad1.right_bumper){ scoringGripper.setState(OPEN); }

            if (gamepad1.left_bumper){ scoringGripper.setState(ScoringGripperState.OPEN); }

            if (gamepad1.dpad_down){
                scoringGripper.setState(ScoringGripperState.CLOSED);
                setpoint = scoringSlide.setState(ScoringSlideState.GROUND_PICKUP);
                scoringArm.setState(ScoringArmState.GROUND_PICKUP);
                scoringGripper.setState(ScoringGripperState.OPEN);
            }

            if (gamepad1.dpad_up){
                setpoint = scoringSlide.setState(ScoringSlideState.HIGH_BASKET_SCORE_PREP);
                scoringArm.setState(ScoringArmState.HIGH_CHAMBER_SCORE_PREP);
                scoringGripper.setState(ScoringGripperState.CLOSED);
            }

            if (gamepad1.dpad_left){ }

            if (gamepad1.dpad_right){ }

            if (gamepad1.a){
                scoringGripper.setState(ScoringGripperState.CLOSED);
                sleep(100);
                scoringArm.setState(ScoringArmState.WALL_PICKUP_PREP);
                //sleep(200);
                setpoint = scoringSlide.setState(ScoringSlideState.WALL_PICKUP_PREP);
                scoringGripper.setState(ScoringGripperState.OPEN);
            }

            if (gamepad1.b){
                scoringArm.setState(ScoringArmState.WALL_PICKUP);
                sleep(100);
                scoringGripper.setState(ScoringGripperState.CLOSED);
                sleep(300);
                setpoint = scoringSlide.setState(ScoringSlideState.HIGH_CHAMBER_SCORE_PREP);
                scoringArm.setState(ScoringArmState.HIGH_CHAMBER_SCORE_PREP);
            }

            if (gamepad1.y) {
                setpoint = scoringSlide.setState(ScoringSlideState.HIGH_CHAMBER_SCORE);
                scoringArm.setState(ScoringArmState.HIGH_CHAMBER_SCORE);
            }

            if (gamepad1.x){ }

            /* ******** GROUP ALL OPERATOR CONTROLS HERE ******** */
            /*
            left stick x    --> move shoulder
            left stick y    --> move slide
            R3              --> intake gripper OPEN
            left bumper     --> drop sample
            right bumper    --> NOT USED
            left trigger    --> change wrist orientation
            right trigger   --> change wrist orientation
            dpad up         --> NOT USED
            dpad down       --> NOT USED
            dpad left       --> NOT USED
            dpad right      --> NOT USED
            a               --> pickup prep
            b               --> pickup sample
            y               --> NOT USED
            x               --> stow arm
            */
            double wrist_move = gamepad2.right_trigger - gamepad2.left_trigger;
            double shoulder_move = -gamepad2.left_stick_x;
            double slide_move = gamepad2.left_stick_y;

            /* calculate new wrist position */
            if (Math.abs(wrist_move) > wristMoveThreshold ) {
                wristNewPosition = intakeWrist.getCurrentPosition() + (wrist_move * wristMoveIncrements);
                intakeWrist.setPosition(wristNewPosition);
            }

            /* calculate new shoulder position */
            if (Math.abs(shoulder_move) > shoulderMoveThreshold ) {
                shoulderNewPosition = intakeShoulder.getCurrentPosition() + (shoulder_move * shoulderMoveIncrements);
                intakeShoulder.setPosition(shoulderNewPosition);
            }

            /* calculate new slide position */
            if (Math.abs(slide_move) > slideMoveThreshold ) {
                slideNewPosition = intakeSlide.getCurrentPosition() + (slide_move * slideMoveIncrements);
                intakeSlide.setPosition(slideNewPosition);
            }

            if (gamepad2.a) {
                intakeElbow.setState(Intake_Elbow.IntakeElbowState.PICKUP_PREP);
                intakeSlide.setState(Intake_Slide.IntakeSlideState.PICKUP_PREP);
                intakeShoulder.setState(Intake_Shoulder.IntakeShoulderState.PICKUP_PREP);
                intakeWrist.setState(Intake_Wrist.IntakeWristState.PICKUP_PREP);
                intakeGripper.setState(Intake_Gripper.IntakeGripperState.OPEN);
            }

            if (gamepad2.b){
                pauseDrive(drive);
                intakeElbow.setState(Intake_Elbow.IntakeElbowState.PICKUP);
                sleep(200);
                intakeGripper.setState(Intake_Gripper.IntakeGripperState.CLOSE);
                sleep(200);
                intakeElbow.setState(Intake_Elbow.IntakeElbowState.PICKUP_PREP);
            }

            if (gamepad2.x) {
                intakeElbow.setState(Intake_Elbow.IntakeElbowState.STOW);
                intakeShoulder.setState(Intake_Shoulder.IntakeShoulderState.STOW);
                intakeSlide.setState(Intake_Slide.IntakeSlideState.STOW);
                intakeWrist.setState(Intake_Wrist.IntakeWristState.STOW);
                intakeGripper.setState(Intake_Gripper.IntakeGripperState.CLOSE);
            }

            if (gamepad2.right_stick_button){
                intakeGripper.setState(Intake_Gripper.IntakeGripperState.OPEN);
                //sleep(200);
                //intakeElbow.setState(Intake_Elbow.IntakeElbowState.PICKUP_PREP);
            }

            if (gamepad2.left_bumper) {
                pauseDrive(drive);
                intakeElbow.setState(Intake_Elbow.IntakeElbowState.DROP);
                intakeShoulder.setState(Intake_Shoulder.IntakeShoulderState.DROP);
                intakeWrist.setState(Intake_Wrist.IntakeWristState.DROP);
                sleep(300);
                intakeGripper.setState(Intake_Gripper.IntakeGripperState.OPEN);
                sleep(200);
                intakeElbow.setState(Intake_Elbow.IntakeElbowState.STOW);
                intakeShoulder.setState(Intake_Shoulder.IntakeShoulderState.STOW);
                intakeWrist.setState(Intake_Wrist.IntakeWristState.STOW);
            }

            viper_SlidePID.setSetPoint(setpoint);
            double power = viper_SlidePID.calculate(scoringSlide.getCurrentPosition());
            scoringSlide.setPower(power);

            // Show the elapsed game time and wheel power.
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            telemetry.addData("intakeGripper: ", intakeGripper.getCurrentPosition());
            telemetry.addData("intakeWrist: ", intakeWrist.getCurrentPosition());
            telemetry.addData("intakeElbow: ", intakeElbow.getCurrentPosition());
            telemetry.addData("intakeShoulder: ", intakeShoulder.getCurrentPosition());
            telemetry.addData("intakeSlide: ", intakeSlide.getCurrentPosition());

            telemetry.addData("scoringGripper: ", scoringGripper.getCurrentPosition());
            telemetry.addData("scoringArm1: ", scoringArm.getSoringArm1position());
            telemetry.addData("scoringArm2: ", scoringArm.getSoringArm2position());
            telemetry.addData("scoringSlide: ", scoringSlide.getCurrentPosition());

            telemetry.update();
            }
        }

        private void pauseDrive(SampleMecanumDrive drive){
            drive.setWeightedDrivePower( new Pose2d(0, 0, 0));
            drive.update();
        }
    }

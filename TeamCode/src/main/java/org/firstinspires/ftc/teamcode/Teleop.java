package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants.*;

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

@TeleOp(name="Teleop", group="A_DriveCode")
public class Teleop extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();

    private static Constants CONSTANTS;

    private static double DRIVE_SCALE = 1;
    private static double STRAFE_SCALE = 1;
    private static double ROT_SCALE = 0.5;

    private static double WRIST_TARGET_POSITION = 0.5;
    private static double SHOULDER_TARGET_POSITION = 0.5;
    private static double SLIDE_TARGGET_POSITION = 0.5;
    private static int SCORING_SLIDE_SETPOINT = 0;
    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Intake_Gripper intakeGripper = new Intake_Gripper(hardwareMap);
        Intake_Wrist intakeWrist = new Intake_Wrist(hardwareMap);
        Intake_Elbow intakeElbow = new Intake_Elbow(hardwareMap);
        Intake_Shoulder intakeShoulder = new Intake_Shoulder(hardwareMap);
        Intake_Slide intakeSlide = new Intake_Slide(hardwareMap);

        Scoring_Gripper scoringGripper = new Scoring_Gripper(hardwareMap);
        Scoring_Arm scoringArm = new Scoring_Arm(hardwareMap);
        Scoring_Slide scoringSlide = new Scoring_Slide(hardwareMap);

        PIDController scoringSlidePID = new PIDController(0.01, 0, 0);
        scoringSlidePID.setTolerance(10,10);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        while (opModeInInit()){
            int slidepos = scoringSlide.getCurrentPosition();
            telemetry.addData("intakeGripper: ", intakeGripper.getCurrentPosition());
            telemetry.addData("intakeWrist: ", intakeWrist.getCurrentPosition());
            telemetry.addData("intakeElbow: ", intakeElbow.getCurrentPosition());
            telemetry.addData("intakeShoulder: ", intakeShoulder.getCurrentPosition());
            telemetry.addData("intakeSlide: ", intakeSlide.getCurrentPosition());

            telemetry.addData("scoringGripper: ", scoringGripper.getCurrentPosition());
            telemetry.addData("scoringArm1: ", scoringArm.getSoringArm1position());
            telemetry.addData("scoringArm2: ", scoringArm.getSoringArm2position());
            telemetry.addData("scoringSlide: ", slidepos);

            telemetry.update();
        }

        waitForStart();
        runtime.reset();

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
                DRIVE_SCALE = CONSTANTS.DRIVE_NORMAL_SCALE;
                STRAFE_SCALE = CONSTANTS.STRAFE_NORMAL_SCALE;
                ROT_SCALE = CONSTANTS.ROT_NORMAL_SCALE;

            }else{
                DRIVE_SCALE = CONSTANTS.DRIVE_SLOW_SCALE;
                STRAFE_SCALE = CONSTANTS.STRAFE_SLOW_SCALE;
                ROT_SCALE = CONSTANTS.ROT_SLOW_SCALE;
            }

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y) * DRIVE_SCALE,
                            -gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x) * STRAFE_SCALE,
                            -gamepad1.right_stick_x * Math.abs(gamepad1.right_stick_x) * ROT_SCALE
                    )
            );
            drive.update();

            //if (gamepad1.right_bumper){ scoringGripper.setState(OPEN); }

            if (gamepad1.left_bumper){ scoringGripper.setState(ScoringGripperState.OPEN); }

            if (gamepad1.dpad_down){
                scoringGripper.setState(ScoringGripperState.CLOSED);
                SCORING_SLIDE_SETPOINT = scoringSlide.setState(ScoringSlideState.GROUND_PICKUP);
                scoringArm.setState(ScoringArmState.GROUND_PICKUP);
                scoringGripper.setState(ScoringGripperState.OPEN);
            }

            if (gamepad1.dpad_up){
                SCORING_SLIDE_SETPOINT = scoringSlide.setState(ScoringSlideState.HIGH_BASKET_SCORE_PREP);
                scoringArm.setState(ScoringArmState.HIGH_CHAMBER_SCORE_PREP);
                scoringGripper.setState(ScoringGripperState.CLOSED);
            }

            if (gamepad1.dpad_left){ }

            if (gamepad1.dpad_right){ }

            if (gamepad1.a){
                //scoringGripper.setState(ScoringGripperState.CLOSED);
                //sleep(100);
                scoringArm.setState(ScoringArmState.WALL_PICKUP_PREP);
                //sleep(200);
                SCORING_SLIDE_SETPOINT = scoringSlide.setState(ScoringSlideState.WALL_PICKUP_PREP);
                scoringGripper.setState(ScoringGripperState.OPEN);
            }

            if (gamepad1.b){
                pauseDrive(drive);
                scoringArm.setState(ScoringArmState.WALL_PICKUP);
                sleep(100);
                scoringGripper.setState(ScoringGripperState.CLOSED);
                sleep(300);
                SCORING_SLIDE_SETPOINT = scoringSlide.setState(ScoringSlideState.HIGH_CHAMBER_SCORE_PREP);
                scoringArm.setState(ScoringArmState.HIGH_CHAMBER_SCORE_PREP);
            }

            if (gamepad1.y) {
                //pauseDrive(drive);
                SCORING_SLIDE_SETPOINT = scoringSlide.setState(ScoringSlideState.HIGH_CHAMBER_SCORE);
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
            double wrist_move = gamepad2.right_trigger - gamepad2.left_trigger;
            double shoulder_move = -gamepad2.left_stick_x;
            double slide_move = gamepad2.left_stick_y;

            /* calculate new wrist position */
            if (Math.abs(wrist_move) > CONSTANTS.WRIST_MOVE_THRESHOLD) {
                WRIST_TARGET_POSITION = intakeWrist.getCurrentPosition() + (wrist_move * CONSTANTS.WRIST_MOVE_INCREMENTS);
                intakeWrist.setPosition(WRIST_TARGET_POSITION);
            }

            /* calculate new shoulder position */
            if (Math.abs(shoulder_move) > CONSTANTS.SHOULDER_MOVE_THRESHOLD) {
                SHOULDER_TARGET_POSITION = intakeShoulder.getCurrentPosition() + (shoulder_move * CONSTANTS.SHOULDER_MOVE_INCREMENTS);
                intakeShoulder.setPosition(SHOULDER_TARGET_POSITION);
            }

            /* calculate new slide position */
            if (Math.abs(slide_move) > CONSTANTS.SLIDE_MOVE_THRESHOLD) {
                SLIDE_TARGGET_POSITION = intakeSlide.getCurrentPosition() + (slide_move * CONSTANTS.SLIDE_MOVE_INCREMENTS);
                intakeSlide.setPosition(SLIDE_TARGGET_POSITION);
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
                sleep(300);
                intakeGripper.setState(Intake_Gripper.IntakeGripperState.CLOSE);
                sleep(300);
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

            scoringSlidePID.setSetPoint(SCORING_SLIDE_SETPOINT);
            double power = scoringSlidePID.calculate(scoringSlide.getCurrentPosition());
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

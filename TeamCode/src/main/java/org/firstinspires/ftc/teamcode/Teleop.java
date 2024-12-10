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

//import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Specimen_Gripper;
import org.firstinspires.ftc.teamcode.teamcode.MecanumDrive;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Basket;
import org.firstinspires.ftc.teamcode.subsystems.ClimbArm;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Park_Arm;
import org.firstinspires.ftc.teamcode.subsystems.Viper_Slide;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

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
    private static Gripper Gripper_Right = null;
    private static Gripper Gripper_Left = null;
    private static Specimen_Gripper Gripper = null;
    private static Viper_Slide Viper_Slide = null;
    private static Arm arm1 = null;
    private static Intake intake = null;
    private static Basket basket = null;
    private static Wrist wrist = null;
    private static Park_Arm Park_Arm = null;
    private static ClimbArm ClimbArm = null;
    public static double viperkP = 0.005;
    public static double viperkD= 0.0000;
    public static double viperkI = 0.000;
    public static double armkP = 0.01;
    public static double armkD = 0.00001;
    public static double armkI = 0.0001;
    public static double climbkP = 0.001;
    public static double climbkD = 0.0000;
    public static double climbkI = 0.000;
    private static int armCurrentPosition = 0;
    private static int newArmPosition = 0;
    private static int armHighChamberPosition = 3554;
    private static int armLowChamberPosition = 5900;
    private static int armPickUpPosition = 2150;
    private static int armClearPosition = 550;
    private static int armStartPosition = 0;
    private static int armPickEndPosition = 1742;
    private static int armPreClimbPosition = 6051;
    private static int armClimbPosition = 750;
    public static double arm_move = 0;
    public static int armRotateScale = 200;
    public static int newViperPosition = 0;
    public static double viperPower = 0;
    public static double viperUsedPower = 0;
    private static int viperCurrentPosition = 0;
    private static int viperStartPosition = 0;
    private static int viperHighBasketPosition = -3200; //uses 0.23
    private static int viperLowBasketPosition = -1921; //uses 0.075
    private static int viperHighSpecimenPrepPosition = -1389;
    private static int viperHighSpecimenPosition = -1790;
    private static double driveSlowScale = 0.5;
    private static double newClimbPosition = 0;
    private static double climbCurrentPositon = 0;
    private static double climbStartPosition = 0;
    private static double climbPrepPosition = -11500;
    private static double climbClimbPosition = -1300;
    private static double DriveScale = 1;
    private static double DriveRotSlowScale = 0;
    private static double wrist_move = 0;
    private static double intake_spin = 0;
    private static double intake_roller_position = 0;
    private static double climbPower = 0;
    private static double armPower = 0;
    private static double wristOrientation = 0;
    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
      //  intake = new Intake(hardwareMap);
        Gripper = new Specimen_Gripper(hardwareMap);
       Gripper_Left = new Gripper(hardwareMap);
       Gripper_Right = new Gripper(hardwareMap);
        arm1 = new Arm(hardwareMap);
        Viper_Slide = new Viper_Slide(hardwareMap);
        basket = new Basket(hardwareMap);
        ClimbArm = new ClimbArm(hardwareMap);
        /*wrist = new Wrist(hardwareMap);
        Park_Arm = new Park_Arm(hardwareMap);*/

        PIDController armPID = new PIDController(armkP, armkI, armkD);
        armPID.setTolerance(50, 10);

        PIDController viperPID = new PIDController(viperkP, viperkI, viperkD);
        viperPID.setTolerance(200, 50);

        PIDController climbPID = new PIDController(climbkP, climbkI, climbkD);
        climbPID.setTolerance(200, 200);

        // Wait for the game to start (driver presses START)
        while (opModeInInit()) {
            viperCurrentPosition = Viper_Slide.getCurrentPosition();
            telemetry.addData("Status", "Initialized");
            telemetry.addData("ViperPos", viperCurrentPosition);
            telemetry.update();
        }
        intake_roller_position = 0;


        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            /*drive.setWeightedDrivePower(
                    new Pose2d(
                            (-gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y)) * DriveScale,
                            (-gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x)) * DriveScale,
                            (-gamepad1.right_stick_x * Math.abs(gamepad1.right_stick_x)) * DriveRotSlowScale
                    )
            );*/
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));
            drive.updatePoseEstimate();
            //Park_Arm.setPosition(1);
            double arm_move = gamepad2.left_stick_y;
            double viper_move = gamepad2.right_stick_y;
            double climbMove = gamepad1.right_stick_y;
            Gripper.setPosition(gamepad1.left_bumper? 0.7 : 0.5);


            if (gamepad2.left_bumper) {
                armPID.setSetPoint(armClearPosition);
                 basket.setPosition(0.25);
            } else {
                basket.setPosition(0.55);
            }

          /*  if (viper_move !=0) {
                Viper_Slide.setPower(viper_move);
            } else {
                Viper_Slide.setPower(0);
            }*/
            /*
             if (arm_move != 0) {
                arm1.setPower1(arm_move);
            } else {
                arm1.setPower1(0);
            }*/

           /* if (climbMove != 0) {
                if  (ClimbArm.getCurrentPosition() < 0) {

                }
            } else {
                ClimbArm.setPower(0);
            }*/
            if (viper_move != 0){
                Viper_Slide.setPower(viper_move);
            }else{
                Viper_Slide.setPower(0);
            }
           /* if (climb_move != 0){
                ClimbArm.setPower(climb_move);
     else {
                ClimbArm.setPower(0);
            }*/

            if (gamepad2.right_bumper){
                Gripper_Right.setPosition2(0.625);
                Gripper_Left.setPosition1(0.4);
            } else {
                Gripper_Right.setPosition2(0.5);
                Gripper_Left.setPosition1(0.5);
            }

          /*  if (gamepad2.left_bumper){
                if (wristOrientation == 0){
                    wristOrientation = 1;
                } else{
                    wristOrientation = 0;
                }
            }*/

            if (gamepad2.dpad_right && gamepad2.left_bumper){
                viperPID.setSetPoint(viperHighSpecimenPrepPosition);
            }

            if (gamepad2.dpad_up && gamepad2.left_bumper){
                viperPID.setSetPoint(viperHighSpecimenPosition);
            }

           // wrist.setPosition((wristOrientation==0)? 0.3 : 0.66);
            if (gamepad1.y) {
                Gripper_Right.setPosition2(0.625);
                Gripper_Left.setPosition1(0.4);
                armPID.setSetPoint(armStartPosition);
                sleep(1000);
                climbPID.setSetPoint(climbPrepPosition);
            }
            if (gamepad1.a) {
                Gripper_Right.setPosition2(0.625);
                Gripper_Left.setPosition1(0.4);
                armPID.setSetPoint(armStartPosition);
                sleep(1000);
                climbPID.setSetPoint(climbClimbPosition);
            }

            if (gamepad2.y) {
                armkP = 0.01;
                armPID.setSetPoint(armStartPosition);
            }

            if (gamepad2.a){
                armkP = 0.01;
                armPID.setSetPoint(armPickUpPosition);
            }

            if (gamepad2.x){
                armkP = 0.01;
                armPID.setSetPoint(armPickEndPosition);
            }
            /*if (gamepad2.x){
                armPID.setSetPoint(armClearPosition);
            }*/

         /*   if (gamepad1.a && gamepad1.left_bumper) {
                armPID.setSetPoint(armPreClimbPosition);
            }

            if (gamepad1.b && gamepad1.left_bumper) {
                armPID.setSetPoint(armClimbPosition);
            }*/

            if (gamepad2.dpad_down){
                armPID.setSetPoint(armClearPosition);
                viperPID.setSetPoint(viperStartPosition);
            }

            if (gamepad2.dpad_right){
                armPID.setSetPoint(armClearPosition);
                viperPID.setSetPoint(viperLowBasketPosition);
            }

            if (gamepad2.dpad_up){
                armPID.setSetPoint(armClearPosition);
                viperPID.setSetPoint(viperHighBasketPosition);
            }

            viperCurrentPosition = Viper_Slide.getCurrentPosition();
           /*if(!(viper_move == 0)){
                //viper_SlidePID.setSetPoint(viper_Current_Position - (10 * viper_move));
                newViperPosition = (int)(viperCurrentPosition + (50 * viper_move));
                if (newViperPosition > -100){
                    newViperPosition = -100;
                }
                viperPID.setSetPoint(newViperPosition);
                viperCurrentPosition = Viper_Slide.getCurrentPosition();
            }*/

          if(!(arm_move == 0)){
              armkP = 0.01;
              armCurrentPosition = arm1.getCurrentPosition();
                newArmPosition = (int)(armCurrentPosition + (20 * arm_move));
                if (newArmPosition < 10){
                    newArmPosition = 10;
                }
                if (newArmPosition > 2200){
                    newArmPosition = 2200;
                }
                armPID.setSetPoint(newArmPosition);
                //armCurrentPosition = arm1.getCurrentPosition();
          }
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.

           /* if(!(climbMove == 0)){
                climbCurrentPositon = ClimbArm.getCurrentPosition();
                newClimbPosition = (int)(climbCurrentPositon + (100 * arm_move));
                if (newClimbPosition > 0){
                    newClimbPosition = 0;
                }
                climbPID.setSetPoint(newClimbPosition);
            }*/

            if (gamepad1.right_bumper)
            {
                DriveScale = driveSlowScale;
                DriveRotSlowScale = 0.25;
            }else
            {
                DriveScale = 1;
                DriveRotSlowScale = 0.5;
            }

            armCurrentPosition = arm1.getCurrentPosition();
            armPower = armPID.calculate(armCurrentPosition);
            //arm1.setPower1(gamepad2.left_stick_y);
            arm1.setPower1(armPower);
            viperCurrentPosition = Viper_Slide.getCurrentPosition();
            viperPower = viperPID.calculate(viperCurrentPosition);

           Viper_Slide.setPower(viperPower);

           viperUsedPower = Viper_Slide.getPower();

           climbPower = climbPID.calculate(climbCurrentPositon);

            climbCurrentPositon =  ClimbArm.getCurrentPosition();

            ClimbArm.setPower(climbPower);

            // Show the elapsed game time and wheel power.
             telemetry.addData("x", drive.pose.position.x);
             telemetry.addData("y", drive.pose.position.y);
             telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.addData("ArmPos1", armCurrentPosition);
            telemetry.addData("ViperPos", viperCurrentPosition);
            telemetry.addData("ClimbPos", climbCurrentPositon);
            telemetry.addData("ViperPower",viperUsedPower);
            //telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("WristOrientation", wristOrientation);

            telemetry.update();
         }
    }
}

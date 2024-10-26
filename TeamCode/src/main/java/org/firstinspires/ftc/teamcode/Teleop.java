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

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Intake_Touch;
import org.firstinspires.ftc.teamcode.subsystems.Limit_Switch;
import org.firstinspires.ftc.teamcode.subsystems.Park_Arm;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Viper_Slide;

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
    private static Intake IntakeLeft = null;
    private static Intake IntakeRight = null;
    private static Intake_Touch Intake_Touch  = null;
    private static Viper_Slide Viper_Slide1;
    private static Viper_Slide Viper_Slide2;
    private static Gripper gripper = null;
    public static double viperkP = 0.01;
    public static double viperkD= 0.0000;
    public static double viperkI = 0.000;
    private static int armCurrentPosition = 0;
    private static int newArmPosition = 0;
    private static int armHighChamberPosition = 3554;
    private static int armLowChamberPosition = 5900;
    private static int armPickUpPositon = 7427;
    private static int armStartPosition = 0;
    private static int armPreClimbPositon = 6051;
    private static int armClimbPosition = 750;
    public static double viper_move = 0;
    public static double viper_target_position = 0;
    public static double viperSlidePositionStart = 0;
    public static double viperSlidePositionLowRung = -800;
    public static double viperSlidePositionHighRung = -1600;
    public static double viperSlidePositionHighBasket = -2400;
    public static int viperRotateScale = 200;
    private static double driveSlowScale = 0.5;
    private static double DriveScale = 1;
    private static double viper_Power = 0;
    private static double viper_Current_Position = 0;
    private static double wrist_move = 0;
    private static double intake_spin = 0;
    private static double intake_roller_position = 0;
    private static double armPower = 0;
    private static double wristOrientation = 0;
    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        IntakeLeft = new Intake(hardwareMap);
        IntakeRight = new Intake(hardwareMap);
        Viper_Slide1 = new Viper_Slide(hardwareMap);
        Viper_Slide2 = new Viper_Slide(hardwareMap);
        gripper = new Gripper(hardwareMap);
        Intake_Touch = new Intake_Touch(hardwareMap);

        PIDController viper_SlidePID = new PIDController(viperkP, viperkI, viperkD);
        viper_SlidePID.setTolerance(10, 10);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        intake_roller_position = 0;


        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * DriveScale,
                            -gamepad1.left_stick_x * DriveScale,
                            -gamepad1.right_stick_x * DriveScale
                    )
            );

            drive.update();
            double viper_move = gamepad2.left_stick_y;
            gripper.setPosition(gamepad2.left_bumper? 0.35 : 0.6);

            if(!(viper_move == 0))
            {
                viper_Current_Position = Viper_Slide1.getCurrentPosition();
                if(viper_move < 0){
                    viperRotateScale = 10 + (armCurrentPosition / 15);
                    viperRotateScale = viperRotateScale < 0? 50 : viperRotateScale;
                }
                viperRotateScale = 50
                ;
                newArmPosition = (int) (armCurrentPosition + (viperRotateScale * viper_move));
                viper_SlidePID.setSetPoint(newArmPosition);
            }

           /* if (viper_move != 0){
                Viper_Slide1.setPower1(viper_move);
                Viper_Slide2.setPower2(viper_move);
            }*/
            if (gamepad2.b){
              while(Intake_Touch.getState()) {
                  IntakeRight.setPower1(0.7);
                  IntakeLeft.setPower2(0.7);
              }
            } else if(gamepad2.x){
                IntakeRight.setPower1(-0.7);
                IntakeLeft.setPower2(-0.7);
            } else {
                IntakeRight.setPower1(0);
                IntakeLeft.setPower2(0);
            }

            if (gamepad2.dpad_down){
                viper_SlidePID.setSetPoint(viperSlidePositionStart);
            }

            if (gamepad2.dpad_left){
                viper_SlidePID.setSetPoint(viperSlidePositionLowRung);
            }

            if (gamepad2.dpad_right){
                viper_SlidePID.setSetPoint(viperSlidePositionHighRung);
            }

            if (gamepad2.dpad_up){
                viper_SlidePID.setSetPoint(viperSlidePositionHighBasket);
            }

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.

            if (gamepad1.right_bumper)
            {
                DriveScale = driveSlowScale;
            }else
            {
                DriveScale = 1;
            }

            viper_Current_Position = Viper_Slide1.getCurrentPosition();
            viper_Power = viper_SlidePID.calculate(viper_Current_Position);
            Viper_Slide1.setPower1(viper_Power);
            Viper_Slide2.setPower2(viper_Power);
            // Show the elapsed game time and wheel power.
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("WristOrientation", wristOrientation);
            telemetry.addData("ViperSlideCurrentPosition", Viper_Slide1.getCurrentPosition());
            telemetry.addData("Touch Sensor State", Intake_Touch.getState());

            telemetry.update();
        }
    }}

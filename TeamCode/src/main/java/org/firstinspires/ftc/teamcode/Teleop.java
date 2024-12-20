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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

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
    private Arm arm = null;
    private Slide slide = null;
    private Gripper gripper = null;

    private static double DriveScale = 0.5;
    private static double DriveRotSlowScale = 0;
    private static  double DriveSlowScale = 0.5;
    private static double armkP = 0.1;
    private static int armCurrentPosition = 0;
    private static int newArmPosition = 0;
    private static int ArmMinimum = 80;
    private static int ArmMaximum = 2360;
    private static int armPower = 0;
    private static int SlideMinimum = 80;
    private static int SlideMaximum = 3480;
    private static int slideCurrentPosition = 0;
    private static int newSlidePosition = 0;
    private static int slidePower = 0;

    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        arm = new Arm(hardwareMap);
        slide = new Slide(hardwareMap);
        gripper = new Gripper(hardwareMap);
      //  intake = new Intake(hardwareMap);
        /*wrist = new Wrist(hardwareMap);
        Park_Arm = new Park_Arm(hardwareMap);*/


        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        while(opModeInInit()){
            telemetry.addData("armCurrentPosition", arm.getCurrentPosition());
            telemetry.addData("slideCurrentPosition", slide.getCurrentPosition());
            telemetry.update();
        }
        telemetry.update();


        PIDController armPID = new PIDController(0.01, 0.01, 0);
        armPID.setTolerance(10,50);

        PIDController slidePID = new PIDController(0.01, 0.01, 0);
        slidePID.setTolerance(10,50);

        waitForStart();
        runtime.reset();

        armPID.setSetPoint(ArmMinimum);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            drive.setWeightedDrivePower(
                    new Pose2d(
                            (-gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y)) * DriveScale,
                            (-gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x)) * DriveScale,
                            (-gamepad1.right_stick_x * Math.abs(gamepad1.right_stick_x)) * DriveRotSlowScale
                    )
            );
            drive.update();
            gripper.setPosition((gamepad1.right_bumper)? 0.7 : 0.5);



            // control the arm
            armCurrentPosition = arm.getCurrentPosition();
            if(gamepad1.dpad_up){
                newArmPosition = (armCurrentPosition + (25));
            }
            if(gamepad1.dpad_down){
                newArmPosition = (armCurrentPosition - (25));
            }

            if (newArmPosition < ArmMinimum){
                newArmPosition = ArmMinimum;
            }
            if (newArmPosition > ArmMaximum){
                newArmPosition = ArmMaximum;
            }

            armPID.setSetPoint(newArmPosition);
            double armPower = armPID.calculate(armCurrentPosition);
            arm.setPower1(armPower);

            slideCurrentPosition = slide.getCurrentPosition();
            if(gamepad1.dpad_right){
                newSlidePosition = (slideCurrentPosition + (25));
            }
            if(gamepad1.dpad_left){
                newSlidePosition = (slideCurrentPosition - (25));
            }

            if (newSlidePosition < SlideMinimum){
                newSlidePosition = SlideMinimum;
            }
            if (newSlidePosition > SlideMaximum){
                newSlidePosition = SlideMaximum;
            }

            slidePID.setSetPoint(newSlidePosition);
            double slidePower = slidePID.calculate(slideCurrentPosition);
            slide.setPower(slidePower);

            if (gamepad1.right_bumper)
            {
                DriveScale = DriveSlowScale;
                DriveRotSlowScale = 0.25;
            }else
            {
                DriveScale = DriveSlowScale;
                DriveRotSlowScale = 0.25;
            }



            // Show the elapsed game time and wheel power.
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("armCurrentPosition", arm.getCurrentPosition());
            telemetry.addData("slideCurrentPosition", slide.getCurrentPosition());
            //telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("WristOrientation", wristOrientation);

            telemetry.update();
        }
    }}

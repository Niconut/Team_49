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

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.scoring.Scoring_Shooter;
import org.firstinspires.ftc.teamcode.subsystems.scoring.scoring_commands.SpinScoringShooterCommand;

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

@TeleOp(name="RoadrunnerTeleOp", group="TeleOp")
public class CommandBasedTeleOp extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private Scoring_Shooter scoringShooter;
    private Servo Pusher;
    private double START = 0.21;
    private double EXTEND = 0.35;
    private double RETRACT = 0.21;
    private double STRAFE_SCALE = 1;
    private double DRIVE_SCALE = 1;
    private double ROT_SCALE = 1;
    private double STRAFE_SLOW_SCALE = 0.25;
    private double DRIVE_SLOW_SCALE = 0.25;
    private double ROT_SLOW_SCALE = 0.25;
    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        scoringShooter = new Scoring_Shooter(hardwareMap);
        Pusher = hardwareMap.get(Servo.class, "Pusher");

        GamepadEx driver = new GamepadEx(gamepad1);
        GamepadEx operator = new GamepadEx(gamepad2);

        CommandScheduler.getInstance().reset();

        /* ******** GROUP ALL DRIVER CONTROLS HERE ******** */
        /*
        left stick      --> drive/strafe
        right stick     --> rotate
        left bumper     --> not used
        right bumper    --> slow driving speed when pressed
        left trigger    --> NOT USED
        right trigger   --> NOT USED
        dpad up         --> NOT USED
        dpad down       --> NOT USED
        dpad left       --> NOT USED
        dpad right      --> NOT USED
        a               --> wall pickup prep
        b               --> wall pickup then transition to high chamber score prep
        y               --> score high chamber
        x               --> not used
        */

        Button ShootBall = new GamepadButton(driver, GamepadKeys.Button.B);
        Button StopShooter = new GamepadButton(driver, GamepadKeys.Button.A);

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        Pusher.setPosition(START);

        waitForStart();
        runtime.reset();

        if (gamepad1.right_bumper){
            DRIVE_SCALE = DRIVE_SLOW_SCALE;
            ROT_SCALE = ROT_SLOW_SCALE;
            STRAFE_SCALE = STRAFE_SLOW_SCALE;
        }
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                    -gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y) * DRIVE_SCALE,
                    -gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x) * STRAFE_SCALE
                    ),
                -gamepad1.right_stick_x * Math.abs(gamepad1.right_stick_x) * ROT_SCALE
            ));


            ShootBall.whenPressed(
                    new SpinScoringShooterCommand(scoringShooter, Scoring_Shooter.ScoringShooterState.OUTTAKE)
            );
            StopShooter.whenPressed(
                    new SpinScoringShooterCommand(scoringShooter, Scoring_Shooter.ScoringShooterState.INIT)
            );
            scoringShooter.setState(Scoring_Shooter.ScoringShooterState.INIT);

            if (gamepad1.y) {
                Pusher.setPosition(EXTEND);
            } else {
                Pusher.setPosition(RETRACT);
            }
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }}

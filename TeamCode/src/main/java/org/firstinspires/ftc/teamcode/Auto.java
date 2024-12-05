package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group="A_DriveCode")
public class Auto extends LinearOpMode {
    private static Wrist wrist;
    public static double armkP = 0.01;
    public static double armkD = 0.00001;
    public static double armkI = 0.0001;
    private static int armCurrentPosition = 0;
    private static int newArmPosition = 0;
    private static int armHighChamberPosition = 3554;
    private static int armParkPosition = 4000;
    private static int armLowChamberPosition = 5900;
    private static int armPickUpPositon = 7427;
    private static int armStartPosition = 0;
    private static int armPreClimbPositon = 6051;
    private static int armClimbPosition = 750;
    private static double armTimeout = 5;
    private static double armPower = 0;
    private static int toPark = 68;
    Pose2d initPose;
    TrajectorySequence trajPark;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        wrist = new Wrist(hardwareMap);
        initPose = new Pose2d(0, 0, Math.toRadians(0));

        drive.setPoseEstimate(initPose);

        PIDController armPID = new PIDController(armkP, armkI, armkD);
        armPID.setTolerance(30, 5);


        waitForStart();
        resetRuntime();

        runAutoSequence(drive);
        sleep(5000);
    }
    public void runAutoSequence(SampleMecanumDrive drive){
        trajPark = drive.trajectorySequenceBuilder(initPose)
                .strafeRight(toPark)
                .back(8)
                .build();
        drive.followTrajectorySequence(trajPark);
    }
/*    public void armAutoPIDPosition(PIDController armPID, int targetPosition) {
        armPID.setSetPoint(targetPosition);
        resetRuntime();
        while (!armPID.atSetPoint() && (getRuntime() <= armTimeout)) {
            armCurrentPosition = arm1.getCurrentPosition();
            armPower = armPID.calculate(armCurrentPosition);
            arm1.setPower1(armPower);
            telemetry.addData("Arm Power ", "%4.2f", armPower);
            telemetry.update();
        }
        arm1.setPower1(0);
    }
    public void stopMotors(){
        arm1.setPower1(0);
    }*/
}

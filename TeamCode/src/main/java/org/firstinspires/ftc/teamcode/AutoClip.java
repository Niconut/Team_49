package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Basket;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Park_Arm;
import org.firstinspires.ftc.teamcode.subsystems.Viper_Slide;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group="A_DriveCode")
public class AutoClip extends LinearOpMode {
    private static Arm arm1;
    private static Viper_Slide viperSlide;
    private static Basket bucket;
    private static Gripper gripperLeft;
    private static Gripper gripperRight;
    private static Wrist wrist;
    private static Intake intake;
    private static Park_Arm Park_Arm;
    public static double armkP = 0.01;
    public static double armkD = 0.00001;
    public static double armkI = 0.0001;
    public static double viperkP = 0.005;
    public static double viperkD= 0.0000;
    public static double viperkI = 0.000;
    private static int armCurrentPosition = 0;
    private static int newArmPosition = 0;
    private static int armHighChamberPosition = 3554;
    private static int armParkPosition = 4000;
    private static int armLowChamberPosition = 5900;
    private static int armPickUpPosition = 2130;
    private static int armClearPosition = 300;
    private static int armAutoScorePosition = 1390;
    private static int armStartPosition = 0;
    private static int armClimbPosition = 750;
    private static int viperCurrentPosition = 0;
    private static int viperStartPosition = 0;
    private static int viperHighBasketPosition = -3300;
    private static int viperLowBasketPosition = -1840;
    private static double armTimeout = 2;
    private static double viperTimeout = 3;
    private static double armPower = 0;
    private static double viperPower = 0;
    private static int toPark = 68;
    Pose2d initPose;
    Pose2d BucketPose, ClearPose;
    TrajectorySequence trajPark, trajSetUp2;
    Trajectory trajSetUp, trajPark2, trajScore;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        arm1 = new Arm(hardwareMap);
        viperSlide = new Viper_Slide(hardwareMap);
        bucket = new Basket(hardwareMap);
        gripperLeft = new Gripper(hardwareMap);
        gripperRight = new Gripper(hardwareMap);
        /*intake = new Intake(hardwareMap);
        wrist = new Wrist(hardwareMap);
        Park_Arm = new Park_Arm(hardwareMap);*/

        initPose = new Pose2d(33, 63.5, Math.toRadians(-90));

        drive.setPoseEstimate(initPose);

        PIDController armPID = new PIDController(armkP, armkI, armkD);
        armPID.setTolerance(30, 5);

        PIDController viperPID = new PIDController(viperkP, viperkI, viperkD);
        viperPID.setTolerance(50, 10);

        stopMotors();

        waitForStart();
        resetRuntime();

        closeGripper();
        getCoordinates();
        runAutoSequence(drive, viperPID, armPID);
    }
    public void runAutoSequence(SampleMecanumDrive drive, PIDController viperPID, PIDController armPID) {
        closeGripper();
        armAutoPIDPosition(armPID, armAutoScorePosition);
        trajSetUp = drive.trajectoryBuilder(initPose)
                .forward(26)
                .build();
        drive.followTrajectory(trajSetUp);


        sleep(500);
        openGripper();
        sleep(500);
        armAutoPIDPosition(armPID, armStartPosition);

        trajPark = drive.trajectorySequenceBuilder(trajSetUp.end())
                .back(18)
                .turn(Math.toRadians(-135))
                .forward(24)
                .build();
        drive.followTrajectorySequence(trajPark);
        armAutoPIDPosition(armPID, armPickUpPosition);
        sleep(500);
        closeGripper();
        sleep(500);
        armAutoPIDPosition(armPID, armAutoScorePosition);

        trajSetUp2 = drive.trajectorySequenceBuilder(trajPark.end())
                .back(22)
                .turn(Math.toRadians(135))
                .forward(20)
                .build();
        drive.followTrajectorySequence(trajSetUp2);
    }
    public void armAutoPIDPosition(PIDController armPID, int targetPosition) {
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
    public void viperAutoPIDPosition(PIDController viperPID, int targetPosition) {
        viperPID.setSetPoint(targetPosition);
        resetRuntime();
        while (!viperPID.atSetPoint() && (getRuntime() <= viperTimeout)) {
            viperCurrentPosition = viperSlide.getCurrentPosition();
            viperPower = viperPID.calculate(viperCurrentPosition);
            viperSlide.setPower(viperPower);
            telemetry.addData("Viper Power ", "%4.2f", viperPower);
            telemetry.update();
        }
        viperSlide.setPower(0);
    }
    public void stopMotors(){
        arm1.setPower1(0);
        viperSlide.setPower(0);
    }
    public void scoreSample(){
        bucket.setPosition(0.25);
    }
    public void holdSample(){
        bucket.setPosition(0.5);
    }
    public void openGripper(){
        gripperRight.setPosition2(0.5);
        gripperLeft.setPosition1(0.5);
    }
    public void closeGripper() {
        gripperRight.setPosition2(0.625);
        gripperLeft.setPosition1(0.4);
    }
    public void bucketScore() {
        bucket.setPosition(0.25);
    }
    public void  bucketStart() {
        bucket.setPosition(0.5);
    }
    public void getCoordinates(){
        BucketPose = new Pose2d(72, 52, Math.toRadians(-160));
        ClearPose = new Pose2d(66, 48, Math.toRadians(-160));
    }
}

package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.teamcode.OpModes.Autonomous.FinalAutonomous.PoseInTiles;
import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.EverglowLibrary.Systems.CameraSystem;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.ThreePose;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.ThreeTrajectories;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Disabled
@Autonomous(group = "drive", name = "SuperSplineTest")
public class SuperSplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        double T_LEN = 60.5;
        Pose2d start = new Pose2d(0, 0, PI);//start.plus()
        Pose2d between = new Pose2d(-0.2 * T_LEN, 1 * T_LEN - start.getY(), PI);
        drive.setPoseEstimate(start);
        drive.followTrajectory(
                drive.trajectoryBuilder(new Pose2d(0,0,PI))
                        .splineToConstantHeading(between.vec(),between.getHeading())
                        .splineToConstantHeading(
                                between.plus(new Pose2d(T_LEN/1.6, 0,0)).vec(),between.getHeading())
                        .build()
        );

        while (opModeIsActive()) {
            Pose2d velocityEstimate = drive.getPoseVelocity();
            telemetry.addData("X", velocityEstimate.getX());
            telemetry.addData("Y", velocityEstimate.getY());
            telemetry.addData("Heading", velocityEstimate.getHeading());
            telemetry.update();
        }

        Pose2d poseEstimate = drive.getPoseEstimate();

        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

    }
}

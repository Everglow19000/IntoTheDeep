package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.teamcode.OpModes.Autonomous.FinalAutonomous.PoseInTiles;
import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.EverglowLibrary.Systems.CameraSystem;
import org.EverglowLibrary.Systems.FourBarSystem;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.FinalAutonomous;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.ThreePose;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.ThreeTrajectories;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Disabled
@Autonomous(group = "drive", name = "SplineTest")
public class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        Pose2d StartLocation = PoseInTiles(3.71 , 5.6, -PI/2);
        telemetry.addLine("move the pose...");
        StartLocation = StartLocation.minus(PoseInTiles(2.42, 0, 0));

        drive.setPoseEstimate(StartLocation);

        ThreeTrajectories trajectories = new ThreeTrajectories(drive);
        trajectories.createThreePoseStart(new ThreePose(StartLocation));

        trajectories.trajbuilderLeft
                .splineToLinearHeading(PoseInTiles(1.4, 4.43, PI),PI);

        trajectories.trajbuilderMiddle
                .splineToLinearHeading(PoseInTiles(1.4, 4.43, PI),PI);

        trajectories.trajbuilderRight
                .splineToLinearHeading(PoseInTiles(1.4, 4.43, PI),PI);

        trajectories.buildTrajs();

        trajectories.driveCorrectTrajectory(CameraSystem.DetectionLocation.LEFT);

        while (opModeIsActive()) {
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("firstX", poseEstimate.getX());
            telemetry.addData("firstY", poseEstimate.getY());
            telemetry.addData("firstHeading", poseEstimate.getHeading());
            telemetry.update();
        }


        /*sleep(2000);
        drive.followTrajectory(
                drive.trajectoryBuilder(traj.end(), true)
                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                        .build()
        );
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();*/

    }
}

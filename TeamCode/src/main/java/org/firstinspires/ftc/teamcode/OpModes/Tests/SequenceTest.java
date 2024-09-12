package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;

//more imports

@TeleOp(name = "SequenceTest", group = "test")
public class SequenceTest extends LinearOpMode {



    public static double TILE_LENGTH = 60.5;

    public Pose2d locationInTiles() {
        Pose2d pos = drive.getPoseEstimate();
        return new Pose2d(pos.getX() / TILE_LENGTH, pos.getY() / TILE_LENGTH, pos.getHeading());
    }
    SampleMecanumDrive drive;
    @Override
    public void runOpMode(){
        drive = new SampleMecanumDrive(hardwareMap);
        waitForStart();

        Trajectory traj = drive.trajectoryBuilder(new Pose2d(0,0))
                    .splineTo(new Vector2d(50, 50),0)
                    .build();




        while (!isStopRequested()) {

            Pose2d currentTilePosition = locationInTiles();
            Pose2d currentPosition = drive.getPoseEstimate();
            telemetry.addData("X ", currentPosition.getX());
            telemetry.addData("Y ", currentPosition.getY());
            telemetry.addData("Angle ", currentPosition.getHeading());
            telemetry.addData("X Tile ", currentTilePosition.getX());
            telemetry.addData("Y Tile ", currentTilePosition.getY());

            telemetry.update();
        }
    }

}

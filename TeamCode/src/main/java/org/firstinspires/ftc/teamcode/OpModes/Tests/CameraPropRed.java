package org.firstinspires.ftc.teamcode.OpModes.Tests;

import android.util.Pair;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.EverglowLibrary.Systems.CameraSystem;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Dictionary;
import java.util.Enumeration;
import java.util.Hashtable;
import java.util.List;

@TeleOp(name = "Camera Prop Red", group = "test")
public class CameraPropRed extends LinearOpMode {

    //make more attributes & functions
    VisionPortal camera;
    AprilTagProcessor aprilTag;

    @Override
    public void runOpMode(){
        CameraSystem cs = new CameraSystem(this,true);
        telemetry.addLine("ready");
        telemetry.update();
        waitForStart();
        telemetry.addLine("start");
        telemetry.update();
        while (opModeIsActive()) {
            List<Recognition> recognitions = cs.DetectProp();

            for (Recognition rec :
                    recognitions) {
                telemetry.addData("rec -> x: ", CameraSystem.ConvertInchToCm(cs.ConvertRecognitionToPos(rec, true)));
                telemetry.addData("y: ", CameraSystem.ConvertInchToCm(cs.ConvertRecognitionToPos(rec, false)));
            }
            if (!gamepad1.x)
                telemetry.update();
        }
    }
}
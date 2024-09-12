package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.EverglowLibrary.Systems.FourBarSystem;

@TeleOp(group = "test")
public class ResetFor_bar extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        FourBarSystem fourBarSystem = new FourBarSystem(this);
        this.sleep(1000);
        fourBarSystem.restart();

        telemetry.addLine("restarted!!!");
        telemetry.update();
    }
}

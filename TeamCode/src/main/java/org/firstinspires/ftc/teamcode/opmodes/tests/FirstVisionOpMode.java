package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.VisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "First Vision Processor", group = "vision")
public class FirstVisionOpMode extends OpMode {
    private VisionProcessor visionProcessor;
    private VisionPortal visionPortal;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        visionProcessor = new VisionProcessor(telemetry);
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam Front"), visionProcessor);
        telemetry.addLine("Initialization Complete");
        telemetry.update();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        // visionPortal.stopStreaming ();
    }

    @Override
    public void loop() {
        telemetry.addData("Identified", visionProcessor.getSelection()); // TODO: in a real game, do something with the vision data
        telemetry.update();
    }
}

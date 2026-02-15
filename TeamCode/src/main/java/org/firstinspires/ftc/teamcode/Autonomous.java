package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class Autonomous extends OpMode {
    AprilTagWebcam AprilTagWebcam = new AprilTagWebcam();
    Constants Constants = new Constants();

    @Override
    public void init() {
        AprilTagWebcam.Init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        AprilTagWebcam.Update();
        AprilTagDetection Detection = AprilTagWebcam.GetTagByID(Constants.RED_APRIL_TAG_ID);
        AprilTagWebcam.DisplayTelemetryData(Detection);
    }
}

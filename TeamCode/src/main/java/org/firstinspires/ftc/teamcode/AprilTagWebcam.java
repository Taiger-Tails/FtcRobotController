package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;
import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class AprilTagWebcam extends RobotComponent {
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    private List<AprilTagDetection> detectedTags = new ArrayList<>();

    private static final Size CAMERA_RESOLUTION = new Size(1920, 1080);
    private static final YawPitchRollAngles CAMERA_ORIENTATION =
            new YawPitchRollAngles(AngleUnit.DEGREES, 0, 64, 0, 0);

    public static double WEBCAM_X = 0;
    public static double WEBCAM_Y = 7.3;
    public static double WEBCAM_Z = 36;

    private static final double FX = 500, FY = 500, CX = 320, CY = 240;

    public void Init(HardwareMap hwMap) {
        Position cameraPosition = new Position(
                DistanceUnit.CM, WEBCAM_X, WEBCAM_Y, WEBCAM_Z, 0);

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .setLensIntrinsics(FX, FY, CX, CY)
                .setCameraPose(cameraPosition, CAMERA_ORIENTATION)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hwMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(CAMERA_RESOLUTION)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(aprilTagProcessor)
                .build();
    }

    @SuppressLint("DefaultLocale")
    public void DisplayTelemetryData(AprilTagDetection Detected, Telemetry telemetry) {
        if (Detected == null) { return; }

        if (Detected.metadata != null) {
            telemetry.addLine(String.format("\n==== (ID %d) %s", Detected.id, Detected.metadata.name));
            telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", Detected.ftcPose.x, Detected.ftcPose.y, Detected.ftcPose.z));
            telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", Detected.ftcPose.pitch, Detected.ftcPose.roll, Detected.ftcPose.yaw));
            telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", Detected.ftcPose.range, Detected.ftcPose.bearing, Detected.ftcPose.elevation));
        } else {
            telemetry.addLine(String.format("\n==== (ID %d) Unknown", Detected.id));
            telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", Detected.center.x, Detected.center.y));
        }
    }

    public void Update() {
        detectedTags = aprilTagProcessor.getDetections();
    }

    public List<AprilTagDetection> GetDetectedTags() {
        return detectedTags;
    }

    public AprilTagDetection GetTagByID(int ID) {
        for (AprilTagDetection detection : detectedTags) {
            if (detection.id == ID) {
                return detection;
            }
        }

        return null;
    }

    public void Stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}

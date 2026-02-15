package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp
public class Main extends OpMode {
    // Make variables
    Drive Drive = new Drive();
    Shooter Shooter = new Shooter();
    AprilTagWebcam AprilTagWebcam = new AprilTagWebcam();

    Constants Constants = new Constants();

    boolean ToggleDriveSlowness = false;

    // Initialize driving
    @Override
    public void init() {
        Drive.Init(hardwareMap);
        Shooter.Init(hardwareMap);
        AprilTagWebcam.Init(hardwareMap, telemetry);
    }

    // Initialize gamepad controls
    @Override
    public void loop() {
        final double ServoSpinDirection = gamepad1.left_trigger - gamepad1.right_trigger;

        ToggleDriveSlowness = gamepad1.left_stick_button || gamepad1.right_stick_button || gamepad1.right_bumper || gamepad1.left_bumper;

        telemetry.addData("Drive Slowness", ToggleDriveSlowness);
        telemetry.addData("Current Max Shooter Power", Shooter.MaxShooterPower);
        telemetry.addData("ShooterMotor", Shooter.ShooterMotor);

        final double Aqua = ToggleDriveSlowness ? 0.15 : 1; // Aqua is useful!

        final double Forward = -gamepad1.left_stick_y * Aqua;
        final double Strafe = gamepad1.left_stick_x * Aqua;
        final double Rotate = gamepad1.right_stick_x * Aqua;

        Drive.DriveFieldRelative(Strafe, Forward, Rotate);

        Shooter.SetShooterPower(gamepad1.a ? 1 : gamepad1.b ? -1 : 0);
        Shooter.SetServoPower(ServoSpinDirection > 0 ? 1 : ServoSpinDirection < 0 ? -1 : 0);

        if (gamepad1.dpadDownWasReleased()) {
            Shooter.MaxShooterPower = Math.max(Shooter.MaxShooterPower - 0.1, 0);
          } else if(gamepad1.dpadUpWasReleased()) {
            Shooter.MaxShooterPower = Math.min(Shooter.MaxShooterPower + 0.1, 1);
        }

        if (gamepad1.share) {
            Drive.ResetIMU();
        }

        AprilTagWebcam.Update();
        AprilTagDetection Detection = AprilTagWebcam.GetTagByID(Constants.RED_APRIL_TAG_ID);
        AprilTagWebcam.DisplayTelemetryData(Detection);
    }
}

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

    double PreviousX, PreviousY = 0;

    int LoopItterations = 0;
    boolean ServoWait = false;

    private double Ease(double PointA, double PointB) {
        return PointA + ((PointB - PointA) / Constants.EASE_POWER);
    }

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
        if (ServoWait) {
           LoopItterations += 1;

           if (LoopItterations == 100000) {
               Shooter.SetServoPower(0);
               LoopItterations = 0;
               ServoWait = false;
           }
        }

        final double ServoSpinDirection = gamepad1.left_trigger - gamepad1.right_trigger;

        ToggleDriveSlowness = gamepad1.left_stick_button || gamepad1.right_stick_button || gamepad1.right_bumper || gamepad1.left_bumper;

        telemetry.addData("Drive Slowness", ToggleDriveSlowness);
        telemetry.addData("Current Max Shooter Power", Shooter.MaxShooterPower);

        final double Aqua = ToggleDriveSlowness ? 0.15 : 1; // Aqua is useful!

        final double Forward = Ease(PreviousY, gamepad1.left_stick_y * Aqua);
        final double Strafe = Ease(PreviousX, gamepad1.left_stick_x * Aqua);
        final double Rotate = gamepad1.right_stick_x * Aqua;

        PreviousX = Strafe;
        PreviousY = Forward;

        Drive.DriveFieldRelative(Strafe, Forward, Rotate);

        Shooter.SetShooterPower(gamepad1.a ? 1 : gamepad1.b ? -1 : 0);
        Shooter.SetServoPower(ServoSpinDirection > 0 ? 0.8 : ServoSpinDirection < 0 ? -0.8 : 0);

        if (gamepad1.dpadDownWasReleased()) {
            Shooter.MaxShooterPower = Math.max(Shooter.MaxShooterPower - 0.1, 0);
        } else if (gamepad1.dpadUpWasReleased()) {
            Shooter.MaxShooterPower = Math.min(Shooter.MaxShooterPower + 0.1, 1);
        }

        if (gamepad1.xWasPressed()) {
            ServoWait = true;
            Shooter.SetServoPower(0.8);
        }

//        if (Shooter.ShooterSpeed >= Constants.MIN_SPEED_TO_ENABLE_SERVOS) {
//            gamepad1.rumble(100);
//        }

        if (gamepad1.shareWasPressed()) { Drive.ResetIMU(); }
    }
}

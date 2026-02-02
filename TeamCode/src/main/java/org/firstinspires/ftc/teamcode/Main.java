package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Main extends OpMode {
    // Make variables
    Drive Drive = new Drive();
    Shooter Shooter = new Shooter();

    boolean ToggleDriveSlowness = false;

    // Initialize driving
    @Override
    public void init() {
        Drive.Init(hardwareMap);
        Shooter.Init(hardwareMap);
    }

    // Initialize gamepad controls
    @Override
    public void loop() {
        final double ServoSpinDirection = gamepad1.left_trigger - gamepad1.right_trigger;

        ToggleDriveSlowness = gamepad1.left_stick_button || gamepad1.right_stick_button || gamepad1.right_bumper || gamepad1.left_bumper;

        telemetry.addData("Drive Slowness", ToggleDriveSlowness);
        telemetry.addData("Current Max Shooter Power", Shooter.MaxShooterPower);

        final double Aqua = ToggleDriveSlowness ? 0.15 : 1; // Aqua is useful!

        final double Forward = -gamepad1.left_stick_y * Aqua;
        final double Strafe = gamepad1.left_stick_x * Aqua;
        final double Rotate = gamepad1.right_stick_x * Aqua;

        Drive.DriveFieldRelative(Strafe, Forward, Rotate);

        Shooter.SetShooterPower(gamepad1.a ? 1 : gamepad1.b ? -1 : 0);
        Shooter.SetServoPower(ServoSpinDirection > 0 ? 1 : ServoSpinDirection < 0 ? -1 : 0);
        Shooter.MaxShooterPower = gamepad1.dpadDownWasReleased() ? Shooter.MaxShooterPower - 0.1 : gamepad1.dpadUpWasReleased() ? Shooter.MaxShooterPower + 0.1 : Shooter.MaxShooterPower;

        if (gamepad1.ps) {
            Drive.ResetIMU();
        }
    }
}

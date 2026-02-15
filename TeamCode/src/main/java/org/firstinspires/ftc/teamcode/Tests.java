package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

//@Disabled
@TeleOp
public class Tests extends OpMode {
    // TODO: maybe add the telemetry from tests to main
    Shooter Shooter = new Shooter();
    Drive Drive = new Drive();

    double Position = 0;

    @Override
    public void init() {
        Shooter.Init(hardwareMap);
        Drive.Init(hardwareMap);
    }

    @Override
    public void loop() {
        // Test Shooter methods

        Shooter.SetShooterPower(gamepad1.a ? 1 : gamepad1.b ? -1 : 0);
        telemetry.addData("Shooter: Shooter", "OK");

        Shooter.SetServoPower(0);
        telemetry.addData("Shooter: Servos", "OK");

        telemetry.addData("Shooter Speed", Shooter.ShooterSpeed);

        // Test Test Servo

        Servo TestServo = hardwareMap.get(Servo.class, "TS");
        Servo TestServo2 = hardwareMap.get(Servo.class, "TS2");

        // thinking a lot in this space...

        Position = Position + 0.1 % 1;
        telemetry.addData("Servo Position", TestServo.getPosition());
        TestServo.setPosition(Position);
        TestServo2.setPosition(Position);

        // Test Drive methods

        double Forward = gamepad1.left_stick_y;
        double Strafe = gamepad1.left_stick_x;
        double Rotate = gamepad1.right_stick_x;

        Drive.DriveFieldRelative(Strafe, Forward, Rotate);

        telemetry.addData("Drive: DriveFieldRelative","OK");
        telemetry.addData("Forward", Forward);
        telemetry.addData("Strafe", Strafe);
        telemetry.addData("Rotate", Rotate);
        telemetry.addData("Yaw", Drive.Imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

//        Drive.ResetIMU();
//        telemetry.addData("Drive: ResetIMU", "OK");

        // Test joysticks

        telemetry.addData("gamepad1.left_stick_x", gamepad1.left_stick_x);
        telemetry.addData("gamepad1.left_stick_y", gamepad1.left_stick_y);
        telemetry.addData("gamepad1.right_stick_x", gamepad1.right_stick_x);
        telemetry.addData("gamepad1.right_stick_y", gamepad1.right_stick_y);

        // Test buttons

        telemetry.addData("gamepad1.a", gamepad1.a);
        telemetry.addData("gamepad1.b", gamepad1.b);
        telemetry.addData("gamepad1.x", gamepad1.x);
        telemetry.addData("gamepad1.y", gamepad1.y);
        telemetry.addData("gamepad1.ps", gamepad1.ps);
        telemetry.addData("gamepad.left_bumper",gamepad1.left_bumper);
        telemetry.addData("gamepad1.right_bumper",gamepad1.right_bumper);
        telemetry.addData("gamepad1.right_stick_button", gamepad1.right_stick_button);
        telemetry.addData("gamepad1.left_stick_button", gamepad1.left_stick_button);
    }
}
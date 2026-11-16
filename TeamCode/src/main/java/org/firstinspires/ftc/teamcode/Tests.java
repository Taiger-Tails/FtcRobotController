package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.lang.reflect.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

//@Disabled
@TeleOp
public class Tests extends OpMode {
    // TODO: maybe add the telemetry from tests to main
    Shooter Shooter = new Shooter();
    Drive Drive = new Drive();

    double Position = 0;

    private void DumpFields(Object object, Telemetry telemetry) throws IllegalAccessException {
        Class<?> c = object.getClass();
        Field[] fields = c.getDeclaredFields();

        for (Field f : fields) {
            Class<?> type = f.getType();

            telemetry.addData(type.getName(), f.getName());
            telemetry.addData("", f.get(c));
        }
    }

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

        try {
            DumpFields(gamepad1, telemetry);
        } catch (IllegalAccessException e) {
            telemetry.addData("failed to dump gamepad1 ;-; ", e.getMessage());
        }
    }
}
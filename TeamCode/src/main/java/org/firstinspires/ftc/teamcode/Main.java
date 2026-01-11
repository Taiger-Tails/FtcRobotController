package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Main extends OpMode {
    Drive drive = new Drive();

    double forward, strafe, rotate;

    // Initialize driving
    @Override
    public void init() {
        drive.Init(hardwareMap);
    }

    @Override
    public void loop() {

        forward = gamepad1.left_stick_y;
        strafe = -gamepad1.left_stick_x;
        rotate = -gamepad1.right_stick_x;

        drive.DriveFieldRelative(forward,strafe,rotate);

        if (gamepad1.ps) {
            drive.ResetIMU();
        }
    }
}

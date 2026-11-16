package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.lang.reflect.Field;

public abstract class DragonMaidOpMode extends OpMode {
    public void InitializeRobotComponents() {
        Class<?> c = this.getClass();

        for (Field field : c.getFields()) {
            if (field.getType() == RobotComponent.class) {
                try {
                    RobotComponent robotComponent = ((RobotComponent)field.get(c));

                    assert robotComponent != null;

                    robotComponent.Init(hardwareMap);
                } catch (IllegalAccessException e) {
                    telemetry.addData("failed to run init ;-; ", e.getMessage());
                }
            }
        }
    }
}

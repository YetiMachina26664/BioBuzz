package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Color Sensor Test", group="Linear OpMode")
public class ColorTest extends OpMode {
    private NormalizedRevColorSensorV3 color;

    public void init() {
        color = hardwareMap.get(NormalizedRevColorSensorV3.class, "color");
    }

    public void loop() {
        telemetry.addData("Red: ", (int)(color.red() * 255));
        telemetry.addData("Green: ", (int)(color.green() * 255));
        telemetry.addData("Blue: ", (int)(color.blue() * 255));
        telemetry.update();
    }
}

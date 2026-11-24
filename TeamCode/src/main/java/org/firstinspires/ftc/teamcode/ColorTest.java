package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@TeleOp(name="Color Sensor Test", group="Linear OpMode")
public class ColorTest extends OpMode {
    private NormalizedColorSensor color;

    public void init() {
        color = hardwareMap.get(NormalizedColorSensor.class, "color");
    }

    public void loop() {
        NormalizedRGBA colors = color.getNormalizedColors();

        telemetry.addData("Red: ", (int) (colors.red * 255));
        telemetry.addData("Green: ", (int) (colors.green * 255));
        telemetry.addData("Blue: ", (int)( colors.blue * 255));
        telemetry.update();
    }
}

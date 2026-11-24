package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "Curve Auto", group = "Autonomous")
public class CurveAuto extends LinearOpMode {

    private Follower follower;
    private Servo gateNectar;
    private DcMotorEx flywheel;
    private NormalizedColorSensor colorSensor;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(57.0405, 8.3227, 90);
    private final Pose path1 = poseFactory.of(22, 70, 0);
    private final Pose path1Control1 = poseFactory.of(23, 22, 0);
    private final Pose point2Start = poseFactory.of(22, 70, 0);
    private final Pose point2 = poseFactory.of(59.5, 117.9949, 270);
    private final Pose point2Control1 = poseFactory.of(22.5, 107, 0);
    private final Pose point3 = poseFactory.of(59.5, 133, 270);

    // Color detection tolerance matching OdoDriver pattern
    public static final int colorTolerance = 15;

    // 8 Color RGB Reference Arrays
    public static final int[] redCheck     = {28, 14, 8};    // PLA Matte Scarlet Red (11200)
    public static final int[] greenCheck   = {13, 38, 24};   // PLA Matte Grass Green (11500)
    public static final int[] blueCheck    = {8, 25, 50};    // PLA Basic Cyan / Blue (10603)
    public static final int[] yellowCheck  = {45, 69, 24};   // PLA Matte Lemon Yellow (11400)
    public static final int[] pinkCheck    = {};   // PLA Matte Pink (11201)
    public static final int[] darkCyanCheck = {};   // PLA Matte Dark Cyan (11602)
    public static final int[] brownCheck  = {};   // PETG HF Brown (33801)
    public static final int[] whiteCheck   = {};   // PLA Basic White (10100)

    private Command selectedRoutine = null;
    private String selectedRoutineName = "None (Default)";

    public double toTPS6000(double rpm) {
        return (rpm * 28) / 60;
    }

    public boolean compareColorArrays(int[] rgb, int[] colorCheck) {
        return (Math.abs(rgb[0] - colorCheck[0]) <= colorTolerance) &&
               (Math.abs(rgb[1] - colorCheck[1]) <= colorTolerance) &&
               (Math.abs(rgb[2] - colorCheck[2]) <= colorTolerance);
    }

    // Autonomous routines using IVY commands

    //RCU 4 + 3
    public Command autoRoutineRed() {
        return sequential(
                // 1. Rev flywheel to proper RPM
                instant(() -> flywheel.setVelocity(toTPS6000(3400))),
                // 2. Wait 0.75s for flywheel spin-up
                waitMs(750.0),
                // 3. Open gate
                instant(() -> gateNectar.setPosition(0.7)),
                // 4. Wait 4s with gate open
                waitMs(4000.0),
                // 5. Close gate and stop flywheel
                instant(() -> {
                    gateNectar.setPosition(0.0);
                    flywheel.setVelocity(0);
                }),
                // 6. Follow the curved paths
                follow(follower, path1()),
                follow(follower, path2()),
                follow(follower, path3())
        );
    }

    //BCU 4 + 3
    public Command autoRoutineBlue() {
        return sequential(
                instant(() -> flywheel.setVelocity(toTPS6000(3400))),
                waitMs(750.0),
                instant(() -> gateNectar.setPosition(0.7)),
                waitMs(4000.0),
                instant(() -> {
                    gateNectar.setPosition(0.0);
                    flywheel.setVelocity(0);
                }),
                follow(follower, path1()),
                follow(follower, path2()),
                follow(follower, path3())
        );
    }

    //BFU 4 + 3
    public Command autoRoutineGreen() {
        return sequential(

        );
    }

    //RFU 4 + 3
    public Command autoRoutineYellow() {
        return sequential(

        );
    }

    //BFA 8 + 3
    public Command autoRoutinePink() {
        return sequential(
                // 1. Rev flywheel to proper RPM
                instant(() -> flywheel.setVelocity(toTPS6000(3400))),
                // 2. Wait 0.75s for flywheel spin-up
                waitMs(750.0),
                // 3. Open gate
                instant(() -> gateNectar.setPosition(0.7)),
                // 4. Wait 4s with gate open
                waitMs(4000.0),
                // 5. Close gate and stop flywheel
                instant(() -> {
                    gateNectar.setPosition(0.0);
                    flywheel.setVelocity(0);
                }),
                // 6. Follow the curved paths
                follow(follower, path1()),
                follow(follower, path2()),
                follow(follower, path3())
        );
    }

    //RCA 8 + 3
    public Command autoRoutineDarkCyan() {
        return sequential(

        );
    }

    //RFA 8 + 3
    public Command autoRoutineBrown() {
        return sequential(

        );
    }

    //BCA 8 + 3
    public Command autoRoutineWhite() {
        return sequential(

        );
    }

    private void updateSelectedRoutineFromColor(NormalizedRGBA colorsNormalized) {
        int[] rgb = {
                (int) (colorsNormalized.red * 255),
                (int) (colorsNormalized.green * 255),
                (int) (colorsNormalized.blue * 255)
        };

        if (compareColorArrays(rgb, redCheck)) {
            selectedRoutine = autoRoutineRed();
            selectedRoutineName = "RCU 4 + 3";
        } else if (compareColorArrays(rgb, blueCheck)) {
            selectedRoutine = autoRoutineBlue();
            selectedRoutineName = "BCU 4 + 3";
        } else if (compareColorArrays(rgb, greenCheck)) {
            selectedRoutine = autoRoutineGreen();
            selectedRoutineName = "BFU 4 + 3";
        } else if (compareColorArrays(rgb, yellowCheck)) {
            selectedRoutine = autoRoutineYellow();
            selectedRoutineName = "RFU 4 + 3";
        } else if (compareColorArrays(rgb, pinkCheck)) {
            selectedRoutine = autoRoutinePink();
            selectedRoutineName = "BFA 8 + 3";
        } else if (compareColorArrays(rgb, brownCheck)) {
            selectedRoutine = autoRoutineBrown();
            selectedRoutineName = "RFA 8 + 3";
        } else if (compareColorArrays(rgb, darkCyanCheck)) {
            selectedRoutine = autoRoutineDarkCyan();
            selectedRoutineName = "RCA 8 + 3";
        } else if (compareColorArrays(rgb, whiteCheck)) {
            selectedRoutine = autoRoutineWhite();
            selectedRoutineName = "BCA 8 + 3";
        } else {
            selectedRoutine = autoRoutineRed();
            selectedRoutineName = "Default (RCA 8 + 3 - Unmatched)";
        }

        telemetry.addData("Color Sensor RGB", "%d, %d, %d", rgb[0], rgb[1], rgb[2]);
        telemetry.addData("Selected Routine", selectedRoutineName);
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);

        gateNectar = hardwareMap.get(Servo.class, "gateNectar");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setDirection(DcMotor.Direction.REVERSE);

        try {
            colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color");
        } catch (Exception e) {
            telemetry.addData("Warning", "Color sensor 'color' not found in hardwareMap!");
        }

        follower.setPose(start);
        follower.update();

        // Initialization Loop: continuous color checking until user presses START
        while (!isStarted() && !isStopRequested()) {
            if (colorSensor != null) {
                NormalizedRGBA colorsNormalized = colorSensor.getNormalizedColors();
                updateSelectedRoutineFromColor(colorsNormalized);
            } else {
                selectedRoutine = autoRoutineRed();
                selectedRoutineName = "Default (No Sensor)";
                telemetry.addData("Selected Routine", selectedRoutineName);
            }
            telemetry.update();
        }

        waitForStart();
        if (selectedRoutine != null) {
            schedule(selectedRoutine);
        }

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    public Path path1() {
        return curve(start, path1Control1, path1).linear(start, path1);
    }

    public Path path2() {
        return curve(point2Start, point2Control1, point2).linear(point2Start, point2);
    }

    public Path path3() {
        return line(point2, point3).constant(point3);
    }
}

package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "Curve Auto", group = "Autonomous")
public class CurveAuto extends LinearOpMode {

    private Follower follower;
    private Servo gateNectar;
    private DcMotorEx flywheel;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(57.0405, 8.3227, 90);
    private final Pose path1 = poseFactory.of(22, 70, 0);
    private final Pose path1Control1 = poseFactory.of(23, 22, 0);
    private final Pose point2Start = poseFactory.of(22, 70, 0);
    private final Pose point2 = poseFactory.of(59.5, 117.9949, 270);
    private final Pose point2Control1 = poseFactory.of(22.5, 107, 0);
    private final Pose point3 = poseFactory.of(59.5, 133, 270);

    public double toTPS6000(double rpm) {
        return (rpm * 28) / 60;
    }

    // Autonomous routine using IVY commands
    public Command autoRoutine() {
        return sequential(
                // 1. Rev flywheel to proper RPM (500 RPM)
                instant(() -> flywheel.setVelocity(toTPS6000(3400))),
                // 2. Wait 1 second for flywheel to spin up
                waitMs(1000.0),
                // 3. Open gate
                instant(() -> gateNectar.setPosition(0.7)),
                // 4. Wait 3 seconds with gate open
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

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);

        gateNectar = hardwareMap.get(Servo.class, "gateNectar");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setDirection(DcMotor.Direction.REVERSE);

        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

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

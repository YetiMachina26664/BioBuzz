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

@Autonomous(name = "Curve Auto Ball Pick Up", group = "Autonomous")
public class AutoCurvePark extends LinearOpMode {

    private Follower follower;
    private Servo gateNectar;
    private DcMotorEx flywheel;
    private DcMotor intake = null;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose path1 = poseFactory.of(7.5, 22, 90);
    private final Pose point2Start = poseFactory.of(7.5, 22, 90);
    private final Pose point2 = poseFactory.of(7.503, 7.5, 90);
    private final Pose point3Start = poseFactory.of(7.503, 7.5, 90);
    private final Pose point3 = poseFactory.of(22, 70, 0);
    private final Pose point4Start = poseFactory.of(22, 70, 0);
    private final Pose point4 = poseFactory.of(59, 120, 270);
    private final Pose point4Control1 = poseFactory.of(22, 120, 0);
    private final Pose point5 = poseFactory.of(59, 133.5, -90);
    private final Pose point6Start = poseFactory.of(59, 133.5, 270);
    private final Pose point6 = poseFactory.of(15, 92, 0);

    public double toTPS6000(double rpm) {
        return (rpm * 28) / 60;
    }

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                // 1. Rev flywheel to proper RPM (500 RPM)
                instant(() -> flywheel.setVelocity(toTPS6000(3400))),
                // 2. Wait 3/4 second for flywheel to spin up
                waitMs(750.0),
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
                // 7. Intake balls
                instant(() -> intake.setPower(0.6)),
                follow(follower, path2()),
                // 8. Wait 1.5 seconds for balls to intake
                waitMs(1500),
                instant(() -> intake.setPower(0.0)),
                // 9. Follow the curved paths to the other side of the field
                follow(follower, path3()),
                follow(follower, path4()),
                follow(follower, path5()),
                // 10. Wait for flywheel to spin up
                instant(() -> flywheel.setVelocity(toTPS6000(3400))),
                waitMs(750),
                // 11. LUANCH
                instant(() -> gateNectar.setPosition(0.7)),
                waitMs(4000),
                // 12. Turn off flywheels
                instant(() -> {
                    gateNectar.setPosition(0.0);
                    flywheel.setVelocity(0);
                }),
                // 13. Follow the curved paths to park
                follow(follower, path6())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);

        intake = hardwareMap.get(DcMotor.class, "intake");
        gateNectar = hardwareMap.get(Servo.class, "gateNectar");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setDirection(DcMotor.Direction.REVERSE);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setPower(0.0);

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
        return Paths.line(start, path1).linear(start, path1);
    }

    public Path path2() {
        return Paths.line(point2Start, point2).linear(point2Start, point2);
    }

    public Path path3() {
        return Paths.line(point3Start, point3).linear(point3Start, point3);
    }

    public Path path4() {
        return Paths.curve(point4Start, point4Control1, point4).linear(point4Start, point4);
    }

    public Path path5() {
        return Paths.line(point4, point5).reverseTangent();
    }

    public Path path6() {
        return Paths.line(point6Start, point6).linear(point6Start, point6);
    }
}

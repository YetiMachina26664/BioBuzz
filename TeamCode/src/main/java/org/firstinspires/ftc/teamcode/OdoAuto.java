package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import static com.pedropathing.api.Paths.line;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "Odo Auto", group = "Autonomous")
public class OdoAuto extends OpMode {
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Poses: Start at (0, 0, 0) and move forward 30 inches to (30, 0, 0)
    private final Pose startPose = poseFactory.of(0, 0, 0);
    private final Pose forwardPose = poseFactory.of(30, 0, 0);

    private Path moveForwardPath;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);

        // Build a straight-line path from startPose to forwardPose maintaining 0 degree heading
        moveForwardPath = line(startPose, forwardPose).constant(0);

        follower.update();
    }

    @Override
    public void start() {
        follower.setPose(startPose);
        follower.follow(moveForwardPath);
    }

    @Override
    public void loop() {
        follower.update();

        // When the follower finishes following the forward path, hold the target pose so the robot stops
        if (!follower.isBusy()) {
            follower.hold(forwardPose);
        } else {
            follower.follow(moveForwardPath);
        }

        telemetry.addData("X", "%4.2f in", follower.pose().x());
        telemetry.addData("Y", "%4.2f in", follower.pose().y());
        telemetry.addData("Heading", "%4.2f deg", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.addData("Is Busy", follower.isBusy());
        telemetry.update();
    }
}

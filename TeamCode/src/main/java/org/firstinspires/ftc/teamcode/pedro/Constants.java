package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("left_front");
        c.frontRightName.set("right_front");
        c.backLeftName.set("left_back");
        c.backRightName.set("right_back");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-6.749731349194144);
        c.yPodOffset.set(6.6091276153804746);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.3788463035885131);
                Controller secondaryTranslationalForward = Controller.proportional(0.1399735862162865);
                Controller primaryTranslationalLateral = Controller.proportional(0.5276613056072817);
                Controller secondaryTranslationalLateral = Controller.proportional(0.194956753052133);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.017199344716210735));
                c.brake.set(Controller.proportionalFeedforward(0.014619443008779124));

                c.headingFeedback.set(Controller.proportional(4.948255684405298));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.047273641758774416, 0.009548334254286946));

                c.linearBrakeCoefficients.set(Matrix.diag(0.05424417509786454, 0.04895432853493132));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.001604982145837956, 0.0018292267161962338));

                c.maxAchievableForwardVelocity.set(61.3570641453111);
                c.maxAchievableStrafeVelocity.set(46.217941309725205);
                c.naturalForwardDeceleration.set(34.20137668874942);
                c.naturalStrafeDeceleration.set(67.08966098288278);
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }

    /*public static Follower create_old(HardwareMap h) {
        return null;
    }

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        // Front/back names are intentionally swapped: on this robot the configured motors
        // strafe opposite to Pedro's convention (+strafe = left, +Y = left), which caused
        // the follower's lateral correction to run away. Swapping front<->back per side
        // inverts strafe only, leaving forward and turn unchanged.
        c.frontLeftName.set("left_back");
        c.frontRightName.set("right_back");
        c.backLeftName.set("left_front");
        c.backRightName.set("right_front");

        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);

        c.manualBrakeMode.set(true);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-4.2107322272353285);
        c.yPodOffset.set(4.059937244325172);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.19555166371494526);
                Controller secondaryTranslationalForward = Controller.proportional(0.07225111450598326);
                Controller primaryTranslationalLateral = Controller.proportional(0.23419822407903643);
                Controller secondaryTranslationalLateral = Controller.proportional(0.08652998590540331);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.017839023370993044));
                c.brake.set(Controller.proportionalFeedforward(0.015163169865344087));

                c.headingFeedback.set(Controller.proportional(3.7733021756124896));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.035977561185321444, 0.005437715697055048));

                c.linearBrakeCoefficients.set(Matrix.diag(0.07502487974254253, 0.0459475013259231));
                c.quadraticBrakeCoefficients.set(Matrix.diag(7.711003821702808E-4, 0.0014251719860363442));

                c.maxAchievableForwardVelocity.set(58.21577703662971);
                c.maxAchievableStrafeVelocity.set(49.38231581919872);
                c.naturalForwardDeceleration.set(41.56017712249986);
                c.naturalStrafeDeceleration.set(62.613401493926034);
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }*/
}
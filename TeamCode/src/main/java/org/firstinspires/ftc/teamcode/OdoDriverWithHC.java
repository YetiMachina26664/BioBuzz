package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedro.Constants;

    @TeleOp(name="Odometry Driver With HC", group="Linear OpMode")
    public class OdoDriverWithHC extends OpMode {
        private Follower follower;

        // Heading control (HC): holds the robot's heading while the driver isn't turning,
        // cancelling the rotation caused by uneven wheel traction when strafing.
        private static final double HEADING_KP = 1.0;           // turn power per radian of error; tune this
        private static final double HEADING_MAX_CORRECTION = 0.3; // cap on the automatic turn power
        private static final double TURN_STICK_DEADBAND = 0.05;
        private static final double SETTLED_OMEGA = Math.toRadians(10); // rad/s; wait for rotation to stop before locking
        private double holdHeading = Double.NaN; // NaN = no heading locked
        private double stickTurn;                // raw turn command from the right stick
        public static Pose startingPose;
        private TelemetryManager telemetryM;

        private RevColorSensorV3 color;

        private DcMotorEx flywheel = null;
        private DcMotor intake = null;

        private Servo gateNectar = null;

        boolean aPressed;       // Tracks if A is toggled ON or OFF
        boolean previousAState; // Tracks the button state from the previous loop cycle

        boolean yPressed;       // Tracks if Y is toggled ON or OFF
        boolean previousYState; // Tracks the button state from the previous loop cycle

        boolean xPressed = false;       // Tracks if X is toggled ON or OFF, start with closed position
        boolean previousXState; // Tracks the button state from the previous loop cycle

        boolean bPressed;       // Tracks if B is toggled ON or OFF
        boolean previousBState; // Tracks the button state from the previous loop cycle

        boolean dPadDownToggle;         //Tracks if D-Pad Down is toggled ON or OFF
        boolean previousDPadDownState;  //Tracks the button state from the previous loop cycle

        boolean dPadUpToggle;         //Tracks if D-Pad Up is toggled ON or OFF
        boolean previousDPadUpState;  //Tracks the button state from the previous loop cycle

        boolean dPadLeftToggle;         //Tracks if D-Pad Up is toggled ON or OFF
        boolean previousDPadLeftState;  //Tracks the button state from the previous loop cycle

        boolean previousDPadRightState;  //Tracks the button state from the previous loop cycle
        boolean dPadRightToggle;         //Tracks if D-Pad Up is toggled ON or OFF
        //boolean greenLight;              //flywheel speeds meet requirements to shoot

        boolean adjustSpeed;         //Checks whether or not speed was adjusted
        boolean autoAimActive;     //true while we are rotating to tgtTheta

        //Initial flywheel percentages for far and close shots respectively
        double liftoffPoly;
        double liftoffLin;

        //Intake and Belt initial speeds
        double intakeSpeed = 0.0;

        double gateNectarPos = 0.0;

        public static final double MAX_TICKS_PER_SECOND = 2840;
        public static final double MAX_TICKS_BELT = 2840;
        public Pose BLUE_GOAL_FAR = new Pose(84.75, 90.57);
        public Pose BLUE_GOAL_CLOSE = new Pose(84.75, 53.43);
        public Pose RED_GOAL_FAR=new Pose(59.25,90.57);
        public Pose RED_GOAL_CLOSE=new Pose(59.25,53.43);

        public Pose tempPose = new Pose(0,0,0);

        double distFromGoal;
        double tgtTheta;
        double hypotenuse;
        double negativeTol;
        double positiveTol;

        boolean isAtPose = false;

        boolean isBlueTeam = false;
        boolean isFieldFar = false;

	public static final int colorTolerence = 1000;
	// Change these values
	// Colours to Check against in RGB
	public static int[] redCheck = {799245,419690,250390};
	public static int[] greenCheck = {359550,968490,635460};
	public static int[] blueCheck = {231285,654630,1390260};
	public static int[] yellowCheck = {1066665,1602420,591090};

	// Change these values
	// Locations for positions for starting locations
	public Pose BLUE_START_FAR = new Pose(84.75,144, Math.toRadians(180));
	public Pose BLUE_START_CLOSE = new Pose(84.75,0.0,Math.toRadians(180));
	public Pose RED_START_FAR = new Pose(59.25,0.0,0.0);
	public Pose RED_START_CLOSE = new Pose(59.25,144,0.0);

    double adjustedSpeed = 0.0;

    // Function to get the distance from the robot to the goal, relies on calcHypotenuse function
        public double getDist(Follower follower) {
            if (isBlueTeam && isFieldFar) {
                return calcHypotenuse(
                        BLUE_GOAL_FAR.x() - follower.pose().x(),
                        BLUE_GOAL_FAR.y() - follower.pose().y());
            } else if (isBlueTeam && !isFieldFar) {
                return calcHypotenuse(
                        BLUE_GOAL_CLOSE.x() - follower.pose().x(),
                        BLUE_GOAL_CLOSE.y() - follower.pose().y());
            } else if (!isBlueTeam && isFieldFar) {
                return calcHypotenuse(
                        RED_GOAL_FAR.x() - follower.pose().x(),
                        RED_GOAL_FAR.y() - follower.pose().y());
            } else {
                return calcHypotenuse(
                        RED_GOAL_CLOSE.x() - follower.pose().x(),
                        RED_GOAL_CLOSE.y() - follower.pose().y());
            }
        }

        //get the target heading for the robot to the goal
        public double getTargetAngle(Follower follower) {
            double tgtY, tgtX;
            if (isBlueTeam && !isFieldFar) {
                tgtY = Math.abs(BLUE_GOAL_CLOSE.y() - follower.pose().y());
                tgtX = Math.abs(BLUE_GOAL_CLOSE.x() - follower.pose().x());
            } else if (isBlueTeam && isFieldFar){
                tgtY = Math.abs(BLUE_GOAL_FAR.y() - follower.pose().y());
                tgtX = Math.abs(BLUE_GOAL_FAR.x() - follower.pose().x());
            } else if (!isBlueTeam && isFieldFar) {
                tgtY = Math.abs(RED_GOAL_FAR.y() - follower.pose().y());
                tgtX = Math.abs(RED_GOAL_FAR.x() - follower.pose().x());
            } else {
                tgtY = Math.abs(RED_GOAL_CLOSE.y() - follower.pose().y());
                tgtX = Math.abs(RED_GOAL_CLOSE.x() - follower.pose().x());
            }
            return Math.abs(((Math.PI)/2) - Math.atan2(tgtX, tgtY));
        }

        //Calculation for the straightline distance between the robot and the goal
        public double calcHypotenuse(double x, double y) {
            return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
        }

        public boolean isOnFarSide(Follower follower) {
            return (follower.pose().y() >= 0);
        }

        public double powerRegressionPoly(double x) {
            if (x < 60) {
                return 960;
            } else if (x < 160) {
                //Return (int) (Math.round((1227 - 14.4 * x + 0.194 * Math.pow(x, 2) - .0007 * Math.pow(x, 3))/ 20) * 20);
                return (int) (Math.round((480 + 5.0 * x )/ 20) * 20);
            } else {
                return 1460;
            }
            //Return (0.0014 * Math.pow(x, 2)) + (2.5179 * x) + 788.18;
        }

        public double powerRegressionLin(double x) {
            return (3.0058 * x) + 756.46;
        }

        public double toTPS6000(double rpm) {
            //return (MAX_TICKS_PER_SECOND / 28) * 60;
            return (rpm *28) / 60;
        }

        public double toRPM6000(double tps) {
            return (tps/28) * 60;
        }
		public boolean compareColorArrays(int[] rgb,int[] colorCheck){
			return (((rgb[0]>(colorCheck[0]-colorTolerence))&&(rgb[0]<(colorCheck[0]+colorTolerence)))&&
			((rgb[1]>(colorCheck[1]-colorTolerence))&&(rgb[1]<(colorCheck[1]+colorTolerence)))&&
			((rgb[2]>(colorCheck[2]-colorTolerence))&&(rgb[2]<(colorCheck[2]+colorTolerence))));
		}
        public Pose getStartingPose() {
		// Get RGB values
		int[] rgb={(int) (color.red() * 255),(int) (color.green() * 255),(int) (color.blue() * 255)};
		// Set isBlueTeam to false just in case it is on red team
		isBlueTeam=false;
		// Check RGB values against red colour to print
		// Check to see if each channel is in tolerances(10 on each side)
		if(compareColorArrays(rgb,redCheck)) return RED_START_CLOSE;

		if(compareColorArrays(rgb,yellowCheck)) return RED_START_FAR;

		// Assume Blue team because
		isBlueTeam=true;
		if(compareColorArrays(rgb,blueCheck)) return BLUE_START_CLOSE;

		if(compareColorArrays(rgb,greenCheck)) return BLUE_START_FAR;

		return new Pose(-1.0,-1.0);
        }

        @Override
        public void init() {
            follower = Constants.create(hardwareMap);

            color = hardwareMap.get(RevColorSensorV3.class, "color");

            /*NormalizedColorSensor normalizedSensor = color;
            normalizedSensor.setGain(40);
            NormalizedRGBA colors = normalizedSensor.getNormalizedColors();

            follower.setPose(getStartingPose(colors));*/
            follower.update();
            telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

            //Hardware map declarations
            gateNectar = hardwareMap.get(Servo.class, "gateNectar");
            flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
            intake = hardwareMap.get(DcMotor.class, "intake");
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intake.setPower(0.0);

            //Zero power behaviors for Flywheels
            flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Set Flywheel modes for velocity running instead of power percentage
            //flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            //comment out set modes for the fly wheels above if we want to use the PID for the flywheels
            //PIDFCoefficients newPID = new PIDFCoefficients(3.0,0.8,0.0,10.0);
            //lFlywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newPID);
            //rFlywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newPID);

            flywheel.setDirection(DcMotorSimple.Direction.REVERSE);

            aPressed = false;       // Tracks if A is toggled ON or OFF
            previousAState = false; // Tracks the button state from the previous loop cycle

            yPressed = false;       // Tracks if Y is toggled ON or OFF
            previousYState = false; // Tracks the button state from the previous loop cycle

            xPressed = true;       // Tracks if X is toggled ON or OFF
            previousXState = false; // Tracks the button state from the previous loop cycle

            bPressed = false;       // Tracks if B is toggled ON or OFF
            previousBState = false; // Tracks the button state from the previous loop cycle

            dPadDownToggle = false;         //Tracks if D-Pad Down is toggled ON or OFF
            previousDPadDownState = false;  //Tracks the button state from the previous loop cycle

            dPadUpToggle = false;         //Tracks if D-Pad Up is toggled ON or OFF
            previousDPadUpState = false;  //Tracks the button state from the previous loop cycle

            dPadLeftToggle = false;         //Tracks if D-Pad Up is toggled ON or OFF
            previousDPadLeftState = false;  //Tracks the button state from the previous loop cycle

            dPadRightToggle = false;
            previousDPadRightState = false;
            autoAimActive = false;

            adjustSpeed = true;         //Checks whether or not speed was adjusted
            //greenLight = false;         //Permission to shoot
        }

        public void init_loop() {
			follower.setPose(getStartingPose());
            follower.update();
            while (follower.pose() == null ) {
                follower.setPose(getStartingPose());
                follower.update();
            }

            follower.update();

            telemetry.addData("Red", color.red() * 255);
            telemetry.addData("Green", color.green() * 255);
            telemetry.addData("Blue", color.blue() * 255);
            telemetry.addData("Starting Pose", follower.pose());
            telemetry.update();
        }


        @Override
        public void start() {
            double forward = -gamepad1.left_stick_y;
            double lateral = -gamepad1.left_stick_x; // Pedro: +strafe = left
            double turn = gamepad1.right_stick_x;

            gateNectar.setPosition(gateNectarPos);
            follower.manual(forward, lateral, turn);
            follower.update();
        }



        @Override
        public void loop() {
            double forward = -gamepad1.left_stick_y;
            double lateral = -gamepad1.left_stick_x; // Pedro: +strafe = left
            double turn = -gamepad1.right_stick_x;

            follower.update();
            telemetryM.update();

            stickTurn = turn;
            if (autoAimActive || Math.abs(stickTurn) > TURN_STICK_DEADBAND) {
                // Driver or auto-aim is turning; release the lock.
                holdHeading = Double.NaN;
            } else {
                if (Double.isNaN(holdHeading)
                        && Math.abs(follower.velocity().omega) < SETTLED_OMEGA) {
                    holdHeading = follower.pose().heading();
                }
                if (Double.isNaN(holdHeading)) {
                    turn = 0.0; // still coasting from a turn; let it settle before locking
                } else {
                    double headingError = AngleUnit.normalizeRadians(holdHeading - follower.pose().heading());
                    turn = Math.max(-HEADING_MAX_CORRECTION,
                            Math.min(HEADING_MAX_CORRECTION, HEADING_KP * headingError));
                }
            }

            if (!autoAimActive) {
                follower.manual(forward, lateral, turn);
            }

            isFieldFar = isOnFarSide(follower);
            tgtTheta = getTargetAngle(follower);
            distFromGoal = getDist(follower);
            hypotenuse = calcHypotenuse(follower.pose().x(), follower.pose().y());
            //calculate shooting speed tolerances based on distance from the goal
            /*if (distFromGoal < 110) {
                negativeTol = 40.0;
                positiveTol = 80.0;
            } else {
                negativeTol = 40.0;
                positiveTol = 80.0;
            }*/

            // --- TOGGLE LOGIC ---
            // This detects a single button press event for each button.
            if (gamepad1.a && !previousAState) { aPressed = !aPressed; }
            previousAState = gamepad1.a;

            if (gamepad1.y && !previousYState) { yPressed = !yPressed; }
            previousYState = gamepad1.y;

            if (gamepad1.x && !previousXState) { xPressed = !xPressed; }
            previousXState = gamepad1.x;

            if (gamepad1.b && !previousBState) { bPressed = !bPressed; }
            previousBState = gamepad1.b;

            if (gamepad1.dpad_down && !previousDPadDownState) { dPadDownToggle = !dPadDownToggle; }
            previousDPadDownState = gamepad1.dpad_down;

            if (gamepad1.dpad_up && !previousDPadUpState) { dPadUpToggle = !dPadUpToggle; }
            previousDPadUpState = gamepad1.dpad_up;

            if (gamepad1.dpad_left && !autoAimActive) {
                autoAimActive = true;

                //freeze the current x/y
                Pose currentPose = follower.pose();
                tempPose = new Pose(
                        currentPose.x(),
                        currentPose.y(),
                        tgtTheta
                );

                follower.hold(tempPose);
                //dPadLeftToggle = !dPadLeftToggle;
            }

            //previousDPadLeftState = gamepad1.dpad_left;

            //if (gamepad1.dpad_right&& !previousDPadRightState) { dPadRightToggle = !dPadRightToggle; }
            //previousDPadRightState = gamepad1.dpad_left;
            //previousDPadRightState = gamepad1.dpad_right; //previous was left, was this intentional?

            // --- ACTION LOGIC ---

            //determine if there is a greenLight to shoot
            //if (flywheel.getVelocity() >= liftoffPoly - negativeTol && flywheel.getVelocity() <= liftoffPoly + positiveTol && flywheel.getVelocity() >= liftoffPoly - negativeTol && flywheel.getVelocity() <= liftoffPoly + positiveTol) {
            //    greenLight = true;
            //} else {
            //    greenLight = false;
            //}

            // Set intake speed.
            if (aPressed) {
                intakeSpeed = 0.6;
            //} else if (greenLight) {
            //    intakeSpeed = 0.7;
            } else if (bPressed) {
                intakeSpeed = -0.5;
            } else {
                intakeSpeed = 0.0;
            }

            // Nectar Gate control: Y = Open Gate (0.7), X = Close Gate (0.0)
            if (gamepad1.y) {
                gateNectarPos = 0.7;
            } else if (gamepad1.x) {
                gateNectarPos = 0.0;
            }

            if (gamepad2.y && isBlueTeam) {
                follower.setPose(new Pose(0, 0, 0));
                holdHeading = Double.NaN; // heading was reset; re-lock to the new value
            } else if (gamepad2.y && !isBlueTeam) {
                follower.setPose(new Pose(141.5, 0, Math.toRadians(180)));
                holdHeading = Double.NaN;
            }

            distFromGoal = getDist(follower);

            liftoffPoly = powerRegressionPoly(distFromGoal) + adjustedSpeed;
            liftoffLin = powerRegressionLin(distFromGoal);

        if (gamepad1.dpad_left && !follower.isBusy()) {
            tgtTheta = getTargetAngle(follower);
            follower.hold(new Pose(follower.pose().x(), follower.pose().y(), tgtTheta));
            isAtPose = follower.pose() == tempPose;
        }

        if (autoAimActive) {
            // Compute heading error in degrees
            double headingErrorRad = AngleUnit.normalizeRadians(tgtTheta - follower.pose().heading());
            double headingErrorDeg = Math.toDegrees(headingErrorRad);

            // Stop auto-aim when within 1 degree of the target
            if (Math.abs(headingErrorDeg) <= 0.8) {
                autoAimActive = false;
            }
        }

            if (gamepad1.dpad_right && autoAimActive) {
                autoAimActive = false;
                follower.manual(forward, lateral, turn);
                previousDPadRightState = gamepad1.dpad_right;
            }



            // Increment and Decrement Flywheel Speed
            // max speed is 40% of max rotation, min is 10% of min rotation.
            if (gamepad1.right_bumper && adjustSpeed) {
                adjustedSpeed += toTPS6000(25);
                adjustSpeed = false;
            }
            // Check if speed was adjusted.
            if (gamepad1.rightBumperWasReleased()) {
                adjustSpeed = true;
            }

            //if (gamepad1.left_bumper && liftoffPoly >= 0.1 && adjustSpeed) {
            if (gamepad1.left_bumper && adjustSpeed) {
                adjustedSpeed -= toTPS6000(25);
                adjustSpeed = false;
            }
            // Check if speed was adjusted.
            if (gamepad1.leftBumperWasReleased()) {
                adjustSpeed = true;
            }

            // Set Intake power percentage
            intake.setPower(intakeSpeed);

            // Set flywheel velocity (LB = High power shot, RB = Low power shot)
            double targetFlywheelTicksPerSecond;
            if (gamepad1.right_trigger > 0) {
                targetFlywheelTicksPerSecond = toTPS6000(3425) + adjustedSpeed;
            } else if (gamepad1.left_trigger > 0) {
                targetFlywheelTicksPerSecond = toTPS6000(3425)+adjustedSpeed;
            } else { // Set -100 by default.
                targetFlywheelTicksPerSecond = 0;
                //greenLight = false;
            }
            flywheel.setVelocity(targetFlywheelTicksPerSecond);
            double measuredFlywheelTicksPerSecond = flywheel.getVelocity();
            if (gamepad1.right_trigger <= 0 && gamepad1.left_trigger <= 0
                    && measuredFlywheelTicksPerSecond == 0.0) {
                gateNectarPos = 0.0;
            }
            gateNectar.setPosition(gateNectarPos);
            telemetry.addData("Sticks (raw)", "LX %.2f  LY %.2f  RX %.2f  RY %.2f",
                    gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_stick_y);
            telemetry.addData("Drive cmd", "fwd %.2f  lat %.2f  stickTurn %.2f  turnSent %.2f",
                    forward, lateral, stickTurn, turn);
            telemetry.addData("Heading hold", Double.isNaN(holdHeading) ? "off"
                    : String.format("%.1f deg (err %.2f deg)", Math.toDegrees(holdHeading),
                        Math.toDegrees(AngleUnit.normalizeRadians(holdHeading - follower.pose().heading()))));
            telemetry.addData("Pose","%4.1f, %4.1f", follower.pose().x(), follower.pose().y());
            telemetry.addData("Angle","%4.2f(degrees), %4.2f(radians)",Math.toDegrees(follower.pose().heading()), follower.pose().heading());
            telemetry.addData("Intake commanded on", intake.getPower() != 0.0);
            telemetry.addData("Intake power command", "%.1f%%", intake.getPower() * 100.0);
            telemetry.addData("Intake mode", intake.getMode());
            telemetry.addData("Intake toggles", "A: %b, B: %b", aPressed, bPressed);
            telemetry.addData("Flywheel commanded on", targetFlywheelTicksPerSecond != 0);
            telemetry.addData("Flywheel target ticks/sec", "%.2f", targetFlywheelTicksPerSecond);
            telemetry.addData("Flywheel target RPM", "%.2f", toRPM6000(targetFlywheelTicksPerSecond));
            telemetry.addData("Flywheel adjustment RPM", "%.2f", toRPM6000(adjustedSpeed));
            telemetry.addData("Flywheel measured ticks/sec", "%.2f", measuredFlywheelTicksPerSecond);
            telemetry.addData("Flywheel motor RPM (28 ticks/rev)", "%.2f", toRPM6000(measuredFlywheelTicksPerSecond));
            telemetry.addData("Flywheel encoder ticks", flywheel.getCurrentPosition());
            telemetry.addData("Team Color","%s",isBlueTeam ? "Blue":"Red");
            telemetry.addData("Field Side","%s", isFieldFar ? "Far":"Close");
            telemetry.addData("Goal","Distance:%4.1f,X:4.1f,Y:4.1f", distFromGoal, isBlueTeam ? (isFieldFar ? BLUE_GOAL_FAR.x():BLUE_GOAL_CLOSE.x()) : (isFieldFar ? RED_GOAL_FAR.x():RED_GOAL_CLOSE.x()), isBlueTeam ? (isFieldFar ? BLUE_GOAL_FAR.y():BLUE_GOAL_CLOSE.y()) : (isFieldFar ? RED_GOAL_FAR.y():RED_GOAL_CLOSE.y()));
            telemetry.addData("Target Angle","%4.2f(degrees), %4.2f(radians)", Math.toDegrees(tgtTheta), tgtTheta);
            telemetry.addData("Difference in Angles","%4.2f(degrees), %4.2f(radians)",Math.toDegrees(tgtTheta-follower.pose().heading()), tgtTheta-follower.pose().heading());
            /*
            telemetry.addData("Flywheel TPS Left/Right", "%4.2f, %4.2f", rFlywheel.getVelocity(), lFlywheel.getVelocity());
            telemetry.addData("Belt TPS", "%4.2f", belt.getVelocity());
            telemetry.addData("Belt TPS Target", "%4.2f", beltSpeed);
            telemetry.addData("Ballz target", "%4.2f", ballzPos);
            telemetry.addData("Ballz", "%4.2f", ballz.getPosition());
            telemetry.addData("Intake", "%4.2f", intakeSpeed);
            telemetry.addData("PedroPose", "X: %.2f in, \nY: %.2f in, \nHeading %.2f degrees", follower.pose().x(), follower.pose().y(), Math.toDegrees(follower.pose().heading()));
            telemetry.addData("Auto-Aim Active", autoAimActive ? "Yes" : "No");
            telemetry.addData("Target Heading","%4.2f degrees", tgtTheta*57.296);
            telemetry.addData("Target Distance","%4.2f in", distFromGoal);
            telemetry.addData("GOAL POS", isBlueTeam ? BLUEGOAL.y() : new Pose(REDGOAL.x(),REDGOAL.y()));
            telemetry.addData("isBusy",follower.isBusy());
            telemetry.update();

            telemetryM.debug("position", follower.pose());
            telemetryM.debug("velocity", follower.velocity());
            telemetryM.debug("L Current Velocity", lFlywheel.getVelocity());
            telemetryM.debug("R Current Velocity", rFlywheel.getVelocity());
            telemetryM.debug("Target V % Polynomial", liftoffPoly);
            telemetryM.debug("Target V % Linear", liftoffLin);
            telemetryM.debug("belt", beltSpeed);
            telemetryM.debug("intake", intakeSpeed);
            telemetryM.debug("adjustSpeed", adjustSpeed);
            telemetryM.debug("isAtPose", isAtPose);*/
        }
    }

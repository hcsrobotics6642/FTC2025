package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@TeleOp(name = "Robot: Field Relative Mecanum Drive", group = "Robot")
//@Disabled
public class MecanumDrive extends OpMode {

    private int servoPresetIndex = 0;                     // 0, 1, or 2
    private final double[] SERVO_PRESETS = {0.0, 0.5, 1.0};  // ← change to your 3 positions

    // Optional: for debounce so holding dpad doesn't spam
    private boolean dpadLeftWasPressed  = false;
    private boolean dpadRightWasPressed = false;
    private RobotHardware robot;
    private ElapsedTime lifterTimer = new ElapsedTime();
    private boolean lifterIsFiring = false;
    private boolean targeting = false;
    private Pose2D targetScoringPose = null;

    @Override
    public void init() {
        robot = new RobotHardware(hardwareMap);
        if (RobotData.autoCompleted && RobotData.finalAutoPose != null) {
            robot.getPinpoint().setPosition(RobotData.finalAutoPose);
            telemetry.addData("Pose Loaded from Auto", "X: %.2f, Y: %.2f, Heading: %.2f",
                    RobotData.finalAutoPose.getX(DistanceUnit.INCH),
                    RobotData.finalAutoPose.getY(DistanceUnit.INCH),
                    RobotData.finalAutoPose.getHeading(AngleUnit.DEGREES));
        } else {
            robot.getPinpoint().setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
            telemetry.addData("Pose", "Initialized to (0, 0, 0)");
        }

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
    }

    @Override
    public void loop() {
        Pose2D odoPose = robot.updatePoseWithFusion();

        // Detect alliance and pattern if not set
        LLResult result = robot.getLimelight().getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                int id = fr.getFiducialId();
                if (RobotData.alliance.equals("NotSet")) {
                    if (id == RobotHardware.RED_APRILTAG_ID) {
                        RobotData.alliance = "Red";
                        robot.setTeamIndicator("Red");
                    } else if (id == RobotHardware.BLUE_APRILTAG_ID) {
                        RobotData.alliance = "Blue";
                        robot.setTeamIndicator("Blue");
                    }
                }
                if (RobotData.pattern.equals("NotSet")) {
                    if (id == RobotHardware.GPP_APRILTAG_ID) {
                        RobotData.pattern = "GPP";
                        robot.setPattern("GPP");
                    } else if (id == RobotHardware.PGP_APRILTAG_ID) {
                        RobotData.pattern = "PGP";
                        robot.setPattern("PGP");
                    } else if (id == RobotHardware.PPG_APRILTAG_ID) {
                        RobotData.pattern = "PPG";
                        robot.setPattern("PPG");
                    }
                }
            }
        }

        telemetry.addData("Alliance", RobotData.alliance);
        telemetry.addData("Pattern", RobotData.pattern);

        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        if (gamepad1.right_bumper) {robot.startIntake();}
        else if (gamepad1.left_bumper) {robot.reverseIntake();}
        else {robot.stopIntake();}

        if (gamepad1.right_trigger > 0.5 ) {robot.startFlywheel();}
        else {robot.stopFlywheel();}

        double currentRPM = robot.getFlywheelVelocity();

        if (!lifterIsFiring && currentRPM >= robot.TARGET_RPM * 0.95) {
            // First time we hit 95% → fire!
            robot.lifterServo.setPosition(0.5);
            lifterTimer.reset();
            lifterIsFiring = true;
        }

        // Has it been 500 ms yet?
        if (lifterIsFiring && lifterTimer.milliseconds() >= 500) {
            robot.lifterServo.setPosition(0.0);
            lifterIsFiring = false;          // ready for next shot
        }
        // ─────────────────────────────────────────────────────────────────────


        if (gamepad1.a) {
            //robot.getImu().resetYaw();
            double odoX = odoPose.getX(DistanceUnit.INCH);
            double odoY = odoPose.getY(DistanceUnit.INCH);
            robot.getPinpoint().setPosition(new Pose2D(DistanceUnit.INCH, odoX, odoY, AngleUnit.DEGREES, 0));
        }

        if (gamepad1.x) {
            if (RobotData.alliance.equals("NotSet")) {
                telemetry.addData("Error", "Alliance not set - cannot move to scoring location");
            } else {
                targetScoringPose = RobotData.alliance.equals("Red") ? RobotData.RED_SCORING_POSE : RobotData.BLUE_SCORING_POSE;
                targeting = true;
            }
        }


        // ────── CYCLE SERVO PRESETS WITH D-PAD LEFT / RIGHT ──────
        boolean dpadLeftPressed  = gamepad1.dpad_left || gamepad2.dpad_left;
        boolean dpadRightPressed = gamepad1.dpad_right || gamepad2.dpad_right;

// Detect button press (not hold)
        if (dpadLeftPressed && !dpadLeftWasPressed) {
            servoPresetIndex--;
            if (servoPresetIndex < 0) servoPresetIndex = SERVO_PRESETS.length - 1;
            robot.getIndexerServo().setPosition(SERVO_PRESETS[servoPresetIndex]);
        }

        if (dpadRightPressed && !dpadRightWasPressed) {
            servoPresetIndex++;
            if (servoPresetIndex >= SERVO_PRESETS.length) servoPresetIndex = 0;
            robot.getIndexerServo().setPosition(SERVO_PRESETS[servoPresetIndex]);
        }

// Save button state for next loop
        dpadLeftWasPressed  = dpadLeftPressed;
        dpadRightWasPressed = dpadRightPressed;

// ────── Optional: show current preset on Driver Station ──────
        telemetry.addData("Servo Preset", "%d → %.2f", servoPresetIndex + 1, SERVO_PRESETS[servoPresetIndex]);
        if (targeting) {
            double dx = targetScoringPose.getX(DistanceUnit.INCH) - odoPose.getX(DistanceUnit.INCH);
            double dy = targetScoringPose.getY(DistanceUnit.INCH) - odoPose.getY(DistanceUnit.INCH);
            double dheading = AngleUnit.normalizeDegrees(targetScoringPose.getHeading(AngleUnit.DEGREES) - odoPose.getHeading(AngleUnit.DEGREES));

            double positionError = Math.hypot(dx, dy);
            if (positionError < 2.0 && Math.abs(dheading) < 5.0) {
                targeting = false;
                robot.drive(0, 0, 0);
                telemetry.addData("Scoring Location", "Reached");
            } else {
                double headingRad = odoPose.getHeading(AngleUnit.RADIANS);
                double localForward = -dx * Math.sin(headingRad) + dy * Math.cos(headingRad);
                double localRight = dx * Math.cos(headingRad) + dy * Math.sin(headingRad);

                double k_p = 0.01; // tune position gain
                double k_h = 0.01; // tune heading gain

                forward = k_p * localForward;
                right = k_p * localRight;
                rotate = k_h * dheading;

                forward = Math.max(-1, Math.min(1, forward));
                right = Math.max(-1, Math.min(1, right));
                rotate = Math.max(-1, Math.min(1, rotate));

                robot.drive(forward, right, rotate);
            }
        } else {
            if (gamepad1.left_bumper) {
                robot.drive(forward, right, rotate);
            } else {
                driveFieldRelative(forward, right, rotate, odoPose);
            }
        }

        telemetry.update();
    }

    private void driveFieldRelative(double forward, double right, double rotate, Pose2D odoPose) {
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);
        theta = AngleUnit.normalizeRadians(theta - odoPose.getHeading(AngleUnit.RADIANS));
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);
        robot.drive(newForward, newRight, rotate);
    }
}
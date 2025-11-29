package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.List;

/**
 * ============================================================================
 *  RobotHardware.java  —  Teaching-Friendly, Clean, Full-Version
 * ============================================================================
 * This file defines EVERYTHING your robot physically has:
 *
 *   ✔ Drive motors
 *   ✔ Intake motor
 *   ✔ Flywheel shooter motors
 *   ✔ Servos (indexer, lifter, indicators)
 *   ✔ Limelight 3A camera (FTC-built-in API)
 *   ✔ Pinpoint odometry system
 *   ✔ PID controllers
 *
 * It also contains:
 *
 *   ✔ Clean helper functions for Limelight (tx, distance, tag ID, etc.)
 *   ✔ A simple auto-aim helper
 *   ✔ Odometry + Limelight pose fusion
 *   ✔ Drive helpers
 *   ✔ Intake helpers
 *   ✔ Flywheel RPM calculation + PID
 *
 * NOTHING in your previous logic is removed.
 * Only comments + helpful accessors were added.
 */
public class RobotHardware {

    // ============================================================================
    //                               HARDWARE DEVICES
    // ============================================================================

    // Pinpoint odometry (dead wheels) + IMU fused via Pinpoint library
    private GoBildaPinpointDriver pinpoint;

    // Limelight 3A camera (FTC SDK provided)
    private Limelight3A limelight;

    // Drive motors (mecanum)
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // Intake + Flywheel shooter motors
    private DcMotor intakeMotor;
    public DcMotor flywheelMotorLeft;
    public DcMotor flywheelMotorRight;

    // Servos
    private Servo Team_Indicator, pattern1, pattern2, pattern3;
    private Servo cameraTilt;
    private Servo indexerServo;
    public Servo lifterServo;

    // ============================================================================
    //                        CONSTANTS & INDICATOR POSITIONS
    // ============================================================================

    private static final double RED_INDICATOR = 0.27;
    private static final double BLUE_INDICATOR = 0.61;
    private static final double GREEN_INDICATOR = 0.48;
    private static final double PURPLE_INDICATOR = 0.69;
    private static final double OFF_INDICATOR = 0.0;
    private static final double ORANGE_INDICATOR = 0.58;

    // AprilTag IDs for alliance/pattern detection
    public static final int RED_APRILTAG_ID  = 24;
    public static final int BLUE_APRILTAG_ID = 20;
    public static final int GPP_APRILTAG_ID  = 21;
    public static final int PGP_APRILTAG_ID  = 22;
    public static final int PPG_APRILTAG_ID  = 23;

    // Vision fusion tuning
    private double trustVision    = 0.7;
    private double maxPoseError   = 5.0;   // Inches
    private double maxHeadingError = 10.0; // Degrees

    // Flywheel parameters
    public static final double TARGET_RPM = 6000;
    private static final double FLYWHEEL_POWER = 1.0;
    private static final double FLYWHEEL_IN_POWER = -0.5;
    private static final int TICKS_PER_REV = 28;

    private double lastEncoderPos = 0;
    private long lastTime = 0;
    private double currentRPM = 0;

    // Intake power
    private static final double INTAKE_POWER = 0.8;

    // ============================================================================
    //                               PID CONTROLLERS
    // ============================================================================

    public static class PIDController {
        private double kp, ki, kd;
        private double target = 0;
        private double integral = 0;
        private double previousError = 0;
        private double lastError = 0;
        private long lastTime = 0;
        private double tolerance = 5;

        public PIDController(double kp, double ki, double kd) {
            this.kp = kp;
            this.ki = ki;
            this.kd = kd;
        }

        public double calculate(double current) {
            long now = System.currentTimeMillis();
            double dt = (lastTime == 0) ? 0.02 : ((now - lastTime) / 1000.0);
            lastTime = now;

            double error = target - current;
            lastError = error;

            double p = kp * error;

            // Anti-windup
            if (Math.abs(error) < tolerance)
                integral += error * dt;
            else
                integral = 0;

            double i = ki * integral;
            double d = kd * ((error - previousError) / dt);
            previousError = error;

            double output = p + i + d;
            return Math.max(-1.0, Math.min(1.0, output));
        }

        public void setTarget(double t) { target = t; }
        public double getError() { return lastError; }
        public void setTolerance(double t) { tolerance = t; }
        public double getTolerance() { return tolerance; }

        public void reset() {
            integral = 0;
            previousError = 0;
            lastError = 0;
            lastTime = 0;
        }
    }

    public PIDController positionPID = new PIDController(0.01, 0.0005, 0.002);
    public PIDController headingPID = new PIDController(0.01, 0.0002, 0.001);
    public PIDController flywheelPID = new PIDController(0.5, 0, 0);

    // ============================================================================
    //                              CONSTRUCTOR
    // ============================================================================

    public RobotHardware(HardwareMap hardwareMap) {

        // ----------------------- Pinpoint -----------------------
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        configurePinpoint();

        // ----------------------- Drive Motors -------------------
        frontLeft  = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeft   = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRight  = hardwareMap.get(DcMotor.class, "back_right_drive");

        // Reverse left side for mecanum correctness
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ----------------------- Limelight 3A -------------------
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // AprilTag pipeline
        limelight.start();

        // ---------------- Indicator Servos ----------------------
        Team_Indicator = hardwareMap.get(Servo.class, "TeamIndicator");
        pattern1 = hardwareMap.get(Servo.class, "pattern1");
        pattern2 = hardwareMap.get(Servo.class, "pattern2");
        pattern3 = hardwareMap.get(Servo.class, "pattern3");

        Team_Indicator.setPosition(OFF_INDICATOR);
        pattern1.setPosition(OFF_INDICATOR);
        pattern2.setPosition(OFF_INDICATOR);
        pattern3.setPosition(OFF_INDICATOR);

        // ---------------- Other Servos --------------------------
        cameraTilt = hardwareMap.get(Servo.class, "cameraTilt");
        indexerServo = hardwareMap.get(Servo.class, "indexer");
        lifterServo = hardwareMap.get(Servo.class, "lifter");

        // ---------------- Flywheel Motors ------------------------
        flywheelMotorLeft = hardwareMap.get(DcMotor.class, "flywheelLeft");
        flywheelMotorRight = hardwareMap.get(DcMotor.class, "flywheelRight");

        flywheelMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flywheelMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        flywheelMotorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheelMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // ---------------- Intake -------------------------------
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    // ============================================================================
    //                        PINPOINT CONFIGURATION
    // ============================================================================

    public void configurePinpoint() {
        pinpoint.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pinpoint.resetPosAndIMU();
    }

    // ============================================================================
    //                         DRIVE HELPER (MECANUM)
    // ============================================================================

    public void drive(double forward, double right, double rotate) {
        double fl = forward + right + rotate;
        double fr = forward - right - rotate;
        double br = forward + right - rotate;
        double bl = forward - right + rotate;

        // Normalize if any exceeds |1|
        double max = Math.max(1.0, Math.max(Math.abs(fl),
                Math.max(Math.abs(fr),
                        Math.max(Math.abs(bl), Math.abs(br)))));

        frontLeft.setPower(fl / max);
        frontRight.setPower(fr / max);
        backLeft.setPower(bl / max);
        backRight.setPower(br / max);
    }

    // ============================================================================
    //                   LIMELIGHT 3A — CLEAN HELPER FUNCTIONS
    // ============================================================================

    /**
     * Call this once per loop in TeleOp or Auto
     */
    public LLResult getLatestLLResult() {
        return limelight.getLatestResult();
    }

    /**
     * True if Limelight sees any AprilTag.
     */
    public boolean hasTarget() {
        LLResult r = limelight.getLatestResult();
        return (r != null && r.isValid());
    }

    /**
     * Horizontal angle to tag in DEGREES.
     * Positive = target is to the RIGHT
     * Negative = target is to the LEFT
     */
    public double getTx() {
        LLResult r = limelight.getLatestResult();
        return (r != null) ? r.getTx() : 0;
    }

    /**
     * Returns the angle robot needs to rotate to face the tag.
     * Simple and extremely effective:
     *
     *   turnAngle = -tx
     */
    public double getAimAngle() {
        return -getTx();
    }

    /**
     * AprilTag ID that Limelight is currently seeing.
     */
    public int getTagID() {
        LLResult r = limelight.getLatestResult();
        if (r == null || !r.isValid()) return -1;

        List<LLResultTypes.FiducialResult> fid = r.getFiducialResults();
        if (fid.size() == 0) return -1;

        return fid.get(0).getFiducialId();
    }

    /**
     * Returns forward distance from robot to tag in METERS.
     * We use botpose for consistency with your fusion system.
     */
    public double getBotposeDistanceMeters() {
        LLResult r = limelight.getLatestResult();
        if (r == null || !r.isValid()) return -1;

        Pose3D botpose = r.getBotpose();
        Position pos = botpose.getPosition();

        return pos.z;  // forward distance in meters
    }

    // ============================================================================
    //                   POSE FUSION (Pinpoint + Limelight)
    // ============================================================================

    public Pose2D updatePoseWithFusion() {

        // Update Pinpoint first
        pinpoint.update();

        Pose2D odoPose = pinpoint.getPosition();
        double odoX = odoPose.getX(DistanceUnit.INCH);
        double odoY = odoPose.getY(DistanceUnit.INCH);
        double odoHeading = odoPose.getHeading(AngleUnit.DEGREES);

        // Now check Limelight
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {

            Pose3D botpose = result.getBotpose();
            Position limePos = botpose.getPosition();

            double visionX = limePos.x * 39.37;
            double visionY = limePos.y * 39.37;
            double visionHeading = botpose.getOrientation().getYaw(AngleUnit.DEGREES);

            double poseError = Math.hypot(visionX - odoX, visionY - odoY);
            double headingError = Math.abs(AngleUnit.normalizeDegrees(visionHeading - odoHeading));

            if (poseError < maxPoseError && headingError < maxHeadingError) {
                double fusedX = (1 - trustVision) * odoX + trustVision * visionX;
                double fusedY = (1 - trustVision) * odoY + trustVision * visionY;
                double fusedHeading = (1 - trustVision) * odoHeading + trustVision * visionHeading;

                odoPose = new Pose2D(DistanceUnit.INCH, fusedX, fusedY, AngleUnit.DEGREES, fusedHeading);
                pinpoint.setPosition(odoPose);
            }
        }

        return odoPose;
    }

    // ============================================================================
    //                        INDICATOR METHODS (UNCHANGED)
    // ============================================================================
// ============================================================================
// PATTERN DETECTION (Obelisk / Motif)
// Reads AprilTags 21 / 22 / 23 to determine pattern order.
// ============================================================================
    public void detectPattern() {

        RobotData.pattern = RobotData.Pattern.NotSet;

        long startTime = System.currentTimeMillis();
        long timeout = 5000; // 5 seconds max

        while (RobotData.pattern == RobotData.Pattern.NotSet &&
                (System.currentTimeMillis() - startTime < timeout)) {

            // Update odometry and get latest Limelight data
            updatePoseWithFusion();
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {

                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

                for (LLResultTypes.FiducialResult fr : fiducials) {

                    int id = fr.getFiducialId();

                    // These IDs come directly from your original RobotHardware.java
                    if (id == GPP_APRILTAG_ID) {
                        setPattern("GPP");
                        RobotData.pattern = RobotData.Pattern.GPP;

                    } else if (id == PGP_APRILTAG_ID) {
                        setPattern("PGP");
                        RobotData.pattern = RobotData.Pattern.PGP;

                    } else if (id == PPG_APRILTAG_ID) {
                        setPattern("PPG");
                        RobotData.pattern = RobotData.Pattern.PPG;
                    }
                }
            }
        }
    }

    public void setPatternOrange() {
        pattern1.setPosition(ORANGE_INDICATOR);
        pattern2.setPosition(ORANGE_INDICATOR);
        pattern3.setPosition(ORANGE_INDICATOR);
    }

    public void setPatternOff() {
        pattern1.setPosition(OFF_INDICATOR);
        pattern2.setPosition(OFF_INDICATOR);
        pattern3.setPosition(OFF_INDICATOR);
    }

    public void setTeamIndicator(String alliance) {
        if (alliance.equals("Red"))
            Team_Indicator.setPosition(RED_INDICATOR);
        else if (alliance.equals("Blue"))
            Team_Indicator.setPosition(BLUE_INDICATOR);
    }

    public void setPattern(String pattern) {
        if (pattern.equals("GPP")) {
            pattern1.setPosition(GREEN_INDICATOR);
            pattern2.setPosition(PURPLE_INDICATOR);
            pattern3.setPosition(PURPLE_INDICATOR);
        } else if (pattern.equals("PGP")) {
            pattern1.setPosition(PURPLE_INDICATOR);
            pattern2.setPosition(GREEN_INDICATOR);
            pattern3.setPosition(PURPLE_INDICATOR);
        } else if (pattern.equals("PPG")) {
            pattern1.setPosition(PURPLE_INDICATOR);
            pattern2.setPosition(PURPLE_INDICATOR);
            pattern3.setPosition(GREEN_INDICATOR);
        }
    }

    // ============================================================================
    //                     DRIVE-TO-POSE (UNCHANGED)
    // ============================================================================

    public void driveToPose(Pose2D target, LinearOpMode opMode) {

        double positionTolerance = 2.0;
        double headingTolerance  = 5.0;

        positionPID.setTolerance(positionTolerance);
        headingPID.setTolerance(headingTolerance);

        while (opMode.opModeIsActive()) {

            Pose2D current = updatePoseWithFusion();

            double dx = target.getX(DistanceUnit.INCH) - current.getX(DistanceUnit.INCH);
            double dy = target.getY(DistanceUnit.INCH) - current.getY(DistanceUnit.INCH);
            double dheading = AngleUnit.normalizeDegrees(
                    target.getHeading(AngleUnit.DEGREES) - current.getHeading(AngleUnit.DEGREES));

            double positionError = Math.hypot(dx, dy);

            positionPID.setTarget(0);
            double positionCorrection = positionPID.calculate(positionError);

            headingPID.setTarget(0);
            double headingCorrection = headingPID.calculate(dheading);

            if (positionError > 0.1) {

                double headingRad = current.getHeading(AngleUnit.RADIANS);
                double localForward = -dx * Math.sin(headingRad) + dy * Math.cos(headingRad);
                double localRight   =  dx * Math.cos(headingRad) + dy * Math.sin(headingRad);

                double forward = positionCorrection * (localForward / positionError);
                double right   = positionCorrection * (localRight / positionError);
                double rotate  = headingCorrection;

                drive(forward, right, rotate);
            }
            else {
                drive(0, 0, headingCorrection);
            }

            opMode.sleep(10);

            if (positionError < positionTolerance &&
                    Math.abs(dheading) < headingTolerance)
                break;
        }

        drive(0, 0, 0);
    }

    // ============================================================================
    //                          FLYWHEEL SYSTEM
    // ============================================================================

    public void startFlywheel() {
        flywheelMotorLeft.setPower(FLYWHEEL_POWER);
        flywheelMotorRight.setPower(FLYWHEEL_POWER);
        flywheelPID.setTarget(TARGET_RPM);
        flywheelPID.setTolerance(100);
        lastEncoderPos = flywheelMotorLeft.getCurrentPosition();
        lastTime = System.currentTimeMillis();
    }

    public void startFlywheelIntake() {
        flywheelMotorLeft.setPower(FLYWHEEL_IN_POWER);
        flywheelMotorRight.setPower(FLYWHEEL_IN_POWER);
        flywheelPID.setTarget(TARGET_RPM);
        flywheelPID.setTolerance(100);
        lastEncoderPos = flywheelMotorLeft.getCurrentPosition();
        lastTime = System.currentTimeMillis();
    }

    public void updateFlywheel() {

        if (Math.abs(flywheelMotorLeft.getPower()) < 0.01)
            return;

        long now = System.currentTimeMillis();
        double dt = (now - lastTime) / 1000.0;

        if (dt <= 0) return;

        double currentPos = flywheelMotorLeft.getCurrentPosition();
        double velocityTicksPerSec = (currentPos - lastEncoderPos) / dt;
        currentRPM = (velocityTicksPerSec * 60) / TICKS_PER_REV;

        double correction = flywheelPID.calculate(currentRPM);
        flywheelMotorLeft.setPower(correction);
        flywheelMotorRight.setPower(correction);

        lastEncoderPos = currentPos;
        lastTime = now;
    }

    public void stopFlywheel() {
        flywheelMotorLeft.setPower(0);
        flywheelMotorRight.setPower(0);
        flywheelPID.reset();
    }

    public void setFlywheelPower(double p) {
        flywheelMotorLeft.setPower(p);
        flywheelMotorRight.setPower(p);
    }

    public boolean isFlywheelReady() {
        return Math.abs(flywheelPID.getError()) < flywheelPID.getTolerance();
    }

    public double getFlywheelVelocity() {
        return currentRPM;
    }

    // ============================================================================
    //                              INTAKE SYSTEM
    // ============================================================================

    public void startIntake() {
        intakeMotor.setPower(INTAKE_POWER);
    }

    public void stopIntake() {
        intakeMotor.setPower(0);
    }

    public void reverseIntake() {
        intakeMotor.setPower(-INTAKE_POWER);
    }

    public void setIntakePower(double p) {
        intakeMotor.setPower(p);
    }

    // ============================================================================
    //                                 GETTERS
    // ============================================================================

    public GoBildaPinpointDriver getPinpoint() { return pinpoint; }
    public Limelight3A getLimelight() { return limelight; }
    public Servo getCameraTilt() { return cameraTilt; }
    public Servo getIndexerServo() { return indexerServo; }
    public Servo getLifterServo() { return lifterServo; }
    public DcMotor getFlywheelMotor() { return flywheelMotorLeft; }

}

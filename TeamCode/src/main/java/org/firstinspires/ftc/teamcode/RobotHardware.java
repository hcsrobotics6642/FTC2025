package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.List;

public class RobotHardware {
    // Odometry and sensors
    private GoBildaPinpointDriver pinpoint;
    private Limelight3A limelight;

    // Drive motors
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // Intake and shooter motors
    private DcMotor intakeMotor;   // Active intake to pull artifacts in
    public DcMotor flywheelMotorLeft; // Flywheel shooter
    public DcMotor flywheelMotorRight; // Flywheel shooter
    // Indicator servos (for showing alliance and pattern)
    private Servo Team_Indicator, pattern1, pattern2, pattern3;

    // New servos for indexer system
    private Servo cameraTilt;    // Tilts the Limelight up/down
    private Servo indexerServo;  // Rotates the indexer wheel
    public Servo lifterServo;   // Lifts artifacts into flywheel

    // Servo positions for indicators
    private static final double RED_INDICATOR = 0.27;
    private static final double BLUE_INDICATOR = 0.61;
    private static final double GREEN_INDICATOR = 0.48;
    private static final double PURPLE_INDICATOR = 0.69;
    private static final double OFF_INDICATOR = 0.0;
    private static final double ORANGE_INDICATOR = 0.58;

    // AprilTag IDs
    public static final int RED_APRILTAG_ID = 24;
    public static final int BLUE_APRILTAG_ID = 20;
    public static final int GPP_APRILTAG_ID = 21;
    public static final int PGP_APRILTAG_ID = 22;
    public static final int PPG_APRILTAG_ID = 23;

    // Vision fusion parameters
    private double trustVision = 0.7;
    private double maxPoseError = 5.0;
    private double maxHeadingError = 10.0;

    // Flywheel parameters
    private static final double FLYWHEEL_POWER = 1;
    private static final double FLYWHEEL_IN_POWER = -0.5;
    public static final double TARGET_RPM = 6000;
    private static final int TICKS_PER_REV = 28;
    private double lastEncoderPos = 0;
    private long lastTime = 0;
    private double currentRPM = 0;

    // PID Controllers
    public static class PIDController {
        private double kp, ki, kd;
        private double target = 0;
        private double integral = 0;
        private double previousError = 0;
        private double lastError = 0;     // For getError()
        private double tolerance = 5.0;   // Default tolerance
        private long lastTime = 0;

        public PIDController(double kp, double ki, double kd) {
            this.kp = kp;
            this.ki = ki;
            this.kd = kd;
        }

        public double calculate(double current) {
            long now = System.currentTimeMillis();
            double dt = (lastTime == 0) ? 0.02 : (now - lastTime) / 1000.0;
            lastTime = now;

            double error = target - current;
            lastError = error;  // Save for getError()

            // Proportional
            double p = kp * error;

            // Integral (with anti-windup)
            if (Math.abs(error) < tolerance) {
                integral += error * dt;
            } else {
                integral = 0;
            }
            double i = ki * integral;

            // Derivative
            double derivative = (dt > 0) ? (error - previousError) / dt : 0;
            double d = kd * derivative;
            previousError = error;

            double output = p + i + d;
            return Math.max(-1.0, Math.min(1.0, output)); // Clamp to [-1, 1]
        }

        // --- Setters ---
        public void setTarget(double target) { this.target = target; }
        public void setTolerance(double tolerance) { this.tolerance = tolerance; }

        // --- Getters ---
        public double getError() { return lastError; }
        public double getTolerance() { return tolerance; }

        // --- Utility ---
        public void reset() {
            integral = 0;
            previousError = 0;
            lastError = 0;
            lastTime = 0;
        }
    }

    public PIDController positionPID = new PIDController(0.01, 0.0005, 0.002);
    public PIDController headingPID   = new PIDController(0.01, 0.0002, 0.001);
    public PIDController flywheelPID  = new PIDController(0.5, 0, 0);

    // Intake parameters
    private static final double INTAKE_POWER = 0.8;

    public RobotHardware(HardwareMap hardwareMap) {
        // Initialize Pinpoint odometry
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        configurePinpoint();

        // Initialize drive motors
        frontLeft = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeft = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRight = hardwareMap.get(DcMotor.class, "back_right_drive");

        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        // Initialize indicator servos
        Team_Indicator = hardwareMap.get(Servo.class, "TeamIndicator");
        Team_Indicator.setPosition(OFF_INDICATOR);
        pattern1 = hardwareMap.get(Servo.class, "pattern1");
        pattern1.setPosition(OFF_INDICATOR);
        pattern2 = hardwareMap.get(Servo.class, "pattern2");
        pattern2.setPosition(OFF_INDICATOR);
        pattern3 = hardwareMap.get(Servo.class, "pattern3");
        pattern3.setPosition(OFF_INDICATOR);

        // Initialize new servos
        cameraTilt = hardwareMap.get(Servo.class, "cameraTilt");
        indexerServo = hardwareMap.get(Servo.class, "indexer");
        lifterServo = hardwareMap.get(Servo.class, "lifter");

        // Initialize flywheel motor
        flywheelMotorLeft = hardwareMap.get(DcMotor.class, "flywheelLeft");
        flywheelMotorRight = hardwareMap.get(DcMotor.class, "flywheelRight");
        flywheelMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelMotorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheelMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize intake motor
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void configurePinpoint() {
        pinpoint.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
    }

    public void drive(double forward, double right, double rotate) {
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        frontLeft.setPower(frontLeftPower / maxPower);
        frontRight.setPower(frontRightPower / maxPower);
        backLeft.setPower(backLeftPower / maxPower);
        backRight.setPower(backRightPower / maxPower);
    }

    public Pose2D updatePoseWithFusion() {
        pinpoint.update();  // Ensure fresh data
        Pose2D odoPose = pinpoint.getPosition();
        double odoX = odoPose.getX(DistanceUnit.INCH);
        double odoY = odoPose.getY(DistanceUnit.INCH);
        double odoHeading = odoPose.getHeading(AngleUnit.DEGREES);  // Pinpoint heading

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();
            Position limePos = botpose.getPosition();
            double visionX = limePos.x * 39.3701;
            double visionY = limePos.y * 39.3701;
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

    public void detectAlliance() {
        RobotData.teamIndicatorSeen = false;
        RobotData.alliance = "NotSet";
        long startTime = System.currentTimeMillis();
        long timeout = 5000;

        while (!RobotData.teamIndicatorSeen && (System.currentTimeMillis() - startTime < timeout)) {
            updatePoseWithFusion();
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    int id = fr.getFiducialId();
                    if (id == RED_APRILTAG_ID) {
                        setTeamIndicator("Red");
                        RobotData.teamIndicatorSeen = true;
                        RobotData.alliance = "Red";
                    } else if (id == BLUE_APRILTAG_ID) {
                        setTeamIndicator("Blue");
                        RobotData.teamIndicatorSeen = true;
                        RobotData.alliance = "Blue";
                    }
                }
            }
        }
    }

    public void detectPattern() {
        RobotData.pattern = "NotSet";
        long startTime = System.currentTimeMillis();
        long timeout = 5000;

        while (RobotData.pattern.equals("NotSet") && (System.currentTimeMillis() - startTime < timeout)) {
            updatePoseWithFusion();
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    int id = fr.getFiducialId();
                    if (id == GPP_APRILTAG_ID) {
                        setPattern("GPP");
                        RobotData.pattern = "GPP";
                    } else if (id == PGP_APRILTAG_ID) {
                        setPattern("PGP");
                        RobotData.pattern = "PGP";
                    } else if (id == PPG_APRILTAG_ID) {
                        setPattern("PPG");
                        RobotData.pattern = "PPG";
                    }
                }
            }
        }
    }

    public void setTeamIndicator(String alliance) {
        if (alliance.equals("Red")) {
            Team_Indicator.setPosition(RED_INDICATOR);
        } else if (alliance.equals("Blue")) {
            Team_Indicator.setPosition(BLUE_INDICATOR);
        }
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

    public void driveToPose(Pose2D target, LinearOpMode opMode) {
        double positionTolerance = 2.0;
        double headingTolerance = 5.0;

        positionPID.setTolerance(positionTolerance);
        headingPID.setTolerance(headingTolerance);

        while (opMode.opModeIsActive()) {
            Pose2D current = updatePoseWithFusion();
            double dx = target.getX(DistanceUnit.INCH) - current.getX(DistanceUnit.INCH);
            double dy = target.getY(DistanceUnit.INCH) - current.getY(DistanceUnit.INCH);
            double dheading = AngleUnit.normalizeDegrees(target.getHeading(AngleUnit.DEGREES) - current.getHeading(AngleUnit.DEGREES));

            double positionError = Math.hypot(dx, dy);
            positionPID.setTarget(0);
            double positionCorrection = positionPID.calculate(positionError);

            headingPID.setTarget(0);
            double headingCorrection = headingPID.calculate(dheading);

            if (positionError > 0.1) {
                double headingRad = current.getHeading(AngleUnit.RADIANS);
                double localForward = -dx * Math.sin(headingRad) + dy * Math.cos(headingRad);
                double localRight = dx * Math.cos(headingRad) + dy * Math.sin(headingRad);

                double forward = positionCorrection * (localForward / positionError);
                double right = positionCorrection * (localRight / positionError);
                double rotate = headingCorrection;

                drive(forward, right, rotate);
            } else {
                drive(0, 0, headingCorrection);
            }

            opMode.sleep(10);

            if (positionError < positionTolerance && Math.abs(dheading) < headingTolerance) {
                break;
            }
        }
        drive(0, 0, 0);
    }

    public double getHeading(AngleUnit unit) {
        return pinpoint.getPosition().getHeading(unit);
    }

    public void resetAllPIDs() {
        positionPID.reset();
        headingPID.reset();
        flywheelPID.reset();
    }

    // ========== FLYWHEEL METHODS ==========

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
        if (Math.abs(flywheelMotorLeft.getPower()) < 0.01) return;

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
        flywheelMotorLeft.setPower(0.0);
        flywheelMotorRight.setPower(0.0);
        flywheelPID.reset();
    }

    public void setFlywheelPower(double power) {
        flywheelMotorLeft.setPower(power);
        flywheelMotorRight.setPower(power);
    }

    public boolean isFlywheelReady() {
        return Math.abs(flywheelPID.getError()) < flywheelPID.getTolerance();
    }

    public double getFlywheelVelocity() {
        return currentRPM;
    }

    // ========== INTAKE METHODS ==========

    public void startIntake() {
        intakeMotor.setPower(INTAKE_POWER);
    }

    public void stopIntake() {
        intakeMotor.setPower(0.0);
    }

    public void reverseIntake() {
        intakeMotor.setPower(-INTAKE_POWER);
    }

    public void setIntakePower(double power) {
        intakeMotor.setPower(power);
    }

    public boolean isIntakeRunning() {
        return Math.abs(intakeMotor.getPower()) > 0.01;
    }

    // ========== GETTERS ==========

    public GoBildaPinpointDriver getPinpoint() { return pinpoint; }
    public Limelight3A getLimelight() { return limelight; }
    public Servo getCameraTilt() { return cameraTilt; }
    public Servo getIndexerServo() { return indexerServo; }
    public Servo getLifterServo() { return lifterServo; }
    public DcMotor getFlywheelMotor() { return flywheelMotorLeft; }


}

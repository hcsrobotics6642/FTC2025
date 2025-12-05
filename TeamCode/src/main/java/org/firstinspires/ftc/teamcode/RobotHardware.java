package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

public class RobotHardware {

    // ====================================================
    // ------------------ DRIVE + ODOM ---------------------
    // ====================================================

    private GoBildaPinpointDriver pinpoint;
    private Limelight3A limelight;

    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // ====================================================
    // -------- INTAKE, SHOOTER, INDEXER SERVO ------------
    // ====================================================

    private DcMotor intakeMotor;
    public DcMotor flywheelMotorLeft;
    public DcMotor flywheelMotorRight;

    private Servo indexerServo;   // rotates hopper
    public Servo lifterServo;     // lifts artifact into flywheel
    // ====================================================
// APRILTAG IDs FOR OBELISK MOTIFS (DECODE 2025-2026)
// ====================================================
// Tune these based on your printed/tested tags (likely 1-3 from 36h11 family)
    public static final int GPP_APRILTAG_ID = 21;  // Green-Purple-Purple motif
    public static final int PGP_APRILTAG_ID = 22;  // Purple-Green-Purple motif
    public static final int PPG_APRILTAG_ID = 23;  // Purple-Purple-Green motif
    // ====================================================
    // -------------- INDEXER SLOT POSITIONS ---------------
    // ====================================================
    // SHOOT POSITIONS (placeholders, tune later)
    public static final double SLOT_1_SHOOT_POS = 0.325;
    public static final double SLOT_2_SHOOT_POS = 0.39;
    public static final double SLOT_3_SHOOT_POS = 0.465;

    // INTAKE POSITIONS (180 degrees away, tune later)
    public static final double SLOT_1_INTAKE_POS = 0.21;
    public static final double SLOT_2_INTAKE_POS = 0.28;

    public static final double SLOT_3_INTAKE_POS = 0.35;

    // Lifter servo positions
    private static final double LIFTER_LOAD_POS = 0.05;   // resting
    private static final double LIFTER_FIRE_POS = 0.5;   // pushes artifact in

    // PID + flywheel tuning
    public static final int TICKS_PER_REV = 28;
    public static final double TARGET_RPM = 6000;
    private long lastTime = 0;
    private double lastEncoder = 0;
    private double currentRPM = 0;

    // ====================================================
    // --------------------- CONSTRUCTOR -------------------
    // ====================================================

    public RobotHardware(HardwareMap hardwareMap) {

        // ----------------- ODOMETRY -----------------
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        configurePinpoint();

        // ----------------- DRIVE MOTORS -----------------
        frontLeft = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeft = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRight = hardwareMap.get(DcMotor.class, "back_right_drive");

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // ---------------- LIMELIGHT -----------------
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        // ---------------- SHOOTER MOTORS ----------------
        flywheelMotorLeft = hardwareMap.get(DcMotor.class, "flywheelLeft");
        flywheelMotorRight = hardwareMap.get(DcMotor.class, "flywheelRight");

        flywheelMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheelMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // ---------------- INTAKE MOTOR ----------------
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // ---------------- SERVOS ----------------
        indexerServo = hardwareMap.get(Servo.class, "indexer");
        lifterServo = hardwareMap.get(Servo.class, "lifter");

        lifterServo.setPosition(LIFTER_LOAD_POS);
    }

    // ====================================================
    // ----------------- DRIVE + ODOMETRY ------------------
    // ====================================================

    public void configurePinpoint() {
        pinpoint.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pinpoint.resetPosAndIMU();
    }

    public void drive(double forward, double strafe, double rotate) {
        double fl = forward + strafe + rotate;
        double fr = forward - strafe - rotate;
        double bl = forward - strafe + rotate;
        double br = forward + strafe - rotate;

        double max = Math.max(1.0,
                Math.max(Math.abs(fl),
                        Math.max(Math.abs(fr),
                                Math.max(Math.abs(bl), Math.abs(br)))));

        frontLeft.setPower(fl / max);
        frontRight.setPower(fr / max);
        backLeft.setPower(bl / max);
        backRight.setPower(br / max);
    }

    // Update odometry with optional Limelight fusion
    public Pose2D updatePose() {
        pinpoint.update();
        return pinpoint.getPosition();
    }

    // ====================================================
    // ------------------- INTAKE METHODS -----------------
    // ====================================================

    public void intakeForward() { intakeMotor.setPower(0.85); }
    public void intakeReverse() { intakeMotor.setPower(-0.85); }
    public void intakeOff()     { intakeMotor.setPower(0); }

    public String getIntakeState() {
        double p = intakeMotor.getPower();
        if (p > 0.1) return "IN";
        if (p < -0.1) return "OUT";
        return "OFF";
    }

    // ====================================================
    // ------------------- INDEXER (HOPPER) ----------------
    // ====================================================
    public void rotateToIntakeSlot(int slot) {
        switch (slot) {
            case 1: rotateToSlot1Intake(); break;
            case 2: rotateToSlot2Intake(); break;
            case 3: rotateToSlot3Intake(); break;
            default: rotateToSlot1Intake(); break;
        }
    }

    public void rotateToShootSlot(int slot) {
        switch (slot) {
            case 1: rotateToSlot1Shoot(); break;
            case 2: rotateToSlot2Shoot(); break;
            case 3: rotateToSlot3Shoot(); break;
            default: rotateToSlot1Shoot(); break;
        }
    }
    public void rotateToSlot1Shoot()  { indexerServo.setPosition(SLOT_1_SHOOT_POS); }
    public void rotateToSlot2Shoot()  { indexerServo.setPosition(SLOT_2_SHOOT_POS); }
    public void rotateToSlot3Shoot()  { indexerServo.setPosition(SLOT_3_SHOOT_POS); }

    public void rotateToSlot1Intake() { indexerServo.setPosition(SLOT_1_INTAKE_POS); }
    public void rotateToSlot2Intake() { indexerServo.setPosition(SLOT_2_INTAKE_POS); }
    public void rotateToSlot3Intake() { indexerServo.setPosition(SLOT_3_INTAKE_POS); }

    // ====================================================
    // ---------------------- SHOOTER -----------------------
    // ====================================================

    public void startFlywheel() {
        flywheelMotorLeft.setPower(1.0);
        //flywheelMotorRight.setPower(1.0);
        lastTime = System.currentTimeMillis();
        lastEncoder = flywheelMotorLeft.getCurrentPosition();
    }

    public void stopFlywheel() {
        flywheelMotorLeft.setPower(0);
        //flywheelMotorRight.setPower(0);
    }

    // Calculate RPM from encoder ticks
    private void updateRPM() {
        long now = System.currentTimeMillis();
        double dt = (now - lastTime) / 1000.0;

        if (dt <= 0) return;

        double current = flywheelMotorLeft.getCurrentPosition();
        double velocity = (current - lastEncoder) / dt;
        currentRPM = (velocity * 60) / TICKS_PER_REV;

        lastEncoder = current;
        lastTime = now;
    }

    public double getFlywheelRPM() { return currentRPM; }

    // -------------------- prepareShot() --------------------
    // Spins up flywheel and waits until RPM is stable
    public void prepareShot() {
        startFlywheel();

        long stableStart = 0;
        long requiredStableTime = 250;  // 250 ms stable window

        while (true) {
            updateRPM();
            double error = Math.abs(TARGET_RPM - currentRPM);

            if (error < 150) {
                if (stableStart == 0) {
                    stableStart = System.currentTimeMillis();
                }
                if (System.currentTimeMillis() - stableStart > requiredStableTime) {
                    break;  // READY TO FIRE
                }
            } else {
                stableStart = 0;  // reset stability timer
            }
        }
    }

    // -------------------- FIRE ARTIFACT --------------------
    public void shootArtifact() {
        lifterServo.setPosition(LIFTER_FIRE_POS);
        sleep(1000);
        lifterServo.setPosition(LIFTER_LOAD_POS);
        sleep(200);
    }

    private void sleep(long ms) {
        try { Thread.sleep(ms); }
        catch (InterruptedException ignored) {}
    }

    // ====================================================
    // ------------------ LIMELIGHT HELPERS ----------------
    // ====================================================
    public Limelight3A getLimelight() {
        return limelight;
    }

    public double getLimelightTX() {
        LLResult r = limelight.getLatestResult();
        if (r == null || !r.isValid()) return 0;
        return r.getTx();
    }

    public double getLimelightDistanceInches() {
        LLResult r = limelight.getLatestResult();
        if (r == null || !r.isValid()) return 0;

        Pose3D bot = r.getBotpose();
        return bot.getPosition().z * 39.37;  // meters → inches
    }

    // ====================================================
    // -------- PATTERN → SLOT ORDER (helper for auto) ----
    // ====================================================

    public int[] getMotifShotOrder() {

        // Force Java to treat this as a String to avoid Pattern class conflicts
        String pat = RobotData.pattern.toString();

        switch (pat) {
            case "GPP":
                return new int[]{1, 2, 3};   // Green, Purple, Purple
            case "PGP":
                return new int[]{2, 1, 3};   // Purple, Green, Purple
            case "PPG":
                return new int[]{3, 1, 2};   // Purple, Purple, Green
            default:
                // If pattern detection failed, always shoot in default GPP order
                return new int[]{1, 2, 3};
        }
    }


    // ====================================================
    // ---------------- PARKING ROUTINES -------------------
    // ====================================================

    public void parkRed() {
        // Strafe LEFT into backstage
        drive(0, -0.4, 0);
        sleep(1000);
        drive(0,0,0);
    }

    public void parkBlue() {
        // Strafe RIGHT into backstage
        drive(0, 0.4, 0);
        sleep(1000);
        drive(0,0,0);
    }
}

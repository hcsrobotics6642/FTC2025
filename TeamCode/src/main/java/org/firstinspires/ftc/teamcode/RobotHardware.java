package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class RobotHardware {

    // Hardware
    private GoBildaPinpointDriver pinpoint;
    private Limelight3A limelight;

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx intakeMotor;
    public DcMotorEx flywheelMotorLeft;

    private Servo indexerServo;
    public Servo lifterServo;

    private LinearOpMode opMode;

    // Constants
    public static final int GPP_APRILTAG_ID = 21;
    public static final int PGP_APRILTAG_ID = 22;
    public static final int PPG_APRILTAG_ID = 23;

    public static final double SLOT_1_SHOOT_POS = 0.325;
    public static final double SLOT_2_SHOOT_POS = 0.39;
    public static final double SLOT_3_SHOOT_POS = 0.465;

    public static final double SLOT_1_INTAKE_POS = 0.21;
    public static final double SLOT_2_INTAKE_POS = 0.28;
    public static final double SLOT_3_INTAKE_POS = 0.35;

    private static final double LIFTER_LOAD_POS = 0.05;
    private static final double LIFTER_FIRE_POS = 0.5;

    // Shooter
    public static final int    TICKS_PER_REV_FLYWHEEL = 28;
    public static final double TARGET_RPM            = 6000;

    // Intake
    private static final double INTAKE_RPM_IN  = 1800;
    private static final double INTAKE_RPM_OUT = -1800;

    // AUTO-AIM TUNING: Tune this to your ideal shooting distance (inches)
    public static final double AUTO_AIM_DISTANCE_INCHES = 78.0;  // e.g., 72-84 based on testing
    // Shooter elevation adjustments
    public Servo elevationServo;

    // Elevation tuning — only ONE distance variable used!
    public static final double ELEVATION_AT_PERFECT   = 0.45;   // Tune at AUTO_AIM_DISTANCE_INCHES
    public static final double ELEVATION_AT_FAR       = 0.62;   // Tune at ~100–110 inches
    private static final double ELEVATION_GAIN_PER_INCH = 0.008; // Raise ~0.2 over 25 extra inches
    // Constructor
    public RobotHardware(HardwareMap hardwareMap, LinearOpMode opMode) {
        this.opMode = opMode;

        // Odometry
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        configurePinpoint();

        // Drive motors
        frontLeft  = hardwareMap.get(DcMotorEx.class, "front_left_drive");
        frontRight = hardwareMap.get(DcMotorEx.class, "front_right_drive");
        backLeft   = hardwareMap.get(DcMotorEx.class, "back_left_drive");
        backRight  = hardwareMap.get(DcMotorEx.class, "back_right_drive");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        // Flywheel
        flywheelMotorLeft = hardwareMap.get(DcMotorEx.class, "flywheelLeft");
        flywheelMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelMotorLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Intake
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Servos
        indexerServo = hardwareMap.get(Servo.class, "indexer");
        lifterServo  = hardwareMap.get(Servo.class, "lifter");
        lifterServo.setPosition(LIFTER_LOAD_POS);

        elevationServo = hardwareMap.get(Servo.class, "elevation");
        elevationServo.setPosition(ELEVATION_AT_FAR);  // Start at perfect shot
    }

    public void configurePinpoint() {
        pinpoint.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pinpoint.resetPosAndIMU();
    }

    public Pose2D updatePose() {
        pinpoint.update();
        return pinpoint.getPosition();
    }
    /**
     * Auto-adjusts shooter elevation based on current distance to goal
     * Uses only AUTO_AIM_DISTANCE_INCHES — no duplicate variables!
     */
    public void setElevationForDistance() {
        double distance = getDistanceToAllianceGoalInches();

        if (distance <= 0) {
            elevationServo.setPosition(ELEVATION_AT_PERFECT);
            return;
        }

        double targetDist = AUTO_AIM_DISTANCE_INCHES;  // ← Only one source of truth!

        if (Math.abs(distance - targetDist) < 3.0) {
            elevationServo.setPosition(ELEVATION_AT_PERFECT);
        }
        else if (distance > targetDist) {
            double extra = distance - targetDist;
            double raise = extra * ELEVATION_GAIN_PER_INCH;
            double pos = ELEVATION_AT_PERFECT + raise;
            pos = Math.min(pos, ELEVATION_AT_FAR);
            elevationServo.setPosition(pos);
        }
        else {
            double lower = (targetDist - distance) * 0.004;
            double pos = ELEVATION_AT_PERFECT - lower;
            pos = Math.max(pos, 0.30);
            elevationServo.setPosition(pos);
        }
    }
    // Robot-centric drive
    public void drive(double forward, double strafe, double rotate) {
        double fl = forward + strafe + rotate;
        double fr = forward - strafe - rotate;
        double bl = forward - strafe + rotate;
        double br = forward + strafe - rotate;

        double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(fr),
                Math.max(Math.abs(bl), Math.abs(br)))));

        frontLeft.setPower(fl / max);
        frontRight.setPower(fr / max);
        backLeft.setPower(bl / max);
        backRight.setPower(br / max);
    }

    // Field-centric drive
    public void driveFieldCentric(double forward, double strafe, double rotate) {
        double botHeading = pinpoint.getPosition().getHeading(AngleUnit.RADIANS);

        double rotX = strafe * Math.cos(-botHeading) - forward * Math.sin(-botHeading);
        double rotY = strafe * Math.sin(-botHeading) + forward * Math.cos(-botHeading);

        double fl = rotY + rotX + rotate;
        double fr = rotY - rotX - rotate;
        double bl = rotY - rotX + rotate;
        double br = rotY + rotX - rotate;

        double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(fr),
                Math.max(Math.abs(bl), Math.abs(br)))));

        frontLeft.setPower(fl / max);
        frontRight.setPower(fr / max);
        backLeft.setPower(bl / max);
        backRight.setPower(br / max);
    }

    public void turnToHeading(double targetDegrees) {
        double current = pinpoint.getPosition().getHeading(AngleUnit.DEGREES);
        double error = AngleUnit.DEGREES.normalize(targetDegrees - current);

        while (opMode.opModeIsActive() && Math.abs(error) > 2.0) {
            double power = error > 0 ? 0.35 : -0.35;
            power = Math.copySign(Math.max(0.22, Math.abs(power)), power);
            drive(0, 0, power);

            pinpoint.update();
            current = pinpoint.getPosition().getHeading(AngleUnit.DEGREES);
            error = AngleUnit.DEGREES.normalize(targetDegrees - current);
        }
        drive(0, 0, 0);
    }

    public void driveStraight(double power, double inches) {
        pinpoint.update();
        Pose2D start = pinpoint.getPosition();
        double targetY = start.getY(DistanceUnit.INCH) + inches;

        drive(power, 0, 0);

        while (opMode.opModeIsActive()) {
            pinpoint.update();
            double currentY = pinpoint.getPosition().getY(DistanceUnit.INCH);
            if (power > 0 && currentY >= targetY) break;
            if (power < 0 && currentY <= targetY) break;
        }
        drive(0, 0, 0);
    }

    // Intake – velocity controlled
    public void intakeForward() {
        double ticksPerSec = (INTAKE_RPM_IN * intakeMotor.getMotorType().getTicksPerRev()) / 60.0;
        intakeMotor.setVelocity(ticksPerSec);
    }

    public void intakeReverse() {
        double ticksPerSec = (INTAKE_RPM_OUT * intakeMotor.getMotorType().getTicksPerRev()) / 60.0;
        intakeMotor.setVelocity(ticksPerSec);
    }

    public void intakeOff() {
        intakeMotor.setPower(0);
    }

    public String getIntakeState() {
        double rpm = (intakeMotor.getVelocity() * 60.0) / intakeMotor.getMotorType().getTicksPerRev();
        if (rpm > 200) return "IN";
        if (rpm < -200) return "OUT";
        return "OFF";
    }

    // Indexer
    public void rotateToIntakeSlot(int slot) {
        switch (slot) {
            case 1: indexerServo.setPosition(SLOT_1_INTAKE_POS); break;
            case 2: indexerServo.setPosition(SLOT_2_INTAKE_POS); break;
            case 3: indexerServo.setPosition(SLOT_3_INTAKE_POS); break;
        }
    }

    public void rotateToShootSlot(int slot) {
        switch (slot) {
            case 1: indexerServo.setPosition(SLOT_1_SHOOT_POS); break;
            case 2: indexerServo.setPosition(SLOT_2_SHOOT_POS); break;
            case 3: indexerServo.setPosition(SLOT_3_SHOOT_POS); break;
        }
    }

    // Shooter – VELOCITY PID
    public void setFlywheelRPM(double rpm) {
        double ticksPerSecond = (rpm * TICKS_PER_REV_FLYWHEEL) / 60.0;
        flywheelMotorLeft.setVelocity(ticksPerSecond);
    }

    public void stopFlywheel() {
        flywheelMotorLeft.setPower(0);
    }

    public double getFlywheelRPM() {
        return (flywheelMotorLeft.getVelocity() * 60.0) / TICKS_PER_REV_FLYWHEEL;
    }

    public void prepareShot() {
        setFlywheelRPM(TARGET_RPM);
        long stableStart = 0;
        long timeout = System.currentTimeMillis() + 3500;

        while (opMode.opModeIsActive() && System.currentTimeMillis() < timeout) {
            if (Math.abs(TARGET_RPM - getFlywheelRPM()) < 120) {
                if (stableStart == 0) stableStart = System.currentTimeMillis();
                else if (System.currentTimeMillis() - stableStart > 200) return;
            } else {
                stableStart = 0;
            }
        }
    }

    public void shootArtifact() {
        lifterServo.setPosition(LIFTER_FIRE_POS);
    }

    public void returnLifter() {
        lifterServo.setPosition(LIFTER_LOAD_POS);
    }

    public Limelight3A getLimelight() { return limelight; }

    // Alliance-specific high goal distance (Tag 20 = Blue, 24 = Red)
    public double getDistanceToAllianceGoalInches() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return -1;

        int targetId = (RobotData.alliance == RobotData.Alliance.Blue) ? 20 : 24;

        for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
            if (fiducial.getFiducialId() == targetId) {
                Pose3D botPose = result.getBotpose();
                return botPose.getPosition().z * 39.3701; // meters → inches
            }
        }
        return -1;
    }

    public double getAngleToAllianceGoal() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return 0;

        int targetId = (RobotData.alliance == RobotData.Alliance.Blue) ? 20 : 24;

        for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
            if (fiducial.getFiducialId() == targetId) {
                return fiducial.getTargetXDegrees(); // degrees, positive = right
            }
        }
        return 0;
    }

    /**
     * Auto-aims to the alliance goal: turns to center it, then drives to AUTO_AIM_DISTANCE_INCHES.
     * Non-blocking — returns when done or tag lost (timeout 5s).
     * Tune AUTO_AIM_DISTANCE_INCHES to your perfect shooting range (e.g., 78 inches).
     */
    public void autoAimToGoal() {
        long startTime = System.currentTimeMillis();
        long timeout = startTime + 5000;  // 5s max

        while (opMode.opModeIsActive() && System.currentTimeMillis() < timeout) {
            double angleError = getAngleToAllianceGoal();
            double currentDist = getDistanceToAllianceGoalInches();
            setElevationForDistance();  // ← Keeps elevation perfect during aim
            if (currentDist < 0) {  // Tag not visible

                break;
            }

            // Step 1: Turn to center (within 2°)
            if (Math.abs(angleError) > 2.0) {
                double turnPower = angleError * 0.025;  // P-gain (tune 0.02-0.03)
                turnPower = Math.copySign(Math.max(0.2, Math.abs(turnPower)), turnPower);
                drive(0, 0, turnPower);
                continue;
            }

            // Step 2: Drive to target distance
            double distError = AUTO_AIM_DISTANCE_INCHES - currentDist;
            if (Math.abs(distError) > 2.0) {  // Within 2 inches
                double drivePower = distError > 0 ? 0.4 : -0.4;  // Approach or back off
                drive(drivePower, 0, 0);
            } else {
                drive(0, 0, 0);  // Done!

                return;
            }
        }
        drive(0, 0, 0);  // Safety stop
    }

    public int[] getMotifShotOrder() {
        switch (RobotData.pattern) {
            case GPP: return new int[]{1, 2, 3};
            case PGP: return new int[]{2, 1, 3};
            case PPG: return new int[]{3, 1, 2};
            default:  return new int[]{1, 2, 3};
        }
    }
}
package org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class TurretLimelight {
    public DcMotorEx TurretMotor;   // Motor port used to read encoder

    private PIDController pid;
    public static double p = 0.005;
    public static double i = 0.1;
    public static double d = 0.0001;
    public static int target = 0;
    public static double tolerance = 1.0;
    public static double turetSpeed = 0.8;
    public static double targetDegrees = 0.0;
    double coefficient = 67.0/18.0;
    private static final double TURRET_MIN_TICKS = -850;
    private static final double TURRET_MAX_TICKS = 730;
    public Limelight3A limelight;
    boolean blueAlliance;
    double groundDistanceCM;
    double power;
    public double currentPos;
    public TurretLimelight(HardwareMap hardwareMap) {
        TurretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        pid = new PIDController(p, i, d);
    }
    public void resetTurretEncoder(){
        TurretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TurretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void updateEncoderPos(){
        currentPos = TurretMotor.getCurrentPosition();
    }


    public void setPid(){
        pid.setPID(p,i,d);
    }
    public void toTargetInTicks(){
        currentPos = TurretMotor.getCurrentPosition();
        double error = target - currentPos;
        if (Math.abs(error) <= tolerance) {
            power = 0.0;
        } else {
            double pidOutput = pid.calculate(currentPos, target);
            power = Math.max(-turetSpeed, Math.min(turetSpeed, pidOutput));
        }
        TurretMotor.setPower(power);
    }
    public void setTicksTarget(int targett){
        target = targett;
    }

    public void toTargetInDegrees(){
        currentPos = TurretMotor.getCurrentPosition();
        // PID control
        double targetTicks = targetDegrees*coefficient;
        double error = targetTicks - currentPos;

        if (Math.abs(error) <= tolerance) {
            power = 0.0;
        } else {
            double pidOutput = pid.calculate(currentPos, targetTicks);
            power = Math.max(-turetSpeed, Math.min(turetSpeed, pidOutput));
        }
        TurretMotor.setPower(power);
    }
    public void toTargetInDegrees2(double targetDegrees2){
//        double targetDegrees2 = Math.toDegrees(cur.getHeading() - headingTotag);
        double targetTicks = targetDegrees2 * coefficient;

        // Clamp target to limits
        targetTicks = Math.max(TURRET_MIN_TICKS,
                Math.min(TURRET_MAX_TICKS, targetTicks));

        double error = targetTicks - currentPos;

        if (Math.abs(error) <= tolerance) {
            power = 0.0;
        } else {
            double pidOutput = pid.calculate(currentPos, targetTicks);
            power = Math.max(-turetSpeed, Math.min(turetSpeed, pidOutput));
        }
        TurretMotor.setPower(power);
    }
    public void setDegreesTarget(double deg){
        targetDegrees = deg;
    }
    public void setRedAlliance(){ blueAlliance = false;}
    public void setBlueAlliance(){ blueAlliance = true;}
    public double getTagDistance(){
        return groundDistanceCM;
    }


}

//hi

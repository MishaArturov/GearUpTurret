package org.firstinspires.ftc.teamcode.pedroPathing.Tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_A_bot.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.C_Bot_Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.TurretLimelight;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "Gear up poop", group = "TeleOp")
public class turret_tracking_for_GearUp extends OpMode {
    private double distanceToGoal;

    public boolean tagInitializing;
    Servo led;
    double speed;

    TurretLimelight turret;

    private Follower follower;

    private final Pose startPose = new Pose(0,0,0);
    private final Pose redGoalClose = new Pose(62,140,0);//used for close turret aim //this pose and the one below are the poses you will tune based on your robot calibration
    private final Pose redGoalFixed = new Pose(72,144,0);//used to calculate distance
    private final Pose redGoalfar = new Pose(60,144,0);//used for far turret aim

    //below is all camera stuff
    private static final boolean USE_WEBCAM = true;
    private Position cameraPosition = new Position(DistanceUnit.INCH, 0, 9, 6, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -70, 0, 0);

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    Pose ftcPose, yourWantedPose;
    boolean tagDetected;
    double turretDeg;
    Timer turretTimer;
    double totalHedOffset;


    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        if (USE_WEBCAM) {
            try { builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")); }
            catch (Exception e) { telemetry.addLine("Warning: Webcam not found"); }
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    }
    private void updateAprilTagLocalization() {
        if (aprilTag == null) return;

        List<AprilTagDetection> dets = aprilTag.getDetections();
        tagDetected = false;

        for (AprilTagDetection d : dets) {
            if (d.metadata == null) continue;
            if (d.metadata.name.contains("Obelisk")) continue;

            tagDetected = true;

            double xIn = d.robotPose.getPosition().x;
            double yIn = d.robotPose.getPosition().y;
            double hDeg = d.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);

            ftcPose = new Pose(xIn, yIn, Math.toRadians(hDeg));
            yourWantedPose = new Pose(ftcPose.getY(), -ftcPose.getX() + 72, ftcPose.getHeading()+Math.toRadians(turretDeg));
            break;
        }
    }

    private enum Mode { nothing, findTag, faceGoal} //modes of turret
    private turret_tracking_for_GearUp.Mode mode = Mode.nothing;


    double headingToGoal;
    @Override
    public void init() {
        turret = new TurretLimelight(hardwareMap);
        follower = C_Bot_Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        initAprilTag();//ignore ts
    }
    @Override
    public void start(){
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        Pose cur = follower.getPose(); //gets current position of robot
        follower.getTotalHeading();
        turret.updateEncoderPos();

        if (gamepad1.triangleWasPressed()) {//this is what will set your pose to what you want it to be
            //you could literally just say mode = Mode.faceGoal;
            //and
////            if (tagInitializing) {                     //this what starts happending when we localize the robots position based on camera, but you guys dont need the complicated camera logic
////                tagInitializing = false;
////                led.setPosition(0);
////                mode = Mode.nothing;
////                pauseAprilTagDetection();
////            } else {
////                turretTimer.startTimer();
////                mode = Mode.findTag;
////                turret.setDegreesTarget(0);
////                led.setPosition(0.34);
////                resumeAprilTagDetection(); // Resume when starting new detection
////                movingCase = 1;
////                timerDiddyMoment = 0;
//            }
        }
        if (tagInitializing) {
            updateAprilTagLocalization();//this stuff finds the robots location with camera but in your case just set the pose to a consistent value
            if (tagDetected && yourWantedPose != null) {//in your case when triangle is pressed you should run the stuff inside this if statement
                mode = Mode.faceGoal;
                telemetry.addData("local x", yourWantedPose.getX());
                telemetry.addData("local y", yourWantedPose.getY());
                telemetry.addData("local hed", Math.toDegrees(yourWantedPose.getHeading()));
                telemetry.addData("turret angle", turretDeg);
                follower.setPose(yourWantedPose.getPose());//this is what sets the current robot pose to what you want, pedro pose
                totalHedOffset = follower.getTotalHeading() - yourWantedPose.getHeading();//this offset is important to know your actual total heading not the total heading of the pose you started with
                tagInitializing = false;
                led.setPosition(0.6);

                pauseAprilTagDetection();//useless bs
            }
        }


        Pose targett;//used for turret alignment can be both red or blue and far or close
        Pose targett2;//used to calculate distanceToGoal can only be red or blue close
        if (distanceToGoal > 125) {//logic that if robot's distance from goal is more than 125 than should align to the far goal pose
            targett = redGoalfar;
        } else {//else align for close goal
            targett = redGoalClose;
        }

        targett2 = redGoalFixed;//sets the target to do distance calculations on
        double rawAngle = Math.atan2(targett.getY() - cur.getY(), targett.getX() - cur.getX()); //calculates the angle between the robots x y and the goals x y. This doesn't account for the robots heading

        double flippedAngle = rawAngle + Math.PI;
        flippedAngle = ((flippedAngle + Math.PI) % (2 * Math.PI)) - Math.PI;
        headingToGoal = flippedAngle + Math.PI;
        //i think the 3 lines above convert that heading to appropriate pedro coordinates but i dont remember cause we i did this some time ago

        double robHeading = follower.getTotalHeading() - totalHedOffset; //gets the correct total robot heading
        while (robHeading >= Math.toRadians(210)) robHeading -= 2 * Math.PI;//when that heading exceeds 210 degrees it locks it below that so if it was 310 it would convevrt to 50
        while (robHeading < -Math.PI) robHeading += 2 * Math.PI; //locks it from other side

        //important part is that if you were to get the robot heading by just doing follower.getheading then you will always be locked to the pedros -180 to 180 degrees which means your turret will only loop around at that one spot

        if (gamepad1.shareWasPressed()) {
            turret.resetTurretEncoder();
        }
        if (mode == Mode.faceGoal) {//this is what happens when it actually aligns to goal
            turret.toTargetInDegrees2(Math.toDegrees(robHeading - headingToGoal));//sets the turret position to the robots heading minus the desired heading to goal as if the bots heading was 0
        } //we subtract the rob heading from heading to goal instead of other way around because in our case turret ticks are positive clockwise and pedro heading is positve counterclockwise
        if (mode == Mode.nothing) {
            turret.TurretMotor.setPower(0);
        }
        if (mode == Mode.findTag) {//this literally sets the turret to tick 0 which is facing forward.
            turret.toTargetInDegrees();
        }



        follower.setTeleOpDrive(-gamepad1.left_stick_y * speed, (gamepad1.left_trigger - gamepad1.right_trigger) * speed, -gamepad1.right_stick_x * speed, true);
        //the teleopdrivve setup above is how we drive you can change ts
        distanceToGoal = cur.distanceFrom(targett2);// distance from the goal, we use it to set the correct flywheel speeds and aim to the right goal
        telemetry.addData("turret tick pos", turret.currentPos);
        telemetry.addData("distance to goal", distanceToGoal);
        telemetry.addData("X", cur.getX());
        telemetry.addData("y", cur.getY());
        telemetry.addData("heading", Math.toDegrees(cur.getHeading()));//this is a pedropathing heading that it gives between -pi and pi
        telemetry.addData("total heading", Math.toDegrees(follower.getTotalHeading() - totalHedOffset));//this shows the robots total heading and can go past 180 and negative 180
        // Telemetry

        telemetry.update();
    }
    private void pauseAprilTagDetection() {
        if (visionPortal != null && aprilTag != null) {
            visionPortal.setProcessorEnabled(aprilTag, false);
        }
    }

    // Add this method to resume detection
    private void resumeAprilTagDetection() {
        if (visionPortal != null && aprilTag != null) {
            visionPortal.setProcessorEnabled(aprilTag, true);
        }
    }

}
package org.firstinspires.ftc.teamcode.pedroPathing.Tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_A_bot.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.C_Bot_Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.TurretLimelight;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "Gear up poop", group = "TeleOp")
public class turret_tracking_for_GearUp extends OpMode {
    private double distanceToGoal;

    public boolean tagInitializing;
    Servo led;
    double speed;

    TurretLimelight turret;

    private Follower follower;

    private final Pose startPose = new Pose(0,0,0);
    private final Pose redGoalClose = new Pose(62,140,0);//used for close turret aim //this pose and the one below are the poses you will tune based on your robot calibration
    private final Pose redGoalFixed = new Pose(72,144,0);//used to calculate distance
    private final Pose redGoalfar = new Pose(60,144,0);//used for far turret aim

    //below is all camera stuff
    private static final boolean USE_WEBCAM = true;
    private Position cameraPosition = new Position(DistanceUnit.INCH, 0, 9, 6, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -70, 0, 0);

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    Pose ftcPose, yourWantedPose;
    boolean tagDetected;
    double turretDeg;
    Timer turretTimer;
    double totalHedOffset;


    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        if (USE_WEBCAM) {
            try { builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")); }
            catch (Exception e) { telemetry.addLine("Warning: Webcam not found"); }
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    }
    private void updateAprilTagLocalization() {
        if (aprilTag == null) return;

        List<AprilTagDetection> dets = aprilTag.getDetections();
        tagDetected = false;

        for (AprilTagDetection d : dets) {
            if (d.metadata == null) continue;
            if (d.metadata.name.contains("Obelisk")) continue;

            tagDetected = true;

            double xIn = d.robotPose.getPosition().x;
            double yIn = d.robotPose.getPosition().y;
            double hDeg = d.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);

            ftcPose = new Pose(xIn, yIn, Math.toRadians(hDeg));
            yourWantedPose = new Pose(ftcPose.getY(), -ftcPose.getX() + 72, ftcPose.getHeading()+Math.toRadians(turretDeg));
            break;
        }
    }

    private enum Mode { nothing, findTag, faceGoal} //modes of turret
    private turret_tracking_for_GearUp.Mode mode = Mode.nothing;


    double headingToGoal;
    @Override
    public void init() {
        turret = new TurretLimelight(hardwareMap);
        follower = C_Bot_Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        initAprilTag();//ignore ts
    }
    @Override
    public void start(){
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        Pose cur = follower.getPose(); //gets current position of robot
        follower.getTotalHeading();
        turret.updateEncoderPos();

        if (gamepad1.triangleWasPressed()) {//this is what will set your pose to what you want it to be
            //you could literally just say mode = Mode.faceGoal;
            //and
////            if (tagInitializing) {                     //this what starts happending when we localize the robots position based on camera, but you guys dont need the complicated camera logic
////                tagInitializing = false;
////                led.setPosition(0);
////                mode = Mode.nothing;
////                pauseAprilTagDetection();
////            } else {
////                turretTimer.startTimer();
////                mode = Mode.findTag;
////                turret.setDegreesTarget(0);
////                led.setPosition(0.34);
////                resumeAprilTagDetection(); // Resume when starting new detection
////                movingCase = 1;
////                timerDiddyMoment = 0;
//            }
        }
        if (tagInitializing) {
            updateAprilTagLocalization();//this stuff finds the robots location with camera but in your case just set the pose to a consistent value
            if (tagDetected && yourWantedPose != null) {//in your case when triangle is pressed you should run the stuff inside this if statement
                mode = Mode.faceGoal;
                telemetry.addData("local x", yourWantedPose.getX());
                telemetry.addData("local y", yourWantedPose.getY());
                telemetry.addData("local hed", Math.toDegrees(yourWantedPose.getHeading()));
                telemetry.addData("turret angle", turretDeg);
                follower.setPose(yourWantedPose.getPose());//this is what sets the current robot pose to what you want, pedro pose
                totalHedOffset = follower.getTotalHeading() - yourWantedPose.getHeading();//this offset is important to know your actual total heading not the total heading of the pose you started with
                tagInitializing = false;
                led.setPosition(0.6);

                pauseAprilTagDetection();//useless bs
            }
        }


        Pose targett;//used for turret alignment can be both red or blue and far or close
        Pose targett2;//used to calculate distanceToGoal can only be red or blue close
        if (distanceToGoal > 125) {//logic that if robot's distance from goal is more than 125 than should align to the far goal pose
            targett = redGoalfar;
        } else {//else align for close goal
            targett = redGoalClose;
        }

        targett2 = redGoalFixed;//sets the target to do distance calculations on
        double rawAngle = Math.atan2(targett.getY() - cur.getY(), targett.getX() - cur.getX()); //calculates the angle between the robots x y and the goals x y. This doesn't account for the robots heading

        double flippedAngle = rawAngle + Math.PI;
        flippedAngle = ((flippedAngle + Math.PI) % (2 * Math.PI)) - Math.PI;
        headingToGoal = flippedAngle + Math.PI;
        //i think the 3 lines above convert that heading to appropriate pedro coordinates but i dont remember cause we i did this some time ago

        double robHeading = follower.getTotalHeading() - totalHedOffset; //gets the correct total robot heading
        while (robHeading >= Math.toRadians(210)) robHeading -= 2 * Math.PI;//when that heading exceeds 210 degrees it locks it below that so if it was 310 it would convevrt to 50
        while (robHeading < -Math.PI) robHeading += 2 * Math.PI; //locks it from other side

        //important part is that if you were to get the robot heading by just doing follower.getheading then you will always be locked to the pedros -180 to 180 degrees which means your turret will only loop around at that one spot

        if (gamepad1.shareWasPressed()) {
            turret.resetTurretEncoder();
        }
        if (mode == Mode.faceGoal) {//this is what happens when it actually aligns to goal
            turret.toTargetInDegrees2(Math.toDegrees(robHeading - headingToGoal));//sets the turret position to the robots heading minus the desired heading to goal as if the bots heading was 0
        } //we subtract the rob heading from heading to goal instead of other way around because in our case turret ticks are positive clockwise and pedro heading is positve counterclockwise
        if (mode == Mode.nothing) {
            turret.TurretMotor.setPower(0);
        }
        if (mode == Mode.findTag) {//this literally sets the turret to tick 0 which is facing forward.
            turret.toTargetInDegrees();
        }



        follower.setTeleOpDrive(-gamepad1.left_stick_y * speed, (gamepad1.left_trigger - gamepad1.right_trigger) * speed, -gamepad1.right_stick_x * speed, true);
        //the teleopdrivve setup above is how we drive you can change ts
        distanceToGoal = cur.distanceFrom(targett2);// distance from the goal, we use it to set the correct flywheel speeds and aim to the right goal
        telemetry.addData("turret tick pos", turret.currentPos);
        telemetry.addData("distance to goal", distanceToGoal);
        telemetry.addData("X", cur.getX());
        telemetry.addData("y", cur.getY());
        telemetry.addData("heading", Math.toDegrees(cur.getHeading()));//this is a pedropathing heading that it gives between -pi and pi
        telemetry.addData("total heading", Math.toDegrees(follower.getTotalHeading() - totalHedOffset));//this shows the robots total heading and can go past 180 and negative 180
        // Telemetry

        telemetry.update();
    }
    private void pauseAprilTagDetection() {
        if (visionPortal != null && aprilTag != null) {
            visionPortal.setProcessorEnabled(aprilTag, false);
        }
    }

    // Add this method to resume detection
    private void resumeAprilTagDetection() {
        if (visionPortal != null && aprilTag != null) {
            visionPortal.setProcessorEnabled(aprilTag, true);
        }
    }

}

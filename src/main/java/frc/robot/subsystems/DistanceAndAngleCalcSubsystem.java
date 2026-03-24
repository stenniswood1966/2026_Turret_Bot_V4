// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants;

//90 percent of this code is borrowed from a blog and the polar pilots :)
  //https://blog.eeshwark.com/robotblog/shooting-on-the-fly
  //https://blog.eeshwark.com/blog/shooting-on-the-fly-pt2
  //https://github.com/frc1498/2026-BuildSeason-1498/blob/main/src/main/java/frc/robot/ShotCalculation.java#L84

public class DistanceAndAngleCalcSubsystem extends SubsystemBase {
  CommandSwerveDrivetrain drivetrain;
  HoodSubsystem hoodsubsystem;
  ShooterSubsystem shootersubsystem;
  TurretSubsystem turretsubsystem;

  //calculator static variables
  SwerveDriveState state;
  double latencyCompensation = .04;//is in seconds

  double turretVectorDistance = constants.kg_TurretOffset.k_TurretOffsetPose2d.getTranslation().getDistance(new Translation2d(0,0));
  double turretVectorAngle = Math.atan2(constants.kg_TurretOffset.k_TurretOffsetPose2d.getY(), constants.kg_TurretOffset.k_TurretOffsetPose2d.getX());

  //hood variables
  double gearRatioHood = 71.25;
  double hoodOffset = 18.1;//in degrees

  //turret variables
  double gearRatioTurret = 37.6;
  double givenTarget;
  double modifier = this.gearRatioTurret;
  double potentialTarget;

  //Simple data class for the outputs of the Look up tables
  public record ShooterParams(double Rps, double hoodAngle, double timeOfFlight) {}

  //the interpolating double maps for scoring and passing do not touch 
  //distance, rps (motor land(rotations)), hood angle (motor land(rotations)), and time of flight (seconds)
  private static final InterpolatingDoubleTreeMap scoringMapShooter = new InterpolatingDoubleTreeMap();
  static {
    scoringMapShooter.put(1.0, 17.0);//distance rps in rps
    scoringMapShooter.put(2.3, 19.0);
    scoringMapShooter.put(2.8, 20.0);
    scoringMapShooter.put(3.3, 21.5);
    scoringMapShooter.put(3.8, 23.5);
    scoringMapShooter.put(4.3, 25.5);
    scoringMapShooter.put(4.8, 29.0);
    scoringMapShooter.put(5.3, 29.75);
    scoringMapShooter.put(5.7, 30.0);
    scoringMapShooter.put(6.0, 30.0);//passing point
    scoringMapShooter.put(15.0, 40.0);//passing point
  }
  private static final InterpolatingDoubleTreeMap scoringMapHood = new InterpolatingDoubleTreeMap();
  static {
    scoringMapHood.put(1.0, 0.0);//distance angle in rotations
    scoringMapHood.put(2.3, 1.65);
    scoringMapHood.put(2.8, 2.5);
    scoringMapHood.put(3.3, 2.7);
    scoringMapHood.put(3.8, 2.7);
    scoringMapHood.put(4.3, 4.0);
    scoringMapHood.put(4.8, 4.15);
    scoringMapHood.put(5.3, 4.75);
    scoringMapHood.put(5.7,4.75);
    scoringMapHood.put(7.0, 7.0);//passing point
    scoringMapHood.put(15.0, 7.0);//passing point
  }
  private static final InterpolatingDoubleTreeMap scoringMapTOF = new InterpolatingDoubleTreeMap();
  static {
    scoringMapTOF.put(1.0, .50);//distance time of flight in seconds
    scoringMapTOF.put(2.3, .67);
    scoringMapTOF.put(2.8, .73);
    scoringMapTOF.put(3.3, .76);
    scoringMapTOF.put(3.8, .82);
    scoringMapTOF.put(4.3, .81);
    scoringMapTOF.put(4.8, .87);
    scoringMapTOF.put(5.3, .95);
    scoringMapTOF.put(5.7, 1.0);
    scoringMapTOF.put(7.0, 1.0);//passing point
    scoringMapTOF.put(15.0, 1.0);//passing point
  }

  /** Creates a new DistanceAndAngleCalcSubsystem. */
  public DistanceAndAngleCalcSubsystem(CommandSwerveDrivetrain drivetrain, 
    HoodSubsystem hoodsubsystem, ShooterSubsystem shootersubsystem, TurretSubsystem turretsubsystem) {
    this.drivetrain = drivetrain;
    this.hoodsubsystem = hoodsubsystem;
    this.shootersubsystem = shootersubsystem;
    this.turretsubsystem = turretsubsystem;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //grabbing the drivetrain state which contains the current veloctiy and position of the drivetrain
    state = this.drivetrain.getState();

    // 1. Project future position
    //as far as i am aware this adjusts for the time that it takes the ball to run through the system and a few other things
    //is technically a fudge factor
    //is in robot frame of reference
    Pose2d velocityAdjustment = state.Pose.exp(
            new Twist2d(
                state.Speeds.vxMetersPerSecond * this.latencyCompensation,
                state.Speeds.vyMetersPerSecond * this.latencyCompensation,
                state.Speeds.omegaRadiansPerSecond * this.latencyCompensation
            )
        );

    //converting into turret land
    Translation2d futurePos = velocityAdjustment.getTranslation()
      .plus(constants.kg_TurretOffset.k_TurretOffsetPose2d.getTranslation());; 

    //grabbing where we want to shot to based on where we are on the field
    Translation2d goalPosition = calcTarget(state.Pose).getTranslation();

    // 2. Get target vector 
    //based from turret position and in field relative
    Translation2d toGoal = goalPosition.minus(futurePos);
    double distance = toGoal.getNorm();
    Translation2d targetDirection = toGoal.div(distance);

    // 3. Look up baseline velocity from tables
    ShooterParams baseline = new ShooterParams(
      scoringMapShooter.get(distance),
      scoringMapHood.get(distance),
      scoringMapTOF.get(distance));
    //i believe that you dont have to account for the hood angle here because it is implicitly baked into the time of flight
    double baselineVelocity = distance / baseline.timeOfFlight;

    // 4. Build target velocity vector
    Translation2d targetVelocity = targetDirection.times(baselineVelocity);

    //getting the turret velocity and all that jazz this is from the polar pilots 
      // Determine the field relative velocity of the turret.  This is mostly equal to the robot velocity, with some adjustment based on the current rotational speed of the robot.
      // Part one is just the X (or Y) component of the field relative robot speed.
      // Part two - an application of the rotation matrix, broken out into the x and y components and multiplied by a constant (the magnitude of the rotational speed).
      // The X and Y terms from the rotation matrix are swapped, because which axis are conventionally regarded as X and Y are swapped in the robot relative frame.
      // At least, that's what I think.

    ChassisSpeeds turretSpeedFieldRelative = this.toFieldRelative(state.Speeds, velocityAdjustment.getRotation());

    // I'm breaking this into multiple lines, just because it's a little more readable.
    double turretVelocityX = turretSpeedFieldRelative.vxMetersPerSecond + 
        turretSpeedFieldRelative.omegaRadiansPerSecond * (
            constants.kg_TurretOffset.k_TurretOffsetPose2d.getY() * Math.cos(velocityAdjustment.getRotation().getRadians()) - 
            constants.kg_TurretOffset.k_TurretOffsetPose2d.getX() * Math.sin(velocityAdjustment.getRotation().getRadians())
        );

    double turretVelocityY = turretSpeedFieldRelative.vyMetersPerSecond + 
        turretSpeedFieldRelative.omegaRadiansPerSecond * (
            constants.kg_TurretOffset.k_TurretOffsetPose2d.getY() * Math.sin(velocityAdjustment.getRotation().getRadians()) - 
            constants.kg_TurretOffset.k_TurretOffsetPose2d.getX() * Math.cos(velocityAdjustment.getRotation().getRadians())
        );

    Translation2d robotVelocity = new Translation2d(turretVelocityX, turretVelocityY);
    
    // 5. THE MAGIC: subtract robot velocity
    Translation2d shotVelocity = targetVelocity.minus(robotVelocity);

    // 6. Extract results
    Rotation2d turretAngle = shotVelocity.getAngle();
    double requiredVelocity = shotVelocity.getNorm();

    // 7. Use table in reverse: velocity → effective distance → Rps
    double velocityRatio = requiredVelocity / baselineVelocity;

    // Split the correction: sqrt gives equal "contribution" from each
    double RpsFactor = Math.sqrt(velocityRatio);
    double hoodFactor = Math.sqrt(velocityRatio);

    // Apply Rps scaling
    double adjustedRps = baseline.Rps * RpsFactor;

    // Apply hood adjustment (changes horizontal component)
    double totalVelocity = baselineVelocity / Math.cos(Math.toRadians(calcHoodAngleDegrees(baseline.hoodAngle)));
    double targetHorizFromHood = baselineVelocity * hoodFactor;
    double ratio = MathUtil.clamp(targetHorizFromHood / totalVelocity, 0.0, 1.0);
    double adjustedHood = Math.toDegrees(Math.acos(ratio));

    //comanding the motors to go to the correct positions after converting them into motor units
      //for the turret rotation you have to deal with the fact that on the red side zero theta is not towards the blue driver
      //station wall and that the angle will be different as the bot rotates also you have to deal with the fact that cc is positive
      //on the turret while cc is negative in field space
    this.shootersubsystem.setTargetMethod(() -> adjustedRps);
    this.hoodsubsystem.enablemotionmagic(calcHoodAngleMotorRotations(adjustedHood));
    this.turretsubsystem.enablemotionmagic(calcTurretAngle((turretAngle.getDegrees() - state.Pose.getRotation().getDegrees()) * -1));

    //this just posts to smart dashboard a bunch of numbers for debugging should probably get rid of it before comp
    //for better performace 
    SmartDashboard.putNumber("Distance", distance);
    SmartDashboard.putNumber("Angle Field Relative", targetDirection.getAngle().getDegrees());
    SmartDashboard.putNumber("Angle Turret Relative", calcTurretAngle((turretAngle.getDegrees() - state.Pose.getRotation().getDegrees()) * -1));
    SmartDashboard.putNumber("Angle Hood", calcHoodAngleMotorRotations(adjustedHood));
  }

  //takes in degrees converts to motor rotations 
  //This is done because the math behind SOTM expects zero on the release angle to be level with the floor so we have to convert
  private double calcHoodAngleMotorRotations(double adjustedHood){
    double target = 90 - adjustedHood - hoodOffset;
    target = target * this.gearRatioHood / 360;

    return target;
  }

  //takes in motor rotations converts to degrees relative to zero being level with the floor
  //This is done because the math behind SOTM expects zero on the release angle to be level with the floor so we have to convert
  private double calcHoodAngleDegrees(double adjustedHood){
    double target = adjustedHood * (360 / this.gearRatioHood);

    target = 90 - (target + hoodOffset);

    return target;
  }

  //accepts degrees works in rotations
  //this mess programmically takes in a angle in degrees converts that to motor rotations and then calculates out a motor rotation
  //that is within the soft limits. This works because the turret has a continous range of 360 so their is no dead zone
  private double calcTurretAngle(double outputAngle){
    //converting to motor rotations based off of the gear ratio of the turret
    this.givenTarget = outputAngle * this.gearRatioTurret / 360;
    this.modifier = this.gearRatioTurret;
    this.potentialTarget = givenTarget;

    //this mess is where the magic happens. First it checks if it is within the soft limits and if it is not then it walks through
    //the for loop adding i * moifier to the potential target until it is within the soft limits. This also has the benifit of 
    //making the value be within one rotation either way so we dont have to solve for that
    if (this.givenTarget >= constants.kg_TurretSubsystem.k_Motor1ReverseSoftLimitThreshold 
        && this.givenTarget <= constants.kg_TurretSubsystem.k_Motor1ForwardSoftLimitThreshold) {
      //do nothing
    } else{
      for (int i = -4; i < 5; i++) {
        this.potentialTarget =  this.givenTarget + (modifier * i);
        if (this.potentialTarget >= constants.kg_TurretSubsystem.k_Motor1ReverseSoftLimitThreshold 
            && this.potentialTarget <= constants.kg_TurretSubsystem.k_Motor1ForwardSoftLimitThreshold) {
          //we have found a value within the range leave the loop
          break;
        }
      }

    }

    return potentialTarget;
  }

  //this literly just converts chassis speeds from robot relative to field relative could be done in line but it got made a 
  //method for later use
  //this is borrowed from polar pilots :)
  public ChassisSpeeds toFieldRelative(ChassisSpeeds robotRelativeSpeeds, Rotation2d robotAngle) {
    return ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, robotAngle);
  }

  //this takes in the position of the robot and throws out a goal position based upon where on the field the robot thinks it is
  //you have to take in account the alliance you are on since the origion is always from the blue alliance side
  private Pose2d calcTarget(Pose2d robotPose){
    Pose2d targetPose;

    //figuring out where we should be targeting based on where we are on the field
    if (isAllianceRed() == 1) {
        //red alliance
        if (robotPose.getX() > constants.kg_TargetsAndField.k_RedAllianceZone) {
            targetPose = constants.kg_TargetsAndField.k_HubPose2dRed;
        } else {
            if (robotPose.getY() > constants.kg_TargetsAndField.k_MiddleOfField) {
                //in left side of field shoot to left red target
                targetPose = constants.kg_TargetsAndField.k_PassRightPose2dRed;
            } else {
                targetPose = constants.kg_TargetsAndField.k_PassLeftPose2dRed;
            }
        }
    } else {
        //blue alliance
        if (robotPose.getX() < constants.kg_TargetsAndField.k_BlueAllianceZone) {
            targetPose = constants.kg_TargetsAndField.k_HubPose2dBlue;
        } else {
            if (robotPose.getY() > constants.kg_TargetsAndField.k_MiddleOfField) {
                //in right side of field shoot to right red target
                targetPose = constants.kg_TargetsAndField.k_PassLeftPose2dBlue;
            } else {
                targetPose = constants.kg_TargetsAndField.k_PassRightPose2dBlue;
            }
        }
    }

    return targetPose;
  }

  //this querys the driver station for what alliance we are on is used in the target calculation step
  private static int isAllianceRed() {
    var alliance = DriverStation.getAlliance().orElse(Alliance.Red);
    if (alliance == DriverStation.Alliance.Red) {
        return 1;
    } else {
        return 0;
    }
  }
}

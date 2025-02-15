package frc.robot.subsystems;

import java.security.Timestamp;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import com.pathplanner.lib.auto.NamedCommands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.ArmPosition;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.Notification.NotificationLevel;
import frc.robot.util.PoseUtils;

public class Auto extends SubsystemBase {
    // variables to set the correct height and angle TODO: not needed?
    // public double autoElevatorHeight = 0;
    // public double autoArmAngle = 0;

    private final double SCORING_TIMEOUT = 0.3;
    private final double INTAKE_TIMEOUT = 0.3;

    public int estimatedScore = 3; //3 because leave bonus!
    private String prevAutoString = "";
    private double prevProgressBar = 0;
    private ArrayList<PathPlannerPath> pathList = new ArrayList<PathPlannerPath>();
    private ArrayList<Pose2d> allPosesList = new ArrayList<Pose2d>();
    private ArrayList<PathPoint> allPathPoints = new ArrayList<PathPoint>();

    private Command autoCmd = Commands.none();

    private double timeSeconds = 0; //time the auto routine will take

    private boolean isUsingProgressBar;

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable auto = inst.getTable("Auto");

    StringTopic autoTopic = auto.getStringTopic("Auto String");
    StringEntry stringEnt = autoTopic.getEntry("");
    DoubleTopic progressBarTopic = auto.getDoubleTopic("Progress Bar");
    DoubleEntry progressBarEnt = progressBarTopic.getEntry(0);
    DoubleTopic timeStampTopic = auto.getDoubleTopic("Time (s)");
    DoublePublisher timeStampPub = timeStampTopic.publish();
    // DoublePublisher progressBarPub = progressBarTopic.publish();

    StringTopic feedbackTopic = auto.getStringTopic("Auto Feedback");
    StringPublisher feedbackPub = feedbackTopic.publish();
    StringSubscriber stringSub = autoTopic.subscribe("");
    Alliance prevAlliance = Alliance.Blue;

    public static Field2d field = new Field2d();

    public static int mapCoralLevels(int x) {
        if (x == 1) {
            return 3;
        } else if (x == 2) {
            return 4;
        } else if (x == 3) {
            return 6;
        } else if (x == 4) {
            return 7;
        } else {
            return 0;
        }
    }

    public void setFeedback(String s, NotificationLevel level) {
        feedbackPub.set(s);
        Notification not = new Notification(level, "Auto Feedback", s, 3000);
        Elastic.sendNotification(not);
    }

    public Auto() {

        autoTopic.setRetained(true); // Should be retained?
        stringEnt.set("");
        progressBarEnt.set(0);
        progressBarTopic.setRetained(true);
        // progressBarPub.set(0);
        feedbackTopic.setRetained(true);
        feedbackPub.set("Enter auto string!");
        timeStampTopic.setRetained(true);
        timeStampPub.set(0);
    }

    public int calculateEstimatedScore(int estimatedScore) {
        estimatedScore = 3;

        for (int i = 2; i < prevAutoString.length(); i += 3) {
            if (i + 1 >= prevAutoString.length()) {
                setFeedback("Error: Auto string is too short for a scoring level at index " + i,
                        NotificationLevel.ERROR);
                return estimatedScore;
            }

            String coralLevelStr = prevAutoString.substring(i + 1, i + 2);

            if (coralLevels.contains(coralLevelStr)) {
                try {
                    int coralLevel = mapCoralLevels(Integer.parseInt(coralLevelStr));
                    estimatedScore += coralLevel;
                } catch (NumberFormatException e) {
                    setFeedback("Error: Invalid coral level '" + coralLevelStr + "' at index " + (i + 1),
                            NotificationLevel.ERROR);
                }
            }
        }
        return estimatedScore;
    }

    /* ArrayLists to hold the values of various waypoints */
    private ArrayList<String> startingLocations = new ArrayList<String>(
            Arrays.asList("S1", "S2", "S3", "S4", "S5"));
    private ArrayList<String> scoringLocations = new ArrayList<String>(
            Arrays.asList("A", "a", "B", "b", "C", "c", "D", "d", "E", "e", "F", "f"));
    private ArrayList<String> pickupLocations = new ArrayList<String>(
            Arrays.asList("1", "2", "3", "4", "5", "6"));
    private ArrayList<String> coralLevels = new ArrayList<String>(
            Arrays.asList("1", "2", "3", "4"));

    public boolean isAutoStringValid(String autoString) {

        if (autoString.length() < 2) {
            setFeedback("Not long enough!", NotificationLevel.ERROR);
            return false;
        }

        /* Checks if there is a valid starting location */
        if (!startingLocations.contains(autoString.substring(0, 2))) {
            setFeedback("Invalid starting location", NotificationLevel.ERROR);
            return false;
        }

        /* Starts at 2 because you don't need to loop through the starting location */
        for (int i = 2; i < autoString.length(); i += 3) { // Loops every 3

            /* Creates strings to store each part of the 3 character interval */
            String first = autoString.substring(i, i + 1);
            String second = i < autoString.length() - 1 ? autoString.substring(i + 1, i + 2) : null;
            String third = i < autoString.length() - 2 ? autoString.substring(i + 2, i + 3) : null;

            /* Checks if we are going to a valid scoring location */
            if (!scoringLocations.contains(first)) {
                setFeedback(
                        "Character " + (i + 1) + ": " + autoString.charAt(i)
                                + " is not a scoring location, when it should be!",
                        NotificationLevel.ERROR);
                return false;
            }
            /* Checks if the coral level we want to score at is valid */
            if (second == null || !coralLevels.contains(second)) {
                setFeedback("Invalid or missing Coral level at " + (i + 2),
                        NotificationLevel.ERROR);
                return false;
            }
            /* Checks if the pickup location we want to score at is valid */
            if (third != null && !pickupLocations.contains(third)) {
                setFeedback(
                        "Character " + (i + 3) + ": " + autoString.charAt(i + 2)
                                + " is not a valid pickup location, when it should be!",
                        NotificationLevel.ERROR);
                return false;
            }

        }

        setFeedback("Auto String Valid!", NotificationLevel.INFO);
        return true;

    }

    public void drawPaths() { // TODO rotate for red alliance?

        for (int i = 0; i < pathList.size(); i++) {
            PathPlannerPath path = pathList.get(i);
            Pose2d[] posList = path.getPathPoses().toArray(new Pose2d[0]);
            List<PathPoint> points = path.getAllPathPoints();

            for (int j = 0; j < posList.length; j++) {
                posList[j] = PoseUtils.flipPoseAlliance(posList[j]);
                allPosesList.add(posList[j]); // adds the pose to an ArrayList that stores all of the auto poses
            }
            // Trajectory traj = TrajectoryGenerator.generateTrajectory(path.getPathPoses(),
            // new TrajectoryConfig(Drivetrain.MAX_SPEED, Drivetrain.FORWARD_ACCEL));
            field.getObject("obj" + i).setPoses(posList);
            // field.getObject("obj" + i).setTrajectory(traj);
            for (PathPoint point : points) {
                allPathPoints.add(point);
            }
        }
    }

    public Command buildAutoCommand(String autoString) {

        timeSeconds = 0;

        if (!isAutoStringValid(autoString)) {
            return Commands.none();
            // Possibly make this return just a simple path so we get the leave bonus
        }

        SequentialCommandGroup autoCommand = new SequentialCommandGroup();

        // S1 B3 1
        // TODO: write the rest of this
        for (int i = 1; i < autoString.length(); i += 3) {

            /* Creates strings to store each part of the 3 character interval */
            String first = autoString.substring(i, i + 1); // pickup location
            String second = i < autoString.length() - 1 ? autoString.substring(i + 1, i + 2) : null; // scoring location
            String third = i < autoString.length() - 2 ? autoString.substring(i + 2, i + 3) : null; // coral level
            String fourth = i < autoString.length() - 3 ? autoString.substring(i + 3, i + 4) : null; // next pickup
                                                                                                     // location

            //pure driving commands to be added
            Command path1Cmd = Commands.none(); //path towards scoring location
            Command path2Cmd = Commands.none(); //path towards intake location

            // try {
            //     if (autoElevatorHeight == 0 || autoArmAngle == 0) {
            //         throw new IllegalStateException("Something is null");
            //     }
            //     autoElevatorHeight = ArmPosition.fromString(third).getHeight();
            //     // autoArmAngle = ArmPosition.fromString(third).getAngle(); // TODO: SET
            //     // THIS!!!!
            // } catch (Exception e) {

            // }

            String pathName; //String name so that you can access the path file
            String middleChar = "-"; //Stores the character connecting the two waypoints
            boolean firstPath = false; //If its the first path in auto
            PathPlannerPath path1 = null;


            /* ------------------------------- FIRST PATH ------------------------------------- */


            boolean path1Exists = first != null && second != null; //Whether or not we have a valid path

            if (path1Exists) {
                //If either character is lowercase, the character connecting them will be a "_"
                if (Character.isLowerCase(first.charAt(0)) || Character.isLowerCase(second.charAt(0))) {
                    middleChar = "_";
                }
                //Constructing the path string 
                pathName = first + middleChar + second;
                //Adds an S to denote a starting location, if we are at the first path
                if (i == 1) {
                    pathName = "S" + pathName;
                    firstPath = true;
                }

                //creates the path name!
                pathName = i == 1 ? "S" + first + middleChar + second : first + middleChar + second; 


                try {
                    // Load the path you want to follow using its name in the GUI
                    path1 = PathPlannerPath.fromPathFile(pathName);

                    // Store path to be drawn on dashboard
                    // Create a path following command using AutoBuilder. This will also trigger
                    // event markers.
                    path1Cmd = Commands.parallel(Commands.print("Running path 1"), AutoBuilder.followPath(path1));

                    //resets pose to the starting pose if we are at the first path!
                    if (firstPath) {
                        Optional<Pose2d> opPose = path1.getStartingHolonomicPose();
                        Pose2d pose = opPose.isPresent() ? PoseUtils.flipPoseAlliance(opPose.get()) : new Pose2d();
                        autoCommand.addCommands(Commands.runOnce(() -> {
                            System.out.println("resetting auto pose");
                            RobotContainer.drivetrain.resetPose(pose);
                        }));

                        // TODO this can be deleted, is here for testing purposes
                        RobotContainer.drivetrain.resetPose(pose);
                    }

                    // TODO needs to be .generateTrajectory()? maybe only if the ideal one doesn't
                    // exist?
                    pathList.add(path1);

                } catch (Exception e) {
                    DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
                    setFeedback("Path " + pathName + " not found!", NotificationLevel.ERROR);
                    pathList.clear();
                    allPosesList.clear();
                    return Commands.none();
                }
            }

            /* -------------------- SECOND PATH ----------------------------  */

            boolean path2Exists = second != null && fourth != null;
            PathPlannerPath path2 = null;
            if (path2Exists) {
                middleChar = "-";
                if (Character.isLowerCase(third.charAt(0)) || Character.isLowerCase(fourth.charAt(0))) {
                    middleChar = "_";
                }
                pathName = second + middleChar + fourth;
                try {
                    // Load the 2nd path you want to follow using its name in the GUI

                    path2 = PathPlannerPath.fromPathFile(pathName);

                    // Store path to be drawn on dashboard
                    // Create a path following command using AutoBuilder. This will also trigger
                    // event markers.
                    path2Cmd = Commands.parallel(AutoBuilder.followPath(path2), Commands.print("Running path 2"));

                    // TODO needs to be .generateTrajectory()? maybe only if the ideal one doesn't
                    // exist?
                    pathList.add(path2);
                } catch (Exception e) {
                    DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
                    setFeedback("Path " + pathName + " not found!", NotificationLevel.ERROR);
                    return Commands.none();
                }
            }

            //Default armPos is none (which is 0)
            ArmPosition armPos = ArmPosition.DrivingNone;
            //if we have a scoring location, set the arm position to that scoring location!
            if (third != null) {
                armPos = ArmPosition.fromString(third); 
            }

            /* adds command form scoring location to next pickup location */
            if (third != null && path1 != null) {
                Optional<PathPlannerTrajectory> opTraj;
                try {
                    opTraj = path1.getIdealTrajectory(RobotConfig.fromGUISettings());
                    if (opTraj.isPresent()) {
                        double pathTime = opTraj.get().getTotalTimeSeconds(); //time path will take
                        double raiseTime = RobotContainer.elevator.getRaiseTime(armPos); //time elevator will take to rise
                        double waitTime = pathTime - raiseTime; //how much time we should wait before raising elevator
                        timeSeconds += pathTime;
                        System.out.println("Path 1 Arm height: " + armPos.getHeight());

                        System.out.println("Path 1 Raise Time: " + raiseTime);
                        //runs the path command along with a command that waits to raise the elevator
                        autoCommand.addCommands(Commands.parallel(
                            path1Cmd,
                                Commands.sequence(
                                    Commands.waitSeconds(waitTime),
                                    Commands.parallel(
                                        Commands.print("Starting elevator command path 1"),
                                        RobotContainer.elevator.setScoringHeightCommand(armPos).withTimeout(raiseTime),
                                        RobotContainer.arm.goToAngleCommand(armPos.getAngle())
                                    ),
                                    Commands.print("Finished elevator command path 1")
                                )    
                        ));

                    } else {
                        System.out.println("Ideal Trajectory failed to load, skipping path");
                    }
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }

            //Command to shoot!!!
            
            autoCommand.addCommands(RobotContainer.rollers.outtakeCoralCommand().withTimeout(SCORING_TIMEOUT)); // add command to
            timeSeconds += SCORING_TIMEOUT;
            
            //TODO: not needed
            // // Bring elevator and arm to driving position after scoring
            // autoCommand.addCommands(Commands.parallel(
            //         Commands.print("Setting elevator to default position after scoring"),
            //         RobotContainer.elevator.setScoringHeightCommand(ArmPosition.DrivingNone).withTimeout(1)
            //         //RobotContainer.arm.goToLocationCommand(ArmPosition.DrivingNone)
            //         ));


            //Path 2 command group

            if (path2 != null) {
                Optional<PathPlannerTrajectory> opTraj;
                try {
                    opTraj = path2.getIdealTrajectory(RobotConfig.fromGUISettings());
                    if (opTraj.isPresent()) {
                        double pathTime = opTraj.get().getTotalTimeSeconds(); 
                        timeSeconds += pathTime;
                        // double waitTime = pathTime - Elevator.ELEVATOR_RAISE_TIME; //TODO: not needed, we want to immediately set elevator height
                        armPos = ArmPosition.Intake; //SETS ARM POSITION
                        System.out.println("Path 2 Arm height: " + armPos.getHeight());
                        double raiseTime = RobotContainer.elevator.getRaiseTime(armPos); //time needed to raise the elevator, for timeout
                        System.out.println("Path 2 Raise Time: " + raiseTime);
                        autoCommand.addCommands(Commands.parallel(
                            path2Cmd,
                            Commands.sequence(
                                //Commands.waitSeconds(waitTime),
                                Commands.parallel(
                                    Commands.print("Running elevator Command path 2"),
                                    RobotContainer.elevator.setScoringHeightCommand(armPos).withTimeout(raiseTime),
                                    //.withTimeout(raiseTime) TODO: timeout needed?
                                    RobotContainer.arm.goToLocationCommand(ArmPosition.Intake)
                                ),
                                Commands.print("Finishing path 2 elevator command")
                            )
                        ));

                    } else {
                        System.out.println("Ideal Trajectory failed to load, skipping path");
                    }
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }

            /* ADDS AN INTAKING COMMAND */
            autoCommand.addCommands(RobotContainer.rollers.intakeCoralCommand().withTimeout(INTAKE_TIMEOUT));
            timeSeconds += INTAKE_TIMEOUT;

            //TODO: not needed
            // Set elevator and arm to driving position after intaking
            // autoCommand.addCommands(Commands.parallel(
            //         Commands.print("Setting elevator to default position after intaking"),
            //         RobotContainer.elevator.setScoringHeightCommand(ArmPosition.DrivingNone).withTimeout(RobotContainer.elevator.getRaiseTime(ArmPosition.DrivingNone))
            //         //RobotContainer.arm.goToLocationCommand(ArmPosition.DrivingNone)
            //         ));

        }

        setFeedback("Auto built successfully!", NotificationLevel.INFO);
        return autoCommand;
    }

    public void validateAndCreatePaths(String str) {
        clearField();
        if (isAutoStringValid(str)) {
            autoCmd = buildAutoCommand(str);
            drawPaths();
        }
    }

    public void clearField() {
        pathList.clear();
        allPosesList.clear();
        for (int i = 0; i < 100; i++) {
            FieldObject2d obj = field.getObject("obj" + i);
            obj.setTrajectory(new Trajectory());
        }
    }

    /**
     * Moves the robot pose in the auto dashboard so that it moves along the inputed
     * auto based on an input from
     */
    public void addRobotPoseProgressBar() {
        isUsingProgressBar = true;
        int index = (int) (
        // multiplies progress from 0 to 1 by the number of poses in our auto paths
        // altogether to get an index
        MathUtil.clamp(progressBarEnt.getAsDouble(), 0, 1) * (allPosesList.size() - 1)); // casts as int to get nearest
                                                                                         // pose

        // int index = (int) (progressBarEntry.getAsDouble() *
        // (allPathPoints.size()-1));
        // sets the robotPose on the field to that pose
        if (index >= 0 && allPosesList.size() > 0) {
            Pose2d pose = allPosesList.get(index);
            field.setRobotPose(pose);
            // poseOnField = pose; //TODO check if I can delete this
            // PathPoint point = allPathPoints.get(index);
            // field.setRobotPose(point.position.getX(), point.position.getY(),
            // point.rotationTarget.rotation());
        }
    }

    public void displayTimestampSeconds() {
        double time = timeSeconds;
        double rounded = Math.round(time * 10.0) / 10.0;
        timeStampPub.set(rounded);
    }

    public Command getAutoCommand() {
        return autoCmd;
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            String str = stringEnt.get("");
            SmartDashboard.putNumber("Score", calculateEstimatedScore(estimatedScore));
            SmartDashboard.putData(field);

            // String autoString = str.replace(' ', Character.MIN_VALUE); // check
            String autoString = "";
            Optional<Alliance> alliance = DriverStation.getAlliance();
            for (char a : str.toCharArray()) {
                if (a != ' ' && a != ',') {
                    autoString += a;
                }
            }
            if (!autoString.equals(prevAutoString)) {
                System.out.println("Running auto sequencer & Command Builder");
                prevAutoString = autoString;
                validateAndCreatePaths(autoString);
                addRobotPoseProgressBar();
                displayTimestampSeconds();
            } else if (alliance.isPresent() && alliance.get() != prevAlliance) {
                System.out.println(" 2 -> Running auto sequencer & Command Builder");
                prevAlliance = alliance.get();
                validateAndCreatePaths(autoString);
                addRobotPoseProgressBar();
                displayTimestampSeconds();
            }

            if (progressBarEnt.getAsDouble() != prevProgressBar) {
                prevProgressBar = progressBarEnt.getAsDouble();
                addRobotPoseProgressBar();
                displayTimestampSeconds();
            }

        }

        if ((DriverStation.isAutonomous() || DriverStation.isDisabled()) && !isUsingProgressBar) {
            // System.out.println("setting the robot pose auto periodic");
            field.setRobotPose(RobotContainer.drivetrain.getRobotPose());
        }
    }
}

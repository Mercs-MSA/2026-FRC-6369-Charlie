package frc.robot.math;

import java.io.IOException;
import java.util.Arrays;
import java.util.NavigableMap;
import java.util.TreeMap;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.turret.TurretConstants;

public class ShooterMathProvider {
    @AutoLogOutput
    public double shooterVelocityTarget;
    @AutoLogOutput
    public double shooterHoodAngle;
    @AutoLogOutput
    public double shooterTurretDelta;
    @AutoLogOutput
    public boolean hoodStow;
    @AutoLogOutput
    public double dist;
    @AutoLogOutput
    public double runTime;
    
    // position of hub opening on blue side
    public Translation2d targetPositionBlueSide = new Translation2d(4.625, 4.034);   // TODO: verify accuracy of position
    public Translation2d hubPositionBlueSide = new Translation2d(4.625, 4.034);   // TODO: verify accuracy of position
    // stow
    public static final Rectangle2d[] stowEnablePositions = new Rectangle2d[]{
        new Rectangle2d(new Translation2d(3.986, 8.147), new Translation2d(5.5, 6.849)), 
        new Rectangle2d(new Translation2d(3.986, 1.282), new Translation2d(5.5, 0)),
        new Rectangle2d(new Translation2d(16.54-3.986, 8.147), new Translation2d(16.54-5.5, 6.849)), 
        new Rectangle2d(new Translation2d(16.54-3.986, 1.282), new Translation2d(16.54-5.5, 0))
    };
    public static final Rectangle2d[] stowDisablePositions = new Rectangle2d[]{
        new Rectangle2d(new Translation2d(3.886, 8.147), new Translation2d(5.7, 6.849)), 
        new Rectangle2d(new Translation2d(3.886, 1.282), new Translation2d(5.7, 0)),
        new Rectangle2d(new Translation2d(16.54-3.886, 8.147), new Translation2d(16.54-5.7, 6.849)), 
        new Rectangle2d(new Translation2d(16.54-3.886, 1.282), new Translation2d(16.54-5.7, 0))
    };

    private final NavigableMap<Double, Double[]> shotMapRPS = new TreeMap<>();
    private final NavigableMap<Double, Double> TOFMap = new TreeMap<>();
    


    public ShooterMathProvider() {
        shotMapRPS.put(1.83, new Double[]{45.0, 0.00});
        shotMapRPS.put(3.09, new Double[]{50.0, 0.018});
        shotMapRPS.put(3.64, new Double[]{51.0, 0.028});
        shotMapRPS.put(5.32, new Double[]{59.0, 0.048});
        TOFMap.put(0.0, 0.0);
        TOFMap.put(3.3, -1.0);
        TOFMap.put(5.0, -1.2);
        
    }

    /**
     * Upper/lower bound search for a value in an array
     * @param value Input value
     * @param a Array to search
     * @return Two index positions. First is the upper bound index, second the lower bound index. May return two identical values if the exact input is in the array.
     */
    public static int[] searchInput(double value, double[] a) {
        // Modified from https://stackoverflow.com/a/30245398
        // Posted by David Soroko, modified by community. See post 'Timeline' for change history
        // Retrieved 2026-01-22, License - CC BY-SA 4.0

        if (value <= a[0]) { return new int[]{0, 0}; } // value is first in array? - return the first position
        if (value >= a[a.length - 1]) { return new int[]{a.length - 1, a.length - 1}; } // value is last in array? - return the last position

        int result = Arrays.binarySearch(a, value); // search the array for the value, returns (-(insertion point) - 1) is value not found
        if (result >= 0) { return new int[]{result, result}; } // If value is found, return exact index

        int insertionPoint = -result - 1; // get the insertion point
        return new int[]{insertionPoint, insertionPoint - 1}; // upper/lower
    }

    /**
     * Linear interpolation
     * @param x Input value
     * @param x1 Lower bound of x
     * @param x2 Upper bound of x
     * @param q00 Value at x1
     * @param q01 Value at x2
     * @return Interpolated value at x
     */
    private static double lerp(double x, double x1, double x2, double q00, double q01) {
        if (x1 == x2) {
            return q00;
        }
        if (x == x1) {
            return q00;
        }
        if (x == x2) {
            return q01;
        }
        return ((x2 - x) / (x2 - x1)) * q00 + ((x - x1) / (x2 - x1)) * q01;
    }

    public void update(Pose2d robotPose, ChassisSpeeds velocities, Pose2d turretPose) throws IOException {
        // start runtime stat
        var ta = Utils.getCurrentTimeSeconds();

        // Distance to target
        Translation2d target = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? FlippingUtil.flipFieldPose(new Pose2d(targetPositionBlueSide, new Rotation2d())).getTranslation() : targetPositionBlueSide;
        dist = Math.sqrt(Math.pow(turretPose.getX() - target.getX(), 2) + Math.pow(turretPose.getY() - target.getY(), 2));

        // Search for upper/lower bound indices
        

        shooterTurretDelta = 0.0;
        // Safely get lower/upper map entries with fallbacks to first/last entries when out-of-range
        var lowerEntryShotmap = shotMapRPS.floorEntry(dist);
        if (lowerEntryShotmap == null) {
            lowerEntryShotmap = shotMapRPS.firstEntry();
        }
        var upperEntryShotmap = shotMapRPS.ceilingEntry(dist);
        if (upperEntryShotmap == null) {
            upperEntryShotmap = shotMapRPS.lastEntry();
        }

        double lowerKeyShotmap = lowerEntryShotmap.getKey();
        double upperKeyShotmap = upperEntryShotmap.getKey();
        Double[] lowerValShotmap = lowerEntryShotmap.getValue();
        Double[] upperValShotmap = upperEntryShotmap.getValue();

        shooterVelocityTarget = lerp(dist, lowerKeyShotmap, upperKeyShotmap, lowerValShotmap[0], upperValShotmap[0]);
        shooterHoodAngle = lerp(dist, lowerKeyShotmap, upperKeyShotmap, lowerValShotmap[1], upperValShotmap[1]);

    ChassisSpeeds fieldVel = ChassisSpeeds.fromRobotRelativeSpeeds(velocities, robotPose.getRotation());

    double turretVelocityX =
        fieldVel.vxMetersPerSecond
            + fieldVel.omegaRadiansPerSecond
                * (TurretConstants.kTurretOffsetY * Math.cos(robotPose.getRotation().getRadians())
                    - TurretConstants.kTurretOffsetX * Math.sin(robotPose.getRotation().getRadians()));
    double turretVelocityY =
        fieldVel.vyMetersPerSecond
            + fieldVel.omegaRadiansPerSecond
                * (TurretConstants.kTurretOffsetX * Math.cos(robotPose.getRotation().getRadians())
                    - TurretConstants.kTurretOffsetY * Math.sin(robotPose.getRotation().getRadians()));

        var lowerEntryTof = TOFMap.floorEntry(dist);
        if (lowerEntryTof == null) {
            lowerEntryTof = TOFMap.firstEntry();
        }
        var upperEntryTof = TOFMap.ceilingEntry(dist);
        if (upperEntryTof == null) {
            upperEntryTof = TOFMap.lastEntry();
        }

        double lowerKeyTof = lowerEntryTof.getKey();
        double upperKeyTof = upperEntryTof.getKey();
        double lowerValTof = lowerEntryTof.getValue();
        double upperValTof = upperEntryTof.getValue();

        var timeOfFlight = lerp(dist, lowerKeyTof, upperKeyTof, lowerValTof, upperValTof);

      double offsetX = turretVelocityX * timeOfFlight;
      double offsetY = turretVelocityY * timeOfFlight;
      System.out.println(offsetY);
      targetPositionBlueSide =
          hubPositionBlueSide.plus(new Translation2d(offsetX, offsetY));
    }
}

package frc.robot;

import static frc.robot.Constants.Drivetrain.TROUGH_OFFSET_METERS;

import java.util.HashMap;
import java.util.Map;

import com.techhounds.houndutil.houndauto.Reflector;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class FieldConstants {
    public static final double fieldLength = Units.inchesToMeters(690.876);
    public static final double fieldWidth = Units.inchesToMeters(317);
    // Measured from the inside of starting line
    public static final double startingLineX = Units.inchesToMeters(299.438);
    public static final double algaeDiameter = Units.inchesToMeters(16);

    public class Reef {
        public static final Translation2d center = new Translation2d(Units.inchesToMeters(176.746),
                Units.inchesToMeters(158.501));
        // Side of the reef to the inside of the reef zone line
        public static final double faceToZoneLine = Units.inchesToMeters(12);

        public enum ReefLevel {
            L4(Units.inchesToMeters(72), -90),
            L3(Units.inchesToMeters(47.625), -35),
            L2(Units.inchesToMeters(31.875), -35),
            L1(Units.inchesToMeters(20), 0);

            ReefLevel(double height, double pitch) {
                this.height = height;
                this.pitch = pitch; // in degrees
            }

            public final double height;
            public final double pitch;
        }

        public enum ReefSide {
            ONE(new Pose2d(
                    Units.inchesToMeters(144.003),
                    Units.inchesToMeters(158.500),
                    Rotation2d.fromDegrees(180))),

            TWO(new Pose2d(
                    Units.inchesToMeters(160.375),
                    Units.inchesToMeters(130.144),
                    Rotation2d.fromDegrees(-120))),

            THREE(new Pose2d(
                    Units.inchesToMeters(193.118),
                    Units.inchesToMeters(130.145),
                    Rotation2d.fromDegrees(-60))),
            FOUR(new Pose2d(
                    Units.inchesToMeters(209.489),
                    Units.inchesToMeters(158.502),
                    Rotation2d.fromDegrees(0))),
            FIVE(new Pose2d(
                    Units.inchesToMeters(193.116),
                    Units.inchesToMeters(186.858),
                    Rotation2d.fromDegrees(60))),
            SIX(new Pose2d(
                    Units.inchesToMeters(160.373),
                    Units.inchesToMeters(186.857),
                    Rotation2d.fromDegrees(120)));

            public final Pose2d centerFace;
            public final Pose2d algaeDescorePose;

            private ReefSide(Pose2d centerFace) {
                this.centerFace = centerFace;
                this.algaeDescorePose = centerFace.transformBy(new Transform2d(
                        Constants.Drivetrain.ALGAE_DESCORE_DISTANCE, 0, Rotation2d.kPi));
            }

            public static Pose2d[] facePoses() {
                return new Pose2d[] {
                        ONE.centerFace, TWO.centerFace, THREE.centerFace,
                        FOUR.centerFace, FIVE.centerFace, SIX.centerFace };
            }
        }

        public enum BranchSide {
            LEFT,
            RIGHT;
        }

        public enum ReefBranch {
            A(ReefSide.ONE, BranchSide.LEFT),
            B(ReefSide.ONE, BranchSide.RIGHT),
            C(ReefSide.TWO, BranchSide.LEFT),
            D(ReefSide.TWO, BranchSide.RIGHT),
            E(ReefSide.THREE, BranchSide.LEFT),
            F(ReefSide.THREE, BranchSide.RIGHT),
            G(ReefSide.FOUR, BranchSide.LEFT),
            H(ReefSide.FOUR, BranchSide.RIGHT),
            I(ReefSide.FIVE, BranchSide.LEFT),
            J(ReefSide.FIVE, BranchSide.RIGHT),
            K(ReefSide.SIX, BranchSide.LEFT),
            L(ReefSide.SIX, BranchSide.RIGHT);

            public final ReefSide reefSide;
            public final BranchSide branchSide;
            public final Map<ReefLevel, Pose3d> positions;
            public final Pose2d chassisPose;
            public final Pose2d troughPose;

            private ReefBranch(ReefSide reefSide, BranchSide branchSide) {
                this.reefSide = reefSide;
                this.branchSide = branchSide;
                this.positions = new HashMap<>();

                Pose2d poseDirection = new Pose2d(center, reefSide.centerFace.getRotation());
                double adjustX = Units.inchesToMeters(30.738);
                double adjustY = Units.inchesToMeters(6.469); // 0.1643126 m
                for (ReefLevel level : ReefLevel.values()) {

                    positions.put(
                            level,
                            new Pose3d(
                                    new Translation3d(
                                            poseDirection
                                                    .transformBy(new Transform2d(
                                                            adjustX,
                                                            branchSide == BranchSide.RIGHT
                                                                    ? adjustY
                                                                    : -adjustY,
                                                            Rotation2d.kZero))
                                                    .getX(),
                                            poseDirection
                                                    .transformBy(new Transform2d(
                                                            adjustX,
                                                            branchSide == BranchSide.RIGHT
                                                                    ? adjustY
                                                                    : -adjustY,
                                                            Rotation2d.kZero))
                                                    .getY(),
                                            level.height),
                                    new Rotation3d(
                                            0,
                                            Units.degreesToRadians(
                                                    level.pitch),
                                            poseDirection.getRotation()
                                                    .getRadians() +
                                                    (level.equals(ReefLevel.L1)
                                                            ? Math.PI / 2.0
                                                            : 0))));
                }
                chassisPose = new Pose2d(
                        new Translation2d(
                                poseDirection
                                        .transformBy(new Transform2d(adjustX,
                                                branchSide == BranchSide.RIGHT
                                                        ? adjustY
                                                        : -adjustY,
                                                Rotation2d.kZero))
                                        .getX(),
                                poseDirection
                                        .transformBy(new Transform2d(adjustX,
                                                branchSide == BranchSide.RIGHT
                                                        ? adjustY
                                                        : -adjustY,
                                                Rotation2d.kZero))
                                        .getY()),
                        reefSide.centerFace.getRotation())
                        .transformBy(new Transform2d(
                                Constants.Drivetrain.SCORING_DISTANCE_METERS, 0,
                                Rotation2d.kPi));

                troughPose = chassisPose.transformBy(new Transform2d(0,
                        branchSide == BranchSide.RIGHT ? -TROUGH_OFFSET_METERS
                                : TROUGH_OFFSET_METERS,
                        Rotation2d.kZero));
            }

            public Pose3d node(ReefLevel level) {
                return positions.get(level);
            }

            public static Pose3d[] allPoses() {
                return new Pose3d[] {
                        A.node(ReefLevel.L1), A.node(ReefLevel.L2), A.node(ReefLevel.L3),
                        A.node(ReefLevel.L4),
                        B.node(ReefLevel.L1), B.node(ReefLevel.L2), B.node(ReefLevel.L3),
                        B.node(ReefLevel.L4),
                        C.node(ReefLevel.L1), C.node(ReefLevel.L2), C.node(ReefLevel.L3),
                        C.node(ReefLevel.L4),
                        D.node(ReefLevel.L1), D.node(ReefLevel.L2), D.node(ReefLevel.L3),
                        D.node(ReefLevel.L4),
                        E.node(ReefLevel.L1), E.node(ReefLevel.L2), E.node(ReefLevel.L3),
                        E.node(ReefLevel.L4),
                        F.node(ReefLevel.L1), F.node(ReefLevel.L2), F.node(ReefLevel.L3),
                        F.node(ReefLevel.L4),
                        G.node(ReefLevel.L1), G.node(ReefLevel.L2), G.node(ReefLevel.L3),
                        G.node(ReefLevel.L4),
                        H.node(ReefLevel.L1), H.node(ReefLevel.L2), H.node(ReefLevel.L3),
                        H.node(ReefLevel.L4),
                        I.node(ReefLevel.L1), I.node(ReefLevel.L2), I.node(ReefLevel.L3),
                        I.node(ReefLevel.L4),
                        J.node(ReefLevel.L1), J.node(ReefLevel.L2), J.node(ReefLevel.L3),
                        J.node(ReefLevel.L4),
                        K.node(ReefLevel.L1), K.node(ReefLevel.L2), K.node(ReefLevel.L3),
                        K.node(ReefLevel.L4),
                        L.node(ReefLevel.L1), L.node(ReefLevel.L2), L.node(ReefLevel.L3),
                        L.node(ReefLevel.L4)
                };
            }

            public static Pose2d[] chassisPoses() {
                return new Pose2d[] {
                        A.chassisPose,
                        Reflector.rotatePoseAcrossField(A.chassisPose, fieldLength, fieldWidth),
                        B.chassisPose,
                        Reflector.rotatePoseAcrossField(B.chassisPose, fieldLength, fieldWidth),
                        C.chassisPose,
                        Reflector.rotatePoseAcrossField(C.chassisPose, fieldLength, fieldWidth),
                        D.chassisPose,
                        Reflector.rotatePoseAcrossField(D.chassisPose, fieldLength, fieldWidth),
                        E.chassisPose,
                        Reflector.rotatePoseAcrossField(E.chassisPose, fieldLength, fieldWidth),
                        F.chassisPose,
                        Reflector.rotatePoseAcrossField(F.chassisPose, fieldLength, fieldWidth),
                        G.chassisPose,
                        Reflector.rotatePoseAcrossField(G.chassisPose, fieldLength, fieldWidth),
                        H.chassisPose,
                        Reflector.rotatePoseAcrossField(H.chassisPose, fieldLength, fieldWidth),
                        I.chassisPose,
                        Reflector.rotatePoseAcrossField(I.chassisPose, fieldLength, fieldWidth),
                        J.chassisPose,
                        Reflector.rotatePoseAcrossField(J.chassisPose, fieldLength, fieldWidth),
                        K.chassisPose,
                        Reflector.rotatePoseAcrossField(K.chassisPose, fieldLength, fieldWidth),
                        L.chassisPose,
                        Reflector.rotatePoseAcrossField(L.chassisPose, fieldLength, fieldWidth),
                };
            }

            public static Pose2d[] troughPoses() {
                return new Pose2d[] {
                        A.troughPose,
                        Reflector.rotatePoseAcrossField(A.troughPose, fieldLength, fieldWidth),
                        B.troughPose,
                        Reflector.rotatePoseAcrossField(B.troughPose, fieldLength, fieldWidth),
                        G.troughPose,
                        Reflector.rotatePoseAcrossField(G.troughPose, fieldLength, fieldWidth),
                        H.troughPose,
                        Reflector.rotatePoseAcrossField(H.troughPose, fieldLength, fieldWidth),
                };
            }

            public static ReefBranch[] allBranches() {
                return new ReefBranch[] {
                        ReefBranch.A, ReefBranch.B, ReefBranch.C, ReefBranch.D,
                        ReefBranch.E, ReefBranch.F, ReefBranch.G, ReefBranch.H,
                        ReefBranch.I, ReefBranch.J, ReefBranch.K, ReefBranch.L,
                };
            }

            public static ReefBranch[] troughBranches() {
                return new ReefBranch[] {
                        ReefBranch.A, ReefBranch.B, ReefBranch.C, ReefBranch.D,
                        ReefBranch.F, ReefBranch.I, ReefBranch.K, ReefBranch.L,
                };
            }

            public static ReefBranch getBranch(ReefSide reefSide, BranchSide branchSide) {
                for (ReefBranch branch : values()) {
                    if (branch.reefSide == reefSide && branch.branchSide == branchSide) {
                        return branch;
                    }
                }
                return null;
            }

        }
    }

    public static class Processor {
        public static final Pose2d centerFace = new Pose2d(Units.inchesToMeters(235.726), 0,
                Rotation2d.fromDegrees(90));
    }

    public static class Barge {
        public static final Translation2d farCage = new Translation2d(Units.inchesToMeters(345.428),
                Units.inchesToMeters(286.779));
        public static final Translation2d middleCage = new Translation2d(Units.inchesToMeters(345.428),
                Units.inchesToMeters(242.855));
        public static final Translation2d closeCage = new Translation2d(Units.inchesToMeters(345.428),
                Units.inchesToMeters(199.947));

        // Measured from floor to bottom of cage
        public static final double deepHeight = Units.inchesToMeters(3.125);
        public static final double shallowHeight = Units.inchesToMeters(30.125);
    }

    public static class CoralStation {
        public static final Pose2d leftCenterFace = new Pose2d(
                Units.inchesToMeters(33.526),
                Units.inchesToMeters(291.176),
                Rotation2d.fromDegrees(90 - 144.011));
        public static final Pose2d rightCenterFace = new Pose2d(
                Units.inchesToMeters(33.526),
                Units.inchesToMeters(25.824),
                Rotation2d.fromDegrees(144.011 - 90));
    }
}
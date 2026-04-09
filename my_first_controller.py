import mc_control
import mc_rbdyn
import mc_rtc
import mc_tasks
import eigen
import sva

class MyFirstController(mc_control.MCPythonController):
    def __init__(self, rm, dt):
        self.qpsolver.addConstraintSet(self.dynamicsConstraint)
        self.qpsolver.addConstraintSet(self.contactConstraint)
        self.qpsolver.addConstraintSet(self.selfCollisionConstraint)

        self.qpsolver.addTask(self.postureTask)
        self.postureTask.stiffness(5.0)

        self.addContact(self.robot().name(), b"ground", b"LeftFoot", b"AllGround")
        self.addContact(self.robot().name(), b"ground", b"RightFoot", b"AllGround")

        self.comTask = mc_tasks.CoMTask(self.robots(), 0, 5.0, 1000.0)
        self.qpsolver.addTask(self.comTask)

        # --- Left hand tasks ---
        self.leftPosTask = mc_tasks.PositionTask(b"l_wrist", self.robots(), 0, 5.0, 500.0)
        self.leftPosTask.selectActiveJoints(self.qpsolver, [
            b"L_SHOULDER_P", b"L_SHOULDER_R", b"L_SHOULDER_Y",
            b"L_ELBOW_P", b"L_ELBOW_Y"
        ])
        self.qpsolver.addTask(self.leftPosTask)

        self.leftOriTask = mc_tasks.OrientationTask(b"l_wrist", self.robots(), 0, 5.0, 500.0)
        self.leftOriTask.selectActiveJoints(self.qpsolver, [
            b"L_WRIST_R", b"L_WRIST_Y"
        ])
        self.qpsolver.addTask(self.leftOriTask)

        # --- Right hand tasks ---
        self.rightPosTask = mc_tasks.PositionTask(b"r_wrist", self.robots(), 0, 5.0, 500.0)
        self.rightPosTask.selectActiveJoints(self.qpsolver, [
            b"R_SHOULDER_P", b"R_SHOULDER_R", b"R_SHOULDER_Y",
            b"R_ELBOW_P", b"R_ELBOW_Y"
        ])
        self.qpsolver.addTask(self.rightPosTask)

        self.rightOriTask = mc_tasks.OrientationTask(b"r_wrist", self.robots(), 0, 5.0, 500.0)
        self.rightOriTask.selectActiveJoints(self.qpsolver, [
            b"R_WRIST_R", b"R_WRIST_Y"
        ])
        self.qpsolver.addTask(self.rightOriTask)

        # --- Targets ---
        quat = eigen.Quaterniond(0.0, 0.7, 0.0, 0.7)
        quat.normalize()
        self.targetRot = quat.toRotationMatrix()

        self.leftTargetPos = eigen.Vector3d(0.5, 0.25, 1.1)
        self.rightTargetPos = eigen.Vector3d(0.5, -0.25, 1.1)

        # --- State ---
        self.leftInitPos = None
        self.leftInitRot = None
        self.rightInitPos = None
        self.rightInitRot = None
        self.phase = 0
        self.canSwitch = False
        self.neckYIndex = self.robot().jointIndexByName(b"NECK_Y")

    def active_error(self):
        """Return the max position error of tasks active in the current phase."""
        if self.phase in [0, 1]:
            return self.leftPosTask.eval().norm()
        elif self.phase in [2, 3]:
            return self.rightPosTask.eval().norm()
        else:
            return max(self.leftPosTask.eval().norm(), self.rightPosTask.eval().norm())

    def run_callback(self):
        error = self.active_error()

        if not self.canSwitch:
            if error > 0.15:
                self.canSwitch = True
            return True

        if error < 0.05:
            self.canSwitch = False
            self.phase = (self.phase + 1) % 6
            self.apply_phase()

        return True

    def apply_phase(self):
        if self.phase == 0:
            # Left hand to target — look left
            self.leftPosTask.position(self.leftTargetPos)
            self.leftOriTask.orientation(self.targetRot)
            self.postureTask.target({b"NECK_Y": [0.5]})
            print("Phase 0 - Left hand to target")

        elif self.phase == 1:
            # Left hand returns — look forward
            self.leftPosTask.position(self.leftInitPos)
            self.leftOriTask.orientation(self.leftInitRot)
            self.postureTask.target({b"NECK_Y": [0.0]})
            print("Phase 1 - Left hand returning")

        elif self.phase == 2:
            # Right hand to target — look right
            self.rightPosTask.position(self.rightTargetPos)
            self.rightOriTask.orientation(self.targetRot)
            self.postureTask.target({b"NECK_Y": [-0.5]})
            print("Phase 2 - Right hand to target")

        elif self.phase == 3:
            # Right hand returns
            self.rightPosTask.position(self.rightInitPos)
            self.rightOriTask.orientation(self.rightInitRot)
            self.postureTask.target({b"NECK_Y": [0.0]})
            print("Phase 3 - Right hand returning")

        elif self.phase == 4:
            # Both hands to targets
            self.leftPosTask.position(self.leftTargetPos)
            self.leftOriTask.orientation(self.targetRot)
            self.rightPosTask.position(self.rightTargetPos)
            self.rightOriTask.orientation(self.targetRot)
            self.postureTask.target({b"NECK_Y": [0.0]})
            print("Phase 4 - Both hands to targets")

        elif self.phase == 5:
            # Both hands return
            self.leftPosTask.position(self.leftInitPos)
            self.leftOriTask.orientation(self.leftInitRot)
            self.rightPosTask.position(self.rightInitPos)
            self.rightOriTask.orientation(self.rightInitRot)
            print("Phase 5 - Both hands returning")

    def reset_callback(self, data):
        self.comTask.reset()
        self.leftPosTask.reset()
        self.leftOriTask.reset()
        self.rightPosTask.reset()
        self.rightOriTask.reset()

        self.leftInitPos = self.leftPosTask.position()
        self.leftInitRot = self.leftOriTask.orientation()
        self.rightInitPos = self.rightPosTask.position()
        self.rightInitRot = self.rightOriTask.orientation()

        self.phase = 0
        self.canSwitch = False
        self.apply_phase()

    @staticmethod
    def create(robot, dt):
        env = mc_rbdyn.get_robot_module("env", mc_rtc.MC_ENV_DESCRIPTION_PATH, "ground")
        return MyFirstController([robot, env], dt)
import time
import numpy as np
from numpy.linalg import norm, solve
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer
from pathlib import Path


# --------------------------------------------------
# Load model (fixed base manipulator)
# --------------------------------------------------

urdf_model_path = Path("manuel/manuel1.urdf")
mesh_dir = urdf_model_path.parent

model, collision_model, visual_model = pin.buildModelsFromUrdf(
    str(urdf_model_path), str(mesh_dir)
)

data = model.createData()

# --------------------------------------------------
# Launch Meshcat
# --------------------------------------------------

viz = MeshcatVisualizer(model, collision_model, visual_model)
viz.initViewer(open=True)
viz.loadViewerModel()

# --------------------------------------------------
# Initial configuration
# --------------------------------------------------

q = pin.neutral(model)
viz.display(q)

print("\nRobot ready.")
print("Number of joints:", model.nq)
print("Joint names:", model.names[1:])
print("Use python3 -i robot_console.py for interactive control.")


# ==================================================
# Smooth motion utility
# ==================================================

def move_smooth(q_target, duration=2.0, steps=200):
    global q

    q_target = np.asarray(q_target, dtype=float)
    q_start = q.copy()

    for i in range(steps + 1):
        alpha = i / steps
        q_interp = pin.interpolate(model, q_start, q_target, alpha)
        viz.display(q_interp)
        time.sleep(duration / steps)

    q[:] = q_target


# ==================================================
# Forward Kinematics (auto-display + smooth motion)
# ==================================================

def fk(q_input=None, frame_name="gripper_link", smooth=True):
    global q

    if q_input is not None:
        q_target = np.asarray(q_input, dtype=float)

        if smooth:
            move_smooth(q_target)
        else:
            q[:] = q_target
            viz.display(q)

    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)

    frame_id = model.getFrameId(frame_name)
    oMf = data.oMf[frame_id]

    print("\n--- Forward Kinematics ---")
    print("Frame:", frame_name)
    print("Position:", oMf.translation)
    print("Rotation:\n", oMf.rotation)

    return oMf


# ==================================================
# CLIK Inverse Kinematics (smooth convergence)
# ==================================================

def ik_clik(frame_name,
            target_xyz,
            target_R=None,
            eps=1e-4,
            IT_MAX=1000,
            DT=1e-1,
            damp=1e-6,
            duration=2.0):

    global q

    frame_id = model.getFrameId(frame_name)

    if target_R is None:
        target_R = np.eye(3)

    oMdes = pin.SE3(target_R, np.asarray(target_xyz, dtype=float))

    q_current = q.copy()

    i = 0
    success = False

    while True:
        pin.forwardKinematics(model, data, q_current)
        pin.updateFramePlacements(model, data)

        oMf = data.oMf[frame_id]
        dMf = oMdes.actInv(oMf)
        err = pin.log(dMf).vector

        if norm(err) < eps:
            success = True
            break

        if i >= IT_MAX:
            break

        J = pin.computeFrameJacobian(
            model, data, q_current, frame_id,
            pin.ReferenceFrame.LOCAL
        )

        v = - J.T @ solve(J @ J.T + damp * np.eye(6), err)

        q_current = pin.integrate(model, q_current, v * DT)

        i += 1

    if success:
        print("\nIK Converged.")
    else:
        print("\nWarning: IK did not fully converge.")

    # Smoothly move from current q to solution
    move_smooth(q_current, duration=duration)

    return q.copy()

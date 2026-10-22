# Gangelia Fault Injector

Opens a parallel gripper during a running Isaac Sim 5.1 simulation. The open command goes to the joints. Regrasp also commands the Panda arm, so a planner that is writing those joints will fight it until the lift finishes. **Camera Noise** exposes one existing Camera prim as a short, noisy machine-vision frame. That camera's viewport and a recording of it show the exposure. Other cameras stay clean.

Version 0.2.0 adds the Panda regrasp, the extension icon, and a generated copy of the pickup rule so the extension can enable without the rest of the Gangelia repository.

## How to install

### Community registry

1. Open Isaac Sim and choose **Window → Extensions**.
2. Open the hamburger menu, then **Extension Registries**. If `kit/community` is missing, add name `kit/community` and URL `https://dw290v42wisod.cloudfront.net/exts/kit/community`.
3. Refresh the registries. Under **Third Party**, open **Community - Unverified**, and select **Gangelia Fault Injector**.
4. Install this version, then enable it. If 0.1.0 is already installed, refresh and update. Isaac Sim keeps the installed version until you do that.

Remove any Extension Search Path that points at a local copy of this extension before you install from Community. Isaac Sim uses that local folder instead of the registry package.

### Local checkout

In the Gangelia repository this folder is `isaac/gangelia.fault_injector`. Point the extension search path at the `isaac` folder, then enable **Gangelia Fault Injector**. That search path does not set `GANGELIA_FAULT_INJECTOR_EDITION`, so Camera Noise stays on Demo until the launcher or the environment sets it to `pro`. Edit the fault list and the pickup rule in the repository-root `shared_faults/` package. A Community package contains generated copies of `shared_faults/__init__.py`, `catalog.py`, and `drop_rule.py`. It does not contain `camera_noise.py`. Do not edit those copies. The Pro package is a separate assemble and is not uploaded to NVIDIA.

The **Gangelia Fault Injector** panel opens as its own window. This checkout is a user extension search path, so the Extensions window shows a remove button on that row. Remove it, then enable **Show Community Extensions** under Options, to see whether the registry copy is published. The local folder is what this command loads.

## How to run

From the repository root, one command opens `/home/aviya/Desktop/fault_injection.usd` and enables this extension:

```bash
cd /home/aviya/Projects/gangelia
/home/aviya/IsaacSim/_build/linux-x86_64/release/python.sh isaac/load_fault_injection_scene.py
```

The viewport stays on Perspective. Nothing starts until you press Play. Press Play and the Franka already in that file runs pick and place. A failure runs only after you turn that failure on yourself. That command sets `GANGELIA_FAULT_INJECTOR_EDITION=pro` for this checkout, because this checkout contains the Camera Noise files. The Community package does not set that variable and does not contain those files. The Robotics Examples **LOAD** button builds a new stage, so leave it unused.

## How to use

The fault list has four rows. **Drop** runs in every package. **Camera Noise** runs in the Pro package. The Community package marks it **Demo**. **Object Over Object** and **Object Collision** are marked **Demo**. Choosing a demo row hides the working controls, disarms a running drop, cancels a regrasp, stops camera noise, and shows `Demo Version - Contact Gangelia`. Choosing **Drop**, or **Camera Noise** in the Pro package, brings that panel back.

### Camera Noise

Camera Noise is Pro. The Community package shows **Demo** and does not contain the camera files. In the Pro package, click **Camera Noise**, type the camera prim path, and check **Apply camera noise**. Point a viewport at that prim to watch it. Perspective and every other camera stay clean.

### Drop

Press Play so the timeline is running. Then set the gripper, the articulation prim path, and either press **Inject Drop** or check **Automatic drop after pickup**.

**Inject Drop** sends one open command for the selected gripper and marks that object as released. The status line reads `Open command sent through <gripper>`. A controller that commands the gripper on a later step can close it again. **Regrasp** can then run without the automatic-drop checkbox.

**Automatic drop after pickup** needs the target object's prim path and a drop probability. A pickup counts when all of these are true:

- the jaws are past the gripper's minimum closure
- the object is within the pickup distance of the grasp frame
- the object is at least 3 cm above the lowest height seen while the extension was watching
- that distance stays steady for 6 physics steps

Each later physics step rolls the probability once. A hit holds the gripper open for 30 physics steps, including after the pick-and-place controller's own close command. The status line then reads `Pickup confirmed`, `Drop hit. Holding the gripper open.`, and `Drop finished. Waiting for the object to leave the jaws.`

The probability is per physics step, not per second. `1` drops on the first step after a pickup is confirmed. The default `0.0001` is much rarer. A faster physics rate produces drops sooner at the same number.

Leave the box checked to drop again. After the object moves farther from the grasp frame than the pickup distance, the status line reads `Watching for the next pickup` and the next grasp gets its own roll. Unchecking the box stops watching.

**Regrasp** is under **Inject Drop**. It runs after **Inject Drop**, after an automatic drop finishes, or when the object is already farther from the hand than the pickup distance. Pressing it while the object is still in the hand does not open the gripper. The status line then reads `Regrasp waits until the object has left the gripper.` If the object stays inside the pickup distance, the status line reads `Regrasp stopped: the object did not leave the gripper.` and the fingers are no longer held open. With the timeline playing and **Panda Hand** selected, the fingers stay open while the object settles. The arm then moves the pinch point above the object with the fingers pointing down, descends, closes, and lifts a short distance. A wrist that is already pointing down is kept. The status line reads `Regrasp armed. Waiting for the object to settle.`, then `Regrasp: moving above the object.`, `Regrasp: moving down to the object.`, `Regrasp: closing the gripper.`, and `Regrasp: lifting the object.` **Robotiq 2F-85** and **Custom** report `Regrasp is only implemented for Panda Hand.`

**Automatic regrasp after drop** starts that same motion when a drop finishes. It runs once. The automatic-drop checkbox turns off when the lift finishes, so a probability of `1` does not throw the object again. Check **Automatic drop after pickup** again for another drop. If **Automatic regrasp after drop** is off and the drop checkbox is still on, watching starts over after a manual regrasp. The regrasp itself is not another pickup.

If the status line reports an error for automatic drop, that checkbox clears itself. A missing joint, a bad number, or an empty path does this. Fix the field and check **Automatic drop after pickup** again. **Automatic regrasp after drop** stays as you left it. A failed regrasp also turns the automatic-drop checkbox off.

### Built-in grippers

**Panda Hand** commands `panda_finger_joint1` and `panda_finger_joint2`. The grasp frame is `panda_hand` under the articulation. For the standard Franka, the articulation prim path is `/World/Franka`.

**Robotiq 2F-85** commands `finger_joint`. Open is `0` and closed is 75 degrees. The grasp frame is `base_link` directly under the articulation. When that link is nested deeper, as on a Franka with the Robotiq variant selected, use **Custom** and paste the full `base_link` path into **Grasp frame**.

### Custom

Choose **Custom** to type another parallel gripper. The fields start as a copy of the preset that was showing. Switching back to a preset ignores what was typed.

| Field | Meaning |
| --- | --- |
| Grasp frame | A prim name under the articulation, or a full prim path starting with `/` |
| Joint names | Comma-separated joint names on that articulation |
| Open positions | One number per joint, in the same order |
| Closed positions | One number per joint, different from the open position |
| Minimum closure (0 to 1) | How far the jaws must travel from open toward closed before a grasp counts |
| Pickup distance (m) | How near the object must be to the grasp frame |

The grasp frame should be a prim at the jaw tips. A frame back at the wrist needs a larger pickup distance, because the distance is measured from that prim to the object.

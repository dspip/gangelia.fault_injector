# Gangelia Fault Injector

Opens a parallel gripper during a running Isaac Sim 5.1 simulation. The open command goes to the joints, so it works while a planner, a script, or a model is moving the arm.

## How to install

1. Extract `gangelia.fault_injector-0.1.0.zip`. That produces a folder named `gangelia.fault_injector`.
2. Open Isaac Sim and choose **Window → Extensions**.
3. Open the extension manager settings. Under **Extension Search Paths**, add the folder that contains `gangelia.fault_injector`. If the zip was extracted in `Downloads`, the search path is `Downloads`, not `Downloads/gangelia.fault_injector`.
4. Enable **Gangelia Fault Injector**.

The **Gangelia Fault Injector** panel docks at the lower left.

## How to use

The fault list has four rows. **Drop** is the working fault. **Object Over Object**, **Camera Noise**, and **Object Collision** are marked **Demo**. Choosing one of those hides the drop controls, disarms a running drop, and shows `Demo Version - Contact Gangelia`. Choosing **Drop** again brings the controls back with the checkbox off.

Press Play so the timeline is running. Then set the gripper, the articulation prim path, and either press **Inject Drop** or check **Automatic drop after pickup**.

**Inject Drop** sends one open command for the selected gripper. The status line reads `Open command sent through <gripper>`. A controller that commands the gripper on a later step can close it again.

**Automatic drop after pickup** needs the target object's prim path and a drop probability. A pickup counts when all of these are true:

- the jaws are past the gripper's minimum closure
- the object is within the pickup distance of the grasp frame
- the object is at least 3 cm above the lowest height seen while the extension was watching
- that distance stays steady for 6 physics steps

Each later physics step rolls the probability once. A hit holds the gripper open for 30 physics steps. The status line then reads `Pickup confirmed`, `Drop hit. Holding the gripper open.`, and `Drop finished. Waiting for the object to leave the jaws.`

The probability is per physics step, not per second. `1` drops on the first step after a pickup is confirmed. The default `0.0001` is much rarer. A faster physics rate produces drops sooner at the same number.

Leave the box checked to drop again. After the object moves farther from the grasp frame than the pickup distance, the status line reads `Watching for the next pickup` and the next grasp gets its own roll. Unchecking the box stops watching.

If the status line reports an error, the box clears itself. A missing joint, a bad number, or an empty path does this. Fix the field and check the box again.

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

# X1 Laban gesture player

Play [Microsoft LabanotationSuite](https://github.com/microsoft/LabanotationSuite)
upper-body gesture JSON on X1, from a browser page or the command line. The same
node drives the Isaac Sim twin and the real robot, over the same
`JointTrajectoryController` topics as the rest of this repo.

**中文摘要 / Chinese summary:** [README.zh-Hant.md](README.zh-Hant.md)

```
Laban JSON  ->  decoder.py  ->  x1_ik_mapper.py  ->  /left_arm_controller/joint_trajectory
                                                     /right_arm_controller/joint_trajectory
                                                     /head_controller/joint_trajectory
```

## What you need first

This is a layer on top of the robot stack in this repo, not a standalone package.
Before anything here will move an arm:

1. **The ROS 2 stack is built and running** — `ros2_ws_src` compiled per
   [README](../README.md) §2.2, then either the real robot (§3.2) or Isaac Sim.
   The arm and `head_controller` (`j_101` yaw / `j_102` pitch) must be active.
2. **`~/pink_venv` exists** with `pin` and `pink` — the venv from
   [README](../README.md) §2.1. The IK mapper reads
   `~/xiaobei_X1_ws/src/pkg_robot_model/urdf/pkg_robot_model.urdf`, so the
   `colcon build` in step 1 is what puts the URDF where the mapper looks.
3. **`/tmp/teleop_env.sh` is in place** for anything touching the real robot —
   `pwsh deploy/deploy_teleop.ps1` puts it there. Without it the player uses the
   wrong DDS settings and publishes into the void: the controllers never see it.

> Playing a gesture moves the arms at full speed. Clear the workspace and keep
> the e-stop in reach. Stop the Quest 3 teleop bridges first
> (`bash /tmp/teleop_down.sh`) — they command the same controllers, and two
> publishers fight over the arms.

## Web UI

```bash
python3 -m pip install -r laban/ui/requirements.txt   # once, into ~/pink_venv
bash laban/ui/run_ui.sh                               # http://<host>:9200
```

The page lists the gesture library, plays one on click, and reports when the arms
will start moving and when the gesture ends. The speed slider and the *Move head*
toggle apply to the next gesture; *Stop* cancels the one in flight.

`X1_DDS` picks the target: `real` (default), `isaac` for the twin only, or `both`
to run the robot and the twin off one timeline.

```bash
X1_DDS=isaac bash laban/ui/run_ui.sh    # validate here first
X1_DDS=both  bash laban/ui/run_ui.sh
```

## Command line

One gesture, one process:

```bash
X1_DDS=isaac bash laban/run_player.sh laban/gestures/library/hello.json --mapper ik
X1_DDS=real  bash laban/run_player.sh laban/gestures/library/hello.json \
  --mapper ik --allow-ik-real --speed 0.5
```

| Flag | Meaning |
|------|---------|
| `--dry-run` | print keyframes and joint angles, publish nothing |
| `--check` | report controller subscriber counts, publish nothing |
| `--speed 1.5` | faster playback |
| `--no-head` | leave the head still |
| `--approach 3.0` | cap on the ramp from the current pose into the first keyframe |
| `--no-return` | stay in the closing pose instead of opening back to ready |
| `--loop` | repeat |
| `--mapper ik` | URDF-based Pink elbow+wrist IK (what the UI uses) |
| `--mapper lut` | legacy coarse symbol-to-joint lookup, kept as a fallback |
| `--ik-gain 1.0` | IK amplitude: 1.0 follows the Laban direction fully |
| `--allow-ik-real` | required to run the IK mapper on real hardware |

## Resident player

Spawning a player per gesture costs about 2.3 s before the arms can move (bash,
venv python and IK account for 1.0 s of it; the rest is `rclpy` init and
controller discovery). The daemon holds `rclpy`, the matched controllers and the
Pinocchio model between gestures and caches solved keyframes, which brings that
down to 0.4 s for a cached gesture and 0.8 s the first time one is solved.

```bash
X1_DDS=real bash laban/run_player.sh --daemon --mapper ik

# any python3, no rclpy needed
python3 laban/laban_ctl.py status
python3 laban/laban_ctl.py play laban/gestures/library/hello.json --head
python3 laban/laban_ctl.py stop        # publishes an empty trajectory: a real cancel
python3 laban/laban_ctl.py quit
```

Requests are one JSON object per line on `/tmp/x1_laban_<mode>.sock`. The UI uses
the socket when it is there and spawns a player when it is not, so the daemon is
optional — the status pill on the page says which path it took. Starting a second
daemon on the same socket is refused; two would each publish a full trajectory.

## The gesture library

`gestures/library/` holds the 92 LabanotationLibrary samples from
MSRAbotChatSimulation; `gestures/*.total.json` are the original LabanotationSuite
demo gestures.

At full amplitude 87 of the 92 retarget cleanly and 5 (`hello`, `goodbye`,
`confuse r`, `robot d move`, `interesting`) are automatically retried at reduced
amplitude — `Place/High` asks for an arm straight overhead, which drives a
shoulder joint to ~2.8 rad and makes the next keyframe flip the shoulder to the
other IK branch. The player retries the whole gesture at 0.85, 0.70, 0.55 and
0.40 gain and logs where it settled.

**13 gestures are blocked outright.** At full amplitude the retarget commands
genuine self-collisions in them — collision meshes interpenetrating, minimum gap
0.000 m, and `thanks` does it in 6 of its 8 keyframes. They only clear on the real
robot because gravity opens the shoulders, which is not a clearance the hardware
can be trusted to keep. `gesture_policy.py` enforces this where the catalogue is
built, so neither the UI nor the daemon can reach them:

```bash
python3 laban/gesture_policy.py --profile chat    # what is playable, and why not
```

The measured clearances live in `gesture_allowlist.yaml`; the block list itself is
a compiled-in constant, because a safety property that disappears when a config
file is missing is not a safety property.

## How the retargeting works

Laban keyframes carry discrete direction symbols, so the symbol-to-joint lookup is
a step function and is evaluated **only at keyframes**. The player then
interpolates in joint space with smoothstep easing, attaches finite-difference
velocities, and stretches any segment that `MAX_JOINT_VEL` says is too fast.
Without that the trajectory is a staircase — flat, then ~0.6 rad in one 0.1 s step
— and the torso rocks.

The IK mapper turns each symbolic keyframe into two Cartesian targets per arm:

```
shoulder + upper-arm direction * URDF upper-arm length -> elbow
elbow    + forearm  direction * URDF forearm  length   -> wrist
```

Pink solves the L4/L6 and R54/R56 position tasks with continuity and ready-pose
regularization, from two seeds (previous keyframe and ready pose) so a raised arm
cannot leave the shoulder on a rotated branch. Residuals of 0.05–0.20 m are normal
rather than an error: the targets come from human-proportioned directions, X1's
ready pose does not hang straight down, and elbow plus wrist together
over-constrain a 6-DOF arm.

Safety checks reject failed IK, joints outside ±2.9 rad, and adjacent keyframes
more than 2.2 rad apart (an IK branch flip; a legitimate rest-to-raised step is
about 1.5 rad and gets time-stretched instead).

`--ik-gain` blends each target direction between X1's ready-pose direction and the
Laban direction. It is one robot-wide parameter, not a per-gesture table, and it
changes how a gesture reads: with the wave, the right elbow relative to the
shoulder lands at −0.02 m (waving at chest height) at gain 0.5 and +0.16 m
(elbow raised, a real wave) at gain 1.0.

## The head

Labanotation has no head axes, so the head is driven from two other places: the
`head` limb direction, and the `rotation` field (notated as torso rotation) mapped
onto head yaw at `HEAD_ROTATION_SCALE` = 0.5 to keep it inside a natural range.

The measured axis directions are **the opposite of what the Laban words suggest on
both axes**, which is why `x1_mapper.py` inverts both. Confirmed on the twin and
then on the robot: positive `j_101` turns the head to its right, positive `j_102`
pitches it up.

The URDF declares both head joints `continuous`, with no limits of their own, so
the limits and the velocity ceiling are enforced in software here:
`HEAD_LIMITS` bounds travel, and `MAX_HEAD_VEL` is 1.5 rad/s against the arms'
1.0. The head carries far less inertia, so holding it to the arm limit makes every
nod look laboured — but without a ceiling of its own nothing would have caught
`shake head` asking for 0.7 rad in 500 ms.

19 gestures in the library are head-only: the arms hold one pose throughout and
all the motion is in the head. `nod`, `shake head` and `laugh head` are the
obvious ones. **With the head disabled they are silent no-ops**, which looks like
a broken player, so if a gesture appears to do nothing, check the *Move head*
toggle (or `--no-head`) first.

## Returning to the ready pose

A gesture's last keyframe is chosen for expression, not for resting, and the
controller holds whatever it was last given. Several two-handed gestures close
with the grippers almost touching, so the robot used to park there — and every
following gesture then started from there. The pose left behind by `thanks.json`
had 4 mm of clearance between the hands, against 231 mm at the ready pose.

The player therefore appends a lead-out mirroring the lead-in: hold the closing
pose for `X1_RETURN_HOLD` seconds (default 0.6, so the pose the gesture was
building to is still readable), then ease back to the ready pose over whatever
`MAX_JOINT_VEL` requires, sampled like the rest of the timeline so the Isaac
mirror follows it too. `--no-return` turns it off, as does `--loop`, where
returning between repetitions would break the cycle.

Measured after the change: the controller reference, the robot and the twin all
rest within 0.02 rad of the ready pose, with 0.230 m of hand clearance against
0.231 m nominal.

## Driving the robot and the twin together

`X1_DDS=both` keeps the real stack's DDS settings and adds a mirror. The twin does
not consume trajectories — its OmniGraph listens on `/isaac_joint_commands`
(`sensor_msgs/JointState`) — so the same samples go to the controllers as a
trajectory and to the twin as interpolated `JointState` at 50 Hz, off one timeline.

Do not start a second player with `X1_DDS=isaac` to reach the twin. The real
controllers run in the same WSL, so a LOCALHOST-scoped player discovers the real
controllers again and commands the robot twice while the twin stays still.

Isaac is a late joiner for every player process (1–5 s in testing), so the player
waits `X1_ISAAC_WAIT` seconds (default 3) before starting and warns if the twin
never appears, rather than holding up the robot.

## Layout

```
laban/
  decoder.py            official Laban JSON -> direction symbols / theta-phi
  x1_mapper.py          symbols -> X1 joints (coarse LUT) + head mapping
  x1_ik_mapper.py       symbols -> Cartesian targets -> Pink IK
  gesture_policy.py     which gestures are safe to play
  gesture_allowlist.yaml  measured clearances behind that policy
  laban_ctl.py          socket client for the resident player
  run_player.sh         launcher: DDS setup, venv selection, one-shot or daemon
  nodes/
    laban_player.py     ROS 2 player: resample, ramp, publish
    laban_daemon.py     resident player, one gesture at a time
  gestures/
    library/            92 MSRAbotChatSimulation samples
    *.total.json        original LabanotationSuite demo gestures
  ui/
    server.py           gesture list + play/stop REST API
    gesture_player.py   daemon-or-spawn playback client
    run_ui.sh           launcher
    static/             the page
```

## Nothing moves — where to look

| Symptom | Cause |
|---|---|
| UI plays, log says ok, robot still | Wrong DDS. The real robot needs `/tmp/teleop_env.sh` for the player that actually publishes; `run_ui.sh` warns at startup when it is missing. |
| Gesture is a no-op, no error | Head-only gesture with the head disabled. Enable *Move head*. |
| Gesture greyed out in the UI | Blocked for self-collision. `python3 laban/gesture_policy.py` prints the reason. |
| Arms jump, then stop | Another publisher on the same controllers — Quest 3 teleop still up. `bash /tmp/teleop_down.sh`. |
| `--mapper ik` refuses on the real robot | By design. Validate in Isaac first, then pass `--allow-ik-real`. |
| Every gesture takes 2.3 s to start | No daemon. Start one with `run_player.sh --daemon`. |
| `gesture_policy` warns about no allow-list | `gesture_allowlist.yaml` is missing; the 13 compiled-in blocks still apply, the tiers do not. |

## Credits

Gesture JSON and the Labanotation encoding come from Microsoft
[LabanotationSuite](https://github.com/microsoft/LabanotationSuite) (MIT), whose
MSRAbotChatSimulation library is reused unchanged.

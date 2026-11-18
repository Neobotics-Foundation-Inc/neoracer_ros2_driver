# EPS Dashboard

Episodic Policy Search for the NeoRacer, on port 8084. The gap follower from the wallfollow dashboard is the plant; a (1+1) evolution strategy learns its four PID coefficients from one Success or Fail press per episode. Sustained success raises the speed a notch; the lab's result is the highest speed the car holds and how many attempts it took.

## Service control

Installed with the driver (`racecar service install`, which runs `scripts/setup_dashboards.sh`), stopped and disabled.
Dashboard: `http://<car-ip>:8084`.

| Command | Does |
|---|---|
| `racecar service start eps` | start now |
| `racecar service stop eps` | stop |
| `racecar service enable eps` | start at every boot |
| `racecar service logs eps` | follow the log |

Stop the other driving dashboards (wallfollow, pursuit, smartfollow, linefollow, webteleop) while EPS runs: only one thing may publish /drive.

## Workflow

1. Students enter starting gains and speed in the Starting point card and press Set. It locks immediately; only Reset unlocks it.
2. START RUN arms a mutated gain proposal and drives it. SUCCESS or FAIL ends the run and is the entire training signal; Discard voids a spoiled run, Undo takes back exactly one label.
3. Success accepts the proposal and grows the mutation size sigma; failure keeps the old gains and shrinks it. confirm_runs successes in a row raise the speed; patience fails in a row back it off.
4. A pass must also keep pace: slower than the best lap at that speed times duration_slack scores as slow and is rejected. Green, gold, red dots on the staircase chart.
5. Reset archives the ledger, clears the charts, and returns gains and speed to the starting point.

## Speed control

The staircase speed is the constant throttle while the learned speed gains are zero. With nonzero speed gains, the speed times max_mps, scaled by the road actually available ahead, becomes an m/s target, and speed_kp and speed_kd bend the throttle toward it using the measured speed from /odom, so battery and surface changes are corrected. Odometry silent for half a second drops the correction and leaves the constant throttle.

## Files

eps.yaml holds the shipped learner hyperparameters (hover any panel field for what it does), the plant settings, and the nominal gain vector that scales the search. Save and Set starting point write this car's values to the car profile, `~/.config/racecar/dashboards/eps.yaml`, which loads on top; a driver update leaves that file alone.

ledger.jsonl is the session record, one row per label with the full learner state, so a restart resumes where the session left off. It lives in `dashboards/eps/` under the data root (`~/neoracer_data`), with the lock of the starting point beside it; Reset archives it there as ledger_TIMESTAMP.jsonl.

## Tests

`test/test_dashboard_eps.py` in the driver; no ROS graph, lidar, or browser needed.

## Safety

The NeoRacer mux forwards /drive with no software deadman; the handset's mode switch is the physical gate: with the handset on, the ESP32 follows /drive only with the switch down. With the handset off or out of range it follows /drive too, so keep the handset on and flip the switch up to stop the car. The car drives only while an episode is RUNNING, every other phase publishes zeros, speed_max caps the staircase, and proposals are clamped to 0.5x..2x of the starting gains.

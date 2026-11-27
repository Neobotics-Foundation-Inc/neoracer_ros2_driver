# Neoracer shell tool - `racecar <subcommand>`.
# Sourced from ~/.bashrc by setup_user_env.sh.
# Not executed directly: it defines a `racecar` shell function so the build /
# test / source subcommands can mutate the current shell (PWD, env).

# Where the car's data goes (neoracer_ros2_driver/data_root.py). Login shells
# get it from /etc/profile.d/neoracer-data.sh; this covers a shell that read
# only .bashrc, such as a terminal inside JupyterLab.
if [ -z "${RACECAR_DATA_DIR:-}" ]; then
    if [ -r /run/racecar/data_root ]; then
        RACECAR_DATA_DIR="$(cat /run/racecar/data_root)"
    else
        RACECAR_DATA_DIR="$HOME/neoracer_data"
    fi
    export RACECAR_DATA_DIR
fi

# Core stack, enabled at boot by setup_services.sh. neoracer-autonomy is not
# here: setup installs it, but it rides on teleop and `racecar map` or the map
# viewer starts it on demand.
_RC_CORE_UNITS=(neoracer-teleop neoracer-watchdog neoracer-dashboard
                neoracer-jupyter neoracer-mapview)

# Lab dashboards, installed disabled and started per session. Each holds the
# camera or the GPU for its whole run, so they are opt-in rather than part of
# the stack that comes up on boot.
_RC_DASH_UNITS=(neoracer-camlabel neoracer-wallfollow neoracer-pursuit
                neoracer-eps neoracer-smartfollow neoracer-linefollow
                neoracer-webteleop)
# The lab dashboards that publish /drive, as DRIVING_APPS in
# scripts/map_control.py. Two of them running would both send commands to the
# mux, so starting or restarting one stops the others first. camlabel runs
# alongside them.
_RC_DRIVE_UNITS=(neoracer-wallfollow neoracer-pursuit neoracer-eps
                 neoracer-smartfollow neoracer-linefollow neoracer-webteleop)

# Running as the apps page and scripts/map_control.py count it: a unit that is
# activating, such as one waiting to restart after a crash, comes back on its
# own, so it counts as running.
_rc_unit_running() {
    case "$(systemctl is-active "$1" 2>/dev/null)" in
        active|activating|reloading) return 0 ;;
    esac
    return 1
}

# Units that would come back on their own after a reboot. `restart` with no
# unit named and `racecar update` use this so they never promote a disabled
# service into a running one; systemctl restart starts a stopped unit, which
# would otherwise put a lab dashboard on the graph without the student asking.
# At most one /drive dashboard is listed: the one running, or else the first
# enabled one. Another enabled one stays stopped, and a line on stderr says so.
_rc_enabled_units() {
    local u drive=""
    for u in "${_RC_DRIVE_UNITS[@]}"; do
        if _rc_unit_running "$u"; then
            drive="$u"
            break
        fi
    done
    for u in "${_RC_CORE_UNITS[@]}" "${_RC_DASH_UNITS[@]}"; do
        systemctl is-enabled --quiet "$u" 2>/dev/null || continue
        if [[ " ${_RC_DRIVE_UNITS[*]} " == *" $u "* ]]; then
            if [[ -n "$drive" && "$u" != "$drive" ]]; then
                echo "  leaving $u stopped: $drive also publishes /drive" >&2
                continue
            fi
            drive="$u"
        fi
        echo "$u"
    done
}

_rc_unit_installed() {
    [[ -n "$(systemctl list-unit-files "$1.service" --no-legend 2>/dev/null)" ]]
}

# One unit per call, named in full: /etc/sudoers.d/neoracer allows exactly
# `systemctl start|stop|restart neoracer-<name>.service`, so these need no
# password. enable and disable still ask for it.
_rc_systemctl() {
    local action="$1" u
    shift
    for u in "$@"; do
        sudo systemctl "$action" "${u%.service}.service" || return
    done
}

# One status line. An absent unit reports as such rather than passing
# systemd's "Failed to get unit file state" through to the column. A unit
# whose description names a port gets the page's address.
_rc_unit_status_line() {
    local u="$1" state enabled port
    if ! _rc_unit_installed "$u"; then
        printf "  %-22s  not installed\n" "$u"
        return
    fi
    state=$(systemctl is-active "$u" 2>&1 || true)
    enabled=$(systemctl is-enabled "$u" 2>&1 || true)
    port=$(sed -n 's/^Description=.*(port \([0-9]*\)).*/\1/p' \
        "${RACECAR_SYSTEMD_DIR:-/etc/systemd/system}/$u.service" 2>/dev/null | head -1)
    printf "  %-22s  active=%-12s enabled=%-10s %s\n" "$u" "$state" "$enabled" \
        "${port:+http://$(hostname).local:$port}"
}

racecar() {
    local pkg="neoracer_ros2_driver"
    local ws="$HOME/ros2_ws"
    local pkg_dir="$ws/src/$pkg"
    local cmd="${1:-help}"
    shift || true

    case "$cmd" in
        build)
            # A source file deleted since the last --symlink-install build
            # leaves a dangling link that setup.py's scripts/*.py glob picks up,
            # and the build then fails when setuptools copies it.
            local tree link
            for tree in "$ws/build/$pkg" "$ws/install/$pkg"; do
                [[ -d "$tree" ]] || continue
                while IFS= read -r -d '' link; do
                    rm -- "$link" && echo "  removed dangling symlink: $link"
                done < <(find "$tree" -xtype l -print0)
            done
            ( cd "$ws" && colcon build --packages-select "$pkg" --symlink-install "$@" ) \
                && source "$ws/install/setup.bash"
            ;;

        test)
            ( cd "$ws" \
                && colcon test --packages-select "$pkg" --event-handlers console_direct+ "$@" \
                && colcon test-result --verbose )
            ;;

        lint)
            # ruff, black and mypy read their settings from pyproject.toml.
            local lint_bin="${RACECAR_LINT_BIN:-$HOME/.local/bin}"
            local tool
            local -a failed=()
            for tool in ruff black mypy; do
                if [[ ! -x "$lint_bin/$tool" ]]; then
                    echo "racecar lint: $lint_bin/$tool not found; run scripts/setup_dev_tools.sh" >&2
                    return 3
                fi
            done
            echo "==> ruff check"
            ( cd "$pkg_dir" && "$lint_bin/ruff" check . ) || failed+=("ruff")
            echo "==> black --check"
            ( cd "$pkg_dir" && "$lint_bin/black" --check . ) || failed+=("black")
            echo "==> mypy"
            ( cd "$pkg_dir" && "$lint_bin/mypy" ) || failed+=("mypy")
            echo
            if [[ ${#failed[@]} -gt 0 ]]; then
                echo "racecar lint: failed: ${failed[*]}" >&2
                return 1
            fi
            echo "racecar lint: ruff, black and mypy passed"
            ;;

        profile)
            # This car's settings outside the driver folder; see
            # scripts/neoracer_profile.py and neoracer_ros2_driver/car_profile.py.
            python3 "${RACECAR_PROFILE_SCRIPT:-$pkg_dir/scripts/neoracer_profile.py}" "$@"
            ;;

        data)
            # Where the car's data goes; see neoracer_ros2_driver/data_root.py.
            python3 "$pkg_dir/neoracer_ros2_driver/data_root.py" "$@"
            ;;

        log)
            # ROS 2 bag recording and analysis. Thin wrapper: the work and its
            # tests live in scripts/neoracer_log.py.
            python3 "${RACECAR_LOG_SCRIPT:-$pkg_dir/scripts/neoracer_log.py}" "$@"
            ;;

        calibrate)
            # Guided calibration; see scripts/calibration/calibrate.py.
            python3 "${RACECAR_CALIBRATE_SCRIPT:-$pkg_dir/scripts/calibration/calibrate.py}" "$@"
            ;;

        source)
            # shellcheck disable=SC1091
            source "$ws/install/setup.bash"
            ;;

        ws)
            # Switch the *current shell* between the neoracer workspace and
            # the vendor stack preinstalled in ~/osracer_ws. Both carry a
            # `lakibeam1` package, so only one can be active per shell; this
            # rebuilds the ROS environment from the base distro plus the
            # chosen overlay. Nothing on disk changes - open a new shell (or
            # switch back) to get the other workspace.
            local target="${1:-status}"
            local vendor_ws="$HOME/osracer_ws"
            case "$target" in
                neoracer|osracer)
                    local overlay
                    if [[ "$target" == "neoracer" ]]; then
                        overlay="$ws/install/setup.bash"
                    else
                        overlay="$vendor_ws/install/setup.bash"
                    fi
                    if [[ ! -f "$overlay" ]]; then
                        echo "racecar ws: $overlay not found" >&2
                        return 1
                    fi
                    # Drop overlay state; keep non-workspace entries of the
                    # mixed vars (CUDA paths etc. survive the switch).
                    unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
                    local var val
                    for var in PATH PYTHONPATH LD_LIBRARY_PATH; do
                        val="${!var:-}"
                        [[ -n "$val" ]] || continue
                        val=$(printf '%s' "$val" | tr ':' '\n' \
                              | grep -vE '/(osracer_ws|ros2_ws)(/|$)' | paste -sd: -) || true
                        if [[ -n "$val" ]]; then export "$var=$val"; else unset "$var"; fi
                    done
                    # shellcheck disable=SC1091
                    source /opt/ros/humble/setup.bash
                    # shellcheck disable=SC1090
                    source "$overlay"
                    export RACECAR_WS="$target"
                    echo "This shell now uses the $target workspace ($overlay)."
                    ;;
                status)
                    if [[ -n "${RACECAR_WS:-}" ]]; then
                        echo "active workspace: $RACECAR_WS (set by racecar ws)"
                    elif [[ ":${AMENT_PREFIX_PATH:-}:" == *"/osracer_ws/"* ]]; then
                        echo "active workspace: osracer (vendor, via inherited environment)"
                    elif [[ ":${AMENT_PREFIX_PATH:-}:" == *"/ros2_ws/"* ]]; then
                        echo "active workspace: neoracer (default from .bashrc)"
                    else
                        echo "active workspace: none (base ROS only)"
                    fi
                    echo "AMENT_PREFIX_PATH:"
                    printf '%s' "${AMENT_PREFIX_PATH:-}" | tr ':' '\n' | sed 's/^/  /'
                    echo
                    ;;
                *)
                    echo "usage: racecar ws [neoracer|osracer|status]" >&2
                    return 2
                    ;;
            esac
            ;;

        teleop)
            # Use the launch wrapper so we get a timestamped log dir at
            # ~/logs/<ts>/ and a fresh FastRTPS SHM sweep. Extra args (e.g.
            # `lidar_enable:=false`) forward through to ros2 launch.
            bash "$pkg_dir/scripts/launch_teleop.sh" "$@"
            ;;

        map)
            # SLAM mapping and localization through neoracer-autonomy.service;
            # see scripts/neoracer_map.py and scripts/map_control.py.
            python3 "${RACECAR_MAP_SCRIPT:-$pkg_dir/scripts/neoracer_map.py}" "$@"
            ;;

        update)
            # One-command field update: repo to latest main, full setup, and
            # a service restart. The git reset runs BEFORE setup_all so the
            # script never rewrites itself mid-run; this function is already
            # loaded in shell memory, so replacing the file under it is safe.
            # reset --hard discards local edits in the driver repo.
            # setup_jupyter.sh leaves the labs as they are unless --labs is
            # given. RACECAR_LABS_REFRESH carries the choice through
            # setup_all.sh. It is set on every update, so a value exported in
            # the shell never refreshes the labs without --labs.
            local labs=0 arg
            for arg in "$@"; do
                case "$arg" in
                    --labs) labs=1 ;;
                    *)
                        echo "usage: racecar update [--labs]" >&2
                        return 2
                        ;;
                esac
            done
            echo "Updating $pkg_dir to latest origin/main..."
            ( cd "$pkg_dir" && git fetch origin && git reset --hard origin/main ) || return 1
            RACECAR_LABS_REFRESH="$labs" bash "$pkg_dir/scripts/setup_all.sh" || return 1
            # Restart only what was already enabled, so a field update brings
            # back the services this car runs without starting a lab dashboard
            # nobody asked for.
            echo "Restarting services..."
            local -a live
            mapfile -t live < <(_rc_enabled_units)
            [[ ${#live[@]} -gt 0 ]] && _rc_systemctl restart "${live[@]}"
            echo "Update complete. Open a new terminal (or run: racecar source)"
            echo "so this shell picks up the new environment and racecar tool."
            ;;

        cd)
            # Hop to the package source dir. Has to be a shell function (not
            # subprocess) so the cd sticks in the user's interactive shell.
            cd "$pkg_dir" || return 1
            ;;

        launch)
            local name="$1"
            if [[ -z "$name" ]]; then
                echo "usage: racecar launch <name>   # e.g. racecar launch lidar" >&2
                return 2
            fi
            shift
            ros2 launch "$pkg" "${name}.launch.py" "$@"
            ;;

        compile)
            # TensorRT export, .pt -> .engine, for inference_node. The engine is
            # built against this board's GPU and TensorRT and is not portable;
            # rebuild after a JetPack, Ultralytics, or hardware change.
            local src="" imgsz="" device="0" half="True" force=0
            local cfg="$pkg_dir/config/inference.yaml"
            for arg in "$@"; do
                case "$arg" in
                    --imgsz=*)  imgsz="${arg#*=}" ;;
                    --device=*) device="${arg#*=}" ;;
                    --half)     half="True" ;;
                    --no-half)  half="False" ;;
                    --force)    force=1 ;;
                    -h|--help)
                        cat <<'__RC_COMPILE_HELP__'
usage: racecar compile [model] [--imgsz=N] [--device=N] [--no-half] [--force]

Exports a YOLO .pt to a TensorRT .engine. With no model, reads model_path and
imgsz from config/inference.yaml, so the engine matches what the node loads.
A bare filename resolves against the package's models/ directory.

  racecar compile                       # whatever inference.yaml points at
  racecar compile yolo26n.pt            # models/yolo26n.pt
  racecar compile /path/to/custom.pt    # absolute path

The build peaks near 3 GB of the Orin's 8 GB shared memory, so it refuses to
start while the stack is running. Stop it first (racecar service stop) or pass
--force. A nano model at 640 takes about 8 minutes on a 25 W Orin Nano.
__RC_COMPILE_HELP__
                        return 0
                        ;;
                    -*) echo "racecar compile: unknown flag '$arg'" >&2; return 2 ;;
                    *)  src="$arg" ;;
                esac
            done

            # Defaults come from the node's own config so the engine is built at
            # the imgsz it will be served at; an engine is fixed to that size.
            if [[ -z "$src" && -f "$cfg" ]]; then
                src=$(sed -n 's/^[[:space:]]*model_path:[[:space:]]*"\?\([^"#]*\)"\?.*/\1/p' "$cfg" \
                      | head -1 | tr -d '[:space:]')
            fi
            if [[ -z "$imgsz" && -f "$cfg" ]]; then
                imgsz=$(sed -n 's/^[[:space:]]*imgsz:[[:space:]]*\([0-9]*\).*/\1/p' "$cfg" | head -1)
            fi
            imgsz="${imgsz:-640}"

            if [[ -z "$src" ]]; then
                echo "racecar compile: no model given and none found in $cfg" >&2
                return 2
            fi
            if [[ "$src" == *.engine ]]; then
                echo "racecar compile: '$src' is already a TensorRT engine." >&2
                return 2
            fi

            local resolved="$src"
            [[ "$src" != /* && "$src" != */* ]] && resolved="$pkg_dir/models/$src"
            if [[ ! -f "$resolved" ]]; then
                echo "racecar compile: no such file '$resolved'." >&2
                echo "Fetch stock weights into models/ first, e.g.:" >&2
                echo "  cd $pkg_dir/models && python3 -c \"from ultralytics import YOLO; YOLO('$src')\"" >&2
                return 3
            fi

            local engine="${resolved%.*}.engine"
            if [[ -f "$engine" && $force -eq 0 ]]; then
                echo "racecar compile: $engine already exists. Pass --force to rebuild." >&2
                return 3
            fi

            if ! python3 -c "import ultralytics" 2>/dev/null; then
                echo "racecar compile: ultralytics is not installed. Run: racecar setup ml" >&2
                return 3
            fi

            if [[ $force -eq 0 ]]; then
                local busy=""
                systemctl is-active --quiet neoracer-teleop 2>/dev/null \
                    && busy+="neoracer-teleop "
                # SLAM holds memory the GPU shares, whether neoracer-autonomy
                # or `racecar launch autonomy` started it.
                pgrep -f autonomy.launch.py >/dev/null 2>&1 && busy+="autonomy "
                if [[ -n "$busy" ]]; then
                    echo "racecar compile: ${busy% } running; the export peaks near 3 GB." >&2
                    echo "Stop the stack first (racecar service stop), or pass --force." >&2
                    return 3
                fi
            fi

            echo "Exporting $(basename "$resolved") -> $(basename "$engine")"
            echo "  imgsz=$imgsz device=$device half=$half"
            echo "  ONNX pass then a TensorRT builder pass; do not interrupt."
            if ! RC_SRC="$resolved" RC_IMGSZ="$imgsz" RC_DEVICE="$device" RC_HALF="$half" \
                 python3 - <<'__RC_COMPILE__'
import os
from ultralytics import YOLO

YOLO(os.environ['RC_SRC'], task='detect').export(
    format='engine',
    imgsz=int(os.environ['RC_IMGSZ']),
    half=os.environ['RC_HALF'] == 'True',
    device=os.environ['RC_DEVICE'],
)
__RC_COMPILE__
            then
                echo "racecar compile: export failed." >&2
                return 1
            fi

            if [[ ! -f "$engine" ]]; then
                echo "racecar compile: export reported success but $engine is missing." >&2
                return 1
            fi

            # Ultralytics writes the engine beside its source; the node looks in
            # models/, so a model compiled from elsewhere gets placed there.
            local models_dir="$pkg_dir/models"
            if [[ "$(dirname "$engine")" != "$models_dir" ]]; then
                cp "$engine" "$models_dir/" && echo "Copied to $models_dir/"
            fi

            echo
            echo "Built $engine"
            echo "To serve it:"
            echo "  1. set model_path: \"$(basename "$engine")\" in config/inference.yaml"
            echo "  2. racecar build"
            echo "  3. racecar service start   (or: racecar launch inference)"
            ;;

        clear)
            local target=""
            for arg in "$@"; do
                case "$arg" in
                    --led) target="led" ;;
                    *) echo "racecar clear: unknown flag '$arg'" >&2; return 2 ;;
                esac
            done
            case "$target" in
                led)
                    ros2 topic pub --once /dotmatrix/text std_msgs/String "{data: ' '}"
                    ;;
                "")
                    echo "usage: racecar clear --led" >&2
                    return 2
                    ;;
            esac
            ;;

        udev)
            bash "$pkg_dir/scripts/setup_udev.sh"
            ;;

        watchdog)
            # Run the watchdog in the foreground. Restarts dead nodes via
            # their individual launch files; logs to ~/logs/latest/watchdog.log.
            # When `neoracer-watchdog.service` is installed, prefer
            # `racecar service start watchdog`.
            python3 "$pkg_dir/scripts/watchdog.py" "$@"
            ;;

        service)
            local action="${1:-status}"
            shift || true
            local -a units=("${_RC_CORE_UNITS[@]}")
            case "$action" in
                install)
                    bash "$pkg_dir/scripts/setup_services.sh"
                    ;;
                start|stop|restart)
                    if [[ -n "$1" ]]; then
                        local unit="neoracer-${1%.service}" u
                        if ! _rc_unit_installed "$unit"; then
                            echo "racecar service: $unit is not installed." >&2
                            echo "Run 'racecar service install' first." >&2
                            return 1
                        fi
                        if [[ "$action" != stop && " ${_RC_DRIVE_UNITS[*]} " == *" $unit "* ]]; then
                            for u in "${_RC_DRIVE_UNITS[@]}"; do
                                [[ "$u" == "$unit" ]] && continue
                                if _rc_unit_running "$u"; then
                                    echo "  stopping $u (only one /drive publisher)"
                                    if ! _rc_systemctl stop "$u"; then
                                        echo "racecar service: $u did not stop, so $unit was not ${action}ed." >&2
                                        return 1
                                    fi
                                fi
                            done
                        fi
                        _rc_systemctl "$action" "$unit"
                    elif [[ "$action" == restart ]]; then
                        # Restart what is already enabled, dashboards included,
                        # and leave every disabled unit where it is.
                        local -a live
                        mapfile -t live < <(_rc_enabled_units)
                        if [[ ${#live[@]} -eq 0 ]]; then
                            echo "No enabled services to restart." >&2
                        else
                            _rc_systemctl restart "${live[@]}" || return
                            printf 'restarted: %s\n' "${live[*]}"
                        fi
                    else
                        _rc_systemctl "$action" "${units[@]}"
                    fi
                    ;;
                enable|disable)
                    # Blanket enable/disable covers the core stack only. A lab
                    # dashboard is turned on by name, so a car that wants one
                    # at boot says so: `racecar service enable camlabel`.
                    if [[ -n "$1" ]]; then
                        sudo systemctl "$action" "neoracer-${1%.service}.service"
                    else
                        for u in "${units[@]}"; do
                            sudo systemctl "$action" "$u.service"
                        done
                    fi
                    ;;
                logs)
                    local unit="${1:-teleop}"
                    sudo journalctl -u "neoracer-${unit%.service}" -f
                    ;;
                status|"")
                    local u
                    for u in "${_RC_CORE_UNITS[@]}"; do
                        _rc_unit_status_line "$u"
                    done
                    echo "  -- autonomy (started by racecar map or the map viewer) --"
                    _rc_unit_status_line neoracer-autonomy
                    echo "  -- lab dashboards (off unless started) --"
                    for u in "${_RC_DASH_UNITS[@]}"; do
                        _rc_unit_status_line "$u"
                    done
                    ;;
                -h|--help|help)
                    cat <<'__RC_SVC_HELP__'
usage: racecar service <action> [unit]
actions:
  install        Install units: core stack enabled at first install,
                 autonomy on demand, lab dashboards disabled
  start [name]   Start neoracer-<name>; no name = the core stack.
                 Starting or restarting a lab dashboard that drives the car
                 stops the others that do first.
  stop [name]    Stop neoracer-<name>; no name = the core stack
  restart [name] Restart neoracer-<name>; no name = every enabled unit,
                 dashboards included, and at most one that drives the car.
                 Never starts a disabled one.
  enable [name]  Enable neoracer-<name>; no name = the core stack
  disable [name] Disable neoracer-<name>; no name = the core stack
  logs [name]    journalctl -u neoracer-<name> -f; default = teleop
  status         active/enabled snapshot for every unit (default)
core units:      teleop, watchdog, dashboard (8080), jupyter (8888),
                 mapview (8088)
lab dashboards:  camlabel (8082), wallfollow (8081), pursuit (8083),
                 eps (8084), smartfollow (8085), linefollow (8086), webteleop (8087)
wallfollow, pursuit, eps, smartfollow, linefollow and webteleop publish
/drive, so only one of them runs at a time. camlabel runs alongside them.

neoracer-autonomy (SLAM on top of teleop) is installed but not enabled.
`racecar map` and the map viewer start and stop it. `racecar launch autonomy`
runs the same launch in a terminal instead.

start, stop and restart need no password (/etc/sudoers.d/neoracer).
enable and disable ask for it.

Dashboards install disabled and are started for a session:
  racecar service start camlabel
Each holds the camera or the GPU while it runs, so start only the ones the
session needs.
__RC_SVC_HELP__
                    ;;
                *)
                    echo "racecar service: unknown action '$action'" >&2
                    return 2
                    ;;
            esac
            ;;

        setup)
            local phase="${1:-}"
            shift || true
            case "$phase" in
                "")
                    echo "usage: racecar setup <phase>" >&2
                    echo "  phases: all, dashboards, ml, networking" >&2
                    return 2
                    ;;
                all)
                    bash "$pkg_dir/scripts/setup_all.sh" "$@"
                    ;;
                dashboards)
                    # Installs the seven lab dashboards' units, stopped and
                    # disabled; setup_services.sh also runs it.
                    bash "$pkg_dir/scripts/setup_dashboards.sh" "$@"
                    ;;
                ml|gpu)
                    bash "$pkg_dir/scripts/setup_ml.sh" "$@"
                    ;;
                networking)
                    # Persist any --flag values to ~/.config/racecar/networking.env
                    # so subsequent runs (and reboots) keep the same settings.
                    local cfg_dir="$HOME/.config/racecar"
                    local cfg_file="$cfg_dir/networking.env"
                    mkdir -p "$cfg_dir"

                    # Load existing persisted values into a local assoc array.
                    local -A vals=()
                    if [[ -f "$cfg_file" ]]; then
                        while IFS='=' read -r k v; do
                            [[ -z "$k" || "$k" =~ ^# ]] && continue
                            v="${v%\"}"; v="${v#\"}"
                            vals["$k"]="$v"
                        done < "$cfg_file"
                    fi

                    # Pass 1: parse every flag into either vals[] (overrides)
                    # or an action variable. Don't act yet - that way `--ssid=foo
                    # --show` persists foo before showing, and `--ssid=foo --reset`
                    # is rejected as nonsense (would be lost immediately).
                    local action="apply"   # default: persist + run setup_networking.sh
                    local vals_changed=0
                    while [[ "$1" == --* ]]; do
                        local arg="$1"; shift
                        local key val
                        if [[ "$arg" == *=* ]]; then
                            key="${arg%%=*}"; val="${arg#*=}"
                        else
                            key="$arg"; val="${1:-}"; shift || true
                        fi
                        case "$key" in
                            --ssid)        vals[RACECAR_AP_SSID]="$val";    vals_changed=1 ;;
                            --psk)         vals[RACECAR_AP_PSK]="$val";     vals_changed=1 ;;
                            --channel)     vals[RACECAR_AP_CHANNEL]="$val"; vals_changed=1 ;;
                            --ap-addr)     vals[RACECAR_AP_ADDR]="$val";    vals_changed=1 ;;
                            --eth-static)  vals[RACECAR_ETH_STATIC]="$val"; vals_changed=1 ;;
                            --lidar-host)  vals[RACECAR_LIDAR_HOST]="$val"; vals_changed=1 ;;
                            --wifi-iface)  vals[RACECAR_WIFI_IFACE]="$val"; vals_changed=1 ;;
                            --eth-iface)   vals[RACECAR_ETH_IFACE]="$val";  vals_changed=1 ;;
                            --show)        action="show" ;;
                            --reset)       action="reset" ;;
                            --help|-h)     action="help" ;;
                            *)
                                echo "racecar setup networking: unknown flag '$key'" >&2
                                return 2
                                ;;
                        esac
                    done

                    if [[ "$action" == "help" ]]; then
                        cat <<'__RC_NET_HELP__'
usage: racecar setup networking [--ssid=NAME] [--psk=PASS]
                                [--channel=N] [--ap-addr=CIDR]
                                [--eth-static=CIDR] [--lidar-host=CIDR]
                                [--wifi-iface=NAME] [--eth-iface=NAME]
                                [--show] [--reset]
Persists any --flag values to ~/.config/racecar/networking.env and then runs
scripts/setup_networking.sh (Wi-Fi AP + Ethernet dual-IP + Lakibeam subnet).
  --show   print current persisted overrides and exit
  --reset  delete the saved settings and the access point connection
Persistence runs BEFORE --show or --reset, so
  racecar setup networking --ssid=foo --show
saves foo, then prints the new file contents.
WARNING: this reconfigures Wi-Fi. If you're SSH'd over Wi-Fi, run from a
wired session or the console instead.
__RC_NET_HELP__
                        return 0
                    fi

                    if [[ "$action" == "reset" ]]; then
                        if [[ $vals_changed -eq 1 ]]; then
                            echo "racecar setup networking: --reset cannot be combined with override flags" >&2
                            return 2
                        fi
                        # The access point connection keeps this car's SSID
                        # and stays up until it is deleted. Its name is
                        # AP_CON_NAME in setup_networking.sh. The settings
                        # file goes first, because deleting the access point
                        # can end an SSH session over Wi-Fi.
                        local ap_con="neoracer-ap" con_names
                        echo "WARNING: this reconfigures Wi-Fi. If you're SSH'd over Wi-Fi, run from a"
                        echo "wired session or the console instead."
                        rm -f "$cfg_file"
                        if ! con_names="$(nmcli -t -f NAME connection show 2>&1)"; then
                            echo "$con_names" >&2
                            echo "racecar setup networking: removed $cfg_file, but nmcli cannot list the connections" >&2
                            return 1
                        fi
                        if grep -qxF "$ap_con" <<<"$con_names"; then
                            if ! sudo nmcli connection delete "$ap_con"; then
                                echo "racecar setup networking: removed $cfg_file, but the access point connection $ap_con is still there" >&2
                                return 1
                            fi
                            echo "Removed the saved settings ($cfg_file) and the access point connection $ap_con."
                        else
                            echo "Removed the saved settings ($cfg_file). The access point connection $ap_con does not exist."
                        fi
                        echo "The next 'racecar setup networking' brings the access point back with the default SSID."
                        return 0
                    fi

                    # Persist any new --flag values (applies to apply/show paths).
                    if [[ $vals_changed -eq 1 ]]; then
                        : > "$cfg_file"
                        chmod 600 "$cfg_file"
                        echo "# racecar networking overrides - managed by 'racecar setup networking'" >> "$cfg_file"
                        for k in RACECAR_AP_SSID RACECAR_AP_PSK RACECAR_AP_CHANNEL RACECAR_AP_ADDR RACECAR_ETH_STATIC \
                                 RACECAR_LIDAR_HOST RACECAR_WIFI_IFACE RACECAR_ETH_IFACE; do
                            if [[ -n "${vals[$k]:-}" ]]; then
                                printf '%s="%s"\n' "$k" "${vals[$k]}" >> "$cfg_file"
                            fi
                        done
                        echo "Saved overrides to $cfg_file"
                    fi

                    if [[ "$action" == "show" ]]; then
                        if [[ -s "$cfg_file" ]]; then
                            echo "Persisted networking config ($cfg_file):"
                            cat "$cfg_file"
                        else
                            echo "No persisted networking config - script defaults will apply."
                        fi
                        return 0
                    fi

                    bash "$pkg_dir/scripts/setup_networking.sh"
                    ;;
                *)
                    echo "racecar setup: unknown phase '$phase'" >&2
                    echo "  phases: all, dashboards, ml, networking" >&2
                    return 2
                    ;;
            esac
            ;;

        library)
            # Manage which ~/jupyter_ws/<folder>/library/ is on Python's sys.path
            # by writing a .pth file into the user site-packages directory.
            # Mirrors what the sim installer does inside its venv (see
            # racecar-neo-installer setup.sh), but uses the user-site dir since
            # the Pi has no venv.
            local jws="$HOME/jupyter_ws"
            local site_pkgs
            site_pkgs=$(python3 -c 'import site; print(site.getusersitepackages())' 2>/dev/null)
            local pth_file="$site_pkgs/racecar_student.pth"

            local action=""
            local select_target=""
            while [[ $# -gt 0 ]]; do
                local arg="$1"; shift
                case "$arg" in
                    --select)
                        action="select"
                        select_target="${1:-}"
                        [[ -n "$select_target" ]] && shift
                        ;;
                    --select=*)
                        action="select"
                        select_target="${arg#*=}"
                        ;;
                    --list)    action="list" ;;
                    --reset)   action="reset" ;;
                    --status)  action="status" ;;
                    --help|-h) action="help" ;;
                    *)
                        echo "racecar library: unknown flag '$arg'" >&2
                        return 2
                        ;;
                esac
            done

            if [[ -z "$action" ]]; then
                echo "usage: racecar library [--select <folder> | --list | --reset | --status]" >&2
                return 2
            fi

            if [[ "$action" == "help" ]]; then
                cat <<'__RC_LIB_HELP__'
usage: racecar library <action>
Manages the racecar_student.pth file in user site-packages so Python scripts
(e.g. labs/demo.py) can `import racecar_core` without sys.path hacks.

Actions:
  --select <folder>  Point the .pth at ~/jupyter_ws/<folder>/library/.
                     <folder> must contain library/racecar_core.py.
  --list             List all ~/jupyter_ws/ folders that look like valid
                     student libraries (contain library/racecar_core.py).
                     The currently-selected folder is marked with *.
  --reset            Delete the .pth file (no library on sys.path).
  --status           Show the currently-selected library, or report none.

The .pth is written to:
  ~/.local/lib/pythonX.Y/site-packages/racecar_student.pth
__RC_LIB_HELP__
                return 0
            fi

            if [[ -z "$site_pkgs" ]]; then
                echo "racecar library: could not determine user site-packages directory" >&2
                return 1
            fi

            case "$action" in
                list)
                    if [[ ! -d "$jws" ]]; then
                        echo "No ~/jupyter_ws/ directory found."
                        return 0
                    fi
                    local current=""
                    if [[ -f "$pth_file" ]]; then
                        current=$(head -n 1 "$pth_file")
                    fi
                    local found=0
                    echo "Available libraries in $jws:"
                    local d
                    for d in "$jws"/*/; do
                        [[ -d "$d" ]] || continue
                        local libdir="${d}library"
                        if [[ -f "$libdir/racecar_core.py" ]]; then
                            local name
                            name=$(basename "$d")
                            local marker="  "
                            if [[ "$current" == "${libdir%/}" ]]; then
                                marker=" *"
                            fi
                            printf "%s %s\n" "$marker" "$name"
                            found=1
                        fi
                    done
                    if [[ $found -eq 0 ]]; then
                        echo "  (none - no folder contains library/racecar_core.py)"
                    fi
                    ;;

                select)
                    if [[ -z "$select_target" ]]; then
                        echo "racecar library: --select requires a folder name" >&2
                        echo "  (run 'racecar library --list' to see candidates)" >&2
                        return 2
                    fi
                    local target_dir="$jws/$select_target"
                    local target_lib="$target_dir/library"
                    if [[ ! -d "$target_dir" ]]; then
                        echo "racecar library: '$select_target' is not a folder under $jws" >&2
                        return 2
                    fi
                    if [[ ! -f "$target_lib/racecar_core.py" ]]; then
                        echo "racecar library: '$target_lib/racecar_core.py' not found" >&2
                        echo "  (folder must contain library/racecar_core.py)" >&2
                        return 2
                    fi
                    mkdir -p "$site_pkgs"
                    echo "$target_lib" > "$pth_file"
                    echo "Selected library: $target_lib"
                    echo "  wrote $pth_file"
                    ;;

                reset)
                    if [[ -f "$pth_file" ]]; then
                        rm -f "$pth_file"
                        echo "Reset: removed $pth_file"
                    else
                        echo "Reset: no .pth file to remove ($pth_file)"
                    fi
                    ;;

                status)
                    if [[ -f "$pth_file" ]]; then
                        local current
                        current=$(head -n 1 "$pth_file")
                        echo "Current library: $current"
                        echo "  ($pth_file)"
                        if [[ ! -f "$current/racecar_core.py" ]]; then
                            echo "  WARNING: racecar_core.py not found at this path" >&2
                        fi
                    else
                        echo "No racecar library is currently selected."
                        echo "  Run: racecar library --select <folder>"
                        echo "  (or: racecar library --list)"
                    fi
                    ;;
            esac
            ;;

        cleanup)
            # Find orphaned/stale racecar processes + FastRTPS SHM segments.
            # Dry-run by default; pass --force to actually kill / remove.
            local force=0
            for arg in "$@"; do
                case "$arg" in
                    -f|--force) force=1 ;;
                    -n|--dry-run) force=0 ;;
                    -h|--help)
                        cat <<'__RC_CLEANUP_HELP__'
usage: racecar cleanup [--dry-run | --force]
  Lists racecar processes and FastRTPS SHM orphans. Default is --dry-run.
  --force kills processes (uses sudo for root-owned ones) and removes SHM.
__RC_CLEANUP_HELP__
                        return 0
                        ;;
                    *) echo "racecar cleanup: unknown flag '$arg'" >&2; return 2 ;;
                esac
            done

            # ----- Process inventory -----
            # Match any process whose cmdline mentions the racecar stack.
            local pattern='neoracer_ros2_driver|lakibeam1_scan_node|ros2 launch neoracer|ros2 launch racecar|sg dialout.*racecar'
            local matches
            matches=$(ps -eo pid,user,cmd --no-headers | grep -E "$pattern" | grep -v 'grep\|racecar cleanup' || true)

            if [[ -z "$matches" ]]; then
                echo "No racecar processes running."
            else
                echo "=== Racecar processes ==="
                echo "$matches" | awk '{printf "  pid=%-6s user=%-8s cmd=%s\n", $1, $2, substr($0, index($0,$3))}' | head -30
                local user_pids root_pids
                user_pids=$(echo "$matches" | awk -v u="$USER" '$2 == u {print $1}' | tr '\n' ' ')
                root_pids=$(echo "$matches" | awk '$2 == "root" {print $1}' | tr '\n' ' ')
                if [[ $force -eq 1 ]]; then
                    if [[ -n "$user_pids" ]]; then
                        echo "Killing user-owned: $user_pids"
                        # shellcheck disable=SC2086
                        kill -9 $user_pids 2>/dev/null || true
                    fi
                    if [[ -n "$root_pids" ]]; then
                        echo "Killing root-owned (sudo): $root_pids"
                        # shellcheck disable=SC2086
                        sudo kill -9 $root_pids 2>/dev/null || \
                            echo "  (sudo failed; run as your user: sudo kill -9 $root_pids)"
                    fi
                else
                    echo "(dry-run; pass --force to kill)"
                fi
            fi

            # ----- FastRTPS SHM orphans -----
            local shm_orphans=()
            local shm_locks=()
            for f in /dev/shm/fastrtps_port*; do
                [ -e "$f" ] || continue
                case "$f" in *_el) continue ;; esac
                # Orphan = zero-byte data segment with no live participant.
                if [ ! -s "$f" ]; then
                    shm_orphans+=("$f")
                fi
            done
            for el in /dev/shm/fastrtps_port*_el; do
                [ -e "$el" ] || continue
                local data="${el%_el}"
                # Orphan = lock segment whose data peer is gone.
                if [ ! -e "$data" ]; then
                    shm_locks+=("$el")
                fi
            done

            echo
            if [[ ${#shm_orphans[@]} -eq 0 && ${#shm_locks[@]} -eq 0 ]]; then
                echo "No FastRTPS SHM orphans in /dev/shm."
            else
                echo "=== FastRTPS SHM orphans ==="
                for f in "${shm_orphans[@]}"; do
                    echo "  zero-byte: $f"
                done
                for el in "${shm_locks[@]}"; do
                    echo "  stale lock: $el"
                done
                if [[ $force -eq 1 ]]; then
                    for f in "${shm_orphans[@]}"; do
                        local base
                        base=$(basename "$f")
                        rm -f "$f" "/dev/shm/${base}_el" "/dev/shm/sem.${base}_mutex"
                    done
                    for el in "${shm_locks[@]}"; do
                        local base
                        base=$(basename "${el%_el}")
                        rm -f "$el" "/dev/shm/sem.${base}_mutex"
                    done
                    echo "Removed."
                else
                    echo "(dry-run; pass --force to remove)"
                fi
            fi
            ;;

        selftest)
            local target=""
            local text="NEORACER"
            for arg in "$@"; do
                case "$arg" in
                    --led) target="led" ;;
                    --led=*) target="led"; text="${arg#*=}" ;;
                    *) echo "racecar selftest: unknown flag '$arg'" >&2; return 2 ;;
                esac
            done
            case "$target" in
                led)
                    # Faster than `ros2 node list` (which hangs ~15s when no
                    # daemon is running). Look for the installed entry-point.
                    if ! pgrep -f 'neoracer_ros2_driver/lib/.*led_matrix' >/dev/null; then
                        echo "racecar selftest: led_matrix_node is not running." >&2
                        echo "Start it first: racecar launch led_matrix" >&2
                        return 3
                    fi
                    ros2 topic pub --once /dotmatrix/text std_msgs/String "{data: '$text'}"
                    echo "Sent '$text' to /dotmatrix/text."
                    ;;
                "")
                    cat <<'__RC_SELFTEST_HELP__' >&2
usage: racecar selftest --led[=<text>]
sends test text to the 8x8 display (requires: racecar launch led_matrix)
__RC_SELFTEST_HELP__
                    return 2
                    ;;
            esac
            ;;

        status)
            # Whole-car diagnostic. Read-only: it observes topics and reads
            # sysfs, and never commands the hardware. Exits non-zero when a
            # check fails (--strict: unless every check passed), so it is
            # usable from a script.
            python3 "${RACECAR_DIAGNOSE_SCRIPT:-$pkg_dir/scripts/diagnose.py}" "$@"
            ;;

        help|-h|--help|"")
            cat <<'__RC_HELP__'
racecar - Neoracer developer tool

Usage:
    racecar <command> [args]

Commands:
    build               Build neoracer_ros2_driver (--symlink-install) and source overlay.
    test                Run the package test suite with verbose results.
    lint                Run ruff, black --check and mypy on the package.
    profile [action]    This car's settings, kept in ~/.config/racecar so a
                        driver update or re-clone keeps the calibration.
                          (none)        path, car name, files and dates
                          path          the profile folder
                          migrate       move config/*.local.yaml and dashboard
                                        tuning out of the driver folder
                          export FILE   write the profile to FILE (.tar.gz)
                          import FILE   read a profile written by export
    data                Where the car's data goes (the data root): ~/neoracer_data,
                        outside the folders the installer removes. Programs
                        read it from $RACECAR_DATA_DIR.
    log <action>        Record and analyze ROS 2 bags. Actions:
                          start [name]        record; bag is <timestamp>_<name>
                                                --topics A B    default: all
                                                --exclude REGEX
                                                --dir PATH      default: bags/ under
                                                                the data root
                                                --name NAME     full bag name
                                                --storage mcap|sqlite3
                                                --force         record anyway when
                                                                the rate outruns the disk
                          stop                SIGINT, so rosbag2 finalizes the bag
                          status [--json]     size, rate, elapsed, time until full
                          list                recorded bags, newest first
                          analyze [bag]       duration, size, per-topic counts and
                                              rates; defaults to the newest bag
                          config              show or set persisted defaults
                        Every topic together is about 4.5 MB/s, most of it the
                        camera.
    calibrate [steps]   Calibrate the car in a guided session. Steps:
                          lidar       lidar yaw from the direction of travel;
                                      the car drives itself
                          odom        gyro yaw scale (circles) and encoder
                                      speed scale (straight drive), both
                                      against the lidar; you drive the car
                                      with the handset, mode switch UP
                                      (manual)
                          steering    steering trim, set in the ESP32
                                      firmware (trim set), with a copy in the
                                      car profile; the car drives itself
                          dynamics    steering angles, servo time, speeds
                                      and stop distances for the simulator;
                                      the car drives itself; no check
                        A step that drives the car by itself drives only
                        while the handset's mode switch is DOWN (autonomy).
                        Start with the switch UP and flip it DOWN when the
                        step asks. Flip it UP to stop. Turning the handset
                        off does not stop the car.
                        No arguments: all but dynamics.
                        `racecar calibrate status` lists what is calibrated
                        on this car, when, and the stored values.
                        `racecar calibrate check [steps]` verifies each
                        calibration against the live sensors.
                        To prepare a car: calibrate, restart teleop, then
                        run the check.
    source              Source the workspace overlay into the current shell.
    ws [name]           Switch this shell between workspaces: neoracer or
                        osracer (the vendor stack preinstalled in ~/osracer_ws).
                        Both carry a lakibeam1 package, so one is active per
                        shell; new shells default to neoracer. `racecar ws`
                        shows the active workspace.
    cd                  Change directory to the neoracer_ros2_driver package root.
    teleop              Launch the full teleop stack via launch_teleop.sh wrapper
                        (timestamped ~/logs/<ts>/ + FastRTPS SHM cleanup).
                        Forwards args, e.g. `racecar teleop camera_enable:=false`.
    map <action>        Build, save, and load maps (SLAM on top of teleop).
                        Teleop must be running; drive with the handset as
                        usual. Maps are in maps/ under the data root.
                          start NAME          start mapping; needs the handset's
                                              speed switch UP (slow), and /drive
                                              is held to the mapping cap
                                                --from SAVED    extend SAVED
                                                --pose x,y,yaw  with --from: the
                                                                car's pose on SAVED
                                                --force         replace NAME (the
                                                                old one is kept)
                          save                save the map; waits for the files
                          stop                stop (a mapping run is autosaved)
                          resume              start the last run again; a
                                              mapping run needs the speed
                                              switch UP
                          load NAME           localize on NAME; starts from the
                                              car's last pose on it
                                                --pose x,y,yaw  start pose
                                                --global        search the map
                                                --localizer amcl|slam_toolbox
                          recover NAME        keep an autosave as the saved map
                          status              mode, map, localized, pose
                          list                maps; unsaved ones marked
                          info NAME           size, dates, poses, frames
                          foxglove on|off     foxglove_bridge (port 8765) at the
                                              next start or load
                        Poses are in the map frame: meters, meters, radians.
    update [--labs]     Field update in one command: repo to latest origin/main
                        (discards local repo edits), full `setup all`, restart
                        the enabled services. Needs internet. The labs in
                        ~/jupyter_ws/neoracer-os/labs stay as they are.
                          --labs   refresh the labs too; setup first copies
                                   the labs folder to labs-backup-<date>-<time>
                                   beside it
    launch <name>       Shortcut for `ros2 launch neoracer_ros2_driver <name>.launch.py`.
                        Examples: racecar launch lidar
                                  racecar launch camera
                                  racecar launch led_matrix
    compile [model]     Export a YOLO .pt to a TensorRT .engine for inference_node.
                        With no model, uses model_path and imgsz from
                        config/inference.yaml. A bare name resolves in models/.
                          --imgsz=N    override the export size (default: config)
                          --device=N   CUDA index (default 0)
                          --no-half    export FP32 instead of FP16
                          --force      rebuild over an existing engine; skip the
                                       stack-is-running guard
                        Peaks near 3 GB; stop the stack first. ~8 min at 640.
    clear --led         Clear the 8x8 LED matrix display.
    udev                Re-install the udev rules (refreshes /dev/osrbot_* etc.).
    watchdog            Run the node watchdog (restart-on-failure supervisor).
                        Monitors control + sensor nodes; logs to
                        ~/logs/latest/watchdog.log. Assumes teleop runs separately.
    setup <phase>       Run a setup script. Phases:
                          all          - setup_all.sh (the 8-phase orchestrator)
                          dashboards   - install the lab dashboards' units
                                         (stopped, disabled). Skip with
                                         RACECAR_DASHBOARDS=0.
                          ml           - GPU stack: PyTorch/torchvision for Tegra,
                                         Ultralytics, ONNX export tooling; verifies
                                         the JetPack TensorRT bindings.
                          networking   - Wi-Fi AP + Lakibeam lidar subnet.
                                         Flags persist to ~/.config/racecar/networking.env:
                                           --ssid=NAME   --psk=PASS   --channel=N
                                           --ap-addr=CIDR (default 10.42.0.1/24)
                                           --eth-static=CIDR (default 192.168.10.100/24)
                                           --lidar-host=CIDR (default 192.168.8.1/24)
                                           --show / --reset
    service <action>    systemd service control. Actions:
                          install              setup_services.sh (core enabled
                                               at first install, dashboards
                                               installed disabled)
                          start [name]         no name: the core stack
                          stop [name]          no name: the core stack
                          restart [name]       no name: every enabled unit;
                                               never starts a disabled one
                          enable|disable [name]  no name: the core stack
                          logs [name]          journalctl -f for neoracer-<name>
                          status               active/enabled summary (default)
                        Core units: teleop, watchdog, dashboard (8080),
                          jupyter (8888), mapview (8088)
                        Lab dashboards, off until started. One that drives
                        the car runs at a time, and camlabel runs beside it:
                          camlabel (8082), wallfollow (8081), pursuit (8083),
                          eps (8084), smartfollow (8085), linefollow (8086), webteleop (8087)
                        neoracer-autonomy: SLAM on top of teleop, installed
                        but not enabled; `racecar map` and the map viewer
                        start and stop it.
    library <action>    Manage racecar_student.pth in user site-packages.
                          --select <folder>   point at ~/jupyter_ws/<folder>/library
                          --list              show valid folders in ~/jupyter_ws
                          --reset             delete the .pth file
                          --status            show current selection
    cleanup             List orphaned racecar processes + FastRTPS SHM segments.
                        Defaults to a dry-run. Pass --force to actually kill/remove
                        (uses sudo for root-owned PIDs).
    selftest            Hardware self-tests. Currently supported:
                          racecar selftest --led            (sends test text)
                          racecar selftest --led=HELLO      (sends custom text)
                        Requires led_matrix_node running (racecar launch led_matrix).
    status [flags]      Whole-car diagnostic: devices, sensors, actuators,
                        autonomy, system, services and network in one pass.
                        Read-only. Exits non-zero when a check fails; WARN
                        still exits 0.
                          --quick             skip the ROS sampling phase
                          --json              machine-readable output
                          --strict            also exit non-zero on WARN/SKIP
                          --section a,b       devices, sensors, actuators,
                                              autonomy, system, services,
                                              network
                          --window SEC        ROS sample window (default 5.0)
    help                Show this message.

Extra args are forwarded:
    racecar build --cmake-args -DCMAKE_BUILD_TYPE=Release
    racecar launch camera camera_config:=/tmp/custom.yaml
__RC_HELP__
            ;;

        *)
            echo "racecar: unknown command '$cmd'. Try 'racecar help'." >&2
            return 2
            ;;
    esac
}

# Bash completion: subcommands at position 1, launch-file names after `launch`,
# weights in models/ after `compile`, `--led` after `clear`.
_racecar_complete() {
    local cur="${COMP_WORDS[COMP_CWORD]}"
    local prev="${COMP_WORDS[COMP_CWORD-1]}"
    local sub="${COMP_WORDS[1]:-}"

    if [[ $COMP_CWORD -eq 1 ]]; then
        COMPREPLY=( $(compgen -W "build test profile data log calibrate lint source ws cd teleop map launch compile clear udev watchdog service setup update library cleanup selftest status help" -- "$cur") )
        return
    fi

    case "$sub" in
        calibrate)
            COMPREPLY=( $(compgen -W "status check lidar odom steering dynamics --help" -- "$cur") )
            ;;
        status)
            COMPREPLY=( $(compgen -W "--quick --json --strict --section --window --help" -- "$cur") )
            ;;
        profile)
            if [[ $COMP_CWORD -eq 2 ]]; then
                COMPREPLY=( $(compgen -W "path migrate export import" -- "$cur") )
            else
                COMPREPLY=( $(compgen -f -- "$cur") )
            fi
            ;;
        launch)
            local launch_dir="$HOME/ros2_ws/src/neoracer_ros2_driver/launch"
            if [[ -d "$launch_dir" ]]; then
                local names
                names=$(cd "$launch_dir" && ls *.launch.py 2>/dev/null | sed 's/\.launch\.py$//')
                COMPREPLY=( $(compgen -W "$names" -- "$cur") )
            fi
            ;;
        compile)
            local models_dir="$HOME/ros2_ws/src/neoracer_ros2_driver/models"
            local weights=""
            if [[ -d "$models_dir" ]]; then
                weights=$(cd "$models_dir" && ls *.pt 2>/dev/null)
            fi
            COMPREPLY=( $(compgen -W "$weights --imgsz= --device= --half --no-half --force --help" -- "$cur") )
            ;;
        ws)
            COMPREPLY=( $(compgen -W "neoracer osracer status" -- "$cur") )
            ;;
        clear)
            COMPREPLY=( $(compgen -W "--led" -- "$cur") )
            ;;
        cleanup)
            COMPREPLY=( $(compgen -W "--dry-run --force --help" -- "$cur") )
            ;;
        update)
            COMPREPLY=( $(compgen -W "--labs" -- "$cur") )
            ;;
        library)
            if [[ "$cur" == --select=* || "$prev" == "--select" ]]; then
                # Complete with folder names under ~/jupyter_ws that contain library/racecar_core.py.
                local jws="$HOME/jupyter_ws"
                local candidates=""
                if [[ -d "$jws" ]]; then
                    local d
                    for d in "$jws"/*/; do
                        [[ -d "$d" ]] || continue
                        if [[ -f "${d}library/racecar_core.py" ]]; then
                            candidates+="$(basename "$d") "
                        fi
                    done
                fi
                if [[ "$cur" == --select=* ]]; then
                    local prefix="${cur#--select=}"
                    COMPREPLY=( $(compgen -W "$candidates" -- "$prefix") )
                else
                    COMPREPLY=( $(compgen -W "$candidates" -- "$cur") )
                fi
            else
                COMPREPLY=( $(compgen -W "--select --select= --list --reset --status --help" -- "$cur") )
            fi
            ;;
        setup)
            if [[ $COMP_CWORD -eq 2 ]]; then
                COMPREPLY=( $(compgen -W "all dashboards ml networking" -- "$cur") )
            elif [[ "${COMP_WORDS[2]}" == "networking" ]]; then
                COMPREPLY=( $(compgen -W "--ssid= --psk= --channel= --ap-addr= --eth-static= --lidar-host= --wifi-iface= --eth-iface= --show --reset --help" -- "$cur") )
            elif [[ "${COMP_WORDS[2]}" == "dashboards" ]]; then
                COMPREPLY=( $(compgen -W "--help" -- "$cur") )
            fi
            ;;
        map)
            local maps_dir="${RACECAR_MAP_ROOT:-${RACECAR_DATA_DIR:-$HOME/neoracer_data}/maps}"
            local maps=""
            if [[ -d "$maps_dir" ]]; then
                maps=$(cd "$maps_dir" && ls -d */ 2>/dev/null | sed 's#/$##' | grep -vx replaced)
            fi
            if [[ $COMP_CWORD -eq 2 ]]; then
                COMPREPLY=( $(compgen -W "start save stop resume load recover status list info foxglove" -- "$cur") )
            elif [[ "$prev" == "--from" ]]; then
                COMPREPLY=( $(compgen -W "$maps" -- "$cur") )
            elif [[ "$prev" == "--localizer" ]]; then
                COMPREPLY=( $(compgen -W "amcl slam_toolbox" -- "$cur") )
            else
                case "${COMP_WORDS[2]}" in
                    start)
                        COMPREPLY=( $(compgen -W "--from --pose --force" -- "$cur") )
                        ;;
                    load)
                        if [[ $COMP_CWORD -eq 3 ]]; then
                            COMPREPLY=( $(compgen -W "$maps" -- "$cur") )
                        else
                            COMPREPLY=( $(compgen -W "--pose --global --localizer" -- "$cur") )
                        fi
                        ;;
                    recover|info)
                        COMPREPLY=( $(compgen -W "$maps" -- "$cur") )
                        ;;
                    foxglove)
                        COMPREPLY=( $(compgen -W "on off" -- "$cur") )
                        ;;
                esac
            fi
            ;;
        log)
            if [[ $COMP_CWORD -eq 2 ]]; then
                COMPREPLY=( $(compgen -W "start stop status list analyze config help" -- "$cur") )
            else
                case "${COMP_WORDS[2]}" in
                    start)
                        COMPREPLY=( $(compgen -W "--topics --exclude --dir --name --storage --force --help" -- "$cur") )
                        ;;
                    stop)
                        COMPREPLY=( $(compgen -W "--timeout --help" -- "$cur") )
                        ;;
                    status|analyze)
                        COMPREPLY=( $(compgen -W "--json --help" -- "$cur") )
                        ;;
                    config)
                        COMPREPLY=( $(compgen -W "--topics --dir --storage --reset --help" -- "$cur") )
                        ;;
                    list)
                        COMPREPLY=( $(compgen -W "--dir --help" -- "$cur") )
                        ;;
                esac
            fi
            ;;
        service)
            if [[ $COMP_CWORD -eq 2 ]]; then
                COMPREPLY=( $(compgen -W "install start stop restart enable disable logs status help" -- "$cur") )
            elif [[ $COMP_CWORD -eq 3 ]]; then
                local action="${COMP_WORDS[2]}"
                case "$action" in
                    start|stop|restart|logs|enable|disable)
                        COMPREPLY=( $(compgen -W "teleop watchdog dashboard jupyter mapview autonomy camlabel wallfollow pursuit eps smartfollow linefollow webteleop" -- "$cur") )
                        ;;
                esac
            fi
            ;;
        selftest)
            COMPREPLY=( $(compgen -W "--led --led=" -- "$cur") )
            ;;
    esac
}
complete -F _racecar_complete racecar

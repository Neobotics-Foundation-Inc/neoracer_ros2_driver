# Neoracer shell tool - `racecar <subcommand>`.
# Sourced from ~/.bashrc by setup_user_env.sh.
# Not executed directly: it defines a `racecar` shell function so the build /
# test / source subcommands can mutate the current shell (PWD, env).

_rc_autonomy_env() {
    # Chained env for the osracer layers: underlay beneath the neoracer
    # overlay, same order as launch_autonomy.sh. Run inside a subshell so
    # the caller's interactive shell keeps its own environment.
    if [[ ! -f "$HOME/osracer_ws/install/setup.bash" ]]; then
        echo "osracer workspace not found at ~/osracer_ws" >&2
        return 1
    fi
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
    # shellcheck disable=SC1090
    source "$HOME/osracer_ws/install/setup.bash"
    # shellcheck disable=SC1090
    [[ -f "$HOME/ros2_ws/install/setup.bash" ]] && source "$HOME/ros2_ws/install/setup.bash"
    systemctl is-active --quiet neoracer-teleop 2>/dev/null || \
        echo "warning: teleop service not active; /scan and /odom are missing" >&2
    # The autonomy base ran as neoracer-autonomy.service until that unit was
    # held; until it returns, the base is started in a terminal of its own.
    pgrep -f launch_autonomy.sh >/dev/null 2>&1 || \
        echo "warning: autonomy base not running (TF + bridge); start it with:" \
             "bash ~/ros2_ws/src/neoracer_ros2_driver/scripts/launch_autonomy.sh" >&2
    return 0
}

# Core stack, enabled at boot by setup_services.sh. neoracer-autonomy is not
# here: the unit is held and setup no longer installs it.
_RC_CORE_UNITS=(neoracer-teleop neoracer-watchdog neoracer-dashboard
                neoracer-jupyter)

# Lab dashboards, installed disabled and started per session. Each holds the
# camera or the GPU for its whole run, so they are opt-in rather than part of
# the stack that comes up on boot.
_RC_DASH_UNITS=(neoracer-camlabel neoracer-wallfollow neoracer-pursuit
                neoracer-eps neoracer-smartfollow neoracer-linefollow
                neoracer-webteleop)

# Units that would come back on their own after a reboot. `restart` with no
# unit named uses this so it never promotes a disabled service into a running
# one; systemctl restart starts a stopped unit, which would otherwise put a
# lab dashboard on the graph behind the student's back.
_rc_enabled_units() {
    local u
    for u in "${_RC_CORE_UNITS[@]}" "${_RC_DASH_UNITS[@]}"; do
        systemctl is-enabled --quiet "$u" 2>/dev/null && echo "$u"
    done
}

_rc_unit_installed() {
    systemctl list-unit-files "$1.service" 2>/dev/null | grep -q "^$1.service"
}

# One status line. An absent unit reports as such rather than passing
# systemd's "Failed to get unit file state" through to the column.
_rc_unit_status_line() {
    local u="$1" state enabled
    if ! _rc_unit_installed "$u"; then
        printf "  %-22s  not installed\n" "$u"
        return
    fi
    state=$(systemctl is-active "$u" 2>&1 || true)
    enabled=$(systemctl is-enabled "$u" 2>&1 || true)
    printf "  %-22s  active=%-12s enabled=%s\n" "$u" "$state" "$enabled"
}

racecar() {
    local pkg="neoracer_ros2_driver"
    local ws="$HOME/ros2_ws"
    local pkg_dir="$ws/src/$pkg"
    local cmd="${1:-help}"
    shift || true

    case "$cmd" in
        build)
            ( cd "$ws" && colcon build --packages-select "$pkg" --symlink-install "$@" ) \
                && source "$ws/install/setup.bash"
            ;;

        test)
            ( cd "$ws" \
                && colcon test --packages-select "$pkg" --event-handlers console_direct+ "$@" \
                && colcon test-result --verbose )
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

        mapping)
            # On-demand SLAM. Mapping is an activity, not a daemon: run it
            # while building a map, Ctrl-C when done. Needs teleop (for /scan
            # and /odom) and the autonomy base service (TF + bridge).
            local sub="${1:-slam_toolbox}"
            shift || true
            local maps_dir="$HOME/osracer_ws/src/osracer/osracer_slam/maps"
            [[ "$sub" == "run" ]] && sub="slam_toolbox"
            case "$sub" in
                slam_toolbox|gmapping|cartographer)
                    (
                        _rc_autonomy_env || exit 1
                        echo "SLAM up ($sub). Drive under RC; Ctrl-C when the map is complete."
                        echo "Save from another terminal: racecar mapping save <name>"
                        exec ros2 launch osracer_slam "$sub.launch.py" "$@"
                    )
                    ;;
                save)
                    local name="${1:-map}"
                    (
                        _rc_autonomy_env || exit 1
                        mkdir -p "$maps_dir"
                        ros2 run nav2_map_server map_saver_cli -f "$maps_dir/$name" && \
                            echo "Saved: $maps_dir/$name.yaml  (drive it with: racecar navigation $name)"
                    )
                    ;;
                rviz)
                    # The vendor's pre-configured mapping RViz. Needs a display:
                    # run from the car desktop (monitor or RustDesk), not SSH.
                    # Cartographer has its own debug view: racecar mapping rviz cartographer
                    local view="debug_mapping"
                    [[ "${1:-}" == "cartographer" ]] && view="debug_cartographer"
                    (
                        _rc_autonomy_env || exit 1
                        if [[ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]]; then
                            echo "racecar mapping rviz needs a display; run it from the car desktop (RustDesk or monitor)" >&2
                            exit 1
                        fi
                        exec ros2 launch osracer_debug "$view.launch.py"
                    )
                    ;;
                -h|--help|help)
                    echo "usage: racecar mapping [backend]     start SLAM: slam_toolbox (default), gmapping, cartographer"
                    echo "       racecar mapping save <name>   save the current map (works with any backend)"
                    echo "       racecar mapping rviz [cartographer]  watch the map form (car desktop only)"
                    ;;
                *)
                    echo "racecar mapping: unknown action '$sub'" >&2
                    return 2
                    ;;
            esac
            ;;

        navigation)
            # On-demand Nav2 against a saved map. Foreground; Ctrl-C stops it.
            # slam and nav are mutually exclusive (both would publish
            # map->odom), so don't run this while `racecar mapping` is up.
            # Optional first arg picks the local planner: teb (default) or dwb.
            # `racecar navigation rviz` opens the navigation RViz view instead
            # (set a goal, watch the plan); needs the car desktop.
            if [[ "${1:-}" == "rviz" ]]; then
                (
                    _rc_autonomy_env || exit 1
                    if [[ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]]; then
                        echo "racecar navigation rviz needs a display; run it from the car desktop (RustDesk or monitor)" >&2
                        exit 1
                    fi
                    exec rviz2 -d "$HOME/osracer_ws/src/osracer/osracer_debug/config/navigation.rviz"
                )
                return
            fi
            local planner="teb"
            case "${1:-}" in
                teb|dwb) planner="$1"; shift ;;
            esac
            local map_name="${1:-map}"
            shift || true
            local maps_dir="$HOME/osracer_ws/src/osracer/osracer_slam/maps"
            if [[ ! -f "$maps_dir/$map_name.yaml" ]]; then
                echo "racecar navigation: no map named '$map_name' in $maps_dir" >&2
                local avail=""
                for f in "$maps_dir"/*.yaml; do
                    [[ -e "$f" ]] && avail+="$(basename "${f%.yaml}") "
                done
                echo "  available maps: ${avail:-none (build one with: racecar mapping)}" >&2
                return 2
            fi
            (
                _rc_autonomy_env || exit 1
                exec ros2 launch osracer_navigation nav2.launch.py \
                    use_planner:="$planner" use_map:="$map_name" \
                    use_rviz:='False' use_namespace:='False' "$@"
            )
            ;;

        update)
            # One-command field update: repo to latest main, full setup, and
            # a service restart. The git reset runs BEFORE setup_all so the
            # script never rewrites itself mid-run; this function is already
            # loaded in shell memory, so replacing the file under it is safe.
            # reset --hard discards local edits in the driver repo.
            echo "Updating $pkg_dir to latest origin/main..."
            ( cd "$pkg_dir" && git fetch origin && git reset --hard origin/main ) || return 1
            bash "$pkg_dir/scripts/setup_all.sh" || return 1
            # Restart only what was already enabled, so a field update brings
            # back the services this car runs without starting a lab dashboard
            # nobody asked for.
            echo "Restarting services..."
            local -a live
            mapfile -t live < <(_rc_enabled_units)
            [[ ${#live[@]} -gt 0 ]] && sudo systemctl restart "${live[@]}"
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
                # The autonomy base holds GPU memory too, and now runs as a
                # plain process rather than a unit.
                pgrep -f launch_autonomy.sh >/dev/null 2>&1 && busy+="autonomy "
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
                        local unit="neoracer-$1"
                        if ! _rc_unit_installed "$unit"; then
                            echo "racecar service: $unit is not installed." >&2
                            echo "Run 'racecar service install' first." >&2
                            return 1
                        fi
                        sudo systemctl "$action" "$unit"
                    elif [[ "$action" == restart ]]; then
                        # Restart what is already enabled, dashboards included,
                        # and leave every disabled unit where it is.
                        local -a live
                        mapfile -t live < <(_rc_enabled_units)
                        if [[ ${#live[@]} -eq 0 ]]; then
                            echo "No enabled services to restart." >&2
                        else
                            sudo systemctl restart "${live[@]}"
                            printf 'restarted: %s\n' "${live[*]}"
                        fi
                    else
                        sudo systemctl "$action" "${units[@]}"
                    fi
                    ;;
                enable|disable)
                    # Blanket enable/disable covers the core stack only. A lab
                    # dashboard is turned on by name, so a car that wants one
                    # at boot says so: `racecar service enable camlabel`.
                    if [[ -n "$1" ]]; then
                        sudo systemctl "$action" "neoracer-$1"
                    else
                        for u in "${units[@]}"; do
                            sudo systemctl "$action" "$u"
                        done
                    fi
                    ;;
                logs)
                    local unit="${1:-teleop}"
                    sudo journalctl -u "neoracer-$unit" -f
                    ;;
                status|"")
                    local u
                    for u in "${_RC_CORE_UNITS[@]}"; do
                        _rc_unit_status_line "$u"
                    done
                    echo "  -- lab dashboards (off unless started) --"
                    for u in "${_RC_DASH_UNITS[@]}"; do
                        _rc_unit_status_line "$u"
                    done
                    ;;
                -h|--help|help)
                    cat <<'__RC_SVC_HELP__'
usage: racecar service <action> [unit]
actions:
  install        Install units: core stack enabled, lab dashboards disabled
  start [name]   Start neoracer-<name>; no name = the core stack
  stop [name]    Stop neoracer-<name>; no name = the core stack
  restart [name] Restart neoracer-<name>; no name = every enabled unit,
                 dashboards included. Never starts a disabled one.
  enable [name]  Enable neoracer-<name>; no name = the core stack
  disable [name] Disable neoracer-<name>; no name = the core stack
  logs [name]    journalctl -u neoracer-<name> -f; default = teleop
  status         active/enabled snapshot for every unit (default)
core units:      teleop, watchdog, dashboard, jupyter
lab dashboards:  camlabel (8082), wallfollow (8081), pursuit (8083),
                 eps (8084), smartfollow (8085), linefollow (8086), webteleop (8087)

neoracer-autonomy is held and no longer installed. Run the base in a terminal:
  bash ~/ros2_ws/src/neoracer_ros2_driver/scripts/launch_autonomy.sh

Dashboards install disabled and are started for a session:
  racecar service start camlabel
Each holds the camera or the GPU while it runs, so run one at a time.
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
                    echo "  phases: all, ml, networking" >&2
                    return 2
                    ;;
                all)
                    bash "$pkg_dir/scripts/setup_all.sh" "$@"
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
  --reset  delete the persisted file (revert to script defaults)
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
                        rm -f "$cfg_file"
                        echo "Cleared $cfg_file; defaults will apply on next run."
                        return 0
                    fi

                    # Persist any new --flag values (applies to apply/show paths).
                    if [[ $vals_changed -eq 1 ]]; then
                        : > "$cfg_file"
                        chmod 600 "$cfg_file"
                        echo "# racecar networking overrides - managed by 'racecar setup networking'" >> "$cfg_file"
                        for k in RACECAR_AP_SSID RACECAR_AP_PSK RACECAR_AP_CHANNEL RACECAR_AP_ADDR RACECAR_ETH_STATIC; do
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
                    echo "  phases: all, networking" >&2
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
            echo "=== USB peripherals ==="
            lsusb | grep -iE "303a|1209|1bcf|2993|05a3|espressif|decxin|camera" || echo "  (none of the expected USB devices found)"
            echo
            echo "=== Stable device symlinks ==="
            for s in osrbot_base osrbot_led_matrix osrbot_usb_cam; do
                if [[ -e "/dev/$s" ]]; then
                    printf "  /dev/%-18s -> %s\n" "$s" "$(readlink -f /dev/$s)"
                else
                    printf "  /dev/%-18s MISSING (run: racecar udev)\n" "$s"
                fi
            done
            echo
            echo "=== ros2 nodes running ==="
            if command -v ros2 >/dev/null; then
                ros2 node list 2>/dev/null || echo "  (no ROS daemon / no nodes)"
            else
                echo "  ros2 not on PATH"
            fi
            ;;

        help|-h|--help|"")
            cat <<'__RC_HELP__'
racecar - Neoracer developer tool

Usage:
    racecar <command> [args]

Commands:
    build               Build neoracer_ros2_driver (--symlink-install) and source overlay.
    test                Run the package test suite with verbose results.
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
    mapping [backend]   Start SLAM to build a map (foreground; Ctrl-C to stop).
                        Backends: slam_toolbox (default), gmapping, cartographer.
                        `racecar mapping save <name>` saves from any backend.
    navigation [teb|dwb] [map]
                        Start Nav2 on a saved map (foreground; Ctrl-C to stop).
                        Planner: teb (default) or dwb. Default map name: map.
                        `racecar navigation rviz` opens the goal-setting RViz
                        view (car desktop only).
    update              Field update in one command: repo to latest origin/main
                        (discards local repo edits), full `setup all`, restart
                        all services. Needs internet.
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
                          ml           - GPU stack: PyTorch/torchvision for Tegra,
                                         Ultralytics, ONNX export tooling; verifies
                                         the JetPack TensorRT bindings.
                          networking   - Wi-Fi AP + Lakibeam lidar subnet.
                                         Flags persist to ~/.config/racecar/networking.env:
                                           --ssid=NAME   --psk=PASS   --channel=N
                                           --ap-addr=CIDR (default 10.42.0.1/24)
                                           --eth-static=CIDR (default 192.168.1.101/24)
                                           --lidar-host=CIDR (default 192.168.8.1/24)
                                           --show / --reset
    service <action>    systemd service control. Actions:
                          install              setup_services.sh (core enabled,
                                               dashboards installed disabled)
                          start [name]         no name: the core stack
                          stop [name]          no name: the core stack
                          restart [name]       no name: every enabled unit;
                                               never starts a disabled one
                          enable|disable [name]  no name: the core stack
                          logs [name]          journalctl -f for neoracer-<name>
                          status               active/enabled summary (default)
                        Core units: teleop, watchdog, dashboard, jupyter
                        Lab dashboards, off until started, one at a time:
                          camlabel (8082), wallfollow (8081), pursuit (8083),
                          eps (8084), smartfollow (8085), linefollow (8086), webteleop (8087)
                        neoracer-autonomy is held; setup no longer installs it.
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
    status              Show USB peripherals, device symlinks, and running ros2 nodes.
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
        COMPREPLY=( $(compgen -W "build test source ws cd teleop mapping navigation launch compile clear udev watchdog service setup update library cleanup selftest status help" -- "$cur") )
        return
    fi

    case "$sub" in
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
                COMPREPLY=( $(compgen -W "all networking" -- "$cur") )
            elif [[ "${COMP_WORDS[2]}" == "networking" ]]; then
                COMPREPLY=( $(compgen -W "--ssid= --psk= --channel= --ap-addr= --eth-static= --show --reset --help" -- "$cur") )
            fi
            ;;
        mapping)
            if [[ $COMP_CWORD -eq 2 ]]; then
                COMPREPLY=( $(compgen -W "slam_toolbox gmapping cartographer save rviz" -- "$cur") )
            fi
            ;;
        navigation)
            if [[ $COMP_CWORD -ge 2 ]]; then
                local maps="teb dwb rviz "
                for f in "$HOME"/osracer_ws/src/osracer/osracer_slam/maps/*.yaml; do
                    [[ -e "$f" ]] && maps+="$(basename "${f%.yaml}") "
                done
                COMPREPLY=( $(compgen -W "$maps" -- "$cur") )
            fi
            ;;
        service)
            if [[ $COMP_CWORD -eq 2 ]]; then
                COMPREPLY=( $(compgen -W "install start stop restart enable disable logs status help" -- "$cur") )
            elif [[ $COMP_CWORD -eq 3 ]]; then
                local action="${COMP_WORDS[2]}"
                case "$action" in
                    start|stop|restart|logs|enable|disable)
                        COMPREPLY=( $(compgen -W "teleop watchdog dashboard jupyter camlabel wallfollow pursuit eps smartfollow linefollow webteleop" -- "$cur") )
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

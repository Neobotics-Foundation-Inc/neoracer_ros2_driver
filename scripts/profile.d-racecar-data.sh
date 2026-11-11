# Neoracer: where the car's data goes (neoracer_ros2_driver/data_root.py).
# Installed to /etc/profile.d/racecar-data.sh by scripts/setup_services.sh.
# neoracer-data.service writes the path once per boot.
if [ -r /run/racecar/data_root ]; then
    RACECAR_DATA_DIR="$(cat /run/racecar/data_root)"
else
    RACECAR_DATA_DIR="$HOME/racecar_data"
fi
export RACECAR_DATA_DIR

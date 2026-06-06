# v0.0.6 Networking - Test Checklist

Walk through this on the actual robot to verify the enP8p1s0 dual-IP + wlP1p1s0 isolated AP setup. **Run from a wired (enP8p1s0) connection or the console** - `racecar setup networking` reconfigures wlP1p1s0 and will drop SSH-over-WiFi sessions.

## Pre-flight (no network changes yet)

- [X] Unit tests pass
  ```sh
  racecar test
  ```
  Expected: `350 passed, 2 skipped`.

- [X] Subcommand is wired into the racecar tool
  ```sh
  racecar help
  ```
  Expected: output contains a `setup <phase>` line.

- [X] No-args usage prints phase list
  ```sh
  racecar setup
  ```
  Expected: exit code 2, stderr says `phases: all, networking`.

- [X] `--help` is comprehensive
  ```sh
  racecar setup networking --help
  ```
  Expected: shows all 5 flags (`--ssid`, `--psk`, `--channel`, `--ap-addr`, `--eth-static`) plus `--show` / `--reset`.

- [X] `--show` with no persisted file gives the right message
  ```sh
  racecar setup networking --show
  ```
  Expected: `No persisted networking config - script defaults will apply.`

- [X] Tab completion works
  ```sh
  racecar setup <TAB>
  ```
  Expected: `all  networking`.

  ```sh
  racecar setup networking --<TAB>
  ```
  Expected: `--ssid=  --psk=  --channel=  --ap-addr=  --eth-static=  --show  --reset  --help`.

## Persistence layer (safe - no network changes)

- [X] Flags get written even when combined with `--show`
  ```sh
  racecar setup networking --ssid=test-ssid --psk=test-pass --show
  ```
  Expected: prints `Saved overrides to ~/.config/racecar/networking.env`, then prints the file contents including both `RACECAR_AP_SSID` and `RACECAR_AP_PSK` lines.

- [X] Verify the persisted file exists
  ```sh
  cat ~/.config/racecar/networking.env
  ```
  Expected:
  ```
  # racecar networking overrides - managed by 'racecar setup networking'
  RACECAR_AP_SSID="test-ssid"
  RACECAR_AP_PSK="test-pass"
  ```

- [X] `--show` alone reads the existing file
  ```sh
  racecar setup networking --show
  ```
  Expected: prints `Persisted networking config (...)` and the file contents.

- [X] `--reset` deletes the file
  ```sh
  racecar setup networking --reset
  ls ~/.config/racecar/networking.env
  ```
  Expected: `Cleared ~/.config/racecar/networking.env...`, then `ls` reports no such file.

- [X] `--reset` combined with overrides errors loudly
  ```sh
  racecar setup networking --ssid=foo --reset
  ```
  Expected: exit code 2, stderr says `--reset cannot be combined with override flags`.

## The actual network reconfiguration (destructive - wired session only)

- [X] You're on a wired connection, not WiFi
  ```sh
  ip -br link show wlP1p1s0
  nmcli -t -f NAME con show
  ```
  Note the current state. If `nmcli` shows a WiFi connection you're SSH'd through, **stop here** and connect via enP8p1s0 or the console.

- [X] Apply the networking config
  ```sh
  racecar setup networking --ssid=neoracer-1 --psk='your-password'
  ```
  Expected: 4 numbered steps, then `Applying netplan...`, then `=== Done ===`. ~5-10 seconds. wlP1p1s0 should drop client mode and come up as AP.

## Verify outcomes

- [X] enP8p1s0 has both static + DHCP addresses
  ```sh
  ip -br addr show enP8p1s0
  ```
  Expected: two `inet` entries - `192.168.52.200/24` (or whatever `--eth-static` you set) AND a DHCP-assigned address.

- [X] wlP1p1s0 is in AP mode
  ```sh
  iw dev wlP1p1s0 info
  ```
  Expected: `type AP`, `ssid neoracer-1` (or whatever you set), `channel 6`.

- [X] iptables FORWARD reject rules are in place (the "isolation" half)
  ```sh
  sudo iptables-nft -L FORWARD -nv
  ```
  Expected: two `REJECT` rules near the top - one with `wlP1p1s0` in the `in`
  column, one with `wlP1p1s0` in the `out` column. Both should sit ABOVE the
  `nm-sh-fw-wlP1p1s0` chain (so they evaluate first and reject AP->internet
  traffic before NM's shared-mode FORWARD rules can accept it).

  Why `iptables-nft` and not plain `iptables`? On Ubuntu 24.04, NetworkManager
  installs its `ipv4.method=shared` forward rules via nftables directly; our
  dispatcher installs REJECT rules via `iptables-nft` (which writes to a
  parallel nft table). Both end up evaluated in order, but `iptables -L
  FORWARD -n` without `-v` hides the `in`/`out` columns so the REJECT rules
  look invisible - use `-nv` or check `sudo nft list ruleset`.

- [X] Persisted file matches what you ran
  ```sh
  cat ~/.config/racecar/networking.env
  ```
  Expected: contains `RACECAR_AP_SSID="neoracer-1"` and `RACECAR_AP_PSK="your-password"`.

## Real-world test (with a phone or laptop)

- [X] AP shows up in WiFi scan: SSID `neoracer-1` visible from a separate device.

- [X] Join the AP with the password you set. Client should receive a `10.42.0.0/24` IP via DHCP.

- [X] Dashboard reachable
  ```
  http://10.42.0.1:8080
  ```
  Expected: dashboard loads with node cards.

- [X] JupyterLab reachable
  ```
  http://10.42.0.1:8888
  ```
  Expected: Jupyter UI loads.

- [X] SSH reachable
  ```sh
  ssh racecar@10.42.0.1
  ```
  Expected: login prompt.

- [X] **Isolation check** - internet must NOT be reachable from the AP client. From the client device:
  ```sh
  curl -m 3 https://google.com
  ```
  Expected: timeout or "no route to host". This is the "isolated" half of "wlP1p1s0 isolated AP" - the dispatcher's FORWARD REJECT rules at work. If this curl succeeds, the isolation is broken.

## Idempotency (re-run safety)

- [X] Running the same command twice is a no-op the second time
  ```sh
  racecar setup networking
  ```
  Expected on the second run: `(No configuration changes were necessary - system already matched.)` at the end.

## Reboot persistence (the autostart test)

- [X] Reboot
  ```sh
  sudo reboot
  ```

- [X] After login (still wired ideally), AP is still up
  ```sh
  iw dev wlP1p1s0 info               # should still show type AP
  ip -br addr show enP8p1s0           # should still show static + DHCP
  racecar service status          # all 4 services active=active
  ```

## Done

If every box is checked, v0.0.6 networking is verified end-to-end.

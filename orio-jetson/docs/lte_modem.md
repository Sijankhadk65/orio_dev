# LTE modem — SIM7600E-H (`orio-lte`)

Orio has a SIMCom **SIM7600E-H** LTE modem as a backup uplink and as the
**SOS path** (SMS). Wi-Fi stays the primary route; LTE only carries traffic when
Wi-Fi is gone. The modem also has a GNSS receiver, exposed as NMEA through
ModemManager.

> **Keep this out of Orio's knowledge base.** Anything ingested with
> `orio.kb_ingest` can be retrieved and read aloud. `kb_ingest` only reads the
> paths you pass it, but its own docstring example is
> `kb_ingest --profile … docs/ --replace` — never point it at `docs/`.

| Piece | Value |
|---|---|
| Module | `SIMCOM_SIM7600E-H`, firmware `LE11B14SIM7600M22_211104` |
| USB ID | `1e0e:9001` (kernel `option` driver), behind a self-powered Genesys hub (`05e3:0610`) at `1-1.1` |
| Serial ports | `ttyUSB0` diag (ignored by MM) · `ttyUSB1` GPS NMEA · **`ttyUSB2` AT + PPP (primary)** · `ttyUSB3` AT · `ttyUSB4` audio |
| ModemManager | plugin `simtech`, MM 1.23.4 |
| NetworkManager | connection `orio-lte`, NM 1.46.0 |

The QMI data interface (USB interface 5) has no driver bound on this kernel, so
data runs over **PPP on `ttyUSB2`**, not QMI.

## The `orio-lte` connection

| Setting | Value | Why |
|---|---|---|
| `type` | `gsm` (via ModemManager) | MM owns the modem; NM asks it to dial PPP on the primary port `ttyUSB2`. |
| `gsm.apn` | `internet` | The SIM's APN. `gsm.auto-config` is off. |
| `ipv4.method` | `auto` | Address and DNS from PPP/IPCP. |
| `ipv6.method` | `ignore` | IPv4 only — see below. |
| `ipv4.route-metric` | `700` | Behind Wi-Fi (`600`), so LTE is backup only. |
| `connection.autoconnect` | `yes` | |
| `connection.autoconnect-retries` | `0` (forever) | Note: this does **not** cover the stale-probe failure below — NM never retries that one. |

Equivalent to:

```bash
sudo nmcli con add type gsm ifname '*' con-name orio-lte \
    gsm.apn internet ipv4.route-metric 700 ipv6.method ignore \
    connection.autoconnect yes connection.autoconnect-retries 0
```

### Why IPv6 is off

The first dual-stack attempt failed with *"Connection requested both IPv4 and
IPv6 but dual-stack addressing is unsupported by the modem"*, and IPv6 was
switched off. That error turned out to be the **stale-probe bug below**, not a
PPP limitation: once MM probes the modem properly it advertises
`ipv4, ipv6, ipv4v6`. IPv6 stays off anyway — IPv4 is tested and working, the SOS
path doesn't need IPv6, and dual-stack over PPP on this SIM is untested. Re-test
before turning it on.

### Route metric: `20700`, not `700`

`ip route` shows the LTE default route at **`20700`**. That's NetworkManager's
+20000 penalty for a device whose connectivity is not `full`, and `ppp0` is
always `limited`. The internet works (`curl --interface ppp0` gets `204` from
the check URL). NM just **never runs the check** on this link:

```
connectivity: (ppp0,IPv4) skip connectivity check due to no global route configured
connectivity: (ppp0,IPv4) check completed: LIMITED; no global route configured
```

PPP's default route has no gateway (`default dev ppp0 scope link`), and NM 1.46
doesn't count that as a global route, so it marks the link `limited` without
testing it. (Found with `nmcli general logging level DEBUG domains CONCHECK`.)
Unmanaging the phantom `ppp0` NM device does not help.

**Effect:** none while Wi-Fi is healthy (`600`) or fully gone (its route
disappears). It *does* break one case: Wi-Fi associated but without internet
goes to `20600`, which still beats LTE's `20700`, so traffic stays on dead Wi-Fi.
Not fixed yet — no small, safe fix was found.

## Power: the modem browns out under TX

The modem **drops off USB and re-enumerates during transmit bursts**. There is
no `over-current` in `dmesg`, so the Jetson port isn't tripping — the modem's
own supply is sagging (SIM7600 peaks at ~2 A while transmitting). Observed on
2026-09-18, in a single 20-minute session:

| Time | What the radio was doing | Result |
|---|---|---|
| 16:22:34 | MM enabled the modem, RF on (`power state: on`) 2 s earlier | USB disconnect, back after 6 s |
| 16:30:03 | 3 MB upload over `ppp0` (~110 kB sent) | USB disconnect, back after 7 s |
| 16:38:43 | link just up + curl test | USB disconnect, back after 6 s |
| 16:39–16:43 | link up and idle, then GPS NMEA enabled | stable |
| 17:01–17:07 | link up **and idle**, then each reconnect | 7 resets in 6 min, sometimes only 20 s apart |

It keeps happening, and not just at boot → **treat it as a power problem**. Fix the
supply before trusting the SOS path: give the modem board its own 5 V rail rated
for ≥2 A peak (not shared with the hub's other devices), add bulk capacitance
near the module, and keep the power leads short and thick.

To check again: `sudo dmesg | grep -iE "usb.*(disconnect|new .*device)|over-current"`.
A drop right after power-on can't be told apart from a normal boot reset,
because the RF switches on at the same moment. Any drop once `orio-lte` is up is a
brownout.

## The ModemManager "no SIM" problem

Each re-enumeration (boot or brownout) triggers a race: **MM probes the modem
before its firmware is ready.** It ends in one of two ways:

- **Hard fail:** `mmcli -m any` shows `state: failed`,
  *"unknown-capabilities"*, NM logs *"No SIM object available"*.
- **Soft fail:** the modem looks fine (`registered`, LTE) but
  `mmcli -m any --output-keyvalue | grep supported-ip-families` is `--`. NM then
  refuses to dial: *"Connection requested IPv4 but IPv4 is unsupported by the
  modem"* (or the dual-stack variant).

Either way `orio-lte` stays down **for good**. NM didn't retry within 7 minutes,
and `nmcli con up orio-lte` fails the same way. The current fix:

```bash
sudo systemctl restart ModemManager      # orio-lte comes up ~10 s later
sudo mmcli -m any --location-enable-gps-nmea
```

After a reboot, `orio-lte` did **not** come up by itself (hard fail). It needed
this restart. The [watchdog](#watchdog-orio-lte-watchdog) now does it
automatically.

## Always use `-m any`

The modem's index changes every time MM restarts or the modem re-enumerates
(seen: 6 → 0 → 1). Use `mmcli -m any …` everywhere. Never hardcode a number.

## GPS NMEA needs re-enabling

MM's GPS source is **off after every ModemManager restart and every modem
reset**:

```bash
sudo mmcli -m any --location-enable-gps-nmea
mmcli -m any --location-status       # expect: enabled: 3gpp-lac-ci, gps-nmea
```

**Gotcha:** if MM restarts while the modem stays powered, the modem's GNSS
engine keeps running (NMEA still streams on `ttyUSB1`), but MM thinks it's off.
MM's enable (`AT+CGPS=1`) then fails with
*"Couldn't enable location 'gps-nmea' gathering: Unknown error"*. Turn the engine
off while MM is stopped, then enable it again:

```bash
sudo systemctl stop ModemManager
sudo bash -c 'stty -F /dev/ttyUSB3 115200 raw -echo; printf "AT+CGPS=0\r" > /dev/ttyUSB3'
sudo systemctl start ModemManager
sudo mmcli -m any --location-enable-gps-nmea
```

Enabling GPS did not cause a brownout. A fix still needs the GNSS antenna on the
GNSS connector (not an LTE one) and sky view. The first test indoors saw zero
satellites.

## SMS (SOS path)

```bash
sudo mmcli -m any --messaging-create-sms="text='…',number='+49…'"   # prints SMS/<n>
sudo mmcli -m any -s <n> --send
```

Tested 2026-09-18: **the SMS arrived**, but `--send` returned
`couldn't send the SMS: 'Timeout was reached'` both times (30 s default and
`--timeout=120`). The SMS object's `state` stayed `--`. So:

- **Don't treat a send timeout as a failure and retry blindly.** It probably
  went out, and a retry can send a duplicate.
- SMS only needs the modem **registered**. It went out while `orio-lte` was down.
  The SOS path doesn't depend on the data link.

## Watchdog (`orio-lte-watchdog`)

A systemd timer runs `/usr/local/sbin/orio-lte-watchdog` every 30 s (first run
2 min after boot). Sources live in `deploy/lte-watchdog/`.

| Condition | What it does |
|---|---|
| Healthy | Nothing, except re-enabling GPS NMEA if a modem reset switched it off. |
| Unhealthy for >3 min: `orio-lte` not activated, or MM has no modem / `failed` / no SIM / no IP families | Stops MM, sends `AT+CGPS=0` on the second AT port (see GPS gotcha), starts MM, re-enables GPS NMEA, then runs `nmcli con up orio-lte` if autoconnect hasn't. |
| SIM7600 not on USB at all | Logs once and does nothing. A restart can't bring the modem back. |
| Restarted <5 min ago | Waits. |
| >3 restarts in the last hour | Logs a warning that the modem is probably browning out. |

It only reads MM/NM state and never sends traffic over LTE. It covers for the
brownouts, but it does **not** fix them.

```bash
journalctl -t orio-lte-watchdog                  # what it did and why
systemctl list-timers orio-lte-watchdog.timer
sudo cat /run/orio-lte-watchdog/restarts         # restart times (epoch), last hour
```

Install / update:

```bash
cd deploy/lte-watchdog
sudo install -m 0755 orio-lte-watchdog /usr/local/sbin/
sudo install -m 0644 orio-lte-watchdog.service orio-lte-watchdog.timer /etc/systemd/system/
sudo systemctl daemon-reload && sudo systemctl enable --now orio-lte-watchdog.timer
```

Tested 2026-09-18 with `orio-lte` taken down by hand (which also blocks
autoconnect) and the GNSS engine still running. The watchdog restarted MM after
the down limit. GPS came back 12 s after the restart and `orio-lte` 27 s after. Disable it with
`sudo systemctl disable --now orio-lte-watchdog.timer`.

## Quick health check

```bash
nmcli -g GENERAL.STATE con show orio-lte          # activated
mmcli -m any | grep -E " state:|access tech"      # connected, lte
mmcli -m any --output-keyvalue | grep supported-ip-families   # not "--"
curl -s -o /dev/null -w "%{http_code}\n" --interface ppp0 http://connectivity-check.ubuntu.com/   # 204
```

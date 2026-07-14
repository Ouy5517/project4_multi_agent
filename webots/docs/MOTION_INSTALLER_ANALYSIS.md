# Motion Installer Analysis

## Scope

This document records the minimal static analysis used to extract Booster T1 configuration files for the Webots Runner. The full motion installer was not executed.

## Installer

- Path: `/home/plon/Workspace/booster_official_installers/v1.6.2.2-release-02145-2026-06-03-22.04-x86_64-motion.run`
- Format: Makeself 2.4.5
- Embedded script: `./install.sh`
- SHA256: `ad898aa2fea967ba9d9415f7f27b219e406801a6c24d57b195321c4fad413788`

## Configuration Copy Logic

Only `/home/plon/Workspace/booster_motion_extracted/install_motion.sh` was used for the config-copy decision.

For a robot with:

- `Model: Booster_T1`
- `Model Version: 2.3.4`

the official map in `booster/configs/config_version_maps.toml` selects:

```text
Booster_T1/T1_2.3.4
```

`install_motion.sh` then copies all top-level files from that directory into `/opt/booster/configs`. If `system_settings_config.yaml` is absent, it copies the common file from:

```text
booster/configs/common/system_settings_config.yaml
```

## Files Selected

The minimal config set prepared in staging is:

```text
/home/plon/Workspace/booster_motion_config_extract/configs/Booster_T1_2.3.4_20241219
/home/plon/Workspace/booster_motion_config_extract/configs/motor_calib.yaml
/home/plon/Workspace/booster_motion_config_extract/configs/robot_config.yaml
/home/plon/Workspace/booster_motion_config_extract/configs/security_config.yaml
/home/plon/Workspace/booster_motion_config_extract/configs/system_settings_config.yaml
```

`robot_config.yaml`, `security_config.yaml`, and `motor_calib.yaml` are copied because the official script uses `cp "$package_config_path"/*`. `system_settings_config.yaml` comes from the official common config directory.

## Verification

The staging files were checked with:

```bash
file /home/plon/Workspace/booster_motion_config_extract/configs/*
ls -lh /home/plon/Workspace/booster_motion_config_extract/configs/*
sha256sum /home/plon/Workspace/booster_motion_config_extract/configs/*
python -c 'import yaml; ...'
```

Results:

- Source and staging files match byte-for-byte.
- `robot_config.yaml` is non-empty.
- `system_settings_config.yaml` is non-empty.
- YAML parsing passed for all YAML files.

## Not Performed

The following were intentionally not performed:

- Running `install.sh`
- Installing systemd services
- Replacing Runner `mck`
- Replacing Runner dynamic libraries
- Modifying `/etc`, NetworkManager, Bluetooth, kernel, firmware, CPU settings, or system services

## Current Status

The sudo installation step was attempted with `sudo -n` and did not run because a password is required:

```text
sudo: a password is required
```

No files were installed into `/opt/booster/configs` during that attempt.

## Next Step

Run the reviewed minimal sudo copy command interactively, then validate `/opt/booster/configs` SHA256 values against the staging directory before starting Webots and Runner.

# Shared SSH access and eduroam preference

Follow-up maintenance on 7 September 2026, **19:15–19:18 CEST**, completed the
access-policy change left pending by the [consolidation](consolidation.md).
All three accepted drone-share recipients are now allowed through the Pi's
Tailscale network policy on TCP 22. The exact `ssh seb@seb-is-pm` command was
verified on the maintenance laptop after repairing its SSH alias. Eduroam has
the highest saved-network priority and no access-point lock.

## Tailscale policy

The administrator API credential was retrieved from the GNOME Keyring entry
identified by the user, held in process memory, and used for the authorized
policy update. No credential was printed or added to Git.

Exactly one grant was appended to the existing policy:

```json
{"src": ["autogroup:shared"], "dst": ["tag:pi-drone"], "ip": ["tcp:22"]}
```

The complete proposed policy passed the Tailscale API validator with HTTP 200.
The update used the original ETag in `If-Match`, preventing replacement of a
concurrent policy edit. It succeeded at **17:15:53 UTC**; a subsequent GET
matched the proposed policy. The existing owner, GPU-server and sync grants,
tag ownership, and Tailscale SSH policy were unchanged.

Readback policy SHA-256:
`323ae36753a6103705dc580cbc3430fb21c06a4d2a55056b04ae5b8d9a6cd69d`.

The API confirmed three accepted recipients for the drone's existing device
share. At **17:18:09 UTC**, the Pi's delivered packet rules allowed TCP 22 from
all eight visible devices belonging to those recipients, over both their
IPv4 and IPv6 addresses. The rules also retained the owner's SSH access.
Other destination ports were not added. No invitations or account-membership
changes were made.

The Pi continues to use ordinary OpenSSH; Tailscale SSH is disabled. Its three
existing authorized SSH keys were preserved. Delivered network permission was
verified for the accepted shares; each teammate's actual OpenSSH login was
not exercised from their computer.

## Exact short SSH command

On the maintenance laptop, the existing `Host drone-pi seb-is-pm` block had a
stale `HostName seb-is-pm`. Only that destination was changed to
`seb-is-pm.tail59e6a4.ts.net`, preserving the identity file and other settings.
The original file is backed up at
`/home/abaris/.ssh/config.ai-drone-backup-20260907T171517Z` with mode 0600.
No system SSH configuration permissions needed changing.

`ssh seb@seb-is-pm` succeeded with batch authentication and strict host-key
verification, returned remote user `seb`, and exited with status 0. The
trusted ED25519 host fingerprint matched the existing full-hostname entry:
`SHA256:P7N2ujzytmYWuzpv9Q8NYC+OrFHR/peSEk3S8C50o2I`.

Shared nodes require a fully qualified MagicDNS name across tailnets, as
described in [Tailscale's sharing documentation](https://tailscale.com/docs/features/sharing#sharing-and-magicdns).
Each teammate therefore needs the short alias from the
[networking guide](../../docs/pi-networking.md#shared-teammate-ssh-access)
on their own computer. A central ACL change cannot install that local alias.
Their existing OpenSSH credentials remain necessary.

## Eduroam preference

The boot selector already tried eduroam first, but NetworkManager's saved
priorities put another client at 300 and eduroam at 100. Eduroam was raised to
**400**, above every other saved client profile, while keeping autoconnect
enabled. This also gives it preference during NetworkManager's own connection
selection.

Readback confirmed no BSSID or band lock and an unrestricted channel. Three
eduroam access points were visible. The credentials, institution CA, and server
domain validation were unchanged. USB SSH and the existing eduroam connection
remained active; no reboot or Wi-Fi disconnection was needed.

The installed helper and systemd unit match repository source. The boot unit
is enabled and its last run succeeded, with no preferred-profile override.
Three offline selector cases passed: eduroam is attempted first even when
listed last, another saved client is the next fallback, and the hotspot is
last. The root-only profile backup is under
`/root/ai-drone-maintenance-20260907/eduroam-priority-20260907T171555Z/`.

This stationary check establishes that access-point roaming is unrestricted;
handover while moving was not tested. The boot/auto selector and saved
priorities do not provide a continuous switch-back from an already connected
fallback SSID. `sudo ai-drone-network auto` reruns selection when needed.

## Local evidence

Private policy snapshots and sanitized verification results are under ignored
`artifacts/tailscale-access/20260907/`, including `apply-result.json` and
`effective-verification.json`. The directory is mode 0700 and its files are
mode 0600; raw policy/share records are not published in Git.

The ignored `artifacts/consolidation/20260907/` directory contains
`client-ssh-alias-repair.json` and `pi-eduroam-preference-result.json`, with
the supporting profile and selector checks. No flight-controller command,
sensor recording, or actuator operation was performed in this follow-up.
The updated documentation site built all 18 pages successfully, and whitespace
validation passed. No application source change was needed.

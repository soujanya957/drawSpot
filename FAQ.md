# FAQ

## Networking

### Vicon ping times out after unplugging/replugging Ethernet (macOS)

**Symptom:** `ping 169.254.217.218` times out even though the Ethernet link is active (`status: active` in `ifconfig`).

**Setup:** Vicon Windows PC (`169.254.217.218`) → Netgear switch → Mac (en18, `169.254.100.50`) + Linux machine.

**Root cause:** macOS caches a stale ARP entry that maps the Vicon PC's IP to your own Mac's MAC address. Packets loop back to yourself instead of reaching the Vicon PC.

**Diagnosis:**

```bash
arp -n 169.254.217.218
# Bad: shows your own MAC (e.g. 20:7b:d2:ce:c3:98) — same as `ifconfig en18 | grep ether`
# Good: shows a different MAC (the Vicon PC's NIC)
```

**Fix:**

```bash
sudo arp -d 169.254.217.218
ping -S 169.254.100.50 169.254.217.218
```

Deleting the stale ARP entry forces macOS to re-resolve it correctly via the switch. You do **not** need to add any manual routes — the existing `169.254.0.0/16` link-local route on en18 handles it.

**Verify Vicon connectivity after ping works:**

```bash
python -m tests.probe_vicon --host 169.254.217.218:801
```

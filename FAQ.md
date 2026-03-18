# FAQ

## Networking

### Vicon ping times out after unplugging/replugging Ethernet (macOS)

**Symptom:** `ping <VICON_IP>` times out even though the Ethernet link is active (`status: active` in `ifconfig`).

**Setup:** Vicon Windows PC → Netgear switch → Mac (Ethernet interface) + Linux machine.

**Root cause:** macOS caches a stale ARP entry that maps the Vicon PC's IP to your own Mac's MAC address. Packets loop back to yourself instead of reaching the Vicon PC.

**Diagnosis:**

```bash
arp -n <VICON_IP>
# Bad: shows your own MAC — same as `ifconfig <ETHERNET_IFACE> | grep ether`
# Good: shows a different MAC (the Vicon PC's NIC)
```

**Fix:**

```bash
sudo arp -d <VICON_IP>
ping -S <MAC_LOCAL_IP> <VICON_IP>
```

Deleting the stale ARP entry forces macOS to re-resolve it correctly via the switch. You do **not** need to add any manual routes — the existing `169.254.0.0/16` link-local route on the Ethernet interface handles it.

**Verify Vicon connectivity after ping works:**

```bash
python -m tests.probe_vicon --host <VICON_IP>:801
```

> See `fixes.local.md` for the actual IPs and MAC address for this lab setup.

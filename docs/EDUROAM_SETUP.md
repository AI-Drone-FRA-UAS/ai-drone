# eduroam WLAN auf dem Raspberry Pi

Eingerichtet und verifiziert am 2026-07-15.

Der Raspberry Pi (`seb-is-pm`) verbindet sich auf dem Campus der Frankfurt UAS
automatisch mit dem WLAN `eduroam` und hat dadurch überall auf dem Campus
Internet. Ausserhalb des Campus fällt er weiter auf den eigenen Hotspot
`AI-Drone-Zero` zurück (siehe [README_HOTSPOT.md](../README_HOTSPOT.md)).

---

## 1. So funktioniert es jetzt

`wlan0` betreibt immer genau **einen** Modus. NetworkManager wählt anhand der
`autoconnect-priority`, welches bekannte Netz Vorrang hat:

| Netz (NM-Profil)     | Typ            | Priorität | Wann aktiv                                  |
| -------------------- | -------------- | --------: | ------------------------------------------- |
| `Xyz`                | Client (Handy) |       100 | Handy-Hotspot in Reichweite                 |
| `eduroam`            | Client         |         5 | Auf dem Campus                              |
| `Espresso Macchiato` | Client         |         0 | Dieses WLAN in Reichweite                   |
| `Hotspot`            | Access Point   |       -10 | Fallback, wenn kein Client-Netz erreichbar  |

- **Auf dem Campus** verbindet sich der Pi automatisch mit `eduroam`, bekommt
  eine DHCP-Adresse (`10.x.x.x`) und echtes Internet. Er betreibt dann *keinen*
  eigenen Hotspot.
- **Ausserhalb des Campus** (kein bekanntes Client-Netz) startet der Pi den
  Fallback-Hotspot `AI-Drone-Zero` (`192.168.4.1`). Dieser hat selbst **kein**
  Internet.

Wichtig: Der **Tailscale-Knoten des Pi ist nur online, wenn der Pi Internet
hat** – also auf `eduroam`, nicht am Hotspot. Siehe Abschnitt 4.

---

## 2. eduroam-Profil

Konfiguration nach der offiziellen Anleitung
([frankfurt-university.de/wlan](https://www.frankfurt-university.de/wlan), nutzt
das eduroam Configuration Assistant Tool). Angelegt als NetworkManager-Profil
`eduroam`:

| Parameter            | Wert                                                        |
| -------------------- | ----------------------------------------------------------- |
| SSID                 | `eduroam`                                                   |
| EAP / Phase 2        | `PEAP` / `MSCHAPV2`                                         |
| Identität            | `wlangast1@frankfurt-university.de` (WLAN-Gast-Account)     |
| Anonyme Identität    | `anonymous@frankfurt-university.de`                        |
| CA-Zertifikat        | `/etc/ssl/certs/eduroam-fra-uas-ca.pem`                    |
| Server (Domain-Match)| `cit.frankfurt-university.de` (`rad-srv-01/02...`)         |
| Autoconnect / Prio   | `yes` / `5`                                                |

Das CA-Zertifikat ist die **eduroam Service Root CA** (DFN), gültig bis 2042:

```text
subject   = C=DE, O=Verein zur Foerderung eines Deutschen Forschungsnetzes e. V.,
            CN=eduroam Service Root CA
SHA-256   = 8A:06:11:E8:96:A6:48:B0:9F:0E:46:F0:EF:DB:D8:D6:
            9C:8B:18:8D:B3:80:91:58:62:6F:11:32:75:96:2D:C9
```

> Die WLAN-Gast-Zugangsdaten (Passwort) liegen bewusst **nicht** im Repo. Sie
> sind ausschliesslich im NM-Profil auf dem Pi gespeichert
> (`/etc/NetworkManager/system-connections/eduroam.nmconnection`, nur für root
> lesbar).

---

## 3. Neu einrichten / reproduzieren

Auf dem Pi (Debian 13, NetworkManager). Zwei Wege:

### Variante A – offizielles CAT-Tool (empfohlen)

Am einfachsten mit dem eduroam Configuration Assistant Tool, wie in der
Hochschul-Anleitung beschrieben (Pi braucht dafür kurz Internet):

```bash
python3 eduroam-linux-FUoAS.py   # Profil "FRA-UAS members" von cat.eduroam.org
# Benutzer im Format <account>@frankfurt-university.de + Passwort eingeben
```

### Variante B – manuell per nmcli (so wurde es gemacht)

CA-Zertifikat aus dem CAT-Installer nach `/etc/ssl/certs/eduroam-fra-uas-ca.pem`
kopieren, dann:

```bash
sudo nmcli connection add type wifi ifname wlan0 con-name eduroam \
  ssid eduroam connection.autoconnect no

sudo nmcli connection modify eduroam \
  802-11-wireless-security.key-mgmt wpa-eap \
  802-1x.eap peap \
  802-1x.phase2-auth mschapv2 \
  802-1x.identity "wlangast1@frankfurt-university.de" \
  802-1x.anonymous-identity "anonymous@frankfurt-university.de" \
  802-1x.password "<GAST_PASSWORT>" \
  802-1x.ca-cert /etc/ssl/certs/eduroam-fra-uas-ca.pem \
  802-1x.domain-suffix-match "cit.frankfurt-university.de" \
  connection.autoconnect-priority 5

# Erst nach erfolgreichem Test scharf schalten:
sudo nmcli connection modify eduroam connection.autoconnect yes
```

> **Hinweis zum Verbindungsverlust:** Ist `eduroam` in Reichweite, wechselt der
> Pi beim Einschalten von `autoconnect` sofort vom Hotspot-AP auf `eduroam` –
> die aktuelle SSH-Verbindung über `192.168.4.1` bricht dann ab. Danach läuft
> das Management über Tailscale (Abschnitt 4). Stelle sicher, dass Tailscale
> **vorher** online ist.

Testen, ohne die Kontrolle zu verlieren (kurz auf eduroam, dann zurück auf den
Hotspot-AP – so bleibt der Pi über den Hotspot erreichbar):

```bash
sudo nmcli connection up eduroam        # verbindet mit eduroam
ping -c3 1.1.1.1 && curl -sI https://cat.eduroam.org | head -1
sudo nmcli connection up Hotspot        # zurück zum Fallback-AP
```

---

## 4. Fernzugriff über Tailscale

Sobald der Pi auf `eduroam` (oder einem anderen Netz mit Internet) ist, geht
sein Tailscale-Knoten online und ist erreichbar – unabhängig davon, in welchem
WLAN sich der Laptop befindet (der Laptop braucht nur *irgendeine*
Internetverbindung).

- Pi im Tailnet: `seb-is-pm`, erreichbar unter **`100.84.84.1`**.
- SSH über Tailscale:

  ```bash
  ssh -tt seb@100.84.84.1
  ```

  `-tt` (Pseudo-Terminal erzwingen) ist nötig: Pi und Laptop sind in
  unterschiedlichen Tailnets (per Node-Sharing verbunden), daher läuft die
  Verbindung über ein **DERP-Relay** statt direkt. Eine normale, nicht-tty
  SSH-Session kann über das Relay hängen bleiben; mit `-tt` läuft sie stabil.

- Status/Erreichbarkeit prüfen (vom Laptop):

  ```bash
  tailscale status | grep seb-is-pm
  tailscale ping 100.84.84.1
  ```

---

## 5. Troubleshooting

**„Tailscale connected, aber keine Verbindung zu anderen Geräten."**
Fast immer, weil der Pi gerade am Hotspot-AP hängt und dort **kein Internet**
hat → sein Tailscale-Knoten ist offline. Sobald der Pi auf `eduroam` ist
(Campus), kommt er innerhalb ~15 s online. Prüfen:

```bash
# auf dem Pi:
nmcli -t -f DEVICE,STATE,CONNECTION device status | grep wlan0   # -> wlan0:connected:eduroam ?
ip -4 addr show wlan0 | grep inet                                # bekommt er eine 10.x-IP ?
tailscale status --self=true --peers=false
```

**eduroam verbindet nicht.**

```bash
# EAP-/Zertifikatsfehler live mitlesen:
journalctl -u NetworkManager -f | grep -iE 'eduroam|EAP|802-1X|CERT|TLS'
```

Häufige Ursachen: falsches Passwort im Profil, fehlendes/falsches CA-Zertifikat,
oder Account nicht (mehr) gültig. Profil-Werte prüfen:

```bash
nmcli connection show eduroam | grep -E '802-1x|autoconnect'
```

**Pi ist auf dem Campus nicht per Hotspot erreichbar.**
Das ist so gewollt: auf dem Campus nutzt der Pi `eduroam` und betreibt keinen
eigenen AP. Zugriff dort über Tailscale (Abschnitt 4). Der Hotspot
`AI-Drone-Zero` erscheint erst wieder, wenn kein bekanntes Client-Netz in
Reichweite ist.

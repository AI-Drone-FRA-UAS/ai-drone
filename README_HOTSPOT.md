# Raspberry Pi Wi-Fi Hotspot (Access Point)

Dieses Dokument beschreibt, wie du dich mit dem Wi-Fi-Hotspot des Raspberry Pi verbindest, Fehler behebst und dem Hotspot Internetzugang über deinen Mac verschaffst.

---

## 1. Verbindungsdaten

Der Hotspot wird beim Booten des Pi automatisch gestartet.

* **Netzwerkname (SSID):** `AI-Drone-Zero`
* **Passwort:** `aidrone123`
* **IP-Adresse des Pi (Gateway):** `192.168.4.1`
* **IP-Bereich für Clients:** `192.168.4.10` bis `192.168.4.254`

---

## 2. Verbindung testen (Smartphone / Tablet)

Da das WLAN des Pi standardmäßig **kein Internet** hat, versuchen moderne Smartphones oft, die Verbindung zu ignorieren oder Daten stattdessen über das Mobilfunknetz zu senden.

### Schritt-für-Schritt-Anleitung:
1. **Mobile Daten ausschalten:** Deaktiviere vorübergehend die mobilen Daten (LTE/5G) auf deinem Smartphone.
2. **Mit WLAN verbinden:** Wähle das WLAN `AI-Drone-Zero` aus und gib das Passwort `aidrone123` ein.
3. **Hinweis bestätigen:** Nach dem Verbinden erscheint meist eine Meldung wie *"Dieses WLAN hat keine Internetverbindung. Trotzdem verbinden?"*. Bestätige dies unbedingt mit **"Ja" / "Verbunden bleiben"**.
4. **Testen:** Öffne eine Terminal-App auf dem Smartphone (z. B. *Termux* auf Android oder *iNetTools* auf iOS) und teste die Verbindung:
   * **Ping:** `ping 192.168.4.1`
   * **SSH:** `ssh seb@192.168.4.1` (Passwort: `1234`)

---

## 3. Hotspot mit Internetzugang ausstatten (macOS Internet Sharing)

Du kannst die Internetverbindung deines Macs über das USB-Kabel mit dem Raspberry Pi teilen. Der Pi leitet dieses Internet dann automatisch an alle Geräte weiter, die mit dem WLAN `AI-Drone-Zero` verbunden sind.

### Anleitung für macOS:
1. Öffne die **Systemeinstellungen** auf deinem Mac.
2. Navigiere zu **Allgemein > Teilen** (oder *Freigaben*).
3. Klicke auf das Info-Ikon (i) neben **Internetfreigabe** (noch nicht aktivieren).
4. Stelle folgende Optionen ein:
   * **Verbindung freigeben von:** `WLAN` (dein normales Internet-WLAN)
   * **Mit Computern über:** Aktiviere das Kontrollkästchen für `Raspberry Pi USB Gadget` (oder die entsprechende USB-Schnittstelle, z. B. `en10`).
5. Klicke auf **Fertig** und schalte den Schalter bei **Internetfreigabe** auf **Ein**. Bestätige die Aktivierung.

### Was passiert nun?
* Dein Mac startet einen DHCP-Server auf der USB-Schnittstelle.
* Der Pi erhält automatisch eine IP-Adresse vom Mac (z. B. `192.168.2.x` oder `192.168.3.x`) und nutzt deinen Mac als Gateway zum Internet.
* Geräte, die mit dem WLAN `AI-Drone-Zero` verbunden sind, haben nun vollen Internetzugang über deinen Mac!

### Wichtiger Hinweis zur SSH-Verbindung über USB:
Bei aktiver Internetfreigabe hat der Pi auf der USB-Schnittstelle **nicht mehr** die statische IP `192.168.7.2`. Du erreichst ihn stattdessen über seinen lokalen Hostnamen:
* **SSH-Befehl:** `ssh seb@seb-is-pm.local`

---

## 4. Fallback-Modus (WLAN-Priorisierung)

Der Hotspot ist so konfiguriert, dass er als **Fallback** (Ausweichlösung) dient:
* **Wenn bekannte WLANs in Reichweite sind** (z. B. dein Heim-WLAN oder dein Handy-Hotspot), verbindet sich der Pi automatisch als Client mit diesen Netzen (Priorität: `0` oder höher).
* **Wenn kein bekanntes WLAN gefunden wird** (z. B. wenn du unterwegs bist), startet der Pi automatisch den eigenen Hotspot `AI-Drone-Zero` (Priorität: `-10`).

Du musst also nichts manuell umschalten. Der Pi entscheidet beim Booten selbstständig, welcher Modus gestartet wird.

Auf dem Campus der Frankfurt UAS ist zusätzlich das WLAN `eduroam` als bevorzugtes Client-Netz eingerichtet – der Pi hat dort automatisch Internet und ist über Tailscale erreichbar. Details: [eduroam WLAN auf dem Raspberry Pi](docs/EDUROAM_SETUP.md).

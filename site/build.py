#!/usr/bin/env python3
"""Build the AI-Drone project site from the Markdown files in this repository.

The Markdown in ``docs/`` is the single source of truth. This script renders it
into a static site under ``site/_build`` and rewrites every in-repo link so the
result works both on GitHub and on GitHub Pages.

    uv run --group docs python site/build.py
    uv run --group docs python site/build.py --serve
"""

from __future__ import annotations

import argparse
import html
import re
import shutil
import unicodedata
from dataclasses import dataclass, field
from pathlib import Path
from urllib.parse import urlsplit

from markdown_it import MarkdownIt
from markdown_it.token import Token

SITE_DIR = Path(__file__).resolve().parent
REPO_ROOT = SITE_DIR.parent
BUILD_DIR = SITE_DIR / "_build"

SITE_TITLE = "KI-Drohne mit AprilTag-Abwurf"
REPO_URL = "https://github.com/AI-Drone-FRA-UAS/ai-drone"
BLOB_URL = f"{REPO_URL}/blob/main"


@dataclass(frozen=True)
class Page:
    """One rendered page of the site."""

    slug: str
    title: str
    source: str | None = None
    group: str = ""
    nav_title: str = ""
    lead: str = ""
    body: str = ""
    in_nav: bool = True

    @property
    def output(self) -> str:
        return f"{self.slug}.html"

    @property
    def label(self) -> str:
        return self.nav_title or self.title


# The navigation, in reading order. ``source`` is relative to the repository
# root; pages without one carry their own markup (see ``_special_pages``).
PAGES: list[Page] = [
    Page(
        slug="index",
        title="Übersicht",
        group="Projekt",
        nav_title="Start",
    ),
    Page(
        slug="ziele",
        title="Ziele und Hardware",
        source="docs/drone-project.md",
        group="Projekt",
        nav_title="Ziele & Hardware",
        lead="Was gebaut werden soll, mit welcher Ausrüstung und auf welcher Drohne.",
    ),
    Page(
        slug="apriltag",
        title="AprilTag-Erkennung und Abwurf",
        group="Projekt",
        nav_title="AprilTag & Abwurf",
        lead="Das erreichte Zwischenergebnis: Marker erkennen und daraufhin die Nutzlast fallen lassen.",
    ),
    Page(
        slug="architektur",
        title="Softwarearchitektur",
        source="docs/SOFTWARE_ARCHITECTURE.md",
        group="Projekt",
        nav_title="Softwarearchitektur",
        lead="Welcher Code auf welchem Rechner läuft, alle Kommandos und der Testlauf.",
    ),
    Page(
        slug="plakat",
        title="Projektplakat",
        group="Projekt",
        nav_title="Plakat",
        lead="Das A1-Plakat zur Projektarbeit als Druck-PDF.",
    ),
    Page(
        slug="konfiguration",
        title="Drohnenkonfiguration",
        source="docs/DRONE_CONFIGURATION.md",
        group="Hardware",
        nav_title="Konfiguration",
        lead="Der verifizierte Stand des Flight Controllers: Ports, Parameter, Kamerapfade.",
    ),
    Page(
        slug="abwurf",
        title="Abwurfmechanismus",
        source="docs/PAYLOAD_DROP.md",
        group="Hardware",
        nav_title="Abwurfmechanismus",
        lead="Das 9-g-Servo: Kennwerte, sicherer Stellbereich und die drei Ansteuerungswege.",
    ),
    Page(
        slug="rahmen",
        title="Rahmen und 3D-Druck",
        source="docs/FRAME_AND_3D_PRINTS.md",
        group="Hardware",
        nav_title="Rahmen & 3D-Druck",
        lead="Die Druckteile im Repository, ihre Slicer-Einstellungen und was noch fehlt.",
    ),
    Page(
        slug="flugversuche",
        title="Flugversuche und Absturz",
        group="Fliegen",
        nav_title="Flugversuche & Absturz",
        lead="Jeder Startversuch, der Absturz vom 21.08. und die drei gemessenen Ursachen.",
    ),
    Page(
        slug="mavlink",
        title="Autonome MAVLink-Steuerung",
        source="docs/PI_MAVLINK_CONTROL.md",
        group="Fliegen",
        nav_title="MAVLink-Steuerung",
        lead="DroneController, Geschwindigkeiten im Body-Frame und die Sicherheitsregeln.",
    ),
    Page(
        slug="verbindung",
        title="Verbindung vom Entwicklungsrechner",
        source="docs/DEVELOPER_MACHINE_DRONE_CONNECTION.md",
        group="Fliegen",
        nav_title="Laptop ↔ Drohne",
        lead="Die vollständige USB-Verbindungssequenz, MAVProxy und das Herunterfahren.",
    ),
    Page(
        slug="readme",
        title="Repository-README",
        source="README.md",
        group="Fliegen",
        nav_title="Repository-README",
        lead="Installation und die kürzesten Wege zu einem laufenden Setup.",
    ),
    Page(
        slug="pi-usb-ssh",
        title="Raspberry Pi über USB und SSH",
        source="docs/RPI_ZERO2W_USB_SSH_SETUP.md",
        group="Raspberry Pi",
        nav_title="USB & SSH",
        lead="Den USB-Gadget-Link und SSH auf einem frischen Pi in Betrieb nehmen.",
    ),
    Page(
        slug="eduroam",
        title="eduroam auf dem Raspberry Pi",
        source="docs/EDUROAM_SETUP.md",
        group="Raspberry Pi",
        nav_title="eduroam",
        lead="Campus-WLAN der Frankfurt UAS und Erreichbarkeit über Tailscale.",
    ),
    Page(
        slug="hotspot",
        title="Hotspot AI-Drone-Zero",
        source="README_HOTSPOT.md",
        group="Raspberry Pi",
        nav_title="Hotspot",
        lead="Der eigene Access Point des Pi als Rückfallebene, inklusive Internetfreigabe.",
    ),
    Page(
        slug="session-19-06",
        title="Inbetriebnahme am 19. Juni 2026",
        source="19-06-session.md",
        group="Protokolle",
        nav_title="Inbetriebnahme 19.06.",
        lead="Jedes Kommando, jeder Fehler und jede Lösung der MAVLink- und UART-Inbetriebnahme.",
    ),
    Page(
        slug="servo-arduino",
        title="Servo-Test am Arduino",
        source="servo_instruction.md",
        group="Protokolle",
        nav_title="Servo am Arduino",
        lead="Die arduino-cli-Sketches, mit denen das Servo vermessen wurde.",
    ),
    Page(
        slug="doku-index",
        title="Dokumentationsindex",
        source="docs/README.md",
        group="Protokolle",
        nav_title="Index aller Dokumente",
        lead="Jedes Dokument des Repositories, nach Aufgabe gruppiert.",
        in_nav=False,
    ),
]

# Repository files that have no page of their own but are worth linking to.
EXTRA_LINKS: dict[str, str] = {
    "docs/poster/README.md": "plakat.html",
}

ASSETS = {
    "docs/poster/fotos/drohne-flug-quer.jpg": "assets/drohne-flug-quer.jpg",
    "docs/poster/fotos/drohne-flug-hoch.jpg": "assets/drohne-flug-hoch.jpg",
    "docs/poster/plakat-a0.pdf": "assets/plakat-a0.pdf",
    "docs/poster/plakat-a1.pdf": "assets/plakat-a1.pdf",
    "docs/poster/plakat-a2.pdf": "assets/plakat-a2.pdf",
    "docs/poster/plakat-a3.pdf": "assets/plakat-a3.pdf",
}


# --------------------------------------------------------------------------- #
# Markdown rendering
# --------------------------------------------------------------------------- #


def _slugify(text: str) -> str:
    normalised = unicodedata.normalize("NFKD", text)
    stripped = "".join(ch for ch in normalised if not unicodedata.combining(ch))
    lowered = stripped.lower().replace("ß", "ss")
    cleaned = re.sub(r"[^a-z0-9]+", "-", lowered).strip("-")
    return cleaned or "abschnitt"


@dataclass
class Heading:
    level: int
    anchor: str
    text: str


@dataclass
class Rendered:
    html: str
    title: str | None
    headings: list[Heading] = field(default_factory=list)


def _link_map() -> dict[str, str]:
    mapping = {page.source: page.output for page in PAGES if page.source}
    mapping.update(EXTRA_LINKS)
    return mapping


def _resolve_link(href: str, source: str, mapping: dict[str, str]) -> str:
    """Rewrite an in-repo link so it works in the built site."""

    parts = urlsplit(href)
    if (
        parts.scheme
        or parts.netloc
        or href.startswith("#")
        or href.startswith("mailto:")
    ):
        return href

    fragment = f"#{parts.fragment}" if parts.fragment else ""
    target = (Path(source).parent / parts.path).as_posix() if parts.path else ""
    target = Path(target).as_posix()
    # Collapse "docs/../README.md" into "README.md".
    normalised = Path(target)
    resolved: list[str] = []
    for segment in normalised.parts:
        if segment == "..":
            if resolved:
                resolved.pop()
        elif segment not in (".", ""):
            resolved.append(segment)
    key = "/".join(resolved)

    if key in mapping:
        return mapping[key] + fragment
    if not key:
        return fragment or href
    # Anything else lives in the repository only — send readers to GitHub.
    return f"{BLOB_URL}/{key}{fragment}"


def _make_parser() -> MarkdownIt:
    # "gfm-like" gives tables and strikethrough; linkify needs an extra
    # dependency and would turn bare paths into links, so it stays off.
    return MarkdownIt("gfm-like").disable("linkify")


def render_markdown(text: str, source: str, mapping: dict[str, str]) -> Rendered:
    md = _make_parser()
    tokens = md.parse(text)

    title: str | None = None
    headings: list[Heading] = []
    seen: dict[str, int] = {}
    drop: list[int] = []

    for index, token in enumerate(tokens):
        if token.type == "heading_open":
            inline = tokens[index + 1]
            text_content = inline.content
            plain = re.sub(r"[*`_\[\]]|\(([^)]*)\)", "", text_content).strip()
            level = int(token.tag[1])

            if level == 1 and title is None:
                # The page shell prints its own <h1>; drop the one from the file.
                title = plain
                drop.extend((index, index + 1, index + 2))
                continue

            if level == 1:
                # Some files use "#" for section headings too. Demote them so a
                # page never carries two <h1> elements.
                token.tag = "h2"
                tokens[index + 2].tag = "h2"
                level = 2

            base = _slugify(plain)
            count = seen.get(base, 0)
            seen[base] = count + 1
            anchor = base if count == 0 else f"{base}-{count + 1}"
            token.attrSet("id", anchor)
            if level in (2, 3):
                headings.append(Heading(level, anchor, plain))
                inline.children = [
                    _anchor_token(anchor),
                    *(inline.children or []),
                ]

        elif token.type == "inline":
            # Links live inside inline tokens, not at the block level.
            for child in token.children or []:
                if child.type != "link_open":
                    continue
                href = str(child.attrGet("href") or "")
                resolved = _resolve_link(href, source, mapping)
                child.attrSet("href", resolved)
                if resolved.startswith("http"):
                    child.attrSet("target", "_blank")
                    child.attrSet("rel", "noopener")

    kept = [t for i, t in enumerate(tokens) if i not in set(drop)]
    body = md.renderer.render(kept, md.options, {})
    body = body.replace("<table>", '<div class="tabelle-huelle"><table>')
    body = body.replace("</table>", "</table></div>")
    return Rendered(html=body, title=title, headings=headings)


def _anchor_token(anchor: str) -> Token:
    token = Token("html_inline", "", 0)
    token.content = f'<a class="anker" href="#{anchor}" aria-hidden="true">#</a>'
    return token


# --------------------------------------------------------------------------- #
# Page shell
# --------------------------------------------------------------------------- #


def _nav(active: str) -> str:
    groups: dict[str, list[Page]] = {}
    for page in PAGES:
        if not page.in_nav:
            continue
        groups.setdefault(page.group, []).append(page)

    chunks = []
    for group, pages in groups.items():
        links = []
        for entry in pages:
            css = ' class="aktiv"' if entry.slug == active else ""
            links.append(
                f'<a href="{entry.output}"{css}>{html.escape(entry.label)}</a>'
            )
        chunks.append(
            f'<div class="gruppe"><b>{html.escape(group)}</b>{"".join(links)}</div>'
        )
    return "".join(chunks)


def _toc(headings: list[Heading]) -> str:
    if len(headings) < 2:
        return ""
    links = "".join(
        f'<a href="#{h.anchor}" class="stufe-{h.level}">{html.escape(h.text)}</a>'
        for h in headings
    )
    return f'<nav class="inhaltsnav"><b>Auf dieser Seite</b>{links}</nav>'


def _pager(page: Page) -> str:
    nav_pages = [p for p in PAGES if p.in_nav]
    if page not in nav_pages:
        return ""
    index = nav_pages.index(page)
    previous = nav_pages[index - 1] if index > 0 else None
    following = nav_pages[index + 1] if index + 1 < len(nav_pages) else None

    parts = []
    if previous:
        parts.append(
            f'<a href="{previous.output}"><span>Zurück</span>'
            f"<b>{html.escape(previous.label)}</b></a>"
        )
    if following:
        parts.append(
            f'<a class="weiter" href="{following.output}"><span>Weiter</span>'
            f"<b>{html.escape(following.label)}</b></a>"
        )
    return f'<div class="blaettern">{"".join(parts)}</div>' if parts else ""


HEAD = """<!doctype html>
<html lang="de">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>{title}</title>
<meta name="description" content="{description}">
<meta property="og:title" content="{title}">
<meta property="og:description" content="{description}">
<meta property="og:type" content="website">
<link rel="icon" href="data:image/svg+xml,<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 100 100'><text y='.9em' font-size='90'>🚁</text></svg>">
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link href="https://fonts.googleapis.com/css2?family=Inter:wght@400;500;600;700;800&family=JetBrains+Mono:wght@400;500&display=swap" rel="stylesheet">
<link rel="stylesheet" href="assets/style.css">
</head>
<body>
<header class="kopf">
  <div class="kopf-inner">
    <a class="marke" href="index.html">
      <span class="marke-punkt"></span>KI-Drohne<small>Frankfurt UAS</small>
    </a>
    <nav class="kopf-nav">
      <a href="index.html"{start_aktiv}>Start</a>
      <a href="ziele.html">Dokumentation</a>
      <a href="plakat.html">Plakat</a>
      <a href="{repo}" target="_blank" rel="noopener">GitHub</a>
    </nav>
    <button class="nav-schalter" type="button" aria-expanded="false">Menü</button>
  </div>
</header>
"""

FOOT = """<footer class="fuss">
  <div class="fuss-inner">
    <div>
      <b>Frankfurt University of Applied Sciences</b><br>
      Fachbereich 2 · Projektarbeit · Sommersemester 2026 · Betreuung Prof. Dr. Baun
    </div>
    <div class="fuss-recht">
      <a href="{repo}" target="_blank" rel="noopener">Quellcode auf GitHub</a> ·
      <a href="assets/plakat-a1.pdf">Plakat als PDF (A1)</a>
    </div>
  </div>
</footer>
<script>
document.querySelector('.nav-schalter')?.addEventListener('click', function () {{
  var nav = document.querySelector('.seitennav');
  if (!nav) return;
  var open = nav.classList.toggle('offen');
  this.setAttribute('aria-expanded', String(open));
}});
</script>
</body>
</html>
"""


def shell(*, title: str, description: str, body: str, active: str) -> str:
    head = HEAD.format(
        title=html.escape(title),
        description=html.escape(description),
        repo=REPO_URL,
        start_aktiv=' class="aktiv"' if active == "index" else "",
    )
    return head + body + FOOT.format(repo=REPO_URL)


def doc_page(page: Page, rendered: Rendered) -> str:
    title = page.title
    lead = page.lead or f"{title} — Projektdokumentation der KI-Drohne."
    lead_markup = (
        f'<p class="seiten-lead">{html.escape(page.lead)}</p>' if page.lead else ""
    )
    original = (
        f'<p class="seiten-original">Originaltitel im Repository: '
        f"<b>{html.escape(rendered.title)}</b></p>"
        if rendered.title and rendered.title != title
        else ""
    )
    source_link = (
        f'<a class="quelle" href="{BLOB_URL}/{page.source}" target="_blank" '
        f'rel="noopener">Quelle: {html.escape(page.source)}</a>'
        if page.source
        else ""
    )
    body = f"""<div class="doku">
  <nav class="seitennav">{_nav(page.slug)}</nav>
  <main class="inhalt">
    <div class="brotkrumen"><a href="index.html">Start</a> › {html.escape(page.group)}</div>
    <h1>{html.escape(title)}</h1>
    {lead_markup}
    {original}
    {rendered.html}
    <div class="seiten-fuss">
      <span>Diese Seite wird aus dem Repository erzeugt.</span>{source_link}
    </div>
    {_pager(page)}
  </main>
  {_toc(rendered.headings)}
</div>
"""
    return shell(
        title=f"{title} · {SITE_TITLE}", description=lead, body=body, active=page.slug
    )


# --------------------------------------------------------------------------- #
# Build
# --------------------------------------------------------------------------- #


def _special_pages() -> dict[str, str]:
    """Hand-written page bodies, one ``content/<slug>.html`` per source-less page."""

    bodies: dict[str, str] = {}
    for page in PAGES:
        if page.source:
            continue
        markup = (SITE_DIR / "content" / f"{page.slug}.html").read_text(
            encoding="utf-8"
        )
        bodies[page.slug] = markup.replace("<!--NAV-->", _nav(page.slug))
    return bodies


def build() -> None:
    if BUILD_DIR.exists():
        shutil.rmtree(BUILD_DIR)
    (BUILD_DIR / "assets").mkdir(parents=True)

    for asset in sorted((SITE_DIR / "assets").iterdir()):
        if asset.is_file():
            shutil.copy2(asset, BUILD_DIR / "assets" / asset.name)
    for source, target in ASSETS.items():
        origin = REPO_ROOT / source
        if not origin.exists():
            raise SystemExit(f"missing asset: {source}")
        shutil.copy2(origin, BUILD_DIR / target)

    # GitHub Pages must serve these files as-is, without running Jekyll.
    (BUILD_DIR / ".nojekyll").write_text("", encoding="utf-8")

    mapping = _link_map()
    specials = _special_pages()

    for page in PAGES:
        if page.source:
            text = (REPO_ROOT / page.source).read_text(encoding="utf-8")
            rendered = render_markdown(text, page.source, mapping)
            output = doc_page(page, rendered)
        else:
            output = shell(
                title=(
                    SITE_TITLE
                    if page.slug == "index"
                    else f"{page.title} · {SITE_TITLE}"
                ),
                description=page.lead
                or (
                    "Projektarbeit an der Frankfurt UAS: eine FPV-Drohne wird zur "
                    "GPS-freien Indoor-Plattform, die AprilTags erkennt und eine "
                    "Nutzlast abwirft."
                ),
                body=specials[page.slug],
                active=page.slug,
            )
        (BUILD_DIR / page.output).write_text(output, encoding="utf-8")

    print(f"built {len(PAGES)} pages → {BUILD_DIR.relative_to(REPO_ROOT)}")


def serve(port: int) -> None:
    import functools
    import http.server
    import socketserver

    handler = functools.partial(
        http.server.SimpleHTTPRequestHandler, directory=str(BUILD_DIR)
    )
    with socketserver.TCPServer(("127.0.0.1", port), handler) as httpd:
        print(f"serving http://127.0.0.1:{port}/  (Ctrl+C to stop)")
        httpd.serve_forever()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--serve", action="store_true", help="serve the site locally")
    parser.add_argument("--port", type=int, default=8000)
    args = parser.parse_args()

    build()
    if args.serve:
        serve(args.port)


if __name__ == "__main__":
    main()

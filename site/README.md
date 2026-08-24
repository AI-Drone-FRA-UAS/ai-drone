# Project Site

The GitHub Pages site for this project. It renders the Markdown from `docs/`
and the repository root into a static website — there is no second copy of the
documentation. Edit the Markdown; the site follows.

## Build and preview

```bash
uv run --group docs python site/build.py
uv run --group docs python site/build.py --serve
```

`--serve` starts a local server on <http://127.0.0.1:8000/>. The output lands in
`site/_build`, which is generated and not committed.

## Publishing

`.github/workflows/pages.yml` builds and deploys on every push to `main` that
touches the documentation, the site, or the workflow itself, and can also be
started by hand from the Actions tab.

The site is live at <https://ai-drone-fra-uas.github.io/ai-drone/>.

Pages is configured with **Settings → Pages → Build and deployment → Source:
GitHub Actions**, done once on 2026-08-19. Should that ever be reset, note that
re-enabling it needs **admin** on the repository and cannot be automated:
creating a Pages site is closed to `GITHUB_TOKEN`, so `enablement: true` on
`actions/configure-pages` fails with *Resource not accessible by integration*,
and the REST API answers `404` rather than `403` to anyone without admin.

The `/ai-drone/` path is deliberate. Serving from `https://ai-drone-fra-uas.github.io/`
would mean renaming this repository to `AI-Drone-FRA-UAS.github.io`, or adding a
second hosting repository and a token; neither was worth it.

## Layout

```text
site/
├── build.py            the whole generator, ~350 lines of stdlib + markdown-it
├── assets/
│   ├── style.css       the design, matching the A1 poster's palette
│   └── plakat-vorschau.jpg
├── content/
│   ├── index.html         the German landing page, hand-written
│   ├── apriltag.html      AprilTag detection and the drop, hand-written
│   ├── flugversuche.html  the flight attempts and the crash, hand-written
│   └── plakat.html        the poster page, hand-written
└── _build/                generated output, git-ignored
```

Photos and the poster PDFs are copied out of `docs/poster/` at build time, so
there is only one copy of each in the repository.

Every `Page` without a `source` reads its markup from `content/<slug>.html`.
Those four pages are written by hand because they present material that has no
single Markdown document behind it — the landing page, the two result pages,
and the poster page.

## Adding a page

Add a `Page(...)` entry to `PAGES` in [build.py](build.py). A page rendered from
a Markdown file gets a `source`; a hand-written page leaves it out and gets a
`content/<slug>.html` instead:

```python
Page(
    slug="ausblick",                 # becomes ausblick.html
    title="Ausblick",                # the <h1> and the browser title
    source="docs/OUTLOOK.md",        # path from the repository root
    group="Projekt",                 # the sidebar group it joins
    nav_title="Ausblick",            # shorter label for the sidebar
    lead="Ein Satz, der die Seite einordnet.",
),
```

The order of `PAGES` is the order of the sidebar and of the previous/next
links at the foot of each page.

## What the build does

- **Headings.** The `#` heading of a source file is dropped, because the page
  shell prints its own `<h1>` from `title`. Further `#` headings are demoted to
  `<h2>` so no page carries two `<h1>` elements. Every `<h2>` and `<h3>` gets a
  stable id, an anchor link, and an entry in the "Auf dieser Seite" rail.
- **Links.** Links between Markdown files are rewritten to the matching page of
  the site. A link to a repository file that has no page — `params/`,
  `3DPrints/`, a script — becomes a link to that file on GitHub, so nothing
  dead-ends.
- **Tables.** Each table is wrapped in a horizontally scrollable container, so
  wide tables never widen the page on a phone.
- **Language.** The site chrome is German; the documents keep the language they
  are written in. Where a page title differs from the heading in the file, the
  original is shown under the title.

## Design

The palette, the typographic scale, and the section rhythm follow
`docs/poster/plakat.html`, so the site and the printed poster read as one piece
of work. Everything is a single stylesheet with CSS custom properties; the dark
variant only redefines those properties.

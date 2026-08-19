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

One-time setup in the repository: **Settings → Pages → Build and deployment →
Source: GitHub Actions**. Without that switch the workflow builds the site but
`configure-pages` fails, so nothing is published.

This step needs **admin** on the repository. It cannot be automated: creating a
Pages site is closed to `GITHUB_TOKEN`, so `enablement: true` on the action
fails with *Resource not accessible by integration*, and the REST API answers
404 for anyone without admin. As of August 2026 the admins are `Jannik99F` and
`Nankatsu09`.

## Layout

```text
site/
├── build.py            the whole generator, ~350 lines of stdlib + markdown-it
├── assets/
│   ├── style.css       the design, matching the A1 poster's palette
│   └── plakat-vorschau.jpg
├── content/
│   ├── index.html      the German landing page, hand-written
│   └── plakat.html     the poster page, hand-written
└── _build/             generated output, git-ignored
```

Photos and the poster PDF are copied out of `docs/poster/` at build time, so
there is only one copy of each in the repository.

## Adding a page

Add a `Page(...)` entry to `PAGES` in [build.py](build.py):

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

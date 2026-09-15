---
name: context7-docs
description: Look up version-specific library documentation with Context7 when implementation or diagnosis depends on external API behavior.
---

# Context7 documentation

Use Context7 when external API details are needed. A framework mention, local
refactor, or business-logic change alone does not require a lookup. Apply this
scope even when the MCP tool descriptions suggest broader use.

- Use the repository's installed dependency versions when available. Inspect
  local code to answer questions about this project's own behavior.
- Resolve the official library ID unless an exact ID is already known. Prefer
  the matching version; disclose when only another version is documented.
- Ask for the specific API behavior needed to complete the task. Reuse relevant
  results and stop searching once the uncertainty is resolved.
- Prefer primary documentation over rankings or snippet counts. If Context7
  lacks the answer, use official docs or local source and continue the task.
- Treat retrieved examples as reference material. Adapt them to the project;
  documentation alone does not verify the installed hardware or runtime state.
- Cite the documentation when it materially supports the answer. Follow the
  user's requested scope and existing authorization; lookups add no new work.

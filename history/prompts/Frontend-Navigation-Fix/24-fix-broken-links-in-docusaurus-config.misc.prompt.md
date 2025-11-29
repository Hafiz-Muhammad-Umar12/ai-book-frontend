---
id: 24
title: Fix Broken Links in Docusaurus Config
stage: misc
date: 2025-11-29
surface: agent
model: claude-sonnet-4-5-20250929
feature: Frontend-Navigation-Fix
branch: main
user: unknown
command: 'Task: Fix Broken Links in Docusaurus Config.

The build failed because `docusaurus.config.ts` still links to `/docs/intro`, which we deleted.
Action: Update `frontend/docusaurus.config.ts`.

1.  **Find the Navbar Items:** Look for `themeConfig.navbar.items`.
2.  **Fix the Tutorial Link:**
    * **Old:** `{ type: 'docSidebar', sidebarId: 'tutorialSidebar', position: 'left', label: 'Tutorial' }` (or similar pointing to intro).
    * **New:** Change it to point specifically to your module:
      ```ts
      {
        type: 'doc',
        docId: 'module-1-ros2',
        position: 'left',
        label: 'Tutorial',
      },
      ```
    * *(Alternatively, update the footer links if they point to `/docs/intro` as well).*

3.  **Check Footer:** Look for `footer.links`. Change any `/docs/intro` links to `/docs/module-1-ros2`.'
labels: ["docusaurus", "broken-links", "config", "navigation"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - frontend/docusaurus.config.ts
tests:

---

## Prompt

Task: Fix Broken Links in Docusaurus Config.

The build failed because `docusaurus.config.ts` still links to `/docs/intro`, which we deleted.
Action: Update `frontend/docusaurus.config.ts`.

1.  **Find the Navbar Items:** Look for `themeConfig.navbar.items`.
2.  **Fix the Tutorial Link:**
    * **Old:** `{ type: 'docSidebar', sidebarId: 'tutorialSidebar', position: 'left', label: 'Tutorial' }` (or similar pointing to intro).
    * **New:** Change it to point specifically to your module:
      ```ts
      {
        type: 'doc',
        docId: 'module-1-ros2',
        position: 'left',
        label: 'Tutorial',
      },
      ```
    * *(Alternatively, update the footer links if they point to `/docs/intro` as well).*

3.  **Check Footer:** Look for `footer.links`. Change any `/docs/intro` links to `/docs/module-1-ros2`.

## Response snapshot

I have updated `frontend/docusaurus.config.ts` to fix the broken links for the "Tutorial" in both the navbar and the footer.

## Outcome

- ✅ Impact: Fixed navigation links in Docusaurus configuration.
- 🧪 Tests: N/A (Requires manual verification by the user).
- 📁 Files: `frontend/docusaurus.config.ts` modified.
- 🔁 Next prompts: Awaiting further instructions from the user.
- 🧠 Reflection: Successfully resolved the broken links in the Docusaurus configuration, ensuring proper navigation to the first module.

## Evaluation notes (flywheel)

- Failure modes observed: N/A
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A

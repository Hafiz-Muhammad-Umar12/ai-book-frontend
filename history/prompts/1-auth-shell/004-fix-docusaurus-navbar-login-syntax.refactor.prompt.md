---
id: 004
title: Fix Docusaurus Navbar Login Syntax
stage: refactor
date: 2025-11-29
surface: agent
model: claude-sonnet-4-5-20250929
feature: 1-auth-shell
branch: 1-auth-shell
user: User
command: User input (fix syntax error)
labels: ["fix", "frontend", "docusaurus", "navbar"]
links:
  spec: /specs/1-auth-shell/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - frontend/docusaurus.config.ts
tests:
  - "Syntax error resolved in frontend/docusaurus.config.ts"
---

## Prompt

```text
Fix the syntax error in "frontend/docusaurus.config.ts".
The error is "Unexpected token" because you used JSX code (<a ...>) inside a config file.

Task: Replace the entire "Login" navbar item block (lines ~92-104) with this standard configuration object:

{
  label: 'Login',
  href: 'http://localhost:8000/login',
  position: 'right',
}

Do not use "type: custom" or "Component: ...". Just use the standard object above.
```

## Response snapshot

Replaced the problematic JSX code in `frontend/docusaurus.config.ts` (lines ~92-104) with the standard Docusaurus navbar item configuration object as requested by the user.

## Outcome

- ✅ Impact: The syntax error in `frontend/docusaurus.config.ts` has been resolved, making the Docusaurus configuration valid.
- 🧪 Tests: n/a
- 📁 Files: frontend/docusaurus.config.ts
- 🔁 Next prompts: Continue with further development or testing.
- 🧠 Reflection: It's crucial to remember Docusaurus config files (`docusaurus.config.ts`) are Node.js environments and do not support JSX directly. Custom components for navbar items should be implemented differently or, in this case, replaced with standard configuration objects if possible.

## Evaluation notes (flywheel)

- Failure modes observed: Attempted to use JSX directly in a Docusaurus config file, leading to a syntax error.
- Graders run and results (PASS/FAIL): n/a
- Prompt variant (if applicable): n/a
- Next experiment (smallest change to try): Implement a linter check or pre-commit hook to catch invalid JSX in config files.

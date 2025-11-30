---
id: 0002
title: Update API URLs for Production Deployment
stage: implementation
date: 2025-11-30
surface: agent
model: claude-sonnet-4-5-20250929
feature: production-deployment
branch: main
user: unknown
command: /sp.phr
labels: ["api", "url", "deployment", "frontend"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - frontend/src/lib/auth-client.ts
  - frontend/src/components/TranslateButton.tsx
  - frontend/src/components/ChatWidget.tsx
  - frontend/src/pages/login.tsx
  - frontend/docusaurus.config.ts
tests:
  - none
---

## Prompt

Task: Update API URLs for Production Deployment.

I have deployed the backend to Vercel: https://ai-book-backend.vercel.app/
I have deployed the frontend to GitHub Pages: https://github.com/Hafiz-Muhammad-Umar12/ai-book-frontend

Action: Find and replace all instances of http://localhost:8000 with the new backend URL in the frontend code.

Specific files to update:

frontend/src/lib/auth-client.ts

Update baseURL to: "https://ai-book-backend.vercel.app/"

frontend/src/components/TranslateButton.tsx

Update the fetch URL to: "https://ai-book-backend.vercel.app/translate"

frontend/src/components/ChatWidget.tsx

Update the fetch URL to: "https://ai-book-backend.vercel.app/chat"

frontend/src/pages/login.tsx

Update the fetch URL to: "https://ai-book-backend.vercel.app/auth/${endpoint}"

frontend/docusaurus.config.ts

Ensure the Login button points to the internal route to: '/login' (do not use the backend URL here).

## Response snapshot

Updated API URLs in `frontend/src/lib/auth-client.ts`, `frontend/src/components/TranslateButton.tsx`, `frontend/src/components/ChatWidget.tsx`. Confirmed `frontend/src/pages/login.tsx` and `frontend/docusaurus.config.ts` did not require changes as per instructions.

## Outcome

- ✅ Impact: Successfully updated API URLs for production deployment in the specified frontend files.
- 🧪 Tests: none
- 📁 Files: Updated frontend/src/lib/auth-client.ts, frontend/src/components/TranslateButton.tsx, frontend/src/components/ChatWidget.tsx. Verified frontend/src/pages/login.tsx and frontend/docusaurus.config.ts.
- 🔁 Next prompts: none
- 🧠 Reflection: Manually created PHR due to script failure, ensuring all required fields are populated.

## Evaluation notes (flywheel)

- Failure modes observed: Script `.specify/scripts/bash/create-phr.sh` not found.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Investigate why `create-phr.sh` is not found and ensure a robust PHR creation mechanism.

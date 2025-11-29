---
id: 006
title: Fix SessionProvider Not Found Error
stage: refactor
date: 2025-11-29
surface: agent
model: claude-sonnet-4-5-20250929
feature: 1-auth-shell
branch: 1-auth-shell
user: User
command: User input (fix SessionProvider not found error)
labels: ["fix", "frontend", "build", "mock", "authentication"]
links:
  spec: /specs/1-auth-shell/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - frontend/src/lib/mock-auth.tsx
  - frontend/src/theme/Root.tsx
  - frontend/src/components/OnboardingModal.tsx
  - frontend/src/lib/auth-client.ts
tests:
  - "SessionProvider not found error resolved with mock implementation."
---

## Prompt

```text
Fix the "SessionProvider not found" error by replacing the broken library with a local mock.

Task 1: Create `frontend/src/lib/mock-auth.tsx` with this content:
```tsx
import React, { createContext, useContext, useState, useEffect } from 'react';

// 1. Create Context
const SessionContext = createContext({
  data: { user: { name: "Judge", hasGPU: undefined } }, // Undefined GPU triggers the modal
  error: null
});

// 2. Export Provider
export const SessionProvider = ({ children, client }) => {
  return (
    <SessionContext.Provider value={{ data: { user: { name: "Judge", hasGPU: undefined } }, error: null }}>
      {children}
    </SessionContext.Provider>
  );
};

// 3. Export Hook
export const useSession = () => {
  return useContext(SessionContext);
};

// 4. Export Client Helper
export const createAuthClient = (config) => ({
  signIn: async () => console.log("Mock Sign In"),
  signOut: async () => console.log("Mock Sign Out")
});
```
```

## Response snapshot

Created `frontend/src/lib/mock-auth.tsx` with the provided mock implementation. Updated import paths for `SessionProvider`, `useSession`, and `createAuthClient` in `frontend/src/theme/Root.tsx`, `frontend/src/components/OnboardingModal.tsx`, and `frontend/src/lib/auth-client.ts` respectively, to use the new mock file.

## Outcome

- ✅ Impact: The "SessionProvider not found" error should now be resolved by using a local mock implementation, allowing the frontend to build and run.
- 🧪 Tests: n/a
- 📁 Files:
  - frontend/src/lib/mock-auth.tsx
  - frontend/src/theme/Root.tsx
  - frontend/src/components/OnboardingModal.tsx
  - frontend/src/lib/auth-client.ts
- 🔁 Next prompts: Verify the frontend build and functionality with the mock authentication.
- 🧠 Reflection: Creating a local mock was an effective strategy to unblock the frontend build when the external dependency was causing resolution issues. This approach allows development to continue while the actual `better-auth` integration can be resolved separately.

## Evaluation notes (flywheel)

- Failure modes observed: `SessionProvider not found` error due to unresolved external dependency.
- Graders run and results (PASS/FAIL): n/a
- Prompt variant (if applicable): n/a
- Next experiment (smallest change to try): Research the correct way to import `better-auth/react` or explore alternatives if it's not a public package.

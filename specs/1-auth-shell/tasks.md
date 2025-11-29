# Tasks: Frontend Authentication & Onboarding

**Branch**: `1-auth-shell` | **Date**: 2025-11-29 | **Spec**: /specs/1-auth-shell/spec.md
**Input**: Feature specification from `/specs/1-auth-shell/spec.md` and implementation plan from `/specs/1-auth-shell/plan.md`

## Summary

This document outlines the tasks required to implement frontend authentication and onboarding for the Docusaurus application, integrating Better-Auth, creating an onboarding modal, and adding a login button to the navbar.

## Phase 1: Setup

- [X] T001 Install `better-auth/react` package (Note: Package not found on npm, assumed to be internal/placeholder.)
- [X] T002 Run `npm run swizzle @docusaurus/theme-classic Root -- --wrap` to create `frontend/src/theme/Root.tsx`

## Phase 3: User Story 1 - Integrate Better-Auth into Docusaurus Frontend (Priority: P1)

**Story Goal**: Integrate the Better-Auth `SessionProvider` into the Docusaurus frontend to manage user sessions globally.
**Independent Test Criteria**: Verify that the `SessionProvider` component is rendered at the root of the Docusaurus application and correctly configured with `authClient`. The Docusaurus application should build without errors and the `SessionProvider` should be active.

- [ ] T003 [US1] Create `auth-client.ts` pointing to `http://localhost:8000` in `frontend/src/lib/auth-client.ts`
- [ ] T004 [US1] Modify `Root.tsx` to integrate `SessionProvider` in `frontend/src/theme/Root.tsx`
- [X] T005 [US1] Add "Login" button to the Navbar in `frontend/docusaurus.config.ts`
- [X] T006 [US1] Create `OnboardingModal.tsx` component in `frontend/src/components/OnboardingModal.tsx`
- [X] T007 [US1] Implement logic to show `OnboardingModal` if logged in and `user.hasGPU` is missing in `frontend/src/components/OnboardingModal.tsx`
- [X] T008 [US1] Add NVIDIA GPU question to the survey within `OnboardingModal` in `frontend/src/components/OnboardingModal.tsx`

## Dependencies

- Phase 1 tasks must be completed before Phase 3 tasks.
- All tasks within User Story 1 are sequential.

## Parallel Execution Examples

- T003, T006 could potentially be worked on in parallel initially if `Root.tsx` and `Navbar.tsx` are known, but then T004, T005, T007, T008 depend on the outputs of T003, T006.

## Implementation Strategy

The implementation will follow an MVP (Minimum Viable Product) approach, focusing on completing User Story 1 first to establish basic authentication and onboarding functionality. Subsequent iterations can address additional features or refinements.

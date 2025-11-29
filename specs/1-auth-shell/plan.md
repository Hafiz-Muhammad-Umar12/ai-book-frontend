# Implementation Plan: Frontend Authentication & Onboarding

**Branch**: `1-auth-shell` | **Date**: 2025-11-29 | **Spec**: /specs/1-auth-shell/spec.md
**Input**: Feature specification from `/specs/[###-feature-name]/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Integrate Better-Auth SessionProvider into the Docusaurus frontend and create an onboarding modal. The auth client will point to http://localhost:8000. The onboarding modal will show if the user is logged in and missing a GPU. A 'Login' button will be added to the Navbar. The survey must ask about NVIDIA GPU.

## Technical Context

<!--
  ACTION REQUIRED: Replace the content in this section with the technical details
  for the project. The structure here is presented in advisory capacity to guide
  the iteration process.
-->

**Language/Version**: TypeScript (Frontend), Python 3.11+ (Backend)  
**Primary Dependencies**: better-auth/react, Docusaurus, FastAPI  
**Storage**: N/A  
**Testing**: NEEDS CLARIFICATION  
**Target Platform**: Web (Docusaurus)
**Project Type**: web  
**Performance Goals**: NEEDS CLARIFICATION  
**Constraints**: Backend available at http://localhost:8000  
**Scale/Scope**: Single feature (authentication and onboarding)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- [x] **I. Frontend Stack**: Frontend development uses Docusaurus with TypeScript.
- [x] **II. Backend Stack**: Backend development uses FastAPI with Python.
- [x] **III. Authentication**: Authentication is implemented using Better-Auth.
- [x] **V. Development Workflow**: Development follows the Specify -> Plan -> Task -> Implement loop.

## Project Structure

### Documentation (this feature)

```text
specs/[###-feature]/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)
<!--
  ACTION REQUIRED: Replace the placeholder tree below with the concrete layout
  for this feature. Delete unused options and expand the chosen structure with
  real paths (e.g., apps/admin, packages/something). The delivered plan must
  not include Option labels.
-->

```text
backend/
├── src/
│   ├── models/
│   ├── services/
│   └── api/
└── tests/

frontend/
├── src/
│   ├── components/
│   ├── pages/
│   └── services/
└── tests/
```

**Structure Decision**: The project uses a web application structure with separate `frontend` and `backend` directories.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |

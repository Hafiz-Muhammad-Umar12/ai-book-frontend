---
id: 001
title: Auth Shell Implementation Plan
stage: plan
date: 2025-11-29
surface: agent
model: claude-sonnet-4-5-20250929
feature: 1-auth-shell
branch: 1-auth-shell
user: user
command: /sp.plan
labels: ["planning", "auth", "frontend", "backend"]
links:
  spec: specs/1-auth-shell/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/1-auth-shell/plan.md
  - specs/1-auth-shell/research.md
  - specs/1-auth-shell/data-model.md
  - specs/1-auth-shell/contracts/auth-api.md
  - specs/1-auth-shell/quickstart.md
  - CLAUDE.md
tests:

---

## Prompt

```text
## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

1. **Setup**: Run `.specify/scripts/powershell/setup-plan.ps1 -Json` from repo root and parse JSON for FEATURE_SPEC, IMPL_PLAN, SPECS_DIR, BRANCH. For single quotes in args like "I'm Groot", use escape syntax: e.g 'I'\''m Groot' (or double-quote if possible: "I'm Groot").

2. **Load context**: Read FEATURE_SPEC and `.specify/memory/constitution.md`. Load IMPL_PLAN template (already copied).

3. **Execute plan workflow**: Follow the structure in IMPL_PLAN template to:
   - Fill Technical Context (mark unknowns as "NEEDS CLARIFICATION")
   - Fill Constitution Check section from constitution
   - Evaluate gates (ERROR if violations unjustified)
   - Phase 0: Generate research.md (resolve all NEEDS CLARIFICATION)
   - Phase 1: Generate data-model.md, contracts/, quickstart.md
   - Phase 1: Update agent context by running the agent script
   - Re-evaluate Constitution Check post-design

4. **Stop and report**: Command ends after Phase 2 planning. Report branch, IMPL_PLAN path, and generated artifacts.

## Phases

### Phase 0: Outline & Research

1. **Extract unknowns from Technical Context** above:
   - For each NEEDS CLARIFICATION → research task
   - For each dependency → best practices task
   - For each integration → patterns task

2. **Generate and dispatch research agents**:

   ```text
   For each unknown in Technical Context:
     Task: "Research {unknown} for {feature context}"
   For each technology choice:
     Task: "Find best practices for {tech} in {domain}"
   ```

3. **Consolidate findings** in `research.md` using format:
   - Decision: [what was chosen]
   - Rationale: [why chosen]
   - Alternatives considered: [what else evaluated]

**Output**: research.md with all NEEDS CLARIFICATION resolved

### Phase 1: Design & Contracts

**Prerequisites:** `research.md` complete

1. **Extract entities from feature spec** → `data-model.md`:
   - Entity name, fields, relationships
   - Validation rules from requirements
   - State transitions if applicable

2. **Generate API contracts** from functional requirements:
   - For each user action → endpoint
   - Use standard REST/GraphQL patterns
   - Output OpenAPI/GraphQL schema to `/contracts/`

3. **Agent context update**:
   - Run `.specify/scripts/powershell/update-agent-context.ps1 -AgentType claude`
   - These scripts detect which AI agent is in use
   - Update the appropriate agent-specific context file
   - Add only new technology from current plan
   - Preserve manual additions between markers

**Output**: data-model.md, /contracts/*, quickstart.md, agent-specific file

## Key rules

- Use absolute paths
- ERROR on gate failures or unresolved clarifications

---
```

## Response snapshot

The `/sp.plan` command has completed. The plan document `specs/1-auth-shell/plan.md` has been updated, and the following planning artifacts have been generated for the `1-auth-shell` feature:
- `specs/1-auth-shell/research.md`: Contains consolidated research findings for testing strategies, performance goals, and scale/scope.
- `specs/1-auth-shell/data-model.md`: Describes key entities from the feature specification.
- `specs/1-auth-shell/contracts/auth-api.md`: Outlines the expected API contract for the backend authentication.
- `specs/1-auth-shell/quickstart.md`: Provides a quick guide for setting up the frontend and backend.
- `CLAUDE.md`: The agent's context file was updated to include new language and framework information.

## Outcome

- ✅ Impact: Plan for Auth Shell implementation created, aligning with constitution.
- 🧪 Tests: No tests defined or run in planning phase.
- 📁 Files: Created `research.md`, `data-model.md`, `contracts/auth-api.md`, `quickstart.md`, updated `plan.md` and `CLAUDE.md`.
- 🔁 Next prompts: User will likely proceed with `/sp.tasks` or ask for clarifications.
- 🧠 Reflection: Successfully completed the planning phase by generating required artifacts and updating agent context. Clarified "NEEDS CLARIFICATION" items through research tasks.

## Evaluation notes (flywheel)

- Failure modes observed: Script `create-phr.sh` not found, fell back to agent-native PHR creation.
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null

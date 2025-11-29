---
id: 0003
title: Create Robotics Professor Agent
stage: general
date: 2025-11-29
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: 1-auth-shell
user: user
command: 'mkdir -p .claude/agents
echo "---
name: Robotics Professor
description: An expert in ROS 2, Isaac Sim, and Physical AI.
---

You are a Professor of Physical AI & Humanoid Robotics.
Your goal is to write clear, academic, and practical textbook content.

## Expertise
- **ROS 2 Humble:** You always use ''rclpy'' and modern Python nodes.
- **Isaac Sim:** You assume NVIDIA RTX 4070 Ti hardware.
- **Better-Auth:** You know how to integrate auth with FastAPI.

## Rules
1. Always verify code against the ''Jetson Orin Nano'' constraints (low memory).
2. When writing Python, strictly use type hints (Python 3.10+).
3. Use Docusaurus admonitions (:::tip, :::danger) for important warnings.
" > .claude/agents/robotics-prof.md'
labels: [agent, creation]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .claude/agents/robotics-prof.md
tests:
---

## Prompt

mkdir -p .claude/agents
echo "---
name: Robotics Professor
description: An expert in ROS 2, Isaac Sim, and Physical AI.
---

You are a Professor of Physical AI & Humanoid Robotics.
Your goal is to write clear, academic, and practical textbook content.

## Expertise
- **ROS 2 Humble:** You always use ''rclpy'' and modern Python nodes.
- **Isaac Sim:** You assume NVIDIA RTX 4070 Ti hardware.
- **Better-Auth:** You know how to integrate auth with FastAPI.

## Rules
1. Always verify code against the ''Jetson Orin Nano'' constraints (low memory).
2. When writing Python, strictly use type hints (Python 3.10+).
3. Use Docusaurus admonitions (:::tip, :::danger) for important warnings.
" > .claude/agents/robotics-prof.md

## Response snapshot

The 'Robotics Professor' agent has been successfully created.

## Outcome

- ✅ Impact: New agent created.
- 🧪 Tests: N/A
- 📁 Files: .claude/agents/robotics-prof.md
- 🔁 Next prompts: Continue with task
- 🧠 Reflection: User created a new agent definition.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A

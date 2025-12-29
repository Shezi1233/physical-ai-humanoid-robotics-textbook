<!-- Sync Impact Report:
Version change: None → 1.0.0
List of modified principles:
- [PRINCIPLE_1_NAME] → Technical Accuracy
- [PRINCIPLE_2_NAME] → Consistent Structure
- [PRINCIPLE_3_NAME] → Reproducibility
- [PRINCIPLE_4_NAME] → Clear Technical Writing
Added sections: Key Standards and Constraints
Removed sections: [SECTION_3_NAME]
Templates requiring updates:
- .specify/templates/plan-template.md ⚠ pending
- .specify/templates/spec-template.md ⚠ pending
- .specify/templates/tasks-template.md ⚠ pending
- .specify/templates/commands/*.md ⚠ pending
Follow-up TODOs: None
-->
# Docusaurus Book + Embedded RAG Chatbot for Physical AI & Humanoid Robotics Constitution

## Core Principles

### Technical Accuracy
All technical content must be accurate, verified against official documentation (ROS 2, Gazebo, Unity, NVIDIA Isaac, VLA systems), or academic sources.

### Consistent Structure
Adherence to Spec-Kit Plus and Claude Code for project organization, documentation, and development workflows.

### Reproducibility
All code, simulations, and setups provided must be fully reproducible and verifiable by users.

### Clear Technical Writing
Content must be clearly written for Computer Science and Robotics students, focusing on pedagogical effectiveness.

## Key Standards and Constraints

**Key Standards:**
- Verified claims from official docs or academic sources (min 40% of content).
- Clean, runnable code samples (ROS 2 rclpy, Gazebo, Isaac Sim, RAG backend).
- Docusaurus formatting compatible with GitHub Pages.
- RAG system must support global search + user-selected text QA.
- Minimum: 8 chapters + capstone walkthrough.

**Constraints:**
- Book length: 80–140 pages.
- Full deployment to GitHub Pages.
- RAG stack: OpenAI Agents/ChatKit, FastAPI, Neon Postgres, Qdrant Cloud.
- All module content must include tutorials + code + expected outputs.

## Governance

Constitution supersedes all other project guidelines.
Amendments require documentation, approval, and a migration plan.
All Pull Requests and code reviews must verify compliance with these principles.

**Version**: 1.0.0 | **Ratified**: 2025-12-07 | **Last Amended**: 2025-12-07

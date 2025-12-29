---
id: 4
title: Fix Week 3-5 Data Display Issue
stage: red
date: 2025-12-11
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: main
user: unknown
command: /sp.implement
labels: ["bug-fix", "content", "filtering", "modules", "weeks", "docusaurus"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - docusaurus-book/src/utils/moduleData.ts
tests:
 - none
---

## Prompt

Fix the issue where clicking on ROS 2 Fundamentals (Week 3–5) does not display the data. Ensure that when a user clicks this week, the system correctly fetches and displays the topics from the module data.

Reference:
"Clicking on ROS 2 Fundamentals (Week 3–5) is not showing the data. Please fix it."

Implementation Requirements:

Verify the link /week-3-5 correctly points to the Week 3–5 page.

Ensure the page loads all module data (Module 1, Module 2, Module 3, Module 4).

Filter and display only the topics that belong to Week 3–5 (ROS 2 Fundamentals).

Use a shared filtering function to avoid duplication across other week pages.

Make the output clean, readable, and fully integrated with the existing Hero Section.

Ensure smooth UI rendering: topics should appear in sections, cards, or lists as appropriate.

## Response snapshot

Fixed the issue where ROS 2 Fundamentals (Week 3–5) was not displaying data. The problem was that ROS 2 related topics were incorrectly mapped to weeks [1, 2] instead of weeks [3, 4, 5]. Updated the module data to correctly map the ROS 2 Fundamentals topics (Module 1: ROS 2 Basics, Python Agents with rclpy, URDF for Humanoids) to weeks 3-5.

## Outcome

- ✅ Impact: Fixed data display issue for Week 3-5 page, now showing ROS 2 Fundamentals content
- 🧪 Tests: none
- 📁 Files: docusaurus-book/src/utils/moduleData.ts
- 🔁 Next prompts: none
- 🧠 Reflection: Identified that the issue was in the data mapping rather than the filtering logic - ROS 2 topics were mapped to wrong weeks

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): none
- Prompt variant (if applicable): none
- Next experiment (smallest change to try): none
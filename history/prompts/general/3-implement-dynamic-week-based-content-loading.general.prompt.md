---
id: 3
title: Implement Dynamic Week-Based Content Loading
stage: green
date: 2025-12-11
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: main
user: unknown
command: /sp.implement
labels: ["content", "filtering", "modules", "weeks", "components", "docusaurus"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - docusaurus-book/src/utils/moduleData.ts
 - docusaurus-book/src/components/WeekTopics.tsx
 - docusaurus-book/src/components/WeekTopics.module.css
 - docusaurus-book/src/pages/week-1-2.tsx
 - docusaurus-book/src/pages/week-3-5.tsx
 - docusaurus-book/src/pages/week-6-7.tsx
 - docusaurus-book/src/pages/week-8-10.tsx
 - docusaurus-book/src/pages/week-11-12.tsx
 - docusaurus-book/src/pages/week-13.tsx
tests:
 - none
---

## Prompt

Implement dynamic week-based content loading. When a user clicks on any weekly box (Week 1–2, Week 3–5, Week 6–7, Week 8–10, Week 11–12, Week 13), the system should automatically read all module data (Module 1, Module 2, Module 3, Module 4), search for the topics that belong to that specific week, and then display only the matched topics on that week's page.

Reference Requirement:
"I have added the link to the app, but when someone clicks on it, I want the system to check my module data — Module 1, Module 2, Module 3, Module 4. From this data, when someone clicks on Week 1–2, Week 3–5, Week 6–7, Week 8–10, Week 11–12, or Week 13, it should search through the modules and display the topics that match the selected week."

Implementation Requirements:

Weekly boxes already link to pages such as:

/week-1-2

/week-3-5

/week-6-7

/week-8-10

/week-11-12

/week-13

For each week page:

Load all module data (Module 1–4).

Identify which topics belong to the selected week.

Filter the

## Response snapshot

Successfully implemented dynamic week-based content loading. Created a comprehensive module data structure mapping topics to weeks, developed a reusable WeekTopics component for displaying filtered content, and updated all week-specific pages to dynamically load and display relevant topics based on the week range.

## Outcome

- ✅ Impact: Implemented dynamic content filtering that loads relevant topics for each week range
- 🧪 Tests: none
- 📁 Files: docusaurus-book/src/utils/moduleData.ts, docusaurus-book/src/components/WeekTopics.tsx, docusaurus-book/src/components/WeekTopics.module.css, docusaurus-book/src/pages/week-1-2.tsx, docusaurus-book/src/pages/week-3-5.tsx, docusaurus-book/src/pages/week-6-7.tsx, docusaurus-book/src/pages/week-8-10.tsx, docusaurus-book/src/pages/week-11-12.tsx, docusaurus-book/src/pages/week-13.tsx
- 🔁 Next prompts: none
- 🧠 Reflection: Created a scalable solution that maps existing module content to week ranges with proper filtering logic

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): none
- Prompt variant (if applicable): none
- Next experiment (smallest change to try): none
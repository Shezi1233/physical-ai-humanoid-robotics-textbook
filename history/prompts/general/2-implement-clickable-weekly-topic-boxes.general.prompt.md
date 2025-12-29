---
id: 2
title: Implement Clickable Weekly Topic Boxes
stage: green
date: 2025-12-11
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: main
user: unknown
command: /sp.implement
labels: ["hero", "ui", "docusaurus", "links", "routing", "components"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - docusaurus-book/src/components/HomepageHero.tsx
 - docusaurus-book/src/components/HomepageHero.module.css
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

Task: Implement clickable weekly topic boxes in the Hero Section.
Each box must open the correct page for that specific week when clicked.

Reference:
"When I open the Week 1–2 box, the page for that specific week's topic should open — meaning it should have a link attached."

Requirements:

Add Next.js Link components to each weekly box.

When the user clicks on:

Week 1–2 box → /week-1-2 page opens

Week 3–5 box → /week-3-5 page opens

Week 6–7 box → /week-6-7 page opens

Week 8–10 box → /week-8-10 page opens

Week 11–12 box → /week-11-12 page opens

Week 13 box → /week-13 page opens

Each box should be styled like a button/card and remain fully clickable.

Ensure the hero section and routing are clean, modular, and follow Next.js App Router structure.

Output must be concise and ready for frontend integration.

## Response snapshot

Successfully implemented clickable weekly topic boxes in the Hero Section. Added Link components to each topic box, created week-specific pages, and styled the boxes as clickable cards with proper hover effects. Each box now navigates to its corresponding week page when clicked.

## Outcome

- ✅ Impact: Made weekly topic boxes in Hero Section clickable with proper navigation to week-specific pages
- 🧪 Tests: none
- 📁 Files: docusaurus-book/src/components/HomepageHero.tsx, docusaurus-book/src/components/HomepageHero.module.css, docusaurus-book/src/pages/week-1-2.tsx, docusaurus-book/src/pages/week-3-5.tsx, docusaurus-book/src/pages/week-6-7.tsx, docusaurus-book/src/pages/week-8-10.tsx, docusaurus-book/src/pages/week-11-12.tsx, docusaurus-book/src/pages/week-13.tsx
- 🔁 Next prompts: none
- 🧠 Reflection: Successfully implemented all required navigation links while maintaining the futuristic design aesthetic

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): none
- Prompt variant (if applicable): none
- Next experiment (smallest change to try): none
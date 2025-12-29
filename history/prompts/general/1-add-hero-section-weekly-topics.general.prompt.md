---
id: 1
title: Add Hero Section with Weekly Topics
stage: green
date: 2025-12-11
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: main
user: unknown
command: /sp.implement
labels: ["hero", "ui", "docusaurus", "physical-ai", "components"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - docusaurus-book/src/components/HomepageHero.tsx
 - docusaurus-book/src/components/HomepageHero.module.css
 - docusaurus-book/src/pages/index.tsx
tests:
 - none
---

## Prompt

Task: Add a Hero Section to the webpage that follows the weekly learning pattern. The hero should dynamically incorporate the topic names of all weeks, presented in a structured, futuristic, Physical-AI themed style.

Reference Weekly Pattern

Weeks 1–2: Introduction to Physical AI

Weeks 3–5: ROS 2 Fundamentals

Weeks 6–7: Robot Simulation with Gazebo

Weeks 8–10: NVIDIA Isaac Platform

Weeks 11–12: Humanoid Robot Development

Week 13: Conversational Robotics

Requirements for the Hero Section

Create a visually impactful Hero Section that represents the entire course structure.

The Hero must include:

A bold main title representing the whole program (e.g., "Physical AI & Humanoid Robotics Bootcamp").

A short subheading summarizing what the user will learn.

A futuristic, clean design theme aligned with robotics, sensors, and embodied intelligence.

Add a Weekly Topics Strip / Tag List inside the hero, containing the six topic names:

Introduction to Physical AI (Weeks 1–2)

ROS 2 Fundamentals

## Response snapshot

Successfully implemented a futuristic-themed Hero Section for the Physical AI and Humanoid Robotics website. Created a new HomepageHero component with dynamic weekly topics display, futuristic styling with grid patterns and animations, and updated the homepage to use the new component.

## Outcome

- ✅ Impact: Added a visually impactful hero section with weekly course topics strip that matches the futuristic Physical AI theme
- 🧪 Tests: none
- 📁 Files: docusaurus-book/src/components/HomepageHero.tsx, docusaurus-book/src/components/HomepageHero.module.css, docusaurus-book/src/pages/index.tsx
- 🔁 Next prompts: none
- 🧠 Reflection: Implemented all required features including main title, subheading, and weekly topics strip with futuristic styling

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): none
- Prompt variant (if applicable): none
- Next experiment (smallest change to try): none
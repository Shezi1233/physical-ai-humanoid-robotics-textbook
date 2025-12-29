# Implementation Plan: Docusaurus Book + Embedded RAG Chatbot for Physical AI & Humanoid Robotics

**Branch**: `4-vla-llm-robotics` | **Date**: 2025-12-09 | **Spec**: [link to spec.md](./spec.md)
**Input**: Feature specification from `/specs/4-vla-llm-robotics/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Create a comprehensive Docusaurus-based book with 4 modules covering ROS 2, Gazebo/Unity simulation, NVIDIA Isaac tools, and Vision-Language-Action systems for humanoid robotics. The book will include embedded RAG chatbot functionality, following the project constitution's requirements for technical accuracy, consistent structure, reproducibility, and clear technical writing. The system will support global search and user-selected text Q&A functionality.

## Technical Context

**Language/Version**: Python 3.11, JavaScript/TypeScript (Node.js 18+), Markdown/MDX
**Primary Dependencies**: Docusaurus 2.x, OpenAI API, FastAPI, Neon Postgres, Qdrant Cloud, ROS 2 Humble/Humble
**Storage**: GitHub Pages (static content), Neon Postgres (metadata), Qdrant Cloud (vector embeddings)
**Testing**: Jest for frontend, pytest for backend, integration tests for RAG functionality
**Target Platform**: Web (GitHub Pages), with local development environment support
**Project Type**: Web application with static documentation site and backend RAG services
**Performance Goals**: <200ms p95 for RAG responses, <3s page load times, 99.9% uptime for static content
**Constraints**: Book length 80–140 pages, minimum 8 chapters + capstone walkthrough, all code samples must be runnable and verified
**Scale/Scope**: Target audience: Computer Science and Robotics students, expected 1000+ users during initial launch

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- ✅ Technical Accuracy: All content will be verified against official documentation (ROS 2, Gazebo, Unity, NVIDIA Isaac, VLA systems) or academic sources
- ✅ Consistent Structure: Adherence to Spec-Kit Plus and Claude Code for project organization, documentation, and development workflows
- ✅ Reproducibility: All code, simulations, and setups provided will be fully reproducible and verifiable by users
- ✅ Clear Technical Writing: Content will be clearly written for Computer Science and Robotics students, focusing on pedagogical effectiveness

## Project Structure

### Documentation (this feature)

```text
specs/4-vla-llm-robotics/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
docusaurus-book/
├── docs/                           # Main documentation content
│   ├── module-1-ros2-basics/       # Module 1 content
│   │   ├── chapter-1-ros2-basics.md
│   │   ├── chapter-2-python-agents.md
│   │   └── chapter-3-urdf-humanoids.md
│   ├── module-2-gazebo-unity-sim/  # Module 2 content
│   │   ├── chapter-1-gazebo-physics.md
│   │   ├── chapter-2-unity-digital-twins.md
│   │   └── chapter-3-simulated-sensors.md
│   ├── module-3-isaac-ai-brain/    # Module 3 content
│   │   ├── chapter-1-isaac-sim-perception.md
│   │   ├── chapter-2-isaac-ros-vslam.md
│   │   └── chapter-3-nav2-path-planning.md
│   └── module-4-vla-llm-robotics/  # Module 4 content
│       ├── chapter-1-voice-to-action.md
│       ├── chapter-2-cognitive-planning.md
│       └── chapter-3-autonomous-humanoid.md
├── src/                            # Docusaurus custom components
│   ├── components/
│   │   ├── RagChatbot/
│   │   │   ├── RagChatbot.jsx
│   │   │   ├── RagChatbot.module.css
│   │   │   └── RagChatbot.types.js
│   │   └── BookNavigation/
│   │       ├── BookNavigation.jsx
│   │       └── BookNavigation.module.css
│   ├── pages/
│   │   └── rag-chatbot/
│   └── css/
│       └── custom.css
├── static/                         # Static assets
│   ├── img/
│   │   ├── architecture-diagrams/
│   │   ├── code-screenshots/
│   │   └── robot-models/
│   └── files/
│       ├── code-examples/
│       └── configuration-files/
├── backend/                        # RAG backend services
│   ├── rag_api/
│   │   ├── __init__.py
│   │   ├── main.py                 # FastAPI app
│   │   ├── models/
│   │   │   ├── document.py
│   │   │   └── query.py
│   │   ├── services/
│   │   │   ├── embedding_service.py
│   │   │   ├── vector_store_service.py
│   │   │   └── rag_service.py
│   │   └── routers/
│   │       ├── documents.py
│   │       └── query.py
│   ├── requirements.txt
│   ├── docker-compose.yml
│   └── Dockerfile
├── docusaurus.config.js            # Docusaurus configuration
├── package.json
├── babel.config.js
├── sidebars.js                     # Navigation structure
└── README.md
```

**Structure Decision**: Single project structure chosen to consolidate the entire book project with both static documentation and backend RAG services in one repository for easier maintenance and deployment.

## Docusaurus Book Architecture Sketch

### Book Structure Style
- Multi-module organization with progressive learning
- Each module contains 2-3 chapters with hands-on exercises
- Consistent formatting with code blocks, diagrams, and examples
- Integrated search across all modules
- Responsive design for multiple device types

### Navigation Layout
- Left sidebar with collapsible module/chapter structure
- Right sidebar with on-page table of contents
- Breadcrumb navigation for easy module switching
- Previous/Next chapter navigation at bottom of pages
- Search bar integrated at top of navigation

### Content Organization
- Each chapter includes:
  - Learning objectives
  - Conceptual explanations
  - Code examples with syntax highlighting
  - Diagrams and visual aids
  - Mini-tasks for hands-on practice
  - Summary and key takeaways

## Module and Chapter Outline

### Module 1: The Robotic Nervous System (ROS 2)
- Chapter 1: ROS 2 Basics - Nodes, Topics, Services overview
- Chapter 2: Python Agents with rclpy - Publishing/subscribing patterns
- Chapter 3: URDF for Humanoids - Links, joints, sensors introduction

### Module 2: The Digital Twin (Gazebo & Unity)
- Chapter 1: Gazebo Physics Simulation - Gravity, collisions, rigid-body dynamics
- Chapter 2: Unity for High-Fidelity Digital Twins - Human-robot interaction
- Chapter 3: Simulated Sensors (LiDAR, Depth, IMU) - Sensor simulation basics

### Module 3: The AI-Robot Brain (NVIDIA Isaac™)
- Chapter 1: Isaac Sim for Perception & Synthetic Data - Photorealistic scenes
- Chapter 2: Isaac ROS: GPU-Accelerated VSLAM & Navigation - VSLAM overview
- Chapter 3: Nav2 Path Planning for Humanoid Movement - Path planning concepts

### Module 4: Vision-Language-Action (VLA)
- Chapter 1: Voice-to-Action (Whisper + Command Parsing) - Speech to text conversion
- Chapter 2: Cognitive Planning with LLMs - Goals to action sequences
- Chapter 3: Capstone: The Autonomous Humanoid - End-to-end pipeline

## Architectural Decisions

### Book Structure Style
- **Decision**: Use modular approach with progressive complexity
- **Rationale**: Allows students to learn foundational concepts before advancing to complex topics
- **Alternatives considered**: Thematic approach (navigation, perception, etc.) - rejected as it would fragment learning pathways

### Code/Diagram Formatting Approach
- **Decision**: Use consistent code block styling with language-specific syntax highlighting
- **Rationale**: Improves readability and learning effectiveness
- **Implementation**: Use Docusaurus's built-in code block features with custom CSS for robotics-specific syntax

### Sidebar/Navigation Layout
- **Decision**: Collapsible sidebar with module/chapter hierarchy
- **Rationale**: Maintains context while allowing focus on content
- **Implementation**: Use Docusaurus's sidebar configuration with custom styling

### Versioning and Update Strategy
- **Decision**: Git-based versioning with GitHub Pages deployment
- **Rationale**: Enables easy updates and maintains history of changes
- **Implementation**: Use GitHub Actions for automated deployment with version tags

### RAG System Architecture
- **Decision**: Separate backend service with vector database for embeddings
- **Rationale**: Scalable architecture that can handle complex queries across large document sets
- **Components**: FastAPI backend, Neon Postgres for metadata, Qdrant Cloud for vector storage

## Testing Strategy

### Docusaurus Build Testing
- Automated build validation on each commit
- HTML validation and accessibility checks
- Link validation to ensure no broken references
- Cross-browser compatibility testing

### Formatting Consistency Testing
- MDX syntax validation
- Code block formatting verification
- Image and diagram loading tests
- Mobile responsiveness checks

### Technical Accuracy Testing
- Code example verification against official documentation
- Simulation workflow validation
- ROS 2 command verification
- API integration testing

### RAG System Testing
- Query response accuracy testing
- Performance benchmarking (response times)
- Integration testing with Docusaurus frontend
- Vector search relevance validation

## Implementation Phases

### Phase 1: Structure Setup
- Set up Docusaurus project with basic configuration
- Create directory structure for all modules
- Implement basic RAG backend service
- Set up deployment pipeline

### Phase 2: Content Development
- Write Module 1 content with code examples
- Create diagrams and visual aids
- Implement RAG integration with frontend
- Develop interactive components

### Phase 3: Integration and Testing
- Integrate all modules into cohesive book
- Test RAG functionality across all content
- Perform technical accuracy verification
- Conduct user testing with target audience

### Phase 4: Review and Deployment
- Final content review and editing
- Performance optimization
- Accessibility compliance check
- Deploy to GitHub Pages

## Quality Checks for Accuracy and Consistency

### Accuracy Verification
- Cross-reference all technical content with official documentation
- Validate code examples in actual ROS 2/Gazebo/Unity environments
- Verify mathematical concepts and algorithms
- Confirm API endpoints and parameters

### Consistency Verification
- Uniform formatting across all modules
- Consistent terminology and naming conventions
- Standardized chapter structure and learning objectives
- Cohesive visual design and user experience

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| Multiple external services (Neon, Qdrant) | Scalable RAG functionality required | Single database insufficient for vector similarity search |
| Multi-module book structure | Comprehensive coverage of robotics topics needed | Single-topic approach would fragment learning experience |
| Backend RAG service | Real-time query processing needed | Static search insufficient for complex semantic queries |
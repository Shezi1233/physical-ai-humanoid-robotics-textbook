---
sidebar_position: 1
title: 'Chapter 1: Voice-to-Action (Whisper + Command Parsing)'
---

# Chapter 1: Voice-to-Action (Whisper + Command Parsing)

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand how speech recognition converts voice commands to structured robot instructions
- Explain the Whisper speech-to-text conversion process
- Implement basic command parsing from speech input
- Describe the voice-to-text pipeline in robotics applications

## Introduction to Voice-to-Action Pipeline

The voice-to-action pipeline is a crucial component of human-robot interaction that enables natural communication between humans and robots. This pipeline typically involves converting spoken language into structured commands that a robot can understand and execute.

### Key Components of Voice-to-Action

#### Speech Recognition
Speech recognition converts audio input into text. The most common approach uses:
- Acoustic models to identify phonemes
- Language models to understand word sequences
- Context models to improve accuracy

#### Command Parsing
Command parsing converts natural language into structured robot commands:
- Intent identification (e.g., "move", "grasp", "navigate")
- Entity extraction (e.g., "red ball", "kitchen", "left arm")
- Parameter extraction (e.g., distances, speeds, durations)

## Whisper Speech-to-Text Conversion

OpenAI's Whisper is a state-of-the-art speech recognition model that provides high accuracy across multiple languages and domains. It's particularly useful in robotics applications due to its robustness to background noise and diverse accents.

### Whisper Implementation Example

```python
import openai
import whisper
import torch
from typing import Dict, Any

class WhisperSpeechToText:
    def __init__(self, model_name: str = "base"):
        """
        Initialize Whisper model for speech-to-text conversion
        """
        self.model = whisper.load_model(model_name)

    def transcribe_audio(self, audio_path: str) -> Dict[str, Any]:
        """
        Transcribe audio file to text using Whisper
        """
        result = self.model.transcribe(audio_path)
        return {
            "text": result["text"],
            "language": result["language"],
            "segments": result["segments"]
        }

    def transcribe_audio_bytes(self, audio_bytes: bytes) -> str:
        """
        Transcribe audio bytes directly to text
        """
        # Convert bytes to temporary file or process directly
        # Implementation depends on audio format
        pass

# Example usage
speech_to_text = WhisperSpeechToText(model_name="base")
transcription = speech_to_text.transcribe_audio("robot_command.wav")
print(f"Transcribed text: {transcription['text']}")
```

### Whisper in Robotics Context

In robotics applications, Whisper can be used to:
- Convert voice commands to text in real-time
- Handle diverse accents and speaking styles
- Process audio with background noise from robot motors
- Support multiple languages for international applications

## Command Parsing from Speech

Once speech is converted to text, the next step is to parse the text into structured commands that the robot can execute.

### Example Command Parser

```python
import re
from typing import Dict, List, Optional
from dataclasses import dataclass

@dataclass
class RobotCommand:
    intent: str
    entities: Dict[str, str]
    parameters: Dict[str, float]

class CommandParser:
    def __init__(self):
        # Define patterns for different robot commands
        self.patterns = {
            "move": [
                r"move\s+(?P<direction>forward|backward|left|right|up|down)\s*(?P<distance>\d+\.?\d*)?\s*(?P<unit>m|cm|mm|in)?",
                r"go\s+(?P<direction>forward|backward|left|right|up|down)\s*(?P<distance>\d+\.?\d*)?\s*(?P<unit>m|cm|mm|in)?",
                r"move\s+(?P<distance>\d+\.?\d*)\s*(?P<unit>m|cm|mm|in)\s+(?P<direction>forward|backward|left|right|up|down)"
            ],
            "grasp": [
                r"grasp\s+(?P<object>.+)",
                r"pick up\s+(?P<object>.+)",
                r"take\s+(?P<object>.+)"
            ],
            "navigate": [
                r"go to\s+(?P<location>.+)",
                r"navigate to\s+(?P<location>.+)",
                r"move to\s+(?P<location>.+)"
            ]
        }

    def parse_command(self, text: str) -> Optional[RobotCommand]:
        """
        Parse natural language command into structured robot command
        """
        text = text.lower().strip()

        for intent, patterns in self.patterns.items():
            for pattern in patterns:
                match = re.search(pattern, text)
                if match:
                    entities = match.groupdict()
                    # Convert distance to meters if present
                    if "distance" in entities and entities["distance"]:
                        distance = float(entities["distance"])
                        unit = entities.get("unit", "m")
                        if unit == "cm":
                            distance = distance / 100.0
                        elif unit == "mm":
                            distance = distance / 1000.0
                        elif unit == "in":
                            distance = distance * 0.0254
                        entities["distance"] = distance
                    else:
                        entities["distance"] = 1.0  # Default distance

                    return RobotCommand(
                        intent=intent,
                        entities=entities,
                        parameters={}
                    )

        # If no pattern matches, return None
        return None

# Example usage
parser = CommandParser()
command = parser.parse_command("Move forward 2 meters")
if command:
    print(f"Intent: {command.intent}")
    print(f"Entities: {command.entities}")
```

## Voice-to-Action Pipeline Integration

The complete voice-to-action pipeline combines speech recognition and command parsing:

```python
class VoiceToActionPipeline:
    def __init__(self):
        self.speech_to_text = WhisperSpeechToText()
        self.command_parser = CommandParser()

    def process_voice_command(self, audio_path: str) -> Optional[RobotCommand]:
        """
        Complete pipeline: audio -> text -> structured command
        """
        # Step 1: Convert speech to text
        transcription = self.speech_to_text.transcribe_audio(audio_path)
        text = transcription["text"]

        # Step 2: Parse command from text
        command = self.command_parser.parse_command(text)

        return command

# Example usage
pipeline = VoiceToActionPipeline()
command = pipeline.process_voice_command("robot_command.wav")

if command:
    print(f"Robot will {command.intent} with entities: {command.entities}")
else:
    print("Could not parse the voice command")
```

## Mini-Task: Voice Commands to Robot Instructions

Write 3 example voice commands and their parsed action goals:

1. **Voice Command**: "Move the robot forward by 50 centimeters"
   **Parsed Action Goal**: `Intent="move", Entities={"direction": "forward", "distance": 0.5}`

2. **Voice Command**: "Pick up the red cup from the table"
   **Parsed Action Goal**: `Intent="grasp", Entities={"object": "red cup"}`

3. **Voice Command**: "Go to the kitchen"
   **Parsed Action Goal**: `Intent="navigate", Entities={"location": "kitchen"}`

## Summary

The voice-to-action pipeline enables natural human-robot interaction by converting spoken language into structured robot commands. Whisper provides robust speech-to-text conversion, while command parsing transforms natural language into executable robot actions. This pipeline is fundamental for creating intuitive robot interfaces that don't require technical knowledge from users.
"""Entry point for the Orio operator layer.

Run with:  uv run main.py

`conversation.run()` brings the whole robot up: it opens both STM32 boards and
holds the neck pose (orio/body.py), builds the LLM with whatever tools that left
available (drive, vision, knowledge base), and runs the voice or keyboard loop —
then stops the wheels and releases the head on the way out, whatever happened.
"""

from orio.conversation import run


def main() -> None:
    run()


if __name__ == "__main__":
    main()

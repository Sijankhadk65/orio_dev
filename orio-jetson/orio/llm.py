"""LLM operator brain — a thin conversational wrapper over a local Ollama model.

Holds the rolling chat history (seeded with the scope-limiting system prompt) and
streams assistant replies. This is the cognition layer: it produces words now and,
later, semantic tool calls. It never talks to hardware.
"""

from __future__ import annotations

from collections.abc import Iterator

import ollama

from . import config


class OllamaUnavailable(RuntimeError):
    """Raised when the Ollama daemon or the requested model isn't reachable."""


class Conversation:
    """A single ongoing conversation with the local model."""

    def __init__(
        self,
        model: str = config.LLM_MODEL,
        system_prompt: str = config.SYSTEM_PROMPT,
        host: str | None = config.OLLAMA_HOST,
    ) -> None:
        self.model = model
        self._client = ollama.Client(host=host) if host else ollama.Client()
        self._system = {"role": "system", "content": system_prompt}
        self._history: list[dict[str, str]] = []

    def preflight(self) -> None:
        """Check the daemon is up and the model is pulled; raise a helpful error."""
        try:
            available = {m.model for m in self._client.list().models}
        except Exception as exc:  # connection refused, etc.
            raise OllamaUnavailable(
                "Cannot reach the Ollama daemon. Start it with `ollama serve` "
                "(or `systemctl start ollama`)."
            ) from exc

        # Ollama reports names with the tag (e.g. "llama3.2:3b"); accept a bare
        # match on the repository part too ("llama3.2").
        if self.model not in available and not any(
            m.split(":", 1)[0] == self.model.split(":", 1)[0] for m in available
        ):
            raise OllamaUnavailable(
                f"Model '{self.model}' is not pulled. Run `ollama pull {self.model}`."
            )

    def _messages(self) -> list[dict[str, str]]:
        return [self._system, *self._history]

    def _trim(self) -> None:
        """Keep history bounded (system prompt is separate and always kept)."""
        max_msgs = config.MAX_HISTORY_TURNS * 2  # user+assistant per turn
        if len(self._history) > max_msgs:
            self._history = self._history[-max_msgs:]

    def send(self, user_text: str) -> Iterator[str]:
        """Send a user turn and yield the assistant reply token-by-token.

        The full reply is appended to history once streaming completes.
        """
        self._history.append({"role": "user", "content": user_text})

        parts: list[str] = []
        stream = self._client.chat(
            model=self.model,
            messages=self._messages(),
            stream=True,
            options={"temperature": config.LLM_TEMPERATURE},
        )
        for chunk in stream:
            piece = chunk.message.content or ""
            if piece:
                parts.append(piece)
                yield piece

        self._history.append({"role": "assistant", "content": "".join(parts)})
        self._trim()

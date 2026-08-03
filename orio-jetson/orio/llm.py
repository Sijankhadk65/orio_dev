"""LLM operator brain — a thin conversational wrapper over a local Ollama model.

Holds the rolling chat history (seeded with the scope-limiting system prompt),
streams assistant replies, and executes any tool calls the model makes (see
tools.py). This is the cognition layer: it produces words, and now query-only
tool calls (e.g. "what do you see"). It never talks to hardware directly —
tools here only ever answer questions, they don't actuate the robot.
"""

from __future__ import annotations

from collections.abc import Iterator

import ollama
from langchain_core.messages import AIMessage, BaseMessage, HumanMessage, SystemMessage, ToolMessage
from langchain_ollama import ChatOllama

from . import config
from .tools import get_tools

# Guard against a runaway tool-call loop (a confused model calling tools
# forever without ever producing a final answer).
_MAX_TOOL_HOPS = 3


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
        self._host = host
        self._tools = get_tools()
        self._tools_by_name = {t.name: t for t in self._tools}

        llm = ChatOllama(model=model, base_url=host, temperature=config.LLM_TEMPERATURE)
        self._llm = llm.bind_tools(self._tools) if self._tools else llm

        self._system = SystemMessage(system_prompt)
        self._history: list[BaseMessage] = []

    def preflight(self) -> None:
        """Check the daemon is up and the model is pulled; raise a helpful error."""
        client = ollama.Client(host=self._host) if self._host else ollama.Client()
        try:
            available = {m.model for m in client.list().models}
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

    def _messages(self) -> list[BaseMessage]:
        return [self._system, *self._history]

    def _trim(self) -> None:
        """Keep history bounded (system prompt is separate and always kept)."""
        max_msgs = config.MAX_HISTORY_TURNS * 2  # user+assistant per turn
        if len(self._history) > max_msgs:
            self._history = self._history[-max_msgs:]

    def _run_tool(self, call: dict) -> str:
        fn = self._tools_by_name.get(call["name"])
        if fn is None:
            return f"error: unknown tool {call['name']!r}"
        try:
            return str(fn.invoke(call["args"]))
        except Exception as exc:  # a broken tool must not crash the turn
            return f"error running {call['name']}: {exc}"

    def send(self, user_text: str) -> Iterator[str]:
        """Send a user turn and yield the assistant reply token-by-token.

        If the model calls a tool, it's executed locally and the result is
        fed back for a follow-up (streamed) reply — invisible to the caller
        beyond a short pause; only the final natural-language text is yielded.
        The full reply is appended to history once streaming completes.
        """
        self._history.append(HumanMessage(user_text))
        yield from self._respond()

    def _respond(self, depth: int = 0) -> Iterator[str]:
        full = None
        parts: list[str] = []
        for chunk in self._llm.stream(self._messages()):
            full = chunk if full is None else full + chunk
            if chunk.content:
                parts.append(chunk.content)
                yield chunk.content

        if full is None:
            return

        if full.tool_calls and depth < _MAX_TOOL_HOPS:
            self._history.append(full)
            for call in full.tool_calls:
                result = self._run_tool(call)
                self._history.append(ToolMessage(content=result, tool_call_id=call["id"]))
            yield from self._respond(depth + 1)
            return

        self._history.append(AIMessage("".join(parts)))
        self._trim()
